/// Copyright © 2012-2022 Forschungszentrum Jülich GmbH
/// SPDX-License-Identifier: LGPL-3.0-or-later
#include "VelocityModel.hpp"

#include "Agent.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"
#include "Simulation.hpp"

#include <Logger.hpp>
#include <memory>
#include <numeric>
#include <vector>

double xRight = 26.0;
double xLeft = 0.0;
double cutoff = 2.0;

VelocityModel::VelocityModel(
    double aped,
    double Dped,
    double awall,
    double Dwall,
    const std::vector<VelocityModelAgentParameters> profiles)
    : _aPed(aped), _DPed(Dped), _aWall(awall), _DWall(Dwall)
{
    _parameterProfiles.reserve(profiles.size());
    for(auto&& p : profiles) {
        auto [_, success] = _parameterProfiles.try_emplace(p.id, p);
        if(!success) {
            throw std::runtime_error("Duplicate agent profile id supplied");
        }
    }
}

PedestrianUpdate VelocityModel::ComputeNewPosition(
    double dT,
    const Agent& ped,
    const CollisionGeometry& geometry,
    const NeighborhoodSearch& neighborhoodSearch) const
{
    const double radius = 4.0;
    const auto neighborhood = neighborhoodSearch.GetNeighboringAgents(ped.GetPos(), radius);
    const auto parameters = _parameterProfiles.at(ped._parametersId);
    double min_spacing = 100.0;
    Point repPed = Point(0, 0);
    const Point p1 = ped.GetPos();
    for(const auto* other : neighborhood) {
        if(other->GetUID() == ped.GetUID()) {
            continue;
        }
        if(!geometry.IntersectsAny(Line(p1, other->GetPos()))) {
            repPed += ForceRepPed(&ped, other);
        }
    }
    // repulsive forces to walls and closed transitions that are not my target
    Point repWall = ForceRepRoom(&ped, geometry);

    // calculate new direction ei according to (6)
    PedestrianUpdate update{};
    e0(&ped, ped.destination, parameters.tau, update);
    const Point direction = update.e0 + repPed + repWall;
    for(const auto* other : neighborhood) {
        if(other->GetUID() == ped.GetUID()) {
            continue;
        }
        if(!geometry.IntersectsAny(Line(p1, other->GetPos()))) {
            double spacing = GetSpacing(&ped, other, direction).first;
            min_spacing = std::min(min_spacing, spacing);
        }
    }

    update.velocity = direction.Normalized() * OptimalSpeed(&ped, min_spacing, parameters.timeGap);
    update.position = ped.GetPos() + *update.velocity * dT;
    if(update.velocity->Norm() >= J_EPS_V) {
        update.resetPhi = true;
    }
    return update;
};

void VelocityModel::ApplyUpdate(const PedestrianUpdate& update, Agent& agent) const
{
    if(update.resetTurning) {
        agent.SetSmoothTurning();
    } else {
        agent.IncrementOrientationDelay();
    }
    agent.SetE0(update.e0);
    if(update.resetPhi) {
        agent.SetPhiPed();
    }
    if(update.position) {
        agent.SetPos(*update.position);
    }
    if(update.velocity) {
        agent.SetV(*update.velocity);
    }
}

std::unique_ptr<OperationalModel> VelocityModel::Clone() const
{
    return std::make_unique<VelocityModel>(*this);
}

void VelocityModel::e0(const Agent* ped, Point target, double tau, PedestrianUpdate& update) const
{
    Point desired_direction;
    const auto pos = ped->GetPos();
    const auto dest = ped->destination;
    const auto dist = (dest - pos).Norm();
    if(dist > J_EPS_GOAL) {
        desired_direction = ped->GetE0(target, tau);
    } else {
        update.resetTurning = true;
        desired_direction = ped->GetE0();
    }
    update.e0 = desired_direction;
}

double VelocityModel::OptimalSpeed(const Agent* ped, double spacing, double t) const
{
    double v0 = ped->GetV0();
    double l = 2 * ped->GetEllipse().GetBmax(); // assume peds are circles with const radius
    double speed = (spacing - l) / t;
    speed = (speed > 0) ? speed : 0;
    speed = (speed < v0) ? speed : v0;
    //      (1-winkel)*speed;
    // todo use winkel
    return speed;
}

// return spacing and id of the nearest pedestrian
my_pair VelocityModel::GetSpacing(const Agent* ped1, const Agent* ped2, Point ei) const
{
    Point distp12 = ped2->GetPos() - ped1->GetPos(); // inversed sign
    double Distance = distp12.Norm();
    double l = 2 * ped1->GetEllipse().GetBmax();
    Point ep12;
    if(Distance >= J_EPS) {
        ep12 = distp12.Normalized();
    } else {
        LOG_WARNING(
            "VelocityModel::GetSPacing() ep12 can not be calculated! Pedestrians are to close "
            "to "
            "each other ({:f})",
            Distance);
        exit(EXIT_FAILURE); // TODO
    }

    double condition1 = ei.ScalarProduct(ep12); // < e_i , e_ij > should be positive
    double condition2 =
        ei.Rotate(0, 1).ScalarProduct(ep12); // theta = pi/2. condition2 should <= than l/Distance
    condition2 = (condition2 > 0) ? condition2 : -condition2; // abs

    if((condition1 >= 0) && (condition2 <= l / Distance)) {
        // return a pair <dist, condition1>. Then take the smallest dist. In case of equality the
        // biggest condition1
        return my_pair(distp12.Norm(), ped2->GetUID());
    }
    return my_pair(FLT_MAX, ped2->GetUID());
}
Point VelocityModel::ForceRepPed(const Agent* ped1, const Agent* ped2) const
{
    Point F_rep(0.0, 0.0);
    // x- and y-coordinate of the distance between p1 and p2
    Point distp12 = ped2->GetPos() - ped1->GetPos();
    double Distance = distp12.Norm();
    Point ep12; // x- and y-coordinate of the normalized vector between p1 and p2
    double R_ij;
    double l = 2 * ped1->GetEllipse().GetBmax();

    if(Distance >= J_EPS) {
        ep12 = distp12.Normalized();
    } else {
        LOG_ERROR(
            "VelocityModel::forcePedPed() ep12 can not be calculated! Pedestrians are too near "
            "to "
            "each other (dist={:f}). Adjust <a> value in force_ped to counter this. Affected "
            "pedestrians ped1 {} at ({:f},{:f}) and ped2 {} at ({:f}, {:f})",
            Distance,
            ped1->GetUID(),
            ped1->GetPos().x,
            ped1->GetPos().y,
            ped2->GetUID(),
            ped2->GetPos().x,
            ped2->GetPos().y);
        exit(EXIT_FAILURE); // TODO: quick and dirty fix for issue #158
                            //  (sometimes sources create peds on the same location)
    }
    Point ei = ped1->GetV().Normalized();
    if(ped1->GetV().NormSquare() < 0.01) {
        ei = ped1->GetE0().Normalized();
    }
    double condition1 = ei.ScalarProduct(ep12); // < e_i , e_ij > should be positive
    condition1 = (condition1 > 0) ? condition1 : 0; // abs

    R_ij = -_aPed * exp((l - Distance) / _DPed);
    F_rep = ep12 * R_ij;

    return F_rep;
}

Point VelocityModel::ForceRepRoom(const Agent* ped, const CollisionGeometry& geometry) const
{
    auto walls = geometry.LineSegmentsInDistanceTo(5.0, ped->GetPos());

    auto f = std::accumulate(
        walls.begin(),
        walls.end(),
        Point(0, 0),
        [this, &ped](const auto& acc, const auto& element) {
            return acc + ForceRepWall(ped, element);
        });
    return f;
}

Point VelocityModel::ForceRepWall(const Agent* ped, const Line& w) const
{
    if(const auto distGoal = (ped->destination - ped->GetPos()).Norm(); distGoal < J_EPS_GOAL) {
        return Point{0, 0};
    }

    const Point pt = w.ShortestPoint(ped->GetPos());
    const Point dist_vec = pt - ped->GetPos();
    const double dist = dist_vec.Norm();
    const Point e_iw = dist_vec / dist;

    const double l = ped->GetEllipse().GetBmax();
    const double R_iw = -_aWall * exp((l - dist) / _DWall);
    return e_iw * R_iw;
}
