/// Copyright © 2012-2022 Forschungszentrum Jülich GmbH
/// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "CollisionGeometry.hpp"
#include "GenericAgent.hpp"
#include "NeighborhoodSearch.hpp"
#include "OperationalModel.hpp"

#include <unordered_map>
#include <vector>

using my_pair = std::pair<double, GenericAgent::ID>;
// sort with respect to first element (ascending).
// In case of equality sort with respect to second element (descending)
struct sort_pred {
    bool operator()(const my_pair& left, const my_pair& right)
    {
        return (left.first == right.first) ? (left.second > right.second) :
                                             (left.first < right.first);
    }
};

// forward declaration
class GenericAgent;

struct VelocityModelAgentParameters {
    OperationalModel::ParametersID id;
    double timeGap;
    double tau;
    double v0;
    double radius;
};

/*!
 * \class VelocityModel
 *
 * \brief @todo
 *
 *\ingroup OperationalModels
 *
 * \author Mohcine Chraibi
 */
class VelocityModel : public OperationalModelBase<VelocityModelAgentParameters>
{
public:
    struct Data {
        /// Desired direction
        Point e0{};
        int orientationDelay{0};
    };
    using NeighborhoodSearchType = NeighborhoodSearch<std::tuple<GenericAgent*, Data*>>;

private:
    /// Modellparameter
    double _aPed;
    double _DPed;
    double _aWall;
    double _DWall;

public:
    VelocityModel(
        double aped,
        double Dped,
        double awall,
        double Dwall,
        const std::vector<VelocityModelAgentParameters>& profiles);
    ~VelocityModel() override = default;

    PedestrianUpdate ComputeNewPosition(
        double dT,
        const GenericAgent& ped,
        const Data& data,
        const CollisionGeometry& geometry,
        const NeighborhoodSearchType& neighborhoodSearch) const;

    void ApplyUpdate(const PedestrianUpdate& update, GenericAgent& agent, Data& modelData) const;
    std::unique_ptr<OperationalModel> Clone() const override;

private:
    /**
     * Optimal velocity function \f$ V(spacing) =\min{v_0, \max{0, (s-l)/T}}  \f$
     *
     * @param ped: Pointer to Pedestrians
     * @param spacing: minimum spacing to the neighbors
     * @param winkel: angle between <ped> and the nearest neighbor. Is not yet used!
     *
     * @return double
     */
    double OptimalSpeed(const GenericAgent* ped, double spacing, double t) const;

    /**
     * The desired direction of pedestrian
     *
     * @param ped: Pointer to Pedestrians
     * @param room: Pointer to room
     *
     * @return Point
     */
    void
    e0(const GenericAgent* ped,
       const Data& modelData,
       Point target,
       double deltaT,
       PedestrianUpdate& update) const;
    /**
     * Get the spacing between ped1 and ped2
     *
     * @param ped1 Pointer to Pedestrian: First pedestrian
     * @param ped2 Pointer to Pedestrian: Second pedestrian
     * @param ei the direction of pedestrian.
     * This direction is: \f$ e_0 + \sum_j{R(spacing_{ij})*e_{ij}}\f$
     * and should be calculated *before* calling OptimalSpeed
     * @return Point
     */
    my_pair GetSpacing(const GenericAgent* ped1, const GenericAgent* ped2, Point ei) const;
    /**
     * Repulsive force between two pedestrians ped1 and ped2 according to
     * the Velocity model (to be published in TGF15)
     *
     * @param ped1 Pointer to Pedestrian: First pedestrian
     * @param ped2 Pointer to Pedestrian: Second pedestrian
     *
     * @return Point
     */
    Point ForceRepPed(const GenericAgent* ped1, const GenericAgent* ped2) const;
    /**
     * Repulsive force acting on pedestrian <ped> from the walls in
     * <subroom>. The sum of all repulsive forces of the walls in <subroom> is calculated
     * @see ForceRepWall
     * @param ped Pointer to Pedestrian
     * @param subroom Pointer to SubRoom
     *
     * @return Point
     */
    Point ForceRepRoom(const GenericAgent* ped, const CollisionGeometry& geometry) const;
    /**
     * Repulsive force between pedestrian <ped> and wall <l>
     *
     * @param ped Pointer to Pedestrian
     * @param l reference to Wall
     *
     * @return Point
     */
    Point ForceRepWall(const GenericAgent* ped, const Line& l) const;
};
