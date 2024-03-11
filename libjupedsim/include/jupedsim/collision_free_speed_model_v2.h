/* Copyright © 2012-2024 Forschungszentrum Jülich GmbH */
/* SPDX-License-Identifier: LGPL-3.0-or-later */
#pragma once

#include "error.h"
#include "export.h"
#include "operational_model.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Opaque type for a Collision Free Speed Model v2 Builder
 */
typedef struct JPS_CollisionFreeSpeedModelv2Builder_t* JPS_CollisionFreeSpeedModelv2Builder;

/**
 * Creates a Collision Free Speed Model v2 builder.
 * @return the builder
 */
JUPEDSIM_API JPS_CollisionFreeSpeedModelv2Builder JPS_CollisionFreeSpeedModelv2Builder_Create();

/**
 * Creates a JPS_OperationalModel of type Collision Free Speed Model v2 from the
 * JPS_CollisionFreeSpeedModelv2Builder.
 * @param handle the builder to operate on
 * @param[out] errorMessage if not NULL: will be set to a JPS_ErrorMessage in case of an error
 * @return a JPS_CollisionFreeSpeedModelv2 or NULL if an error occured.
 */
JUPEDSIM_API JPS_OperationalModel JPS_CollisionFreeSpeedModelv2Builder_Build(
    JPS_CollisionFreeSpeedModelv2Builder handle,
    JPS_ErrorMessage* errorMessage);

/**
 * Frees a JPS_CollisionFreeSpeedModelv2Builder
 * @param handle to the JPS_CollisionFreeSpeedModelv2Builder to free.
 */
JUPEDSIM_API void
JPS_CollisionFreeSpeedModelv2Builder_Free(JPS_CollisionFreeSpeedModelv2Builder handle);

/**
 * Opaque type of Collision Free Speed v2 model state
 */
typedef struct JPS_CollisionFreeSpeedModelv2State_t* JPS_CollisionFreeSpeedModelv2State;

/**
 * Read strength neighbor repulsion of this agent.
 * @param handle of the Agent to access.
 * @return strength neighbor repulsion of this agent
 */
JUPEDSIM_API double JPS_CollisionFreeSpeedModelv2State_GetStrengthNeighborRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write strength neighbor repulsion of this agent.
 * @param handle of the Agent to access.
 * @param strengthNeighborRepulsion of this agent.
 */
JUPEDSIM_API void JPS_CollisionFreeSpeedModelv2State_SetStrengthNeighborRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle,
    double strengthNeighborRepulsion);

/**
 * Read range neighbor repulsion of this agent.
 * @param handle of the Agent to access.
 * @return range neighbor repulsion of this agent
 */
JUPEDSIM_API double JPS_CollisionFreeSpeedModelv2State_GetRangeNeighborRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write range neighbor repulsion of this agent.
 * @param handle of the Agent to access.
 * @param rangeNeighborRepulsion of this agent.
 */
JUPEDSIM_API void JPS_CollisionFreeSpeedModelv2State_SetRangeNeighborRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle,
    double rangeNeighborRepulsion);

/**
 * Read strength geometry repulsion of this agent.
 * @param handle of the Agent to access.
 * @return strength geometry repulsion of this agent
 */
JUPEDSIM_API double JPS_CollisionFreeSpeedModelv2State_GetStrengthGeometryRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write strength geometry repulsion of this agent.
 * @param handle of the Agent to access.
 * @param strengthGeometryRepulsion of this agent.
 */
JUPEDSIM_API void JPS_CollisionFreeSpeedModelv2State_SetStrengthGeometryRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle,
    double strengthGeometryRepulsion);

/**
 * Read range geometry repulsion of this agent.
 * @param handle of the Agent to access.
 * @return range geometry repulsion of this agent
 */
JUPEDSIM_API double JPS_CollisionFreeSpeedModelv2State_GetRangeGeometryRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write strength neighbor repulsion of this agent.
 * @param handle of the Agent to access.
 * @param rangeGeometryRepulsion of this agent.
 */
JUPEDSIM_API void JPS_CollisionFreeSpeedModelv2State_SetRangeGeometryRepulsion(
    JPS_CollisionFreeSpeedModelv2State handle,
    double rangeGeometryRepulsion);

/**
 * Read e0 of this agent.
 * @param handle of the Agent to access.
 * @return e0 of this agent
 */
JUPEDSIM_API JPS_Point
JPS_CollisionFreeSpeedModelv2State_GetE0(JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write e0 of this agent.
 * @param handle of the Agent to access.
 * @param e0 of this agent.
 */
JUPEDSIM_API void
JPS_CollisionFreeSpeedModelv2State_SetE0(JPS_CollisionFreeSpeedModelv2State handle, JPS_Point e0);

/**
 * Read time gap of this agent.
 * @param handle of the Agent to access.
 * @return time gap of this agent
 */
JUPEDSIM_API double
JPS_CollisionFreeSpeedModelv2State_GetTimeGap(JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write time gap of this agent.
 * @param handle of the Agent to access.
 * @param time_gap of this agent.
 */
JUPEDSIM_API void JPS_CollisionFreeSpeedModelv2State_SetTimeGap(
    JPS_CollisionFreeSpeedModelv2State handle,
    double time_gap);

/**
 * Read tau of this agent.
 * @param handle of the Agent to access.
 * @return tau of this agent
 */
JUPEDSIM_API double
JPS_CollisionFreeSpeedModelv2State_GetTau(JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write tau of this agent.
 * @param handle of the Agent to access.
 * @param tau of this agent.
 */
JUPEDSIM_API void
JPS_CollisionFreeSpeedModelv2State_SetTau(JPS_CollisionFreeSpeedModelv2State handle, double tau);

/**
 * Read v0 of this agent.
 * @param handle of the Agent to access.
 * @return v0 of this agent
 */
JUPEDSIM_API double
JPS_CollisionFreeSpeedModelv2State_GetV0(JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write v0 of this agent.
 * @param handle of the Agent to access.
 * @param v0 of this agent.
 */
JUPEDSIM_API void
JPS_CollisionFreeSpeedModelv2State_SetV0(JPS_CollisionFreeSpeedModelv2State handle, double v0);

/**
 * Read radius of this agent.
 * @param handle of the Agent to access.
 * @return radius of this agent
 */
JUPEDSIM_API double
JPS_CollisionFreeSpeedModelv2State_GetRadius(JPS_CollisionFreeSpeedModelv2State handle);

/**
 * Write radius of this agent in meters.
 * @param handle of the Agent to access.
 * @param radius (m) of this agent.
 */
JUPEDSIM_API void JPS_CollisionFreeSpeedModelv2State_SetRadius(
    JPS_CollisionFreeSpeedModelv2State handle,
    double radius);

/**
 * Describes parameters of an Agent in Collision Free Speed Model v2
 */
typedef struct JPS_CollisionFreeSpeedModelv2AgentParameters {
    /**
     * Position of the agent.
     * The position needs to inside the accessible area.
     */
    JPS_Point position{0, 0};
    /**
     * Defines the journey this agent will take use
     */
    JPS_JourneyId journeyId = 0;
    /**
     * Defines the current stage of its journey
     */
    JPS_StageId stageId = 0;

    /**
     * @param time_gap of the agents using this profile (T in the OV-function)
     */
    double time_gap = 1.;
    /**
     *@param v0 of the agents using this profile(desired speed) double radius;
     */
    double v0 = 1.2;
    /**
     *@param radius of the agent in 'meters'
     */
    double radius = 0.2;

    /**
     *  Strength of the repulsion from neighbors
     */
    double strengthNeighborRepulsion{8.0};

    /**
     * Range of the repulsion from neighbors
     */
    double rangeNeighborRepulsion{0.1};

    /**
     * Strength of the repulsion from geometry boundaries
     */
    double strengthGeometryRepulsion{5.0};

    /**
     * Range of the repulsion from geometry boundaries
     */
    double rangeGeometryRepulsion{0.02};

} JPS_CollisionFreeSpeedModelv2AgentParameters;

#ifdef __cplusplus
}
#endif
