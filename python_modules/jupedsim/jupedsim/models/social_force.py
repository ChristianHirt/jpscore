# Copyright © 2012-2024 Forschungszentrum Jülich GmbH
# SPDX-License-Identifier: LGPL-3.0-or-later
from dataclasses import dataclass

import jupedsim.native as py_jps


@dataclass(kw_only=True)
class SocialForceModel:
    """Parameters for Social Force Model

    All attributes are initialized with reasonably good defaults.


    Attributes:
        bodyForce: describes the strength with which an agent is influenced by pushing forces from obstacles and neighbors in its direct proximity.
        friction: describes the strength with which an agent is influenced by frictional forces from obstacles and neighbors in its direct proximity.
    """

    bodyForce: float = 120000
    friction: float = 240000


@dataclass(kw_only=True)
class SocialForceModelAgentParameters:
    """
    Parameters required to create an Agent in the Social Force Model.

    Attributes:
        position: Position of the agent.
        orientation: Orientation of the agent.
        journey_id: Id of the journey the agent follows.
        stage_id: Id of the stage the agent targets.
        velocity: current velocity of the agent
        mass: mass of the agent.
        desiredSpeed: desired Speed of the agent.
        reactionTime: reaction Time of the agent.
        agentScale: indicates how strong an agent is influenced by pushing forces from neighbors.
        obstacleScale: indicates how strong an agent is influenced by pushing forces from obstacles.
        forceDistance: indicates how much the distance between an agent and obstacles or neighbors influences social forces.
        radius: radius of the space an agent occupies.
    """
    # todo write force equation from paper
    position: tuple[float, float] = (0.0, 0.0)
    orientation: tuple[float, float] = (0.0, 0.0)
    journey_id: int = -1
    stage_id: int = -1
    velocity: tuple[float, float] = (0.0, 0.0)
    mass: float = 80.0
    desiredSpeed: float = 0.8
    reactionTime: float = 0.5
    agentScale: float = 2000
    obstacleScale: float = 2000
    forceDistance: float = 0.08
    radius: float = 0.3

    def as_native(
        self,
    ) -> py_jps.SocialForceModelAgentParameters:
        return py_jps.SocialForceModelAgentParameters(
            position=self.position,
            orientation=self.orientation,
            journey_id=self.journey_id,
            stage_id=self.stage_id,
            velocity=self.velocity,
            mass=self.mass,
            desiredSpeed=self.desiredSpeed,
            reactionTime=self.reactionTime,
            agentScale=self.agentScale,
            obstacleScale=self.obstacleScale,
            forceDistance=self.forceDistance,
            radius=self.radius,
        )


class SocialForceModelState:
    def __init__(self, backing) -> None:
        self._obj = backing

    @property
    def velocity(self) -> float:
        """velocity of this agent."""
        return self._obj.test_value

    @velocity.setter
    def velocity(self, velocity):
        self._obj.velocity = velocity

    @property
    def mass(self) -> float:
        """mass of this agent."""
        return self._obj.mass

    @mass.setter
    def mass(self, mass):
        self._obj.mass = mass

    @property
    def desiredSpeed(self) -> float:
        """desired Speed of this agent."""
        return self._obj.desiredSpeed

    @desiredSpeed.setter
    def desiredSpeed(self, desiredSpeed):
        self._obj.desiredSpeed = desiredSpeed

    @property
    def reactionTime(self) -> float:
        """reaction Time of this agent."""
        return self._obj.reactionTime

    @reactionTime.setter
    def reactionTime(self, reactionTime):
        self._obj.reactionTime = reactionTime

    @property
    def agentScale(self) -> float:
        """agent Scale of this agent."""
        return self._obj.agentScale

    @agentScale.setter
    def agentScale(self, agentScale):
        self._obj.agentScale = agentScale

    @property
    def obstacleScale(self) -> float:
        """obstacle Scale of this agent."""
        return self._obj.obstacleScale

    @obstacleScale.setter
    def obstacleScale(self, obstacleScale):
        self._obj.obstacleScale = obstacleScale

    @property
    def forceDistance(self) -> float:
        """force Distance of this agent."""
        return self._obj.forceDistance

    @forceDistance.setter
    def forceDistance(self, forceDistance):
        self._obj.forceDistance = forceDistance

    @property
    def radius(self) -> float:
        """radius of this agent."""
        return self._obj.radius

    @radius.setter
    def radius(self, radius):
        self._obj.radius = radius
