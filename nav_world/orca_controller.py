#!/usr/bin/env python3
"""
Lightweight ORCA controller wrapper used by the formation environment.

The controller keeps a PyRVOSimulator instance internally and exposes a small
API to initialise the simulator with the robot positions and query collision-
free velocities that guide the robots towards their goals.
"""

from __future__ import annotations

import numpy as np
from typing import List, Tuple

# Make rvo2 import optional - if not available, ORCA will not work
try:
    import rvo2
    HAS_RVO2 = True
except ImportError:
    HAS_RVO2 = False
    rvo2 = None


class ORCAController:
    """Thin wrapper on top of rvo2 for multi-robot collision avoidance."""

    def __init__(self, robot_radius: float = 0.08, max_speed: float = 1.5, dt: float = 0.05):
        """
        Initialize ORCA controller.
        
        Args:
            robot_radius: Effective collision radius (reduced to 0.08m to allow
                         closer formation spacing)
            max_speed: Maximum robot speed (m/s)
            dt: Simulation timestep (s)
        """
        self.dt = dt
        self.robot_radius = robot_radius
        self.max_speed = max_speed
        self.sim: rvo2.PyRVOSimulator | None = None
        self.agent_ids: List[int] = []
        # Increase neighbor distance and time horizon for better formation behavior
        # Larger values = more conservative collision avoidance
        self.neighbor_dist = 4.0  # Increased from 3.0 for better coordination
        self.time_horizon = 4.0  # Increased from 3.0 for longer lookahead

    def init_sim(self, positions: List[Tuple[float, float]]) -> None:
        """Initialise the ORCA simulator with one agent per robot."""
        if not HAS_RVO2:
            raise ImportError("rvo2 is not installed. Please install it to use ORCA collision avoidance.")
        self.sim = rvo2.PyRVOSimulator(
            self.dt,
            neighborDist=self.neighbor_dist,
            maxNeighbors=20,  # Increased for better multi-robot coordination
            timeHorizon=self.time_horizon,
            timeHorizonObst=self.time_horizon,
            radius=self.robot_radius,
            maxSpeed=self.max_speed,
        )
        self.agent_ids = []
        for (x, y) in positions:
            aid = self.sim.addAgent((x, y))
            self.agent_ids.append(aid)

    def compute_velocities(
        self,
        positions: List[Tuple[float, float]],
        goals: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        """
        Given current and goal positions, return safe velocities from ORCA.
        
        If a robot's distance to its goal is < 0.05m, it is considered arrived
        and returns zero velocity for that agent.
        """
        if self.sim is None or len(self.agent_ids) != len(positions):
            self.init_sim(positions)

        # Update agent positions
        for aid, (px, py) in zip(self.agent_ids, positions):
            self.sim.setAgentPosition(aid, (px, py))

        # Set preferred velocities towards the goals
        # Check goal reached condition before setting velocity
        goal_reached = []
        for aid, pos, goal in zip(self.agent_ids, positions, goals):
            dir_vec = np.array(goal) - np.array(pos)
            dist = np.linalg.norm(dir_vec)
            
            # Goal reached condition: distance < 0.05m
            if dist < 0.05:
                pref_vel = (0.0, 0.0)
                goal_reached.append(True)
            else:
                # Normalize direction and scale to max speed
                pref_vel = tuple(dir_vec / dist * self.max_speed)
                goal_reached.append(False)
            self.sim.setAgentPrefVelocity(aid, pref_vel)

        # Compute collision-free velocities
        self.sim.doStep()

        vels: List[Tuple[float, float]] = []
        for aid, reached in zip(self.agent_ids, goal_reached):
            if reached:
                # Robot has reached goal, return zero velocity
                vels.append((0.0, 0.0))
            else:
                vx, vy = self.sim.getAgentVelocity(aid)
                # Ensure velocity is not NaN or infinite
                if not (np.isfinite(vx) and np.isfinite(vy)):
                    vx, vy = 0.0, 0.0
                vels.append((vx, vy))
        return vels


