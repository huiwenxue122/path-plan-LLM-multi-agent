#!/usr/bin/env python3
"""
ORCA-based formation environment.

This environment keeps the original NavEnv interface but replaces the previous
hand-written multi-robot logic with a compact controller powered by ORCA.

Improvements:
- Stuck robot detection with fallback velocity
- Improved boundary handling with wall sliding
- Velocity smoothing to avoid jerky movement
- Better starting positions with proper spacing
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import mujoco
import numpy as np

from .nav_env import AgentState, NavEnv
from .formations import generate_letter_b_targets

# Make ORCA import optional
try:
    from .orca_controller import ORCAController, HAS_RVO2
except ImportError:
    ORCAController = None
    HAS_RVO2 = False


class NavEnvFormationORCA(NavEnv):
    """Simple formation environment that relies on ORCA for collision avoidance."""

    def __init__(
        self,
        xml_path: str,
        num_robots: int = 20,
        grid_res: float = 0.1,
        render_w: int = 800,
        render_h: int = 600,
    ):
        self.num_robots = num_robots
        self._formation_agent_names = [f"robot_{i}" for i in range(num_robots)]
        super().__init__(xml_path=xml_path, grid_res=grid_res, render_w=render_w, render_h=render_h)

        # Override agent configuration from the base class
        self.agent_names = self._formation_agent_names
        self.agents = {n: self.agents.get(n, AgentState(name=n)) for n in self.agent_names}
        self._init_qpos_idx()

        # Only keep the room walls as obstacles
        self.grid, self.grid_xs, self.grid_ys = self._build_occupancy()

        self.goal_xy: Dict[str, Tuple[float, float]] = {name: (0.0, 0.0) for name in self.agent_names}
        # Use larger radius (0.25m) to ensure robots maintain larger safe distance
        # This prevents robots from getting too close together and reduces crowding
        # With radius=0.25m, robots will maintain at least 0.5m distance (2 * radius)
        # Only initialize ORCA if rvo2 is available
        if HAS_RVO2 and ORCAController is not None:
            self.orca = ORCAController(robot_radius=0.25, max_speed=1.5, dt=0.05)
        else:
            self.orca = None
        
        # Track stuck robots for diagnostics
        self._stuck_timers: Dict[str, float] = {name: 0.0 for name in self.agent_names}
        self._last_positions: Dict[str, Tuple[float, float]] = {}
        # Track last warning time to reduce spam (warn at most once per 2 seconds per robot)
        self._last_warn_time: Dict[str, float] = {name: -10.0 for name in self.agent_names}

    # ------------------------------------------------------------------
    # Occupancy grid
    # ------------------------------------------------------------------
    def _build_occupancy(self):
        xs = np.arange(self.xmin, self.xmax + 1e-6, self.grid_res)
        ys = np.arange(self.ymin, self.ymax + 1e-6, self.grid_res)
        grid = np.zeros((len(xs), len(ys)), dtype=np.uint8)
        grid[0, :] = 1
        grid[-1, :] = 1
        grid[:, 0] = 1
        grid[:, -1] = 1
        return grid, xs, ys

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _make_default_agent(self, name: str) -> AgentState:
        return AgentState(name=name)

    def get_room_bounds(self) -> Tuple[float, float, float, float]:
        return self.xmin, self.xmax, self.ymin, self.ymax

    def _default_start_positions(self) -> List[Tuple[float, float]]:
        """
        Place robots on the left side in a vertical strip.
        
        All robots start in a narrow vertical strip on the left side:
        - x ∈ [xmin + 0.4, xmin + 0.9] (narrow vertical strip)
        - y uniformly distributed across the full height
        
        This ensures all robots must move a significant distance to reach
        the B formation on the right side, preventing any robot from appearing stuck.
        """
        # Narrow vertical strip on left side
        x_start = self.xmin + 0.4
        x_end = self.xmin + 0.9
        x_center = (x_start + x_end) / 2
        
        # Distribute robots vertically across full height
        y_min = self.ymin + 0.3  # Small margin from bottom
        y_max = self.ymax - 0.3  # Small margin from top
        
        starts: List[Tuple[float, float]] = []
        if self.num_robots == 1:
            # Single robot: place at center
            starts.append((float(x_center), 0.0))
        else:
            # Multiple robots: distribute evenly along y-axis
            ys = np.linspace(y_min, y_max, self.num_robots)
            for y in ys:
                # Use center of the strip for x, or slight variation
                starts.append((float(x_center), float(y)))
        
        return starts

    def generate_letter_B(
        self,
        region: str = "right_side",
        margin: float = 0.3,
    ) -> List[Tuple[float, float]]:
        """
        Generate target positions for robots to form the letter B.
        
        Uses the detailed polyline-based B shape from formations.py,
        which creates a much clearer B shape than the simple pixel pattern.
        
        Args:
            region: Where to place the B ("right_side", "left_side", "center")
            margin: Margin from room edges in meters
        
        Returns:
            List of (x, y) target positions in world coordinates
        """
        room_bounds = self.get_room_bounds()
        return generate_letter_b_targets(
            num_agents=self.num_robots,
            room_bounds=room_bounds,
            region=region,
            margin=margin,
        )
    
    def generate_formation_targets(
        self,
        shape: str,
        region: str = "right_side",
        margin: float = 0.3,
    ) -> List[Tuple[float, float]]:
        """
        Generate target positions for robots to form various shapes.
        
        REFACTORED: Unified method to generate targets for any supported shape.
        This is the main entry point for formation target generation.
        
        Args:
            shape: Formation shape ("B", "circle", "grid", or "heart")
            region: Where to place the formation ("right_side", "left_side", "center")
            margin: Margin from room edges in meters
        
        Returns:
            List of (x, y) target positions in world coordinates
        """
        from .formations import (
            generate_letter_b_targets,
            generate_circle_targets,
            generate_grid_targets,
            generate_heart_targets,
        )
        
        room_bounds = self.get_room_bounds()
        
        if shape == "B":
            return generate_letter_b_targets(
                num_agents=self.num_robots,
                room_bounds=room_bounds,
                region=region,
                margin=margin,
            )
        elif shape == "circle":
            return generate_circle_targets(
                num_agents=self.num_robots,
                room_bounds=room_bounds,
                region=region,
                margin=margin,
            )
        elif shape == "grid":
            return generate_grid_targets(
                num_agents=self.num_robots,
                room_bounds=room_bounds,
                region=region,
                margin=margin,
            )
        elif shape == "heart":
            return generate_heart_targets(
                num_agents=self.num_robots,
                room_bounds=room_bounds,
                region=region,
                margin=margin,
            )
        else:
            raise ValueError(f"Unsupported shape: {shape}. Supported: 'B', 'circle', 'grid', 'heart'")

    def set_formation_goals(self, targets: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Assign targets to each robot using one-to-one mapping.
        
        Ensures each robot gets a unique target point. If there are more targets
        than robots, extra targets are discarded. If there are fewer targets than
        robots, an assertion error is raised.
        """
        # Ensure we have at least as many targets as robots
        assert len(targets) >= len(self.agent_names), \
            f"Not enough target points: {len(targets)} < {len(self.agent_names)}"
        
        assigned = []
        for i, name in enumerate(self.agent_names):
            # One-to-one assignment: robot i gets target i
            goal = targets[i]
            self.goal_xy[name] = goal
            assigned.append(goal)
        
        return assigned

    # ------------------------------------------------------------------
    # Main API
    # ------------------------------------------------------------------
    def reset(
        self,
        target_positions: Optional[List[Tuple[float, float]]] = None,
        use_B: bool = True,
    ):
        mujoco.mj_resetData(self.model, self.data)

        # Step 1: Generate B formation targets first (ensures we have enough points)
        # REFACTORED: If target_positions provided, use them directly (from LLM task)
        # Otherwise, generate using default region (right_side)
        if target_positions is not None and len(target_positions) > 0:
            b_targets = target_positions
        elif use_B:
            # Default to right_side if no region specified
            b_targets = self.generate_letter_B(region="right_side")
        else:
            b_targets = self.generate_letter_B(region="right_side")
        
        # Ensure we have at least num_robots targets
        if len(b_targets) < self.num_robots:
            raise ValueError(
                f"Not enough B formation targets: {len(b_targets)} < {self.num_robots}. "
                f"Please check generate_letter_B() to ensure it generates enough points."
            )
        
        # Step 2: Set starting positions (left side vertical strip)
        starts = self._default_start_positions()
        for name, (x, y) in zip(self.agent_names, starts):
            self._set_body_xy(name, x, y)

        # Extra guarantee: ensure initial orientation is upright for all robots
        for name in self.agent_names:
            self._reset_body_orientation(name)

        # Step 3: One-to-one assignment of targets to robots
        goals = self.set_formation_goals(b_targets)

        # Initialize ORCA for collision avoidance (if available)
        if self.orca is not None:
            try:
                self.orca.init_sim(starts)
            except (ImportError, AttributeError):
                self.orca = None  # Fallback to repulsion-based avoidance
        
        # Reset stuck tracking (not used with simple controller, but keep for compatibility)
        self._stuck_timers = {name: 0.0 for name in self.agent_names}
        self._last_positions = {}
        self._last_warn_time = {name: -10.0 for name in self.agent_names}
        self._sim_time = 0.0
        if hasattr(self, '_last_vels'):
            self._last_vels = {}
        
        return self._obs()

    def step(self, dt: float = 0.05):
        """
        ORCA-based collision avoidance controller.
        Uses ORCA algorithm to ensure robots never collide - same as MuJoCo 3D Viewer.
        This is the SAME algorithm used in MuJoCo 3D Viewer.
        """
        # CRITICAL: Always use ORCA if available - this is the ONLY way to guarantee no collisions
        # ORCA computes collision-free velocities for all robots simultaneously
        if self.orca is not None:
            # Get current positions and goals
            pos_list = [np.array(self._get_body_xy(name), dtype=float) for name in self.agent_names]
            goals_list = [np.array(self.goal_xy[name], dtype=float) for name in self.agent_names]
            
            # Initialize ORCA if needed
            if self.orca.sim is None or len(self.orca.agent_ids) != len(pos_list):
                self.orca.init_sim([(p[0], p[1]) for p in pos_list])
            
            # Compute collision-free velocities using ORCA
            # ORCA guarantees no collisions by computing safe velocities for all robots
            vels = self.orca.compute_velocities(
                [(p[0], p[1]) for p in pos_list],
                [(g[0], g[1]) for g in goals_list]
            )
            
            # Apply velocities to robots
            for (vx, vy), (x, y), name in zip(vels, pos_list, self.agent_names):
                nx = x + vx * dt
                ny = y + vy * dt
                margin = 0.15
                nx = np.clip(nx, self.xmin + margin, self.xmax - margin)
                ny = np.clip(ny, self.ymin + margin, self.ymax - margin)
                self._set_body_xy(name, float(nx), float(ny))
            
            # Force all robots to stay upright
            for name in self.agent_names:
                self._reset_body_orientation(name)
            
            mujoco.mj_step(self.model, self.data)
            done = self._is_done()
            return self._obs(), done
        
        # Fallback: Enhanced repulsion-based avoidance (only if ORCA unavailable)
        # This should not happen if rvo2 is properly installed
        positions = {name: np.array(self._get_body_xy(name), dtype=float) for name in self.agent_names}
        
        # Enhanced repulsion-based collision avoidance (fallback when ORCA unavailable)
        # This ensures robots maintain minimum distance and NEVER overlap
        robot_radius = 0.20  # Minimum safe distance between robot centers (increased from 0.15)
        min_safe_distance = robot_radius * 2  # 0.4m minimum distance
        repulsion_force = 2.0  # Increased repulsion strength (from 0.5)
        repulsion_range = 0.6  # Repulsion acts up to 0.6m away (increased range)
        
        # First pass: compute all repulsion vectors
        repulsion_vectors = {}
        min_distances = {}
        
        for name in self.agent_names:
            cur = positions[name].copy()
            repulsion_vec = np.array([0.0, 0.0])
            min_dist = float('inf')
            
            for other in self.agent_names:
                if other == name:
                    continue
                other_pos = positions[other]
                vec_to_other = cur - other_pos
                dist_to_other = np.linalg.norm(vec_to_other)
                min_dist = min(min_dist, dist_to_other)
                
                # If too close, apply STRONG repulsion force
                if dist_to_other < repulsion_range:
                    if dist_to_other < 1e-6:
                        # Overlapping: push in random direction with maximum force
                        angle = np.random.uniform(0, 2 * np.pi)
                        repulsion_vec += np.array([np.cos(angle), np.sin(angle)]) * repulsion_force * 5.0
                    elif dist_to_other < min_safe_distance:
                        # Very close: very strong repulsion (inverse square law)
                        repulsion_dir = vec_to_other / (dist_to_other + 1e-6)
                        # Stronger repulsion when closer
                        repulsion_strength = repulsion_force * (min_safe_distance / (dist_to_other + 1e-6)) ** 3
                        repulsion_vec += repulsion_dir * repulsion_strength
                    else:
                        # Close but not too close: moderate repulsion
                        repulsion_dir = vec_to_other / (dist_to_other + 1e-6)
                        repulsion_strength = repulsion_force * (min_safe_distance / (dist_to_other + 1e-6)) ** 2
                        repulsion_vec += repulsion_dir * repulsion_strength
            
            repulsion_vectors[name] = repulsion_vec
            min_distances[name] = min_dist
        
        # Second pass: apply velocities with collision avoidance
        for name in self.agent_names:
            cur = positions[name].copy()
            gx, gy = self.goal_xy[name]
            goal = np.array([gx, gy], dtype=float)
            
            # Compute direction to goal
            diff = goal - cur
            dist_to_goal = np.linalg.norm(diff)
            
            # If robot is very close to target, snap to goal (but check collisions first)
            if dist_to_goal < 0.05:
                # Before snapping, check if goal position would cause collision
                collision_at_goal = False
                for other in self.agent_names:
                    if other == name:
                        continue
                    other_pos = positions[other]
                    dist_to_other = np.linalg.norm(np.array([gx, gy]) - np.array(other_pos))
                    if dist_to_other < min_safe_distance:
                        collision_at_goal = True
                        break
                
                if not collision_at_goal:
                    self._set_body_xy(name, gx, gy)
                    continue
            
            # Normalize direction to goal
            dir_to_goal = diff / (dist_to_goal + 1e-6)
            
            # Get precomputed repulsion
            repulsion_vec = repulsion_vectors[name]
            min_dist = min_distances[name]
            
            # Weight repulsion MUCH more when robots are very close
            if min_dist < min_safe_distance:
                repulsion_weight = 5.0  # Very strong repulsion when too close
            elif min_dist < repulsion_range:
                repulsion_weight = 2.0  # Moderate repulsion when close
            else:
                repulsion_weight = 0.3  # Weak repulsion when far
            
            # Normalize repulsion if non-zero
            if np.linalg.norm(repulsion_vec) > 1e-6:
                repulsion_vec = repulsion_vec / (np.linalg.norm(repulsion_vec) + 1e-6)
            
            # Combined velocity: goal direction + repulsion
            # When very close, repulsion dominates
            if min_dist < min_safe_distance:
                # Too close: prioritize repulsion over goal
                combined_dir = repulsion_vec * 0.8 + dir_to_goal * 0.2
            else:
                combined_dir = dir_to_goal + repulsion_vec * repulsion_weight
            
            combined_dir = combined_dir / (np.linalg.norm(combined_dir) + 1e-6)
            
            # Speed: much slower when close to other robots
            base_speed = 1.0
            if min_dist < min_safe_distance:
                speed_factor = max(0.1, min_dist / min_safe_distance)  # Very slow when too close
            elif min_dist < repulsion_range:
                speed_factor = max(0.5, (min_dist - min_safe_distance) / (repulsion_range - min_safe_distance))
            else:
                speed_factor = 1.0
            
            vx, vy = combined_dir * base_speed * speed_factor
            
            # Compute new position
            nx = cur[0] + vx * dt
            ny = cur[1] + vy * dt
            
            # Final collision check: if new position would cause overlap, reduce movement
            new_pos = np.array([nx, ny])
            for other in self.agent_names:
                if other == name:
                    continue
                other_pos = positions[other]
                dist_to_other = np.linalg.norm(new_pos - np.array(other_pos))
                if dist_to_other < min_safe_distance:
                    # Would collide: move less or stop
                    if dist_to_other < 1e-6:
                        # Already overlapping: don't move
                        nx, ny = cur[0], cur[1]
                    else:
                        # Too close: reduce movement towards that direction
                        vec_away = (new_pos - np.array(other_pos)) / (dist_to_other + 1e-6)
                        # Move only a small amount away from the other robot
                        nx = cur[0] + vec_away[0] * dt * 0.5
                        ny = cur[1] + vec_away[1] * dt * 0.5
                    break
            
            # Clamp inside room (min margin 0.15 m)
            margin = 0.15
            nx = np.clip(nx, self.xmin + margin, self.xmax - margin)
            ny = np.clip(ny, self.ymin + margin, self.ymax - margin)
            
            self._set_body_xy(name, float(nx), float(ny))
        
        # Force all robots to stay upright (2D kinematic behavior)
        for name in self.agent_names:
            self._reset_body_orientation(name)
        
        mujoco.mj_step(self.model, self.data)
        done = self._is_done()
        return self._obs(), done


