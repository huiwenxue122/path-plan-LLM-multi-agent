#!/usr/bin/env python3
"""
Enhanced NavEnv with Multi-Agent Path Finding (MAPF) support

This module extends NavEnv to use priority-based MAPF planning instead of
independent A* planning for each agent. This ensures collision-free paths
in both space and time.
"""

from __future__ import annotations
import os
from typing import List, Tuple, Optional, Dict
import numpy as np
import mujoco
from mujoco import MjModel, MjData

from .nav_env import NavEnv, AgentState
from .multi_agent_planner import plan_priority, AgentSpec


class NavEnvMAPF(NavEnv):
    """
    Enhanced NavEnv with Multi-Agent Path Finding (MAPF)
    
    Key differences from NavEnv:
    1. Uses priority-based MAPF planning instead of independent A* for each agent
    2. Ensures collision-free paths in both space and time
    3. Supports priority ordering (can be set by LLM or user)
    4. Paths include time stamps for precise coordination
    """
    
    def __init__(
        self,
        xml_path: str,
        grid_res: float = 0.1,
        render_w: int = 800,
        render_h: int = 600,
        priority_order: Optional[List[str]] = None
    ):
        """
        Initialize NavEnv with MAPF support
        
        Args:
            xml_path: Path to MuJoCo XML file
            grid_res: Grid resolution in meters
            render_w: Render width
            render_h: Render height
            priority_order: Priority order for agents (e.g., ['alice', 'bob'])
                          If None, uses default order from agent_names
        """
        # Initialize base NavEnv
        super().__init__(xml_path, grid_res, render_w, render_h)
        
        # Set priority order (default: use agent_names order)
        self.priority_order = priority_order if priority_order else self.agent_names.copy()
        
        # Validate priority order
        if set(self.priority_order) != set(self.agent_names):
            raise ValueError(
                f"Priority order {self.priority_order} must contain all agents: {self.agent_names}"
            )
        
        # Store time-stamped paths (from MAPF planning)
        self.mapf_paths: Optional[Dict[str, List[Tuple[int, int, int]]]] = None
    
    def reset(self, randomize: bool = False, use_mapf: bool = True):
        """
        Reset environment and plan paths using MAPF
        
        Args:
            randomize: Whether to randomize initial positions
            use_mapf: If True, use MAPF planning; if False, use independent A* (original behavior)
        """
        mujoco.mj_resetData(self.model, self.data)
        
        # Initialize robot positions
        alice_xy = (-2.5, -2.0)
        bob_xy = (-2.5, 2.0)
        if randomize:
            alice_xy = (-2.8, np.random.uniform(-2.5, -1.5))
            bob_xy = (-2.8, np.random.uniform(1.5, 2.5))
        
        self._set_body_xy("alice", *alice_xy)
        self._set_body_xy("bob", *bob_xy)
        
        # Calculate goals
        self.goal_xy = {
            agt: self._get_site_xy(self.goal_sites[agt]) 
            for agt in self.agent_names
        }
        
        if use_mapf:
            # Use MAPF planning
            self._plan_paths_mapf()
        else:
            # Use original independent A* planning
            self._plan_paths_independent()
        
        return self._obs()
    
    def _plan_paths_mapf(self):
        """
        Plan paths using Multi-Agent Path Finding (MAPF) with priority planning
        """
        # Create agent specifications for MAPF
        agents = []
        for agt in self.agent_names:
            start_grid = self._world2grid(self._get_body_xy(agt))
            goal_grid = self._world2grid(self.goal_xy[agt])
            agents.append(AgentSpec(
                id=agt,
                start=start_grid,
                goal=goal_grid
            ))
        
        # Plan paths using priority planning
        try:
            self.mapf_paths = plan_priority(
                grid=self.grid,
                agents=agents,
                order=self.priority_order,
                max_time=200
            )
        except RuntimeError as e:
            raise RuntimeError(
                f"MAPF planning failed: {e}\n"
                f"Try adjusting priority order or obstacle layout."
            )
        
        # Convert time-stamped grid paths to world coordinate paths
        # For each agent, extract the spatial path (ignoring time for now)
        for agt in self.agent_names:
            path_grid_time = self.mapf_paths[agt]
            # Extract unique spatial positions (in order)
            path_grid = []
            seen = set()
            for x, y, t in path_grid_time:
                if (x, y) not in seen:
                    path_grid.append((x, y))
                    seen.add((x, y))
            
            # Convert to world coordinates
            path_world = [self._grid2world(ix, iy) for (ix, iy) in path_grid]
            self.agents[agt].path_world = path_world
            self.agents[agt].path_ptr = 0
        
        print(f"✅ MAPF planning completed with priority order: {self.priority_order}")
    
    def _plan_paths_independent(self):
        """
        Plan paths using independent A* (original behavior)
        """
        from .nav_env import astar
        
        for agt in self.agent_names:
            start = self._world2grid(self._get_body_xy(agt))
            goal = self._world2grid(self.goal_xy[agt])
            path_g = astar(self.grid.copy(), start, goal)
            if path_g is None:
                raise RuntimeError(f"[{agt}] A* 找不到路径，从 {start} 到 {goal}")
            path_w = [self._grid2world(ix, iy) for (ix, iy) in path_g]
            self.agents[agt].path_world = path_w
            self.agents[agt].path_ptr = 0
    
    def set_priority_order(self, order: List[str]):
        """
        Set priority order for MAPF planning
        
        Args:
            order: List of agent names in priority order (e.g., ['alice', 'bob'])
        """
        if set(order) != set(self.agent_names):
            raise ValueError(
                f"Priority order {order} must contain all agents: {self.agent_names}"
            )
        self.priority_order = order
        print(f"Priority order updated to: {self.priority_order}")
    
    def get_mapf_paths(self) -> Optional[Dict[str, List[Tuple[int, int, int]]]]:
        """
        Get the time-stamped MAPF paths
        
        Returns:
            Dictionary mapping agent names to time-stamped paths
            Each path is a list of (x, y, t) tuples in grid coordinates
        """
        return self.mapf_paths


# Example usage
if __name__ == "__main__":
    here = os.path.dirname(__file__)
    xml = os.path.join(here, "room.xml")
    
    print("=" * 60)
    print("NavEnv with MAPF Planning - Demo")
    print("=" * 60)
    
    # Create environment with MAPF support
    env = NavEnvMAPF(xml_path=xml, grid_res=0.1, priority_order=['alice', 'bob'])
    
    # Reset with MAPF planning
    print("\n🔄 Resetting environment with MAPF planning...")
    env.reset(use_mapf=True)
    
    # Get MAPF paths
    mapf_paths = env.get_mapf_paths()
    if mapf_paths:
        print("\n📊 MAPF Path Summary:")
        for agent_id, path in mapf_paths.items():
            print(f"  {agent_id}: {len(path)} steps, "
                  f"from {path[0][:2]} to {path[-1][:2]}, "
                  f"duration: {path[-1][2]} timesteps")
    
    # Run simulation
    print("\n🎬 Running simulation...")
    T = 15.0
    fps = 30
    dt = 1.0 / fps
    steps = int(T * fps)
    
    for step_i in range(steps):
        obs, done = env.step(dt=dt)
        if step_i % 30 == 0:
            alice_xy = obs['alice']['xy']
            bob_xy = obs['bob']['xy']
            alice_dist = np.linalg.norm(np.array(alice_xy) - np.array(obs['alice']['goal']))
            bob_dist = np.linalg.norm(np.array(bob_xy) - np.array(obs['bob']['goal']))
            print(f"Step {step_i}: Alice dist={alice_dist:.2f}m, Bob dist={bob_dist:.2f}m")
        if done:
            print(f"✅ Task completed at step {step_i}!")
            break
    
    print("\n✅ Demo completed!")

