#!/usr/bin/env python3
"""
End-to-End Natural Language Navigation Control

This script provides a complete pipeline:
1. User types natural language command
2. LLM parses goals & priority
3. MAPF (Priority Planning) plans collision-free paths
4. MuJoCo executes the paths

Usage:
    python llm_interface/end_to_end_navigation.py
    # Then type: "Robot A go to (3, 2), Robot B go to (-2, 2), A has priority"
"""

import os
import sys
import time
import numpy as np
from typing import Dict, List, Tuple

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from llm_interface.llm_controller import llm_parse_instruction, llm_parse_instruction_offline
from nav_world.nav_env_mapf import NavEnvMAPF
from nav_world.multi_agent_planner import AgentSpec as MAPFAgentSpec


class EndToEndNavigationController:
    """
    End-to-end navigation controller that integrates:
    - LLM natural language parsing
    - MAPF path planning
    - MuJoCo execution
    """
    
    def __init__(self, xml_path: str, use_offline_parser: bool = False):
        """
        Initialize the end-to-end navigation controller
        
        Args:
            xml_path: Path to MuJoCo XML file
            use_offline_parser: If True, use offline parser (no API calls)
        """
        self.use_offline_parser = use_offline_parser
        
        # Initialize navigation environment
        print("📦 Initializing navigation environment...")
        self.env = NavEnvMAPF(xml_path=xml_path, grid_res=0.1)
        print("✅ Environment initialized")
        
        # Agent ID mapping: LLM uses "A"/"B", NavEnv uses "alice"/"bob"
        self.agent_id_map = {
            'A': 'alice',
            'B': 'bob',
            'alice': 'alice',
            'bob': 'bob'
        }
    
    def parse_user_instruction(self, user_text: str):
        """
        Parse natural language instruction using LLM
        
        Args:
            user_text: Natural language instruction
            
        Returns:
            TaskPlan object with parsed goals and priority
        """
        print(f"\n🗣️  Parsing user instruction: \"{user_text}\"")
        
        if self.use_offline_parser:
            print("📝 Using offline parser (no GPT API call)")
            task_plan = llm_parse_instruction_offline(user_text)
        else:
            print("🤖 Using GPT-4o API for instruction parsing...")
            task_plan = llm_parse_instruction(user_text)
        
        print(f"✅ Parsed plan:")
        print(f"   Task: {task_plan.task}")
        print(f"   Agents: {[(a.id, a.goal, f'delay={a.delay}s') for a in task_plan.agents]}")
        print(f"   Priority: {task_plan.priority}")
        
        return task_plan
    
    def set_goals_from_plan(self, task_plan):
        """
        Set agent goals and delays in the environment based on parsed plan
        
        Args:
            task_plan: TaskPlan object from LLM parser
        """
        print("\n🎯 Setting agent goals and delays...")
        
        # Map LLM agent IDs to NavEnv agent names
        goal_mapping = {}
        agent_delays = {}
        for agent_spec in task_plan.agents:
            nav_env_id = self.agent_id_map.get(agent_spec.id, agent_spec.id)
            goal_mapping[nav_env_id] = tuple(agent_spec.goal)  # Convert to tuple
            agent_delays[nav_env_id] = agent_spec.delay  # Store delay
        
        # Store goals and delays for MAPF planning and execution
        self.goal_mapping = goal_mapping
        self.agent_delays = agent_delays
        
        print(f"✅ Goals set: {goal_mapping}")
        if any(d > 0 for d in agent_delays.values()):
            print(f"⏱️  Delays: {agent_delays}")
            for nav_id, delay in agent_delays.items():
                if delay > 0:
                    if delay >= 60.0:
                        print(f"   {nav_id.capitalize()} will wait {delay:.1f} seconds ({delay/60:.1f} minutes) before starting")
                    else:
                        print(f"   {nav_id.capitalize()} will wait {delay:.1f} seconds before starting")
    
    def plan_paths_with_mapf(self, task_plan):
        """
        Plan collision-free paths using MAPF
        
        Args:
            task_plan: TaskPlan object with goals and priority
        """
        print("\n🗺️  Planning paths with MAPF...")
        
        # Reset environment to get current positions
        obs = self.env.reset(use_mapf=False)  # Reset without MAPF first
        
        # Update goal positions in environment
        for nav_env_id, goal_pos in self.goal_mapping.items():
            self.env.goal_xy[nav_env_id] = goal_pos
        
        # Create MAPF agent specifications
        mapf_agents = []
        priority_order = []
        
        for agent_spec in task_plan.agents:
            nav_env_id = self.agent_id_map.get(agent_spec.id, agent_spec.id)
            
            # Get current position in grid coordinates
            current_pos = self.env._get_body_xy(nav_env_id)
            start_grid = self.env._world2grid(current_pos)
            
            # Get goal position in grid coordinates
            goal_world = self.goal_mapping[nav_env_id]
            goal_grid = self.env._world2grid(goal_world)
            
            # Create MAPF agent spec
            mapf_agents.append(MAPFAgentSpec(
                id=nav_env_id,
                start=start_grid,
                goal=goal_grid
            ))
        
        # Validate goal positions before planning
        print("\n🔍 Validating goal positions...")
        from nav_world.nav_env import astar
        invalid_goals = []
        for agent in mapf_agents:
            # Check if goal is within grid bounds
            goal_grid = agent.goal
            if (goal_grid[0] < 0 or goal_grid[0] >= self.env.grid.shape[0] or 
                goal_grid[1] < 0 or goal_grid[1] >= self.env.grid.shape[1]):
                goal_world = self.env._grid2world(goal_grid[0], goal_grid[1])
                invalid_goals.append(f"{agent.id}: goal {goal_world} is out of bounds")
                continue
            
            # Check if goal is in obstacle
            if self.env.grid[goal_grid[0], goal_grid[1]] == 1:
                goal_world = self.env._grid2world(goal_grid[0], goal_grid[1])
                invalid_goals.append(f"{agent.id}: goal {goal_world} is in obstacle")
                continue
            
            # Quick check: try A* to see if path exists
            path_test = astar(self.env.grid.copy(), agent.start, goal_grid)
            if path_test is None:
                goal_world = self.env._grid2world(goal_grid[0], goal_grid[1])
                start_world = self.env._grid2world(agent.start[0], agent.start[1])
                invalid_goals.append(f"{agent.id}: no path from {start_world} to {goal_world}")
        
        if invalid_goals:
            print("\n❌ Invalid goal positions detected:")
            for msg in invalid_goals:
                print(f"   - {msg}")
            print("\n💡 Suggestions:")
            print("   1. Check environment bounds: x ∈ [-4.0, 4.0], y ∈ [-3.0, 3.0]")
            print("   2. Avoid positions on the left side (x < 0) - many obstacles there")
            print("   3. Try goal positions on the right side: x > 2.0")
            print("   4. Use recommended commands from docs/user_guides/VALID_COMMANDS.md")
            print("\n📝 Example valid commands:")
            print("   - Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority")
            print("   - Robot A go to (2.0, 1.0), Robot B go to (2.0, -1.0), A has priority")
            raise RuntimeError(f"Invalid goal positions: {', '.join(invalid_goals)}")
        
        print("✅ All goal positions are valid")
        
        # Map priority order
        for agent_id in task_plan.priority:
            nav_env_id = self.agent_id_map.get(agent_id, agent_id)
            if nav_env_id not in priority_order:
                priority_order.append(nav_env_id)
        
        # Ensure all agents are in priority order
        for agent_spec in task_plan.agents:
            nav_env_id = self.agent_id_map.get(agent_spec.id, agent_spec.id)
            if nav_env_id not in priority_order:
                priority_order.append(nav_env_id)
        
        # Set priority order in environment
        self.env.set_priority_order(priority_order)
        
        # Plan paths using MAPF
        from nav_world.multi_agent_planner import plan_priority
        
        # Calculate max_time based on grid size and path complexity
        grid_size = self.env.grid.size
        # Calculate Manhattan distance for each agent
        max_distance = 0
        for agent in mapf_agents:
            dist = abs(agent.start[0] - agent.goal[0]) + abs(agent.start[1] - agent.goal[1])
            max_distance = max(max_distance, dist)
        
        # Set max_time based on distance (with safety margin)
        # For complex paths, we need more time steps
        if grid_size > 5000:  # Large grid
            max_time = min(200, max(100, max_distance * 2))
        else:
            max_time = min(250, max(150, max_distance * 2))
        
        print(f"   Grid size: {grid_size}, Max distance: {max_distance}, Using max_time={max_time}")
        
        # Debug: Print agent info
        print(f"   Agent details:")
        for agent in mapf_agents:
            dist = abs(agent.start[0] - agent.goal[0]) + abs(agent.start[1] - agent.goal[1])
            print(f"     {agent.id}: start={agent.start}, goal={agent.goal}, distance={dist}")
        
        try:
            mapf_paths = plan_priority(
                grid=self.env.grid,
                agents=mapf_agents,
                order=priority_order,
                max_time=max_time
            )
        except RuntimeError as e:
            error_msg = str(e)
            print(f"\n⚠️  MAPF planning failed: {error_msg}")
            print("\n💡 Trying fallback: Independent A* planning (no collision avoidance)...")
            
            # Fallback: Use independent A* planning
            try:
                self.env.reset(use_mapf=False)
                # Update goals
                for nav_env_id, goal_pos in self.goal_mapping.items():
                    self.env.goal_xy[nav_env_id] = goal_pos
                
                # Use independent planning
                from nav_world.nav_env import astar
                for agt in self.env.agent_names:
                    start = self.env._world2grid(self.env._get_body_xy(agt))
                    goal = self.env._world2grid(self.env.goal_xy[agt])
                    path_g = astar(self.env.grid.copy(), start, goal)
                    if path_g is None:
                        raise RuntimeError(f"Even independent A* failed for {agt} from {start} to {goal}")
                    path_w = [self.env._grid2world(ix, iy) for (ix, iy) in path_g]
                    self.env.agents[agt].path_world = path_w
                    self.env.agents[agt].path_ptr = 0
                
                print("✅ Fallback planning successful (no collision avoidance)")
                return {}  # Return empty dict to indicate fallback was used
            except Exception as fallback_error:
                print(f"\n❌ Fallback also failed: {fallback_error}")
                print("\n💡 Suggestions:")
                print("   1. Check if goal positions are within environment bounds")
                print("   2. Try different goal positions")
                print("   3. Reduce grid resolution: grid_res=0.2")
                print("   4. Check if goals are blocked by obstacles")
                raise RuntimeError(f"Both MAPF and fallback planning failed. Original error: {error_msg}")
        except KeyboardInterrupt:
            print("\n\n⚠️  Path planning interrupted by user")
            print("💡 Tip: Try using simpler goals or reducing grid resolution")
            raise
        
        # Convert MAPF paths to world coordinates and set in environment
        # If fallback was used (empty dict), paths are already set in env
        if not mapf_paths:
            return mapf_paths
        
        for nav_env_id in self.env.agent_names:
            if nav_env_id in mapf_paths:
                path_grid_time = mapf_paths[nav_env_id]
                # Extract spatial path (ignore time)
                path_grid = []
                seen = set()
                for x, y, t in path_grid_time:
                    if (x, y) not in seen:
                        path_grid.append((x, y))
                        seen.add((x, y))
                
                # Convert to world coordinates
                path_world = [self.env._grid2world(ix, iy) for (ix, iy) in path_grid]
                self.env.agents[nav_env_id].path_world = path_world
                self.env.agents[nav_env_id].path_ptr = 0
        
        print("✅ MAPF path planning completed")
        return mapf_paths
    
    def execute_in_mujoco(self, use_viewer: bool = True):
        """
        Execute planned paths in MuJoCo
        
        Args:
            use_viewer: If True, use interactive 3D viewer; if False, run headless
        """
        if use_viewer:
            return self._execute_with_viewer()
        else:
            return self._execute_headless()
    
    def _execute_with_viewer(self):
        """Execute with MuJoCo 3D viewer"""
        import platform
        
        # Check if we're on macOS
        is_macos = platform.system() == 'Darwin'
        
        # Check if mjpython is available
        mjpython_available = False
        if is_macos:
            import shutil
            mjpython_available = shutil.which('mjpython') is not None
        
        try:
            import mujoco.viewer
            
            print("\n" + "=" * 60)
            print("🎬 Starting MuJoCo 3D Viewer...")
            print("=" * 60)
            
            # On macOS, try to use launch_passive, but handle the error gracefully
            if is_macos and not mjpython_available:
                print("⚠️  macOS detected but mjpython not found in PATH")
                print("💡 For best results on macOS, run with: mjpython llm_interface/end_to_end_navigation.py")
                print("   Attempting to launch viewer anyway...")
            
            print("   Controls:")
            print("   - ESC key: Exit")
            print("   - Mouse: Rotate view")
            print("   - Scroll: Zoom in/out")
            print("=" * 60)
            
            # Try to launch viewer
            try:
                with mujoco.viewer.launch_passive(self.env.model, self.env.data) as viewer:
                    print("✅ 3D Viewer started!")
                    print("   Waiting 2 seconds for viewer to fully load...")
                    time.sleep(2)  # Wait for viewer to load
                    
                    if not viewer.is_running():
                        print("❌ Viewer closed immediately")
                        print("💡 Trying headless mode...")
                        return self._execute_headless()
                    
                    print("🚀 Executing planned paths...")
                    print("   Robots will follow MAPF-planned collision-free paths")
                    print("   Press ESC to exit\n")
                    
                    # Calculate simulation time based on delays
                    max_delay = max(self.agent_delays.values()) if self.agent_delays else 0.0
                    # Base simulation time + max delay + buffer for path execution
                    T = 15.0 + max_delay + 5.0  # Add buffer for path execution after delay
                    fps = 30
                    dt = 1.0 / fps
                    steps = int(T * fps)
                    
                    if max_delay > 0:
                        print(f"⏱️  Maximum delay: {max_delay:.1f}s, simulation time: {T:.1f}s")
                    
                    # Track current simulation time
                    current_time = 0.0
                    
                    for step in range(steps):
                        if not viewer.is_running():
                            print("\n👋 Viewer closed by user")
                            break
                        
                        obs, done = self.env.step(dt=dt, current_time=current_time, agent_delays=self.agent_delays)
                        current_time += dt
                        viewer.sync()
                        
                        if step % 30 == 0:
                            alice_pos = self.env._get_body_xy('alice')
                            bob_pos = self.env._get_body_xy('bob')
                            alice_goal = self.goal_mapping.get('alice', obs['alice']['goal'])
                            bob_goal = self.goal_mapping.get('bob', obs['bob']['goal'])
                            
                            alice_dist = np.linalg.norm(np.array(alice_pos) - np.array(alice_goal))
                            bob_dist = np.linalg.norm(np.array(bob_pos) - np.array(bob_goal))
                            
                            # Show delay status
                            alice_delay = self.agent_delays.get('alice', 0.0)
                            bob_delay = self.agent_delays.get('bob', 0.0)
                            alice_status = "⏸️ waiting" if current_time < alice_delay else "▶️ moving"
                            bob_status = "⏸️ waiting" if current_time < bob_delay else "▶️ moving"
                            
                            print(f"   Step {step:3d} (t={current_time:.1f}s): Alice {alice_status} dist={alice_dist:.2f}m, Bob {bob_status} dist={bob_dist:.2f}m")
                        
                        if done:
                            print(f"\n✅ Task completed at step {step}!")
                            print("   Showing final state for 3 seconds...")
                            # Show final state for a few seconds
                            for _ in range(90):
                                if not viewer.is_running():
                                    break
                                viewer.sync()
                                time.sleep(0.033)
                            break
                        
                        time.sleep(0.01)
                    
                    print("\n🎉 Execution completed!")
                    
            except RuntimeError as e:
                error_msg = str(e)
                if 'mjpython' in error_msg.lower() or 'launch_passive' in error_msg.lower():
                    print(f"\n⚠️  Viewer launch error: {error_msg}")
                    print("\n💡 Solution for macOS:")
                    print("   Run the script with mjpython instead:")
                    print("   mjpython llm_interface/end_to_end_navigation.py")
                    print("\n   Or continue in headless mode...")
                    return self._execute_headless()
                else:
                    raise
                
        except ImportError:
            print("⚠️  mujoco.viewer not available, running headless...")
            return self._execute_headless()
        except Exception as e:
            print(f"\n⚠️  Viewer error: {e}")
            print("💡 For macOS, try running with: mjpython llm_interface/end_to_end_navigation.py")
            print("   Continuing in headless mode...")
            return self._execute_headless()
    
    def _execute_headless(self):
        """Execute without viewer (headless mode)"""
        print("\n🏃 Running simulation (headless mode)...")
        
        # Calculate simulation time based on delays
        max_delay = max(self.agent_delays.values()) if self.agent_delays else 0.0
        # Base simulation time + max delay + buffer for path execution
        T = 15.0 + max_delay + 5.0  # Add buffer for path execution after delay
        fps = 30
        dt = 1.0 / fps
        steps = int(T * fps)
        
        if max_delay > 0:
            print(f"⏱️  Maximum delay: {max_delay:.1f}s, simulation time: {T:.1f}s")
        
        # Track current simulation time
        current_time = 0.0
        
        for step in range(steps):
            obs, done = self.env.step(dt=dt, current_time=current_time, agent_delays=self.agent_delays)
            current_time += dt
            
            if step % 30 == 0:
                alice_pos = self.env._get_body_xy('alice')
                bob_pos = self.env._get_body_xy('bob')
                alice_goal = self.goal_mapping.get('alice', obs['alice']['goal'])
                bob_goal = self.goal_mapping.get('bob', obs['bob']['goal'])
                
                alice_dist = np.linalg.norm(np.array(alice_pos) - np.array(alice_goal))
                bob_dist = np.linalg.norm(np.array(bob_pos) - np.array(bob_goal))
                
                # Show delay status
                alice_delay = self.agent_delays.get('alice', 0.0)
                bob_delay = self.agent_delays.get('bob', 0.0)
                alice_status = "⏸️ waiting" if current_time < alice_delay else "▶️ moving"
                bob_status = "⏸️ waiting" if current_time < bob_delay else "▶️ moving"
                
                print(f"   Step {step:3d} (t={current_time:.1f}s): Alice {alice_status} dist={alice_dist:.2f}m, Bob {bob_status} dist={bob_dist:.2f}m")
            
            if done:
                print(f"\n✅ Task completed at step {step}!")
                break
        
        print("🎉 Execution completed!")


def main():
    """Main function for end-to-end navigation"""
    print("=" * 60)
    print("🎯 End-to-End Natural Language Navigation Control")
    print("=" * 60)
    print("\nThis system:")
    print("  1. Parses natural language instructions using LLM")
    print("  2. Plans collision-free paths using MAPF")
    print("  3. Executes paths in MuJoCo 3D environment")
    print("=" * 60)
    
    # Get XML path
    xml_path = os.path.join(project_root, "nav_world", "room.xml")
    
    if not os.path.exists(xml_path):
        print(f"❌ XML file not found: {xml_path}")
        return
    
    # Initialize controller
    controller = EndToEndNavigationController(
        xml_path=xml_path,
        use_offline_parser=False  # Set to True to avoid API calls
    )
    
    # Get user instruction
    print("\n" + "=" * 60)
    print("📝 Enter your navigation instruction:")
    print("=" * 60)
    print("Examples:")
    print('  - "Robot A go to (3, 2), Robot B go to (-2, 2), A has priority"')
    print('  - "Alice goes to 3.0, 1.6. Bob goes to 3.2, -1.0. Alice first"')
    print("=" * 60)
    
    user_text = input("\nYour instruction: ").strip()
    
    if not user_text:
        print("⚠️  No instruction provided, using default plan")
        user_text = "Robot A go to (3, 2), Robot B go to (-2, 2), A has priority"
    
    try:
        # Step 1: Parse instruction
        task_plan = controller.parse_user_instruction(user_text)
        
        # Step 2: Set goals
        controller.set_goals_from_plan(task_plan)
        
        # Step 3: Plan paths with MAPF
        mapf_paths = controller.plan_paths_with_mapf(task_plan)
        
        # Step 4: Execute in MuJoCo
        print("\n" + "=" * 60)
        print("🎬 Starting MuJoCo 3D Visualization...")
        print("=" * 60)
        controller.execute_in_mujoco(use_viewer=True)
        
        print("\n" + "=" * 60)
        print("✅ End-to-end navigation completed successfully!")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

