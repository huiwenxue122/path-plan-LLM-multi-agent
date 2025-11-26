#!/usr/bin/env python3
"""
End-to-End Formation Task Control

This script provides a complete pipeline for formation tasks:
1. User types a natural language command (e.g., "form the letter B")
2. LLM parses the command into a formation plan
3. The formation environment (NavEnvFormationORCA) is initialised
4. ORCA computes collision-free velocities for every robot
5. MuJoCo executes the formation in the 3D viewer

Usage:
    python llm_interface/end_to_end_formation.py
    # Then type: "form the letter B" or "让机器人形成字母B"
"""

import os
import sys
import time
import numpy as np
from typing import Optional

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from llm_interface.llm_formation_controller import (
    llm_parse_formation_instruction,
    llm_parse_formation_instruction_offline,
    FormationTaskPlan
)
from nav_world.nav_env_formation_orca import NavEnvFormationORCA


class EndToEndFormationController:
    """
    End-to-end formation controller that integrates:
    - LLM natural language parsing for formation tasks
    - Formation pattern generation
    - Path planning for all robots
    - MuJoCo execution
    """
    
    def __init__(self, xml_path: str, use_offline_parser: bool = False):
        """
        Initialize the end-to-end formation controller
        
        Args:
            xml_path: Path to MuJoCo XML file (should be room_formation.xml)
            use_offline_parser: If True, use offline parser (no API calls)
        """
        self.use_offline_parser = use_offline_parser
        self.xml_path = xml_path
        self.env = None  # Will be initialized after parsing task plan
        self.formation_plan = None
    
    def parse_user_instruction(self, user_text: str) -> FormationTaskPlan:
        """
        Parse natural language instruction using LLM
        
        Args:
            user_text: Natural language instruction
            
        Returns:
            FormationTaskPlan object
        """
        print(f"\n🗣️  Parsing user instruction: \"{user_text}\"")
        
        if self.use_offline_parser:
            print("📝 Using offline parser (no GPT API call)")
            self.formation_plan = llm_parse_formation_instruction_offline(user_text)
        else:
            print("🤖 Using GPT-4o API for formation task parsing...")
            self.formation_plan = llm_parse_formation_instruction(user_text)
        
        print(f"✅ Parsed formation plan:")
        print(f"   Task: {self.formation_plan.task}")
        print(f"   Formation type: {self.formation_plan.formation_type}")
        print(f"   Number of robots: {self.formation_plan.num_robots}")
        print(f"   Region: {self.formation_plan.region}")
        
        return self.formation_plan
    
    def initialize_environment(self):
        """Initialize the formation environment based on parsed plan"""
        if self.formation_plan is None:
            raise RuntimeError("Must parse instruction first")
        
        # Determine number of robots
        num_robots = self.formation_plan.num_robots
        if num_robots is None:
            # Use default based on formation type
            if self.formation_plan.formation_type == "letter_b":
                num_robots = 20  # Default for letter B (optimal balance)
            else:
                num_robots = 10  # Default for other formations
        
        # Limit to maximum supported robots (24 in XML)
        if num_robots > 24:
            print(f"⚠️  Warning: Requested {num_robots} robots, but only 24 are supported. Using 24.")
            num_robots = 24
        
        print(f"\n📦 Initializing environment with {num_robots} robots...")
        self.env = NavEnvFormationORCA(
            xml_path=self.xml_path,
            num_robots=num_robots,
            grid_res=0.1
        )
        print(f"✅ Environment initialized")
        print(f"   Room bounds: {self.env.get_room_bounds()}")
        print(f"   Grid size: {self.env.grid.shape}")
    
    def plan_and_execute(self, use_viewer: bool = True):
        """Plan paths and execute formation in MuJoCo"""
        if self.env is None:
            raise RuntimeError("Must initialize environment first")
        
        use_letter_b = True
        if self.formation_plan and self.formation_plan.formation_type.lower() not in ["letter_b", "b"]:
            raise NotImplementedError("Currently only letter B formations are supported.")

        # Reset environment with B targets
        print(f"\n🔄 Resetting environment and setting formation targets...")
        obs = self.env.reset(use_B=use_letter_b)
        print(f"✅ Environment reset")

        # Execute in MuJoCo
        print(f"\n🎬 Starting MuJoCo 3D Visualization...")
        print("=" * 70)
        
        # Simulation parameters
        dt = 0.02  # 50 Hz
        max_time = 40.0  # Maximum simulation time
        max_steps = int(max_time / dt)
        
        viewer = None
        if use_viewer:
            try:
                import mujoco
                viewer = mujoco.viewer.launch_passive(self.env.model, self.env.data)
                print(f"✅ MuJoCo 3D viewer launched")
                print(f"   Press ESC or close window to stop")
            except Exception as e:
                print(f"⚠️  Could not launch viewer: {e}")
                print(f"   Continuing in headless mode...")
                use_viewer = False
        
        print(f"\n   Running simulation (max {max_time:.1f}s)...")
        print(f"   Progress: ", end="", flush=True)
        
        start_time = time.time()
        all_done_count = 0
        last_status_print = 0
        
        for step in range(max_steps):
            # Step simulation
            obs, done = self.env.step(dt=dt)
            
            # Update viewer
            if use_viewer and viewer is not None:
                try:
                    viewer.sync()
                except Exception:
                    viewer = None
            
            # Check if all robots reached goals
            # Count how many robots are at their goals
            robots_at_goal = 0
            for agent_name in self.env.agent_names:
                if agent_name in self.env.goal_xy:
                    pos = self.env._get_body_xy(agent_name)
                    goal = self.env.goal_xy[agent_name]
                    dist = np.linalg.norm(np.array(pos) - np.array(goal))
                    if dist < 0.05:  # 5cm threshold
                        robots_at_goal += 1
            
            # Check if all robots reached goals
            if robots_at_goal == len(self.env.agent_names):
                all_done_count += 1
                if all_done_count >= 10:  # Require 10 consecutive checks
                    print(f"\n✅ All robots reached their formation positions!")
                    break
            else:
                all_done_count = 0  # Reset counter if not all done
                
            # Print status every 500 steps
            if step - last_status_print >= 500:
                print(f"\n   Step {step}/{max_steps}: {robots_at_goal}/{len(self.env.agent_names)} robots at goal", end="", flush=True)
                last_status_print = step
            
            # Progress indicator
            if step % 100 == 0:
                print(".", end="", flush=True)
        
        elapsed = time.time() - start_time
        print(f"\n   Simulation completed in {elapsed:.2f}s")
        
        # Final status
        print(f"\n📊 Final Status:")
        for agent_name in self.env.agent_names:
            pos = self.env._get_body_xy(agent_name)
            if agent_name in self.env.goal_xy:
                goal = self.env.goal_xy[agent_name]
                dist = np.linalg.norm(np.array(pos) - np.array(goal))
                status = "✅" if dist < 0.1 else "⏳"
                print(f"   {status} {agent_name}: distance to goal = {dist:.3f}m")
        
        if use_viewer and viewer is not None:
            print(f"\n💡 Viewer is still open. Close it when done.")
            try:
                input("Press Enter to close viewer and exit...")
                viewer.close()
            except:
                pass


def main():
    """Main function for end-to-end formation control"""
    print("=" * 70)
    print("🎯 End-to-End Natural Language Formation Control")
    print("=" * 70)
    print("\nThis system:")
    print("  1. Parses natural language formation instructions using LLM")
    print("  2. Generates formation target positions")
    print("  3. Plans collision-free paths for all robots")
    print("  4. Executes formation in MuJoCo 3D environment")
    print("=" * 70)
    
    # Get XML path
    xml_path = os.path.join(project_root, "nav_world", "room_formation.xml")
    
    if not os.path.exists(xml_path):
        print(f"❌ XML file not found: {xml_path}")
        print("   Please ensure room_formation.xml exists in nav_world/")
        return
    
    # Initialize controller
    controller = EndToEndFormationController(
        xml_path=xml_path,
        use_offline_parser=False  # Set to True to avoid API calls
    )
    
    # Get user instruction
    print("\n" + "=" * 70)
    print("📝 Enter your formation instruction:")
    print("=" * 70)
    print("Examples:")
    print('  - "form the letter B"')
    print('  - "让机器人形成字母B"')
    print('  - "form letter B with 20 robots on the right side"')
    print('  - "让20个机器人形成字母B在右侧"')
    print("=" * 70)
    
    user_text = input("\nYour instruction: ").strip()
    
    if not user_text:
        print("⚠️  No instruction provided, using default: 'form the letter B'")
        user_text = "form the letter B"
    
    try:
        # Step 1: Parse instruction
        formation_plan = controller.parse_user_instruction(user_text)
        
        # Step 2: Initialize environment
        controller.initialize_environment()
        
        # Step 3: Plan and execute
        controller.plan_and_execute(use_viewer=True)
        
        print("\n" + "=" * 70)
        print("✅ End-to-end formation task completed successfully!")
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n\n⏹️  Interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

