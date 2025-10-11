#!/usr/bin/env python3
"""
Safe navigation demo that avoids MuJoCo rendering issues on macOS
"""
import os
import sys
import numpy as np

# Add the project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.insert(0, project_root)

from real_world.nav_world.nav_env import NavEnv

def safe_demo():
    """Run navigation demo without rendering to avoid segfaults"""
    print("🤖 Starting Safe Navigation Demo")
    print("=" * 50)
    
    # Get the XML file path
    here = os.path.dirname(__file__)
    xml_path = os.path.join(here, "room.xml")
    
    if not os.path.exists(xml_path):
        print(f"❌ XML file not found: {xml_path}")
        return
    
    try:
        # Initialize environment
        print("📦 Initializing NavEnv...")
        env = NavEnv(xml_path=xml_path, grid_res=0.1)
        print(f"✅ Environment initialized")
        print(f"   Grid shape: {env.grid.shape}")
        print(f"   Agents: {env.agent_names}")
        
        # Reset environment
        print("\n🔄 Resetting environment...")
        obs = env.reset()
        print("✅ Environment reset")
        
        # Print initial state
        for agent_name in env.agent_names:
            agent_obs = obs[agent_name]
            print(f"   {agent_name}: pos={agent_obs['xy']}, goal={agent_obs['goal']}")
        
        # Run simulation without rendering
        print("\n🏃 Running simulation (no rendering)...")
        T = 10.0  # Shorter simulation
        fps = 30
        dt = 1.0 / fps
        steps = int(T * fps)
        
        for step_i in range(steps):
            obs, done = env.step(dt=dt)
            
            # Print progress every 30 steps
            if step_i % 30 == 0:
                print(f"   Step {step_i}/{steps}")
                for agent_name in env.agent_names:
                    agent_obs = obs[agent_name]
                    print(f"     {agent_name}: pos={agent_obs['xy']}")
            
            if done:
                print(f"🎯 Task completed at step {step_i}")
                break
        
        print("\n✅ Simulation completed successfully!")
        
        # Final positions
        print("\n📊 Final Results:")
        for agent_name in env.agent_names:
            final_pos = env._get_body_xy(agent_name)
            goal_pos = env.goal_xy[agent_name]
            distance = np.linalg.norm(np.array(final_pos) - np.array(goal_pos))
            print(f"   {agent_name}: final_pos={final_pos}, goal={goal_pos}, distance={distance:.3f}")
        
    except Exception as e:
        print(f"❌ Error during simulation: {e}")
        import traceback
        traceback.print_exc()

def render_demo():
    """Try rendering demo (may cause segfault on macOS)"""
    print("🎨 Attempting rendering demo...")
    print("⚠️  This may cause segmentation fault on macOS")
    
    try:
        here = os.path.dirname(__file__)
        xml_path = os.path.join(here, "room.xml")
        env = NavEnv(xml_path=xml_path, grid_res=0.1)
        env.reset()
        
        # Try to render one frame
        frame = env.render_rgb()
        print(f"✅ Rendering successful! Frame shape: {frame.shape}")
        
    except Exception as e:
        print(f"❌ Rendering failed: {e}")

if __name__ == "__main__":
    print("🚀 Co-Robot Pathfinding Demo")
    print("Choose an option:")
    print("1. Safe demo (no rendering)")
    print("2. Try rendering demo (may crash)")
    print("3. Both")
    
    choice = input("Enter choice (1-3): ").strip()
    
    if choice in ["1", "3"]:
        safe_demo()
    
    if choice in ["2", "3"]:
        render_demo()
