#!/usr/bin/env python3
"""
Find Valid Navigation Commands

This script helps you find valid coordinates for robot navigation by:
1. Checking the environment bounds
2. Testing different goal positions
3. Generating working command examples
"""

import os
import sys
import numpy as np

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

from nav_world.nav_env_mapf import NavEnvMAPF
from nav_world.multi_agent_planner import plan_priority, AgentSpec as MAPFAgentSpec
from nav_world.nav_env import astar


def get_environment_info():
    """Get environment information"""
    xml_path = os.path.join(project_root, "nav_world", "room.xml")
    env = NavEnvMAPF(xml_path=xml_path, grid_res=0.1)
    env.reset(use_mapf=False)
    
    # Get initial positions
    alice_start = env._get_body_xy('alice')
    bob_start = env._get_body_xy('bob')
    
    # Get default goals
    alice_default_goal = env.goal_xy['alice']
    bob_default_goal = env.goal_xy['bob']
    
    # Environment bounds
    bounds = {
        'x_min': env.xmin,
        'x_max': env.xmax,
        'y_min': env.ymin,
        'y_max': env.ymax
    }
    
    return {
        'env': env,
        'alice_start': alice_start,
        'bob_start': bob_start,
        'alice_default_goal': alice_default_goal,
        'bob_default_goal': bob_default_goal,
        'bounds': bounds
    }


def test_goal_position(env, agent_name, goal_world):
    """Test if a goal position is reachable"""
    try:
        start = env._world2grid(env._get_body_xy(agent_name))
        goal = env._world2grid(goal_world)
        
        # Check if goal is within bounds
        if goal[0] < 0 or goal[0] >= env.grid.shape[0] or goal[1] < 0 or goal[1] >= env.grid.shape[1]:
            return False, "Goal out of grid bounds"
        
        # Check if goal is in obstacle
        if env.grid[goal[0], goal[1]] == 1:
            return False, "Goal is in obstacle"
        
        # Try A* pathfinding
        path = astar(env.grid.copy(), start, goal)
        if path is None:
            return False, "No path found"
        
        return True, f"Path found: {len(path)} steps"
    except Exception as e:
        return False, f"Error: {e}"


def find_valid_goals(env, agent_name, num_tests=50):
    """Find valid goal positions for an agent"""
    valid_goals = []
    
    # Test positions in a grid pattern, focusing on right side (where goals usually are)
    # Left side: x from -3.5 to -2.0
    # Right side: x from 2.0 to 3.5 (more common for goals)
    # Y: full range
    
    # Test right side first (more likely to be valid)
    x_right = np.linspace(2.0, 3.5, 8)
    y_range = np.linspace(env.ymin + 0.5, env.ymax - 0.5, 10)
    
    for x in x_right:
        for y in y_range:
            goal = (float(x), float(y))
            is_valid, message = test_goal_position(env, agent_name, goal)
            if is_valid:
                valid_goals.append(goal)
            if len(valid_goals) >= 15:  # Get more valid goals
                break
        if len(valid_goals) >= 15:
            break
    
    # Also test some positions in the middle
    x_mid = np.linspace(-1.0, 1.0, 5)
    for x in x_mid:
        for y in y_range[::2]:  # Sample every other y
            goal = (float(x), float(y))
            is_valid, message = test_goal_position(env, agent_name, goal)
            if is_valid and goal not in valid_goals:
                valid_goals.append(goal)
            if len(valid_goals) >= 20:
                break
        if len(valid_goals) >= 20:
            break
    
    return valid_goals


def generate_command_examples():
    """Generate working command examples"""
    print("=" * 60)
    print("🔍 Finding Valid Navigation Commands")
    print("=" * 60)
    
    info = get_environment_info()
    env = info['env']
    
    print("\n📊 Environment Information:")
    print(f"   Bounds: x ∈ [{info['bounds']['x_min']}, {info['bounds']['x_max']}], "
          f"y ∈ [{info['bounds']['y_min']}, {info['bounds']['y_max']}]")
    print(f"   Alice start: {info['alice_start']}")
    print(f"   Bob start: {info['bob_start']}")
    print(f"   Alice default goal: {info['alice_default_goal']}")
    print(f"   Bob default goal: {info['bob_default_goal']}")
    
    # Test default goals
    print("\n✅ Testing Default Goals:")
    alice_ok, alice_msg = test_goal_position(env, 'alice', info['alice_default_goal'])
    bob_ok, bob_msg = test_goal_position(env, 'bob', info['bob_default_goal'])
    print(f"   Alice goal {info['alice_default_goal']}: {'✅' if alice_ok else '❌'} {alice_msg}")
    print(f"   Bob goal {info['bob_default_goal']}: {'✅' if bob_ok else '❌'} {bob_msg}")
    
    # Find valid goals
    print("\n🔍 Finding Valid Goal Positions...")
    alice_valid = find_valid_goals(env, 'alice', num_tests=30)
    bob_valid = find_valid_goals(env, 'bob', num_tests=30)
    
    print(f"\n✅ Found {len(alice_valid)} valid goals for Alice")
    print(f"✅ Found {len(bob_valid)} valid goals for Bob")
    
    # Generate command examples
    print("\n" + "=" * 60)
    print("📝 Working Command Examples:")
    print("=" * 60)
    
    # Example 1: Use default goals
    if alice_ok and bob_ok:
        print("\n1️⃣  Using Default Goals (Recommended):")
        print(f'   "Robot A go to ({info["alice_default_goal"][0]:.1f}, {info["alice_default_goal"][1]:.1f}), '
              f'Robot B go to ({info["bob_default_goal"][0]:.1f}, {info["bob_default_goal"][1]:.1f}), A has priority"')
    
    # Example 2: Use found valid goals (prefer right side)
    if alice_valid and bob_valid:
        # Prefer goals on the right side (x > 0)
        alice_right = [g for g in alice_valid if g[0] > 0]
        bob_right = [g for g in bob_valid if g[0] > 0]
        
        if alice_right and bob_right:
            print("\n2️⃣  Using Valid Goals (Right Side):")
            alice_goal = alice_right[0]
            bob_goal = bob_right[0]
            print(f'   "Robot A go to ({alice_goal[0]:.1f}, {alice_goal[1]:.1f}), '
                  f'Robot B go to ({bob_goal[0]:.1f}, {bob_goal[1]:.1f}), A has priority"')
        
        # Example with different y positions
        if len(alice_valid) > 2 and len(bob_valid) > 2:
            # Find goals with different y positions
            alice_upper = max([g for g in alice_valid if g[0] > 0], key=lambda g: g[1], default=None)
            bob_lower = min([g for g in bob_valid if g[0] > 0], key=lambda g: g[1], default=None)
            if alice_upper and bob_lower:
                print(f'\n3️⃣  Goals with Different Heights:')
                print(f'   "Robot A go to ({alice_upper[0]:.1f}, {alice_upper[1]:.1f}), '
                      f'Robot B go to ({bob_lower[0]:.1f}, {bob_lower[1]:.1f}), A has priority"')
    
    # Example 3: Simple goals (close to start)
    print("\n4️⃣  Simple Goals (Close to Start):")
    simple_alice = (2.0, 1.0)
    simple_bob = (2.0, -1.0)
    alice_simple_ok, _ = test_goal_position(env, 'alice', simple_alice)
    bob_simple_ok, _ = test_goal_position(env, 'bob', simple_bob)
    if alice_simple_ok and bob_simple_ok:
        print(f'   "Robot A go to ({simple_alice[0]:.1f}, {simple_alice[1]:.1f}), '
              f'Robot B go to ({simple_bob[0]:.1f}, {simple_bob[1]:.1f}), A has priority"')
    else:
        print("   (Testing simple goals...)")
        # Try other simple positions
        for x in [1.5, 2.0, 2.5]:
            for y in [-1.5, -1.0, -0.5, 0.5, 1.0, 1.5]:
                test_goal = (x, y)
                alice_ok, _ = test_goal_position(env, 'alice', test_goal)
                bob_ok, _ = test_goal_position(env, 'bob', test_goal)
                if alice_ok and bob_ok:
                    print(f'   "Robot A go to ({test_goal[0]:.1f}, {test_goal[1]:.1f}), '
                          f'Robot B go to ({test_goal[0]:.1f}, {-test_goal[1]:.1f}), A has priority"')
                    break
            if alice_ok and bob_ok:
                break
    
    # Show valid goal lists
    print("\n" + "=" * 60)
    print("📋 Valid Goal Positions:")
    print("=" * 60)
    print(f"\nAlice valid goals (first 5):")
    for i, goal in enumerate(alice_valid[:5], 1):
        print(f"   {i}. ({goal[0]:.2f}, {goal[1]:.2f})")
    
    print(f"\nBob valid goals (first 5):")
    for i, goal in enumerate(bob_valid[:5], 1):
        print(f"   {i}. ({goal[0]:.2f}, {goal[1]:.2f})")
    
    print("\n" + "=" * 60)
    print("💡 Tips:")
    print("=" * 60)
    print("1. Use the default goals for best results")
    print("2. Keep goals within bounds: x ∈ [-4, 4], y ∈ [-3, 3]")
    print("3. Avoid positions too close to walls (keep 0.5m margin)")
    print("4. Test your command with the examples above")
    print("=" * 60)


if __name__ == "__main__":
    try:
        generate_command_examples()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

