#!/usr/bin/env python3
"""
Test script for Multi-Agent Path Finding (MAPF) with visualization

This script demonstrates the priority planning algorithm with:
1. Visual representation of the grid and obstacles
2. Agent paths displayed with different colors
3. Time-step visualization showing agent positions
4. Conflict validation
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import Dict, List, Tuple
from multi_agent_planner import plan_priority, AgentSpec, validate_paths


def visualize_paths(
    grid: np.ndarray,
    agents: List[AgentSpec],
    paths: Dict[str, List[Tuple[int, int, int]]],
    save_path: str = None
):
    """
    Visualize multi-agent paths on the grid
    
    Args:
        grid: 2D array where 0 = free, 1 = obstacle
        agents: List of agent specifications
        paths: Dictionary mapping agent IDs to paths
        save_path: Optional path to save the figure
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Left plot: Static path visualization
    ax1.imshow(grid, cmap='gray_r', origin='lower', alpha=0.3)
    ax1.set_title('Multi-Agent Path Planning - Complete Trajectories', fontsize=12)
    
    # Color map for agents
    colors = ['blue', 'green', 'red', 'orange', 'purple', 'brown']
    agent_colors = {agent.id: colors[i % len(colors)] for i, agent in enumerate(agents)}
    
    # Plot paths
    for agent_id, path in paths.items():
        color = agent_colors[agent_id]
        xs = [x for x, y, t in path]
        ys = [y for x, y, t in path]
        
        # Plot path line
        ax1.plot(ys, xs, color=color, linewidth=2, alpha=0.7, label=f'Agent {agent_id}')
        
        # Plot start and goal
        agent = next(a for a in agents if a.id == agent_id)
        ax1.plot(agent.start[1], agent.start[0], 'o', color=color, markersize=10, 
                markeredgecolor='black', markeredgewidth=1.5, label=f'{agent_id} Start')
        ax1.plot(agent.goal[1], agent.goal[0], 's', color=color, markersize=10,
                markeredgecolor='black', markeredgewidth=1.5, label=f'{agent_id} Goal')
    
    ax1.set_xlabel('Y (grid coordinates)')
    ax1.set_ylabel('X (grid coordinates)')
    ax1.legend(loc='best', fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # Right plot: Time-step visualization (first 20 steps)
    ax2.imshow(grid, cmap='gray_r', origin='lower', alpha=0.3)
    ax2.set_title('Multi-Agent Path Planning - Time Steps (t=0 to t=20)', fontsize=12)
    
    # Plot positions at different time steps
    max_t = min(20, max(max(t for _, _, t in path) for path in paths.values()))
    for t in range(0, max_t + 1, 2):  # Show every 2 steps
        for agent_id, path in paths.items():
            color = agent_colors[agent_id]
            # Find position at time t
            pos = None
            for x, y, path_t in path:
                if path_t == t:
                    pos = (x, y)
                    break
                elif path_t > t:
                    # Use previous position
                    if len(path) > 0:
                        prev_idx = max(0, path.index((x, y, path_t)) - 1)
                        pos = path[prev_idx][:2]
                    break
            
            if pos is None and len(path) > 0:
                pos = path[0][:2]  # Use start position
            
            if pos:
                ax2.plot(pos[1], pos[0], 'o', color=color, markersize=8, alpha=0.6)
                ax2.text(pos[1] + 0.1, pos[0] + 0.1, f't={t}', fontsize=6, color=color)
    
    ax2.set_xlabel('Y (grid coordinates)')
    ax2.set_ylabel('X (grid coordinates)')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"✅ Visualization saved to: {save_path}")
    else:
        plt.show()


def print_path_summary(paths: Dict[str, List[Tuple[int, int, int]]]):
    """Print a summary of the planned paths"""
    print("\n" + "=" * 60)
    print("Path Summary:")
    print("=" * 60)
    
    for agent_id, path in paths.items():
        print(f"\nAgent '{agent_id}':")
        print(f"  Path length: {len(path)} steps")
        print(f"  Start: {path[0][:2]} at t={path[0][2]}")
        print(f"  Goal:  {path[-1][:2]} at t={path[-1][2]}")
        print(f"  Duration: {path[-1][2] - path[0][2]} timesteps")
        
        # Count wait actions
        wait_count = sum(1 for i in range(len(path) - 1) 
                        if path[i][:2] == path[i+1][:2])
        if wait_count > 0:
            print(f"  Wait actions: {wait_count}")


def test_example_1():
    """Test case 1: Simple 2-agent scenario"""
    print("\n" + "=" * 60)
    print("Test Case 1: Simple 2-Agent Scenario")
    print("=" * 60)
    
    grid = np.array([
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0]
    ])
    
    agents = [
        AgentSpec(id='A', start=(0, 0), goal=(2, 3)),
        AgentSpec(id='B', start=(2, 0), goal=(0, 3)),
    ]
    
    print(f"\nGrid shape: {grid.shape}")
    print(f"Agents:")
    for agent in agents:
        print(f"  {agent.id}: {agent.start} -> {agent.goal}")
    
    paths = plan_priority(grid, agents, order=['A', 'B'], max_time=50)
    
    print_path_summary(paths)
    
    # Validate
    is_valid, errors = validate_paths(paths, grid)
    if is_valid:
        print("\n✅ Validation: All paths are collision-free!")
    else:
        print("\n❌ Validation failed:")
        for error in errors:
            print(f"  - {error}")
    
    # Visualize
    visualize_paths(grid, agents, paths, save_path='mapf_test_case1.png')
    
    return paths


def test_example_2():
    """Test case 2: More complex scenario with narrow passage"""
    print("\n" + "=" * 60)
    print("Test Case 2: Narrow Passage Scenario")
    print("=" * 60)
    
    grid = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 0, 0, 0, 0],
    ])
    
    agents = [
        AgentSpec(id='A', start=(0, 0), goal=(3, 4)),
        AgentSpec(id='B', start=(3, 4), goal=(0, 0)),
    ]
    
    print(f"\nGrid shape: {grid.shape}")
    print(f"Agents:")
    for agent in agents:
        print(f"  {agent.id}: {agent.start} -> {agent.goal}")
    
    paths = plan_priority(grid, agents, order=['A', 'B'], max_time=100)
    
    print_path_summary(paths)
    
    # Validate
    is_valid, errors = validate_paths(paths, grid)
    if is_valid:
        print("\n✅ Validation: All paths are collision-free!")
    else:
        print("\n❌ Validation failed:")
        for error in errors:
            print(f"  - {error}")
    
    # Visualize
    visualize_paths(grid, agents, paths, save_path='mapf_test_case2.png')
    
    return paths


if __name__ == "__main__":
    print("=" * 60)
    print("Multi-Agent Path Finding (MAPF) - Test Suite")
    print("=" * 60)
    
    # Run test cases
    paths1 = test_example_1()
    paths2 = test_example_2()
    
    print("\n" + "=" * 60)
    print("All tests completed!")
    print("=" * 60)

