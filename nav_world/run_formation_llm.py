#!/usr/bin/env python3
"""
LLM-Controlled Formation Demo

Allows natural language control of multi-robot formation tasks.
Uses LLM to parse commands like "form a letter B on the right side"
into structured FormationTask objects.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Tuple, List

import numpy as np

# Add project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world.nav_env_formation_orca import NavEnvFormationORCA
from nav_world.llm_formation_controller import LLMFormationController
from nav_world.formation_task import FormationTask
from nav_world import formations


def _distance_to_goals(env: NavEnvFormationORCA) -> List[float]:
    """Calculate distances from all robots to their goals."""
    dists = []
    for name in env.agent_names:
        pos = np.array(env._get_body_xy(name))
        goal = np.array(env.goal_xy[name])
        dists.append(float(np.linalg.norm(pos - goal)))
    return dists


def _generate_targets_from_task(
    task: FormationTask,
    num_agents: int,
    room_bounds: Tuple[float, float, float, float]
) -> List[Tuple[float, float]]:
    """
    Generate target positions based on FormationTask.
    
    REFACTORED: This is a fallback function (kept for compatibility).
    Prefer using env.generate_formation_targets(shape, region) directly.
    
    Args:
        task: FormationTask object (Pydantic validated)
        num_agents: Number of agents (should match task.num_agents)
        room_bounds: (xmin, xmax, ymin, ymax) room boundaries
        
    Returns:
        List of (x, y) target positions
    """
    from nav_world import formations
    
    if task.shape == "B":
        return formations.generate_letter_b_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=task.region,
            margin=0.3
        )
    elif task.shape == "circle":
        return formations.generate_circle_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=task.region,
            margin=0.3
        )
    elif task.shape == "grid":
        return formations.generate_grid_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=task.region,
            margin=0.3
        )
    elif task.shape == "heart":
        return formations.generate_heart_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=task.region,
            margin=0.3
        )
    else:
        raise ValueError(f"Unsupported shape: {task.shape}. Supported: 'B', 'circle', 'grid', 'heart'")


def run_llm_formation(
    xml_path: str,
    prompt: Optional[str],
    num_robots: int,
    sim_time: float,
    dt: float,
    use_viewer: bool
) -> None:
    """
    Run formation demo with LLM control.
    
    Args:
        xml_path: Path to MuJoCo XML file
        prompt: Natural language instruction (None = prompt user)
        num_robots: Number of robots
        sim_time: Maximum simulation time in seconds
        dt: Simulation timestep
        use_viewer: Whether to launch MuJoCo viewer
    """
    # Get user prompt if not provided
    if prompt is None:
        print("\n💬 Enter your formation command:")
        print("   Examples:")
        print("   - 'form a letter B on the right side'")
        print("   - 'move all robots to form B on the left'")
        print("   - 'arrange robots in a B shape in the center'")
        prompt = input("   > ").strip()
        if not prompt:
            print("⚠️  No command provided. Using default: 'form a letter B on the right side'")
            prompt = "form a letter B on the right side"
    
    print(f"\n📝 Parsing command: '{prompt}'")
    
    # REFACTORED: Parse command using LLM first to get num_agents
    # This allows us to create the environment with the correct number of robots
    controller = LLMFormationController(model_name="gpt-4o-mini")
    task = controller.parse_command(prompt)
    
    print(f"✅ Parsed task: {task}")
    print(f"   Shape: {task.shape}")
    print(f"   Region: {task.region}")
    print(f"   Number of agents: {task.num_agents}")
    
    # REFACTORED: Create environment with num_agents from LLM task
    # This ensures the environment matches the LLM's decision
    print(f"\n🚀 Initializing formation environment with {task.num_agents} robots...")
    env = NavEnvFormationORCA(xml_path=xml_path, num_robots=task.num_agents)
    
    # Get room bounds
    room_bounds = env.get_room_bounds()
    print(f"📐 Room bounds: x=[{room_bounds[0]:.1f}, {room_bounds[1]:.1f}], "
          f"y=[{room_bounds[2]:.1f}, {room_bounds[3]:.1f}]")
    
    # REFACTORED: Generate targets using env.generate_formation_targets(shape, region)
    # This is the unified pattern for all formation task execution
    print(f"\n🎯 Generating {task.shape} formation targets in region: {task.region}...")
    
    # Use unified formation target generation method
    targets = env.generate_formation_targets(
        shape=task.shape,
        region=task.region,
        margin=0.3
    )
    print(f"   Generated {len(targets)} target positions")
    
    # Reset environment with targets
    # Note: use_B parameter is legacy, kept for backward compatibility
    env.reset(target_positions=targets, use_B=(task.shape == "B"))
    
    # Setup viewer if requested
    viewer = None
    if use_viewer:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(env.model, env.data)
            print("✅ MuJoCo viewer launched (press ESC to exit)")
        except Exception as exc:
            print(f"⚠️  Could not start viewer ({exc}). Continuing headless.")
            viewer = None
    
    # Run simulation
    max_steps = int(sim_time / dt)
    print(f"\n🏃 Running simulation for up to {sim_time:.1f}s ({max_steps} steps)")
    print(f"   Timestep: {dt:.3f}s, FPS: {1.0/dt:.1f}")
    
    start = time.time()
    for step_idx in range(max_steps):
        obs, done = env.step(dt=dt)
        
        if viewer is not None:
            try:
                viewer.sync()
            except Exception:
                viewer = None
        
        # Print progress periodically
        if step_idx % 50 == 0 or done:
            dists = _distance_to_goals(env)
            reached = sum(d < 0.05 for d in dists)
            print(f"   Step {step_idx:04d}: {reached}/{len(dists)} robots at goal")
        
        if done:
            break
    
    elapsed = time.time() - start
    print(f"\n⏱️  Simulation finished in {elapsed:.2f}s")
    
    # Print final distances
    print("\n📊 Final distances to targets:")
    dists = _distance_to_goals(env)
    reached_count = sum(1 for d in dists if d < 0.15)
    total_count = len(dists)
    
    for name, dist in zip(env.agent_names, dists):
        if dist < 0.05:
            status = "✅"
        elif dist < 0.15:
            status = "✓"
        else:
            status = "⏳"
        print(f"   {status} {name}: {dist:.3f} m")
    
    # Summary
    print(f"\n📈 Summary: {reached_count}/{total_count} robots within 0.15m of target")
    if reached_count == total_count:
        print("✅ All robots successfully reached their formation positions!")
    elif reached_count >= total_count * 0.8:
        print("✓ Most robots reached their targets. Formation looks good.")
    else:
        print("⚠️  Some robots did not reach targets. Formation may need adjustment.")
    
    # Keep viewer open if requested
    if viewer is not None:
        try:
            input("\nPress Enter to close viewer...")
            viewer.close()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="LLM-controlled formation demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python nav_world/run_formation_llm.py --viewer --prompt "form a letter B on the right side"
  python nav_world/run_formation_llm.py --prompt "move all robots to form B on the left"
  python nav_world/run_formation_llm.py --viewer  # Will prompt for command interactively
        """
    )
    parser.add_argument(
        "--prompt",
        type=str,
        default=None,
        help="Natural language formation command (if not provided, will prompt interactively)"
    )
    parser.add_argument(
        "--num-robots",
        type=int,
        default=20,
        help="Number of robots (default: 20)"
    )
    parser.add_argument(
        "--sim-time",
        type=float,
        default=35.0,
        help="Maximum simulation time in seconds (default: 35.0)"
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.05,
        help="Simulation timestep in seconds (default: 0.05)"
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Launch MuJoCo viewer"
    )
    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()
    
    # Find XML file
    xml_path = os.path.join(os.path.dirname(__file__), "room_formation.xml")
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"room_formation.xml not found at {xml_path}")
    
    # Validate arguments
    if args.num_robots <= 0:
        raise ValueError("--num-robots must be > 0")
    if args.sim_time <= 0:
        raise ValueError("--sim-time must be > 0")
    if args.dt <= 0:
        raise ValueError("--dt must be > 0")
    
    # Run demo
    run_llm_formation(
        xml_path=xml_path,
        prompt=args.prompt,
        num_robots=args.num_robots,
        sim_time=args.sim_time,
        dt=args.dt,
        use_viewer=args.viewer,
    )


if __name__ == "__main__":
    main()

