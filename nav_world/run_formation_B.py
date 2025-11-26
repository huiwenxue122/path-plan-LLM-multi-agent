#!/usr/bin/env python3
"""
Simple demo: robots form the letter B using the ORCA-based formation environment.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Tuple

import numpy as np

# Add project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world.nav_env_formation_orca import NavEnvFormationORCA


def _num_robots_type(value: str) -> int:
    n = int(value)
    if n <= 0:
        raise argparse.ArgumentTypeError("num_robots must be > 0")
    return n


def run_demo(xml_path: str, num_robots: int, sim_time: float, dt: float, use_viewer: bool) -> None:
    env = NavEnvFormationORCA(xml_path=xml_path, num_robots=num_robots)
    env.reset(use_B=True)

    max_steps = int(sim_time / dt)
    print(f"🚀 Running formation demo for up to {sim_time:.1f}s ({max_steps} steps)")

    viewer = None
    if use_viewer:
        try:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(env.model, env.data)
            print("✅ MuJoCo viewer launched (press ESC to exit)")
        except Exception as exc:  # pragma: no cover - viewer availability varies
            print(f"⚠️  Could not start viewer ({exc}). Continuing headless.")
            viewer = None

    start = time.time()
    for step_idx in range(max_steps):
        obs, done = env.step(dt=dt)

        if viewer is not None:
            try:
                viewer.sync()
            except Exception:
                viewer = None

        if step_idx % 50 == 0 or done:
            dists = _distance_to_goals(env)
            reached = sum(d < 0.05 for d in dists)
            print(f"   Step {step_idx:04d}: {reached}/{len(dists)} robots at goal")

        if done:
            break

    elapsed = time.time() - start
    print(f"\n⏱️  Simulation finished in {elapsed:.2f}s")
    print("📊 Final distances to targets:")
    
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

    if viewer is not None:
        try:
            input("Press Enter to close viewer...")
            viewer.close()
        except Exception:
            pass


def _distance_to_goals(env: NavEnvFormationORCA):
    dists = []
    for name in env.agent_names:
        pos = np.array(env._get_body_xy(name))
        goal = np.array(env.goal_xy[name])
        dists.append(float(np.linalg.norm(pos - goal)))
    return dists


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Formation demo using ORCA")
    parser.add_argument("--num-robots", type=_num_robots_type, default=20, help="number of robots (default: 20)")
    parser.add_argument("--sim-time", type=float, default=35.0, help="simulation time in seconds (default: 35)")
    parser.add_argument("--dt", type=float, default=0.05, help="simulation timestep (default: 0.05)")
    parser.add_argument("--viewer", action="store_true", help="launch MuJoCo viewer")
    return parser.parse_args()


def main():
    args = parse_args()
    xml_path = os.path.join(os.path.dirname(__file__), "room_formation.xml")
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"room_formation.xml not found at {xml_path}")

    run_demo(
        xml_path=xml_path,
        num_robots=args.num_robots,
        sim_time=args.sim_time,
        dt=args.dt,
        use_viewer=args.viewer,
    )


if __name__ == "__main__":
    main()


