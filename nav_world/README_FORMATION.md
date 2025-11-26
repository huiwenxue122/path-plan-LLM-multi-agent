# Letter B Formation Demo (ORCA Edition)

This module now uses an ORCA-based controller to drive a swarm of robots into
the letter **B** (for Brown). The new implementation is compact, easier to
read, and far more robust than the previous path-planning-heavy version.

## Features

- **Multi-robot support**: 10–20 robots (default: 12)
- **Open room**: Only the room walls are treated as obstacles
- **Letter B pattern**: Built-in generator that maps a coarse template to the
  right side of the room
- **ORCA collision avoidance**: Powered by `rvo2`, eliminating the need for
  complex hand-written speed logic
- **3D visualization**: Works with the MuJoCo viewer or headless mode

## Key Files

- `nav_world/orca_controller.py` – Thin wrapper on top of `rvo2`
- `nav_world/nav_env_formation_orca.py` – ORCA-based formation environment
- `nav_world/run_formation_B.py` – CLI demo to watch robots draw the letter B
- `nav_world/room_formation.xml` – MuJoCo XML with robot bodies and no internal obstacles

## Quick Start

```bash
# Simple run (headless)
python nav_world/run_formation_B.py

# With MuJoCo viewer
python nav_world/run_formation_B.py --viewer

# Different robot count and timestep
python nav_world/run_formation_B.py --num-robots 14 --dt 0.04
```

The script prints live progress (how many robots are within goal tolerance) and
summarises final distances to the targets.

## How It Works

1. **Environment setup** – `NavEnvFormationORCA` creates `robot_0 ... robot_N`.
2. **Start configuration** – Robots spawn on the left side of the room.
3. **Target generation** – A simple bitmap template is sampled to produce the
   letter B near the right wall.
4. **ORCA step** – On every timestep the ORCA controller assigns safe velocities
   towards the goals.
5. **MuJoCo execution** – Robots move smoothly without collisions until all
   targets are reached.

## Notes

- The previous `nav_env_formation.py` (A* + custom collision avoidance) has been
  deprecated. Use the ORCA version (`nav_env_formation_orca.py`) for all new
  demos.
- Make sure `rvo2` is installed (`pip install rvo2`) before running the demo.
- The environment still reuses `room_formation.xml`, so no XML changes are
  required.

