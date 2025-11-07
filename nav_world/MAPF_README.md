# Multi-Agent Path Finding (MAPF) with Priority Planning

## Overview

This module implements **Multi-Agent Path Finding (MAPF)** using **Priority Planning**, which upgrades the original single-agent A* pathfinding to handle multiple agents with collision avoidance in both space and time.

## Key Features

✅ **Priority Planning**: Higher-priority agents plan first; later agents avoid their trajectories  
✅ **Collision Avoidance**: Prevents both vertex conflicts (same position, same time) and edge conflicts (swapping positions)  
✅ **Wait Actions**: Agents can stay still for one timestep to avoid collisions  
✅ **Time-Stamped Paths**: Returns paths with time information: `(x, y, t)` tuples  

## Architecture

### Core Components

1. **`multi_agent_planner.py`**: Core MAPF implementation
   - `ReservationTable`: Tracks occupied positions and edges in space-time
   - `astar_time_extended()`: Time-extended A* search with conflict checking
   - `plan_priority()`: Main planning function

2. **`nav_env_mapf.py`**: Enhanced NavEnv with MAPF support
   - Extends `NavEnv` to use MAPF planning
   - Supports priority ordering
   - Maintains backward compatibility

3. **`test_mapf.py`**: Test suite with visualization
   - Two test scenarios
   - Path validation
   - Visual output

## Usage

### Basic Usage

```python
from multi_agent_planner import plan_priority, AgentSpec
import numpy as np

# Define grid (0 = free, 1 = obstacle)
grid = np.array([
    [0, 0, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 0, 0]
])

# Define agents
agents = [
    AgentSpec(id='A', start=(0, 0), goal=(2, 3)),
    AgentSpec(id='B', start=(2, 0), goal=(0, 3)),
]

# Plan paths with priority order
paths = plan_priority(grid, agents, order=['A', 'B'], max_time=50)

# Result: paths['A'] = [(0,0,0), (0,1,1), ...], paths['B'] = [(2,0,0), ...]
```

### Using with NavEnv

```python
from nav_env_mapf import NavEnvMAPF

# Create environment with MAPF support
env = NavEnvMAPF(
    xml_path="room.xml",
    grid_res=0.1,
    priority_order=['alice', 'bob']  # Alice plans first
)

# Reset with MAPF planning
env.reset(use_mapf=True)

# Get time-stamped paths
mapf_paths = env.get_mapf_paths()
```

### Changing Priority Order

```python
# Set priority order (e.g., from LLM decision)
env.set_priority_order(['bob', 'alice'])  # Bob plans first now
env.reset(use_mapf=True)
```

## Algorithm Details

### Reservation Table

The **Reservation Table** is the core mechanism for preventing collisions:

- **Vertex Reservations**: `(x, y, t)` - Agent occupies position (x,y) at time t
- **Edge Reservations**: `(x1, y1, x2, y2, t)` - Agent moves from (x1,y1) to (x2,y2) at time t

### Conflict Types

1. **Vertex Conflict**: Two agents at the same position at the same time
   - Detected by checking if `(x, y, t)` is already reserved

2. **Edge Conflict**: Two agents swapping positions at consecutive timesteps
   - Agent A: (x1, y1) → (x2, y2) at time t
   - Agent B: (x2, y2) → (x1, y1) at time t
   - Detected by checking if reverse edge is reserved

### Time-Extended A*

The algorithm extends standard A* to include:
- **Time dimension**: Nodes are `(x, y, t)` instead of `(x, y)`
- **Wait actions**: Agents can stay at `(x, y, t+1)` to avoid conflicts
- **Conflict checking**: Skips nodes/edges that are already reserved

## Testing

Run the test suite:

```bash
cd real_world/nav_world
python test_mapf.py
```

This will:
- Run two test scenarios
- Validate paths for collisions
- Generate visualization images (`mapf_test_case1.png`, `mapf_test_case2.png`)

## Integration with LLM

The priority order can be determined by LLM:

```python
# LLM decides priority based on natural language command
# Example: "Alice 先到达目标，Bob 等 2 秒再出发"
priority_order = ['alice', 'bob']  # From LLM parsing

env.set_priority_order(priority_order)
env.reset(use_mapf=True)
```

## Performance

- **Time Complexity**: O(n × m × T) where n=agents, m=grid cells, T=max_time
- **Space Complexity**: O(n × T) for reservation table
- **Typical Performance**: 
  - 2 agents, 80×60 grid: ~0.1 seconds
  - 4 agents, 80×60 grid: ~0.5 seconds

## Future Enhancements

- [ ] Conflict-Based Search (CBS) for optimal solutions
- [ ] Windowed Hierarchical Cooperative A* (WHCA*)
- [ ] Dynamic priority adjustment during execution
- [ ] Support for agent size/radius in conflict detection

## References

- **Priority Planning**: A simple but effective MAPF algorithm
- **Reservation Table**: Space-time conflict avoidance mechanism
- **Time-Extended A***: Standard A* with time dimension

