#!/usr/bin/env python3
"""
Multi-Agent Path Finding (MAPF) using Priority Planning

This module implements a priority-based MAPF algorithm that:
1. Plans paths for agents in priority order
2. Uses a reservation table to prevent vertex and edge conflicts
3. Supports "wait" actions (agents can stay still to avoid collisions)
4. Returns time-stamped paths for each agent

Key Concepts:
- Reservation Table: Stores occupied positions (x, y, t) and edges (x1, y1, x2, y2, t)
- Vertex Conflict: Two agents at the same position at the same time
- Edge Conflict: Two agents swapping positions at consecutive timesteps
- Priority Planning: Higher-priority agents plan first; later agents avoid their trajectories
"""

from __future__ import annotations
import heapq
from typing import List, Tuple, Optional, Dict, Set
from dataclasses import dataclass
import numpy as np


@dataclass
class AgentSpec:
    """Agent specification for path planning"""
    id: str
    start: Tuple[int, int]  # (x, y) in grid coordinates
    goal: Tuple[int, int]    # (x, y) in grid coordinates


class ReservationTable:
    """
    Reservation Table: Tracks occupied positions and edges in space-time
    
    This is the core mechanism for preventing collisions:
    - Vertex reservations: (x, y, t) positions occupied by agents
    - Edge reservations: (x1, y1, x2, y2, t) edges traversed by agents
    """
    
    def __init__(self):
        # Set of occupied vertices: (x, y, t)
        self.vertices: Set[Tuple[int, int, int]] = set()
        
        # Set of occupied edges: (x1, y1, x2, y2, t)
        # Represents agent moving from (x1, y1) to (x2, y2) at time t
        self.edges: Set[Tuple[int, int, int, int, int]] = set()
    
    def reserve_vertex(self, x: int, y: int, t: int):
        """Reserve a vertex (position) at a specific time"""
        self.vertices.add((x, y, t))
    
    def reserve_edge(self, x1: int, y1: int, x2: int, y2: int, t: int):
        """Reserve an edge (movement) at a specific time"""
        self.edges.add((x1, y1, x2, y2, t))
    
    def is_vertex_reserved(self, x: int, y: int, t: int) -> bool:
        """Check if a vertex is already reserved"""
        return (x, y, t) in self.vertices
    
    def is_edge_reserved(self, x1: int, y1: int, x2: int, y2: int, t: int) -> bool:
        """Check if an edge is already reserved"""
        return (x1, y1, x2, y2, t) in self.edges
    
    def add_path(self, path: List[Tuple[int, int, int]]):
        """
        Add a complete path to the reservation table
        
        Args:
            path: List of (x, y, t) tuples representing the agent's trajectory
        """
        for i, (x, y, t) in enumerate(path):
            # Reserve vertex
            self.reserve_vertex(x, y, t)
            
            # Reserve edge (if not the last position)
            if i < len(path) - 1:
                x_next, y_next, t_next = path[i + 1]
                # Edge conflict: check if moving from (x,y) to (x_next, y_next) at time t
                # Also check reverse edge (swapping conflict)
                self.reserve_edge(x, y, x_next, y_next, t)
                self.reserve_edge(x_next, y_next, x, y, t)  # Reverse edge for swap detection
    
    def has_vertex_conflict(self, x: int, y: int, t: int) -> bool:
        """Check for vertex conflict: same position at same time"""
        return self.is_vertex_reserved(x, y, t)
    
    def has_edge_conflict(self, x1: int, y1: int, x2: int, y2: int, t: int) -> bool:
        """
        Check for edge conflict: swapping positions at consecutive timesteps
        
        An edge conflict occurs when:
        - Agent A moves from (x1, y1) to (x2, y2) at time t
        - Agent B moves from (x2, y2) to (x1, y1) at time t
        """
        # Check if the reverse edge is already reserved
        return self.is_edge_reserved(x2, y2, x1, y1, t)


def astar_time_extended(
    grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    reservation_table: ReservationTable,
    max_time: int = 200,
    progress_callback=None
) -> Optional[List[Tuple[int, int, int]]]:
    """
    Time-extended A* search with conflict avoidance
    
    This extends the standard A* algorithm to include:
    1. Time dimension: nodes are (x, y, t) instead of (x, y)
    2. Conflict checking: skips nodes/edges that are already reserved
    3. Wait actions: allows staying at the same position (0,0 move)
    
    Args:
        grid: 2D array where 0 = free, 1 = obstacle
        start: Starting position (x, y)
        goal: Goal position (x, y)
        reservation_table: Reservation table to check for conflicts
        max_time: Maximum timesteps to search
    
    Returns:
        Path as list of (x, y, t) tuples, or None if no path found
    """
    h, w = grid.shape
    
    def in_bounds(p: Tuple[int, int]) -> bool:
        """Check if position is within grid bounds"""
        return 0 <= p[0] < h and 0 <= p[1] < w
    
    def get_neighbors(x: int, y: int, t: int) -> List[Tuple[int, int, int]]:
        """
        Get valid neighbors including wait action
        
        Neighbors include:
        - 4-directional moves: (x±1, y, t+1), (x, y±1, t+1)
        - Wait action: (x, y, t+1) - staying at the same position
        """
        neighbors = []
        
        # 4-directional moves
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if in_bounds((nx, ny)) and grid[nx, ny] == 0:
                nt = t + 1
                # Check for conflicts
                if not reservation_table.has_vertex_conflict(nx, ny, nt):
                    if not reservation_table.has_edge_conflict(x, y, nx, ny, t):
                        neighbors.append((nx, ny, nt))
        
        # Wait action: stay at current position
        if t + 1 < max_time:
            if not reservation_table.has_vertex_conflict(x, y, t + 1):
                neighbors.append((x, y, t + 1))
        
        return neighbors
    
    # A* search with time dimension
    g = {(start[0], start[1], 0): 0}  # g-cost: (x, y, t) -> cost
    came_from: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
    
    # Priority queue: (f-cost, x, y, t)
    # f-cost = g-cost + heuristic
    def heuristic(x: int, y: int) -> int:
        """Manhattan distance heuristic"""
        return abs(x - goal[0]) + abs(y - goal[1])
    
    pq = [(heuristic(start[0], start[1]), start[0], start[1], 0)]
    visited: Set[Tuple[int, int, int]] = set()
    iterations = 0
    max_iterations = max_time * grid.size  # Reasonable limit
    
    while pq:
        iterations += 1
        
        # Progress callback every 1000 iterations
        if progress_callback and iterations % 1000 == 0:
            progress_callback(iterations)
        
        # Safety check to prevent infinite loops
        if iterations > max_iterations:
            return None
        
        f_cost, x, y, t = heapq.heappop(pq)
        node = (x, y, t)
        
        if node in visited:
            continue
        visited.add(node)
        
        # Goal reached
        if (x, y) == goal:
            # Reconstruct path
            path = [(x, y, t)]
            while node in came_from:
                node = came_from[node]
                path.append(node)
            return path[::-1]
        
        # Expand neighbors
        for nx, ny, nt in get_neighbors(x, y, t):
            neighbor = (nx, ny, nt)
            if neighbor in visited:
                continue
            
            # Calculate new g-cost
            # Cost is 1 for moving, 0.5 for waiting (prefer movement)
            move_cost = 1.0 if (nx, ny) != (x, y) else 0.5
            alt_g = g[node] + move_cost
            
            # Update if better path found
            if neighbor not in g or alt_g < g[neighbor]:
                g[neighbor] = alt_g
                came_from[neighbor] = node
                f_new = alt_g + heuristic(nx, ny)
                heapq.heappush(pq, (f_new, nx, ny, nt))
        
        # Time limit check
        if t >= max_time - 1:
            continue
    
    return None  # No path found


def plan_priority(
    grid: np.ndarray,
    agents: List[AgentSpec],
    order: List[str],
    max_time: int = 200
) -> Dict[str, List[Tuple[int, int, int]]]:
    """
    Priority-based Multi-Agent Path Finding (MAPF)
    
    This is the main planning function that:
    1. Plans paths for agents in priority order (order list)
    2. Higher-priority agents plan first
    3. Lower-priority agents plan paths avoiding higher-priority agents' trajectories
    4. Uses reservation table to prevent vertex and edge conflicts
    
    Args:
        grid: 2D array where 0 = free, 1 = obstacle
        agents: List of AgentSpec objects with id, start, and goal
        order: Priority order list (e.g., ['A', 'B'] means A plans first)
        max_time: Maximum timesteps for path planning
    
    Returns:
        Dictionary mapping agent IDs to their paths
        Each path is a list of (x, y, t) tuples
    
    Example:
        >>> grid = np.array([[0,0,0,0], [0,1,1,0], [0,0,0,0]])
        >>> agents = [
        ...     AgentSpec(id='A', start=(0,0), goal=(2,3)),
        ...     AgentSpec(id='B', start=(2,0), goal=(0,3))
        ... ]
        >>> paths = plan_priority(grid, agents, order=['A', 'B'])
        >>> print(paths['A'])  # [(0,0,0), (0,1,1), ...]
        >>> print(paths['B'])  # [(2,0,0), (1,0,1), ...]
    """
    # Create agent lookup
    agent_dict = {agent.id: agent for agent in agents}
    
    # Initialize reservation table
    reservation_table = ReservationTable()
    
    # Store planned paths
    paths: Dict[str, List[Tuple[int, int, int]]] = {}
    
    # Plan paths in priority order
    for idx, agent_id in enumerate(order):
        if agent_id not in agent_dict:
            raise ValueError(f"Agent ID '{agent_id}' not found in agents list")
        
        agent = agent_dict[agent_id]
        
        # Progress indicator
        print(f"[Priority Planning] Planning path for agent '{agent_id}' ({idx+1}/{len(order)})...", end='', flush=True)
        
        # Plan path for this agent (avoiding previously planned paths)
        path = astar_time_extended(
            grid=grid,
            start=agent.start,
            goal=agent.goal,
            reservation_table=reservation_table,
            max_time=max_time
        )
        
        if path is None:
            print(f"\n❌ Failed to find path for agent '{agent_id}'")
            raise RuntimeError(
                f"Failed to find path for agent '{agent_id}' "
                f"from {agent.start} to {agent.goal}. "
                f"This may be due to conflicts with higher-priority agents. "
                f"Try reducing max_time or adjusting priority order."
            )
        
        # Store path
        paths[agent_id] = path
        
        # Add path to reservation table for future agents
        reservation_table.add_path(path)
        
        # Success output
        print(f" ✅ {len(path)} steps, from {agent.start} to {agent.goal}")
    
    return paths


def validate_paths(
    paths: Dict[str, List[Tuple[int, int, int]]],
    grid: np.ndarray
) -> Tuple[bool, List[str]]:
    """
    Validate that all paths are collision-free
    
    Returns:
        (is_valid, list_of_errors)
    """
    errors = []
    
    # Check all pairs of agents
    agent_ids = list(paths.keys())
    for i, agent_a in enumerate(agent_ids):
        for agent_b in agent_ids[i + 1:]:
            path_a = paths[agent_a]
            path_b = paths[agent_b]
            
            # Find maximum time
            max_t = max(max(t for _, _, t in path_a), max(t for _, _, t in path_b))
            
            # Check for conflicts at each timestep
            for t in range(max_t + 1):
                # Get positions at time t (use last position if path ends earlier)
                pos_a = None
                for x, y, path_t in path_a:
                    if path_t == t:
                        pos_a = (x, y)
                    elif path_t > t:
                        break
                if pos_a is None:
                    pos_a = path_a[-1][:2]  # Use last position
                
                pos_b = None
                for x, y, path_t in path_b:
                    if path_t == t:
                        pos_b = (x, y)
                    elif path_t > t:
                        break
                if pos_b is None:
                    pos_b = path_b[-1][:2]  # Use last position
                
                # Check vertex conflict
                if pos_a == pos_b:
                    errors.append(
                        f"Vertex conflict: Agents '{agent_a}' and '{agent_b}' "
                        f"at position {pos_a} at time {t}"
                    )
                
                # Check edge conflict (swapping)
                if t > 0:
                    # Get previous positions
                    prev_pos_a = None
                    for x, y, path_t in path_a:
                        if path_t == t - 1:
                            prev_pos_a = (x, y)
                        elif path_t > t - 1:
                            break
                    if prev_pos_a is None:
                        prev_pos_a = path_a[0][:2]
                    
                    prev_pos_b = None
                    for x, y, path_t in path_b:
                        if path_t == t - 1:
                            prev_pos_b = (x, y)
                        elif path_t > t - 1:
                            break
                    if prev_pos_b is None:
                        prev_pos_b = path_b[0][:2]
                    
                    # Check if agents swapped positions
                    if prev_pos_a == pos_b and prev_pos_b == pos_a:
                        errors.append(
                            f"Edge conflict: Agents '{agent_a}' and '{agent_b}' "
                            f"swapped positions at time {t-1}->{t}"
                        )
    
    return len(errors) == 0, errors


# Simple test/demo function
if __name__ == "__main__":
    # Example usage
    grid = np.array([
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0]
    ])
    
    agents = [
        AgentSpec(id='A', start=(0, 0), goal=(2, 3)),
        AgentSpec(id='B', start=(2, 0), goal=(0, 3)),
    ]
    
    print("=" * 60)
    print("Multi-Agent Path Finding (MAPF) - Priority Planning Demo")
    print("=" * 60)
    print(f"\nGrid shape: {grid.shape}")
    print(f"Obstacles at: (1,1) and (1,2)")
    print(f"\nAgents:")
    for agent in agents:
        print(f"  {agent.id}: {agent.start} -> {agent.goal}")
    
    print(f"\nPriority order: A, B (A plans first)")
    print("\nPlanning paths...")
    
    paths = plan_priority(grid, agents, order=['A', 'B'], max_time=50)
    
    print("\n" + "=" * 60)
    print("Planning Results:")
    print("=" * 60)
    for agent_id, path in paths.items():
        print(f"\nAgent '{agent_id}' path ({len(path)} steps):")
        # Print first 5 and last 5 steps
        if len(path) <= 10:
            for x, y, t in path:
                print(f"  t={t:2d}: ({x}, {y})")
        else:
            for x, y, t in path[:5]:
                print(f"  t={t:2d}: ({x}, {y})")
            print("  ...")
            for x, y, t in path[-5:]:
                print(f"  t={t:2d}: ({x}, {y})")
    
    # Validate paths
    print("\n" + "=" * 60)
    print("Validation:")
    print("=" * 60)
    is_valid, errors = validate_paths(paths, grid)
    if is_valid:
        print("✅ All paths are collision-free!")
    else:
        print("❌ Conflicts detected:")
        for error in errors:
            print(f"  - {error}")

