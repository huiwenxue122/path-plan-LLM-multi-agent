#!/usr/bin/env python3
"""
Formation generation library for backend.

This module wraps the existing formation generators and adds support for
additional formations like "line" and "random".
"""

import os
import sys
import random
from typing import List, Tuple

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world import formations as nav_formations


def generate_formation(
    formation_type: str,
    room_bounds: Tuple[float, float, float, float],
    num_agents: int,
    region: str = "right_side"
) -> List[Tuple[float, float]]:
    """
    Generate target positions for a formation.
    
    Args:
        formation_type: Type of formation ("B", "circle", "grid", "heart", "line", "random")
        room_bounds: (xmin, xmax, ymin, ymax) room boundaries
        num_agents: Number of robots
        region: Where to place formation ("left_side", "right_side", "center")
        
    Returns:
        List of (x, y) target positions
    """
    if formation_type == "B":
        return nav_formations.generate_letter_b_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=region,
            margin=0.5  # Increased margin for better spacing
        )
    elif formation_type == "circle":
        return nav_formations.generate_circle_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=region,
            margin=0.5  # Increased margin for better spacing
        )
    elif formation_type == "grid":
        return nav_formations.generate_grid_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=region,
            margin=0.5  # Increased margin for better spacing
        )
    elif formation_type == "heart":
        return nav_formations.generate_heart_targets(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=region,
            margin=0.5  # Increased margin for better spacing
        )
    elif formation_type == "line":
        return generate_line_formation(
            num_agents=num_agents,
            room_bounds=room_bounds,
            region=region,
            margin=0.3
        )
    elif formation_type == "random":
        return generate_random_formation(
            num_agents=num_agents,
            room_bounds=room_bounds,
            margin=0.5
        )
    else:
        raise ValueError(f"Unknown formation type: {formation_type}")


def generate_line_formation(
    num_agents: int,
    room_bounds: Tuple[float, float, float, float],
    region: str = "right_side",
    margin: float = 0.3
) -> List[Tuple[float, float]]:
    """
    Generate a straight line formation.
    
    Args:
        num_agents: Number of robots
        room_bounds: (xmin, xmax, ymin, ymax) room boundaries
        region: Where to place the line ("left_side", "right_side", "center")
        margin: Margin from room edges
        
    Returns:
        List of (x, y) target positions
    """
    xmin, xmax, ymin, ymax = room_bounds
    room_width = xmax - xmin
    room_height = ymax - ymin
    
    # Line spans 70% of room height
    line_length = room_height * 0.7
    spacing = line_length / max(1, num_agents - 1) if num_agents > 1 else 0
    
    # Determine line position based on region
    if region == "right_side":
        x = xmax - margin - 1.0  # 1m from right wall
        y_start = (ymin + ymax) / 2 - line_length / 2
    elif region == "left_side":
        x = xmin + margin + 1.0  # 1m from left wall
        y_start = (ymin + ymax) / 2 - line_length / 2
    elif region == "center":
        x = (xmin + xmax) / 2
        y_start = (ymin + ymax) / 2 - line_length / 2
    else:
        raise ValueError(f"Unknown region: {region}")
    
    # Generate points along the line
    targets = []
    for i in range(num_agents):
        y = y_start + i * spacing
        targets.append((float(x), float(y)))
    
    return targets


def generate_random_formation(
    num_agents: int,
    room_bounds: Tuple[float, float, float, float],
    margin: float = 0.5
) -> List[Tuple[float, float]]:
    """
    Generate random scattered positions.
    
    Args:
        num_agents: Number of robots
        room_bounds: (xmin, xmax, ymin, ymax) room boundaries
        margin: Margin from room edges
        
    Returns:
        List of (x, y) target positions
    """
    xmin, xmax, ymin, ymax = room_bounds
    
    targets = []
    min_spacing = 0.4  # Minimum distance between robots
    
    for _ in range(num_agents):
        max_attempts = 100
        for attempt in range(max_attempts):
            x = random.uniform(xmin + margin, xmax - margin)
            y = random.uniform(ymin + margin, ymax - margin)
            
            # Check spacing from existing targets
            too_close = False
            for tx, ty in targets:
                dist = ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
                if dist < min_spacing:
                    too_close = True
                    break
            
            if not too_close:
                targets.append((float(x), float(y)))
                break
        
        # If we couldn't find a good position, just add it anyway
        if len(targets) < num_agents and attempt == max_attempts - 1:
            x = random.uniform(xmin + margin, xmax - margin)
            y = random.uniform(ymin + margin, ymax - margin)
            targets.append((float(x), float(y)))
    
    return targets


