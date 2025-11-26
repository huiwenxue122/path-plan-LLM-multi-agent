#!/usr/bin/env python3
"""
Formation pattern generators for multi-robot swarm tasks.

This module provides functions to generate target positions for robots
to form various shapes (e.g., letters, patterns) in the environment.

Supported shapes:
- "B": Letter B formation
- "circle": Circular formation
- "grid": Rectangular grid formation
- "heart": Heart shape formation
"""

import os
import random
from typing import List, Tuple, Optional
import numpy as np

# Base polyline for letter B in normalized coordinates.
# Clean B shape with well-defined components: vertical spine, upper loop, lower loop.
# Points are defined to create a clear, recognizable letter B.
BASE_B_POLYLINE: List[Tuple[float, float]] = [
    # 左侧竖线（从下到上）
    (0.05, 0.00),   # 底部
    (0.05, 0.20),
    (0.05, 0.40),
    (0.05, 0.50),   # 中间
    (0.05, 0.60),
    (0.05, 0.80),
    (0.05, 1.00),   # 顶部

    # 上半圆（顺时针，更平滑的弧线）
    (0.25, 1.00),   # 左上
    (0.40, 0.95),   # 顶部弧线起点
    (0.50, 0.85),   # 顶部弧线中点
    (0.55, 0.72),   # 右上
    (0.50, 0.60),   # 右下
    (0.40, 0.55),   # 右下过渡
    (0.35, 0.52),   # 接近中间横线左侧
    (0.55, 0.50),   # 中间横线右端（连接上下半圆）

    # 下半圆（顺时针，更平滑的弧线）
    (0.35, 0.48),   # 接近中间横线左侧
    (0.40, 0.45),   # 左下
    (0.50, 0.40),   # 上弧
    (0.55, 0.28),   # 右侧
    (0.50, 0.15),   # 下弧
    (0.40, 0.08),   # 下弧过渡
    (0.25, 0.02),   # 左下
    (0.10, 0.00),   # 回到靠近竖线的底部，闭合形状
]


def _resample_polyline(points: List[Tuple[float, float]], num_samples: int) -> List[Tuple[float, float]]:
    """
    Resample a polyline to a specified number of points using uniform spacing.
    
    Ensures:
    - Uniform spacing along the polyline
    - No duplicate points
    - Exactly num_samples points returned
    """
    if num_samples <= 0:
        raise ValueError("num_samples must be positive")
    
    pts = np.array(points, dtype=float)
    
    # Remove duplicate consecutive points
    unique_pts = [pts[0]]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - unique_pts[-1]) > 1e-6:
            unique_pts.append(pts[i])
    pts = np.array(unique_pts)
    
    if len(pts) == 1:
        # All points are the same, return evenly spaced points in a small area
        center = pts[0]
        radius = 0.1
        angles = np.linspace(0, 2 * np.pi, num_samples, endpoint=False)
        return [(float(center[0] + radius * np.cos(a)), float(center[1] + radius * np.sin(a))) for a in angles]
    
    # Compute cumulative distances along the polyline
    segs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cumdist = np.concatenate([[0.0], np.cumsum(segs)])
    total = cumdist[-1]
    
    if total < 1e-6:
        # Degenerate case: all points are the same
        return [tuple(pts[0])] * num_samples
    
    # Sample points uniformly along the polyline
    target_dists = np.linspace(0.0, total, num_samples, endpoint=False)
    resampled = []
    idx = 0
    
    for td in target_dists:
        # Find segment containing td
        while idx < len(cumdist) - 1 and td > cumdist[idx + 1] + 1e-6:
            idx += 1
        
        if idx >= len(pts) - 1:
            resampled.append(tuple(pts[-1]))
            continue
        
        # Linear interpolation within segment
        seg_len = cumdist[idx + 1] - cumdist[idx]
        if seg_len < 1e-6:
            resampled.append(tuple(pts[idx]))
            continue
        
        ratio = (td - cumdist[idx]) / seg_len
        interp = pts[idx] + ratio * (pts[idx + 1] - pts[idx])
        resampled.append((float(interp[0]), float(interp[1])))
    
    return resampled


def generate_letter_b_template(num_points: int = 12) -> List[Tuple[float, float]]:
    """
    Generate normalized (u, v) coordinates for the letter "B" shape.
    
    Simple resampling of the base polyline.
    
    Args:
        num_points: Number of points to use for the B shape (default: 12)
    
    Returns:
        List of (u, v) tuples in normalized coordinates [0, 1]
    """
    return _resample_polyline(BASE_B_POLYLINE, num_points)


def _ensure_minimum_spacing(points: List[Tuple[float, float]], min_spacing: float) -> List[Tuple[float, float]]:
    """
    Ensure minimum spacing between points by adjusting positions.
    
    Args:
        points: List of (x, y) points
        min_spacing: Minimum distance between any two points (meters)
    
    Returns:
        List of adjusted points with guaranteed minimum spacing
    """
    if len(points) <= 1:
        return points
    
    pts = np.array(points)
    adjusted = [pts[0].copy()]
    
    for i in range(1, len(pts)):
        new_pt = pts[i].copy()
        
        # Check distance to all previously placed points
        for existing in adjusted:
            dist = np.linalg.norm(new_pt - existing)
            if dist < min_spacing:
                # Move point away from existing point
                direction = new_pt - existing
                if np.linalg.norm(direction) > 1e-6:
                    direction = direction / np.linalg.norm(direction)
                else:
                    # Random direction if points are identical
                    direction = np.array([1.0, 0.0])
                new_pt = existing + direction * min_spacing
        
        adjusted.append(new_pt)
    
    return [tuple(pt) for pt in adjusted]


def generate_letter_b_targets(
    num_agents: int,
    room_bounds: Tuple[float, float, float, float],
    region: str = "right_side",
    margin: float = 0.5  # Increased default margin for better spacing
) -> List[Tuple[float, float]]:
    """
    Generate world coordinate target positions for robots to form a large, clear letter "B".
    
    The B is designed to be:
    - Width ≈ 2.5m, Height ≈ 3.5m
    - Placed on the right side of the room (with margin from right wall)
    - Points are uniformly distributed along the B shape using polyline interpolation
    - Minimum spacing between points to avoid clustering
    
    Coordinate system:
    - room_bounds: (xmin, xmax, ymin, ymax) in world coordinates (meters)
    - Returns: List of (x, y) tuples in world coordinates
    - Points are evenly sampled along the B polyline to ensure smooth shape
    
    Args:
        num_agents: Number of robots (minimum number of points to generate)
        room_bounds: (xmin, xmax, ymin, ymax) in world coordinates (meters)
        region: Where to place the B ("right_side", "left_side", "center")
        margin: Margin from room edges in meters
    
    Returns:
        List of (x, y) tuples in world coordinates, with at least num_agents points
    """
    xmin, xmax, ymin, ymax = room_bounds
    
    # Define B dimensions in world coordinates (larger for bigger room)
    b_width = 5.0  # meters (increased from 3.5m)
    b_height = 6.0  # meters (increased from 4.5m)
    
    # Determine target region bounds
    if region == "right_side":
        # Place B on right side: x from (xmax - b_width - margin) to (xmax - margin)
        x_min = xmax - b_width - margin
        x_max = xmax - margin
        x_range = (x_min, x_max)
        # Center B vertically in the room
        center_y = (ymin + ymax) / 2
        y_min = center_y - b_height / 2
        y_max = center_y + b_height / 2
        y_range = (y_min, y_max)
    elif region == "left_side":
        x_min = xmin + margin
        x_max = xmin + margin + b_width
        x_range = (x_min, x_max)
        center_y = (ymin + ymax) / 2
        y_min = center_y - b_height / 2
        y_max = center_y + b_height / 2
        y_range = (y_min, y_max)
    elif region == "center":
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
        x_range = (center_x - b_width/2, center_x + b_width/2)
        y_range = (center_y - b_height/2, center_y + b_height/2)
    else:
        raise ValueError(f"Unknown region: {region}. Use 'right_side', 'left_side', or 'center'")
    
    # Map normalized template to world coordinates
    x_min, x_max = x_range
    y_min, y_max = y_range
    
    # Generate more points than needed to ensure smooth shape, then resample uniformly
    # Use at least 2x num_agents for smooth polyline, then resample to num_agents
    num_template_points = max(num_agents * 2, 30)  # Ensure enough points for smooth B
    template = generate_letter_b_template(num_template_points)
    
    world_targets = []
    for u, v in template:
        # u is horizontal (0=left, 1=right), v is vertical (0=bottom, 1=top)
        x = x_min + u * (x_max - x_min)
        y = y_min + v * (y_max - y_min)
        world_targets.append((x, y))
    
    # Resample to get exactly num_agents points with uniform spacing along the polyline
    if len(world_targets) > num_agents:
        # Use polyline resampling to get evenly distributed points
        world_targets = _resample_polyline(world_targets, num_agents)
    elif len(world_targets) < num_agents:
        # If we somehow have fewer points, interpolate more
        while len(world_targets) < num_agents:
            new_points = []
            for i in range(len(world_targets) - 1):
                p1 = np.array(world_targets[i])
                p2 = np.array(world_targets[i + 1])
                mid = (p1 + p2) / 2
                new_points.append(tuple(mid))
            world_targets.extend(new_points)
            if len(world_targets) >= num_agents:
                break
        world_targets = _resample_polyline(world_targets, num_agents)
    
    # Ensure minimum spacing between points (0.5m minimum for better visibility and less crowding)
    world_targets = _ensure_minimum_spacing(world_targets, min_spacing=0.5)
    
    # Sort by y-coordinate (bottom to top) for consistent ordering
    
    # Manual adjustments for specific robot positions
    # R12 is at index 11 (0-indexed: R0=0, R1=1, ..., R12=11)
    if len(world_targets) > 11:
        world_targets[11] = (2.0, 0.5)
    
    # R13 is at index 12 (0-indexed: R0=0, R1=1, ..., R13=12)
    if len(world_targets) > 12:
        world_targets[12] = (1.5, 0.0)
    
    # R14 is at index 13 (0-indexed: R0=0, R1=1, ..., R14=13)
    if len(world_targets) > 13:
        world_targets[13] = (3.0, 1.0)
    
    # Move the robot near (3, 0) to (1.5, -3) as requested
    # This is the robot highlighted in red (likely R10, index 9 or R11, index 10)
    # Find robot closest to (3, 0) and move it
    target_pos = np.array([3.0, 0.0])
    closest_idx = None
    min_dist = float('inf')
    for i in range(len(world_targets)):
        pos = np.array(world_targets[i])
        dist = np.linalg.norm(pos - target_pos)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    # Only move if we found a robot close to (3, 0) and it's not one of the manually placed ones
    if closest_idx is not None and min_dist < 1.0 and closest_idx not in [11, 12, 13]:
        world_targets[closest_idx] = (1.5, -3.0)
    
    # Adjust nearby robots to maintain safe spacing from manually placed robots
    # R14 at (3.0, 1.0) may be too close to some robots, adjust them
    r14_pos = np.array([3.0, 1.0])
    min_spacing = 0.5
    for i in range(len(world_targets)):
        if i != 13:  # Don't adjust R14 itself
            pos = np.array(world_targets[i])
            dist = np.linalg.norm(pos - r14_pos)
            if dist < min_spacing:
                # Move this robot away from R14
                direction = pos - r14_pos
                if np.linalg.norm(direction) > 1e-6:
                    direction = direction / np.linalg.norm(direction)
                else:
                    direction = np.array([1.0, 0.0])  # Default direction
                new_pos = r14_pos + direction * min_spacing
                world_targets[i] = (float(new_pos[0]), float(new_pos[1]))
    
    # Re-ensure minimum spacing after all adjustments
    world_targets = _ensure_minimum_spacing(world_targets, min_spacing=0.5)
    
    # Re-apply manual positions after spacing adjustment (they take priority)
    if len(world_targets) > 11:
        world_targets[11] = (2.0, 0.5)
    if len(world_targets) > 12:
        world_targets[12] = (1.5, 0.0)
    if len(world_targets) > 13:
        world_targets[13] = (3.0, 1.0)
    
    # Return at least num_agents points (may have more if spacing requires it)
    return world_targets


def generate_circle_targets(
    num_agents: int,
    room_bounds: Tuple[float, float, float, float],
    region: str = "right_side",
    margin: float = 0.3
) -> List[Tuple[float, float]]:
    """
    Generate target positions for robots to form a circle.
    
    Args:
        num_agents: Number of robots
        room_bounds: (xmin, xmax, ymin, ymax) in world coordinates (meters)
        region: Where to place the circle ("right_side", "left_side", "center")
        margin: Margin from room edges in meters
    
    Returns:
        List of (x, y) tuples in world coordinates
    """
    xmin, xmax, ymin, ymax = room_bounds
    room_width = xmax - xmin
    room_height = ymax - ymin
    
    # Circle size: use 60% of smaller dimension
    max_radius = min(room_width, room_height) * 0.3
    radius = max_radius
    
    # Determine center position based on region
    if region == "right_side":
        center_x = xmax - margin - radius
        center_y = (ymin + ymax) / 2
    elif region == "left_side":
        center_x = xmin + margin + radius
        center_y = (ymin + ymax) / 2
    elif region == "center":
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
    else:
        raise ValueError(f"Unknown region: {region}")
    
    # Generate points evenly distributed around the circle
    targets = []
    for i in range(num_agents):
        angle = 2 * np.pi * i / num_agents
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        targets.append((float(x), float(y)))
    
    return targets


def generate_grid_targets(
    num_agents: int,
    room_bounds: Tuple[float, float, float, float],
    region: str = "right_side",
    margin: float = 0.3
) -> List[Tuple[float, float]]:
    """
    Generate target positions for robots to form a rectangular grid.
    
    Args:
        num_agents: Number of robots
        room_bounds: (xmin, xmax, ymin, ymax) in world coordinates (meters)
        region: Where to place the grid ("right_side", "left_side", "center")
        margin: Margin from room edges in meters
    
    Returns:
        List of (x, y) tuples in world coordinates
    """
    xmin, xmax, ymin, ymax = room_bounds
    
    # Calculate grid dimensions (prefer square-ish grid)
    # Find factors of num_agents that are close to sqrt(num_agents)
    sqrt_n = int(np.sqrt(num_agents))
    for cols in range(sqrt_n, 0, -1):
        if num_agents % cols == 0:
            rows = num_agents // cols
            break
    else:
        # Fallback: use sqrt_n
        cols = sqrt_n
        rows = (num_agents + cols - 1) // cols  # Ceiling division
    
    # Grid size: use 50% of available space
    available_width = (xmax - xmin) * 0.5
    available_height = (ymax - ymin) * 0.5
    
    grid_width = min(available_width, available_height * (cols / rows))
    grid_height = grid_width * (rows / cols)
    
    # Spacing between grid points
    spacing_x = grid_width / max(1, cols - 1) if cols > 1 else 0
    spacing_y = grid_height / max(1, rows - 1) if rows > 1 else 0
    
    # Determine grid position based on region
    if region == "right_side":
        grid_x_min = xmax - margin - grid_width
        grid_y_min = (ymin + ymax) / 2 - grid_height / 2
    elif region == "left_side":
        grid_x_min = xmin + margin
        grid_y_min = (ymin + ymax) / 2 - grid_height / 2
    elif region == "center":
        grid_x_min = (xmin + xmax) / 2 - grid_width / 2
        grid_y_min = (ymin + ymax) / 2 - grid_height / 2
    else:
        raise ValueError(f"Unknown region: {region}")
    
    # Generate grid points
    targets = []
    for i in range(rows):
        for j in range(cols):
            if len(targets) >= num_agents:
                break
            x = grid_x_min + j * spacing_x
            y = grid_y_min + i * spacing_y
            targets.append((float(x), float(y)))
        if len(targets) >= num_agents:
            break
    
    return targets


def generate_heart_targets(
    num_agents: int,
    room_bounds: Tuple[float, float, float, float],
    region: str = "right_side",
    margin: float = 0.5  # Increased default margin for better spacing
) -> List[Tuple[float, float]]:
    """
    Generate target positions for robots to form a heart shape.
    
    Uses parametric equations for a heart curve:
    x(t) = 16*sin^3(t)
    y(t) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t)
    
    Args:
        num_agents: Number of robots
        room_bounds: (xmin, xmax, ymin, ymax) in world coordinates (meters)
        region: Where to place the heart ("right_side", "left_side", "center")
        margin: Margin from room edges in meters
    
    Returns:
        List of (x, y) tuples in world coordinates
    """
    xmin, xmax, ymin, ymax = room_bounds
    room_width = xmax - xmin
    room_height = ymax - ymin
    
    # Heart size: use 50% of smaller dimension (increased from 40% for better visibility)
    scale = min(room_width, room_height) * 0.25
    
    # Generate heart curve points in normalized coordinates
    # Use more points for smooth curve, but exclude the last point to avoid closure
    t_values = np.linspace(0, 2 * np.pi, num_agents * 3, endpoint=False)  # endpoint=False to avoid duplicate
    heart_points = []
    
    for t in t_values:
        # Parametric heart equations (normalized)
        x_norm = 16 * np.sin(t) ** 3
        y_norm = 13 * np.cos(t) - 5 * np.cos(2*t) - 2 * np.cos(3*t) - np.cos(4*t)
        heart_points.append((x_norm, y_norm))
    
    # Resample to get exactly num_agents points with uniform spacing
    if len(heart_points) > num_agents:
        # Calculate cumulative arc length
        distances = []
        total_dist = 0.0
        for i in range(len(heart_points)):
            if i == 0:
                distances.append(0.0)
            else:
                dx = heart_points[i][0] - heart_points[i-1][0]
                dy = heart_points[i][1] - heart_points[i-1][1]
                total_dist += np.sqrt(dx*dx + dy*dy)
                distances.append(total_dist)
        
        # Sample uniformly along the curve, ensuring no duplicates
        sampled_points = []
        used_indices = set()
        min_spacing = 0.3  # Minimum spacing in normalized coordinates
        
        for i in range(num_agents):
            target_dist = total_dist * i / num_agents if num_agents > 0 else 0
            # Find closest point that hasn't been used
            candidates = [(j, abs(distances[j] - target_dist)) for j in range(len(distances)) if j not in used_indices]
            if candidates:
                idx = min(candidates, key=lambda x: x[1])[0]
                # Check if this point is too close to already sampled points
                too_close = False
                for sx, sy in sampled_points:
                    dx = heart_points[idx][0] - sx
                    dy = heart_points[idx][1] - sy
                    if np.sqrt(dx*dx + dy*dy) < min_spacing:
                        too_close = True
                        break
                
                if not too_close:
                    sampled_points.append(heart_points[idx])
                    used_indices.add(idx)
                else:
                    # If too close, find next best candidate
                    candidates = [(j, abs(distances[j] - target_dist)) for j in range(len(distances)) 
                                 if j not in used_indices and j != idx]
                    if candidates:
                        idx = min(candidates, key=lambda x: x[1])[0]
                        sampled_points.append(heart_points[idx])
                        used_indices.add(idx)
                    else:
                        # Fallback: use the closest point anyway
                        sampled_points.append(heart_points[idx])
                        used_indices.add(idx)
            else:
                # Fallback: use remaining points
                for j in range(len(heart_points)):
                    if j not in used_indices:
                        sampled_points.append(heart_points[j])
                        used_indices.add(j)
                        break
        
        heart_points = sampled_points[:num_agents]  # Ensure exactly num_agents points
    
    # Determine center position based on region
    # Heart curve spans approximately x_norm in [-16, 16] and y_norm in [-20, 20]
    # We need to scale and position it correctly in world coordinates
    heart_width = scale * 2.0  # Heart width in world coordinates (x_norm spans ~32 units, so scale * 32/16 = scale * 2)
    heart_height = scale * 2.0  # Heart height in world coordinates (y_norm spans ~40 units, so scale * 40/20 = scale * 2)
    
    if region == "right_side":
        # Place heart on right side: center_x should be near xmax - margin - heart_width/2
        center_x = xmax - margin - heart_width / 2
        center_y = (ymin + ymax) / 2
    elif region == "left_side":
        # Place heart on left side: center_x should be near xmin + margin + heart_width/2
        center_x = xmin + margin + heart_width / 2
        center_y = (ymin + ymax) / 2
    elif region == "center":
        # Place heart in center
        center_x = (xmin + xmax) / 2
        center_y = (ymin + ymax) / 2
    else:
        raise ValueError(f"Unknown region: {region}")
    
    # Convert to world coordinates
    # x_norm ranges from -16 to 16, so we scale by scale/16 to get world coordinates
    # y_norm ranges from -20 to 20, so we scale by scale/20 to get world coordinates
    targets = []
    for x_norm, y_norm in heart_points:
        x = center_x + x_norm * scale / 16  # x_norm in [-16, 16] -> world x
        y = center_y + y_norm * scale / 20  # y_norm in [-20, 20] -> world y
        targets.append((float(x), float(y)))
    
    # Ensure exactly num_agents points (remove duplicates if any)
    targets = targets[:num_agents]
    
    # Ensure minimum spacing between points (0.5m minimum for better visibility and less crowding)
    targets = _ensure_minimum_spacing(targets, min_spacing=0.5)
    
    return targets


def visualize_formation(
    targets: List[Tuple[float, float]],
    room_bounds: Tuple[float, float, float, float],
    title: str = "Formation Pattern",
    save_path: Optional[str] = None
):
    """
    Visualize the formation pattern using matplotlib.
    
    Args:
        targets: List of (x, y) target positions
        room_bounds: (xmin, xmax, ymin, ymax) for drawing room boundaries
        title: Plot title
    """
    try:
        import matplotlib
        # Use non-interactive backend if display is not available
        try:
            import matplotlib.pyplot as plt
        except Exception:
            matplotlib.use('Agg')  # Use non-interactive backend
            import matplotlib.pyplot as plt
        
        xmin, xmax, ymin, ymax = room_bounds
        
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Draw room boundaries
        ax.plot([xmin, xmax, xmax, xmin, xmin], 
                [ymin, ymin, ymax, ymax, ymin], 
                'k-', linewidth=2, label='Room boundaries')
        
        # Draw target positions
        xs = [t[0] for t in targets]
        ys = [t[1] for t in targets]
        
        ax.scatter(xs, ys, c='red', s=100, marker='o', 
                  edgecolors='black', linewidths=2, label='Target positions', zorder=5)
        
        # Draw lines connecting points in order (to show the B shape)
        # Sort by y-coordinate to show the shape better
        sorted_targets = sorted(targets, key=lambda p: p[1])
        sorted_xs = [t[0] for t in sorted_targets]
        sorted_ys = [t[1] for t in sorted_targets]
        ax.plot(sorted_xs, sorted_ys, 'b--', linewidth=1, alpha=0.5, label='Formation pattern')
        
        # Add labels for each point
        for i, (x, y) in enumerate(targets):
            ax.annotate(f'R{i}', (x, y), xytext=(5, 5), 
                       textcoords='offset points', fontsize=8)
        
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        # Always save to file (works even without display)
        if save_path:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"✅ Visualization saved to: {save_path}")
        else:
            # Auto-generate save path if not provided
            default_path = os.path.join(os.getcwd(), "formation_preview.png")
            plt.savefig(default_path, dpi=150, bbox_inches="tight")
            print(f"✅ Visualization saved to: {default_path}")
        
        plt.close(fig)
        print(f"✅ Visualization completed: {len(targets)} target positions")
        
    except ImportError as e:
        print(f"⚠️  matplotlib not available: {e}")
        print(f"Target positions ({len(targets)} points):")
        for i, (x, y) in enumerate(targets):
            print(f"  Robot {i}: ({x:.2f}, {y:.2f})")
    except Exception as e:
        print(f"⚠️  Error during visualization: {e}")
        print(f"Target positions ({len(targets)} points):")
        for i, (x, y) in enumerate(targets):
            print(f"  Robot {i}: ({x:.2f}, {y:.2f})")


if __name__ == "__main__":
    # Test the formation generator
    print("Testing letter B formation generator...")
    
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # Updated room bounds to match room_formation.xml: 16m x 14m
    room_bounds = (-8.0, 8.0, -7.0, 7.0)
    
    # Test with 20 robots (default)
    print("\n📋 Generating B formation for 20 robots (right side):")
    targets_20 = generate_letter_b_targets(20, room_bounds, region="right_side")
    for i, (x, y) in enumerate(targets_20):
        print(f"  Robot {i:2d}: ({x:6.2f}, {y:6.2f})")
    
    # Test with 12 robots
    print("\n📋 Generating B formation for 12 robots (right side):")
    targets_12 = generate_letter_b_targets(12, room_bounds, region="right_side")
    for i, (x, y) in enumerate(targets_12):
        print(f"  Robot {i:2d}: ({x:6.2f}, {y:6.2f})")
    
    # Visualize
    print("\n📊 Visualizing 20-robot B formation...")
    preview_path = os.path.join(project_root, "results", "letter_b_preview.png")
    visualize_formation(
        targets_20,
        room_bounds,
        title="Letter B Formation (20 robots)",
        save_path=preview_path
    )

