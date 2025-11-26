#!/usr/bin/env python3
"""
Backend controller for MuJoCo formation simulation.

This module provides a high-level interface for controlling the formation
environment and rendering frames for the Streamlit UI.
"""

import os
import sys
import numpy as np
from typing import List, Tuple, Optional
import mujoco

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world.nav_env_formation_orca import NavEnvFormationORCA


class FormationController:
    """
    High-level controller for formation simulation.
    
    Provides methods for:
    - Initializing the environment
    - Setting formation targets
    - Stepping the simulation
    - Rendering frames
    """
    
    def __init__(
        self,
        xml_path: Optional[str] = None,
        num_robots: int = 20,
        render_width: int = 800,
        render_height: int = 600
    ):
        """
        Initialize the formation controller.
        
        Args:
            xml_path: Path to MuJoCo XML file (default: nav_world/room_formation.xml)
            num_robots: Number of robots
            render_width: Frame width for rendering
            render_height: Frame height for rendering
        """
        if xml_path is None:
            xml_path = os.path.join(project_root, "nav_world", "room_formation.xml")
        
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"XML file not found: {xml_path}")
        
        self.xml_path = xml_path
        self.num_robots = num_robots
        self.render_width = render_width
        self.render_height = render_height
        
        # Initialize environment
        self.env = NavEnvFormationORCA(
            xml_path=xml_path,
            num_robots=num_robots,
            render_w=render_width,
            render_h=render_height
        )
        
        # Verify ORCA is available (critical for collision avoidance)
        if self.env.orca is None:
            print("⚠️  WARNING: ORCA not available! Robots may collide.")
            print("   Please ensure rvo2 is installed in Python 3.11+ environment")
        else:
            print(f"✅ ORCA collision avoidance enabled (radius={self.env.orca.robot_radius}m)")
        
        self.is_running = False
        self.step_count = 0
        self.max_steps = 10000  # Maximum simulation steps
        self.record_frames = False  # Whether to record frames for video
        self.recorded_frames = []  # List of recorded frames
        
    def reset(self, target_positions: Optional[List[Tuple[float, float]]] = None):
        """
        Reset the simulation.
        
        Args:
            target_positions: Optional list of (x, y) target positions.
                             If None, uses default start positions.
        """
        if target_positions is not None:
            self.env.reset(target_positions=target_positions, use_B=False)
        else:
            self.env.reset(use_B=False)
        
        self.step_count = 0
        self.is_running = False
        self.record_frames = False
        self.recorded_frames = []
    
    def set_targets(self, target_positions: List[Tuple[float, float]]):
        """
        Set formation target positions for all robots.
        
        Args:
            target_positions: List of (x, y) target positions.
                            Must have at least num_robots positions.
        """
        if len(target_positions) < self.num_robots:
            raise ValueError(
                f"Not enough target positions: {len(target_positions)} < {self.num_robots}"
            )
        
        # Reset environment with new targets
        self.env.reset(target_positions=target_positions, use_B=False)
        self.step_count = 0
        self.record_frames = False
        self.recorded_frames = []
        
        # Verify ORCA is initialized
        if self.env.orca is not None and self.env.orca.sim is not None:
            print(f"✅ ORCA ready: {len(self.env.orca.agent_ids)} agents, radius={self.env.orca.robot_radius}m")
    
    def step(self, dt: float = 0.02) -> Tuple[bool, dict]:
        """
        Step the simulation forward.
        
        Args:
            dt: Time step in seconds
            
        Returns:
            Tuple of (done, info) where:
            - done: True if all robots reached targets or max steps reached
            - info: Dictionary with simulation info
        """
        if not self.is_running:
            return True, {"message": "Simulation not started"}
        
        obs, done = self.env.step(dt=dt)
        self.step_count += 1
        
        # Record frame if recording
        if self.record_frames:
            try:
                frame = self.get_frame()
                self.recorded_frames.append(frame.copy())
            except Exception as e:
                print(f"Warning: Could not record frame: {e}")
        
        # Check if all robots reached targets
        all_reached = self._check_all_reached()
        
        # Check max steps
        if self.step_count >= self.max_steps:
            done = True
        
        info = {
            "step": self.step_count,
            "all_reached": all_reached,
            "done": done,
            "distances": self.get_robot_distances(),
            "positions": self.get_robot_positions(),
        }
        
        return done, info
    
    def _check_all_reached(self, threshold: float = 0.15) -> bool:
        """Check if all robots have reached their targets."""
        for name in self.env.agent_names:
            if name not in self.env.goal_xy:
                return False
            pos = np.array(self.env._get_body_xy(name))
            goal = np.array(self.env.goal_xy[name])
            dist = np.linalg.norm(pos - goal)
            if dist > threshold:
                return False
        return True
    
    def get_frame(self) -> np.ndarray:
        """
        Render and return current frame as numpy array.
        
        IMPORTANT: MuJoCo rendering causes segfaults in Streamlit.
        This method uses a simple matplotlib-based visualization instead.
        
        Returns:
            RGB image array of shape (height, width, 3)
        """
        try:
            # Get robot positions and goals
            positions = self.get_robot_positions()
            room_bounds = self.get_room_bounds()
            xmin, xmax, ymin, ymax = room_bounds
            
            # Get goals
            goals = []
            for name in self.env.agent_names:
                if name in self.env.goal_xy:
                    goals.append(self.env.goal_xy[name])
                else:
                    goals.append((0.0, 0.0))
            
            # Use matplotlib to create a simple visualization
            # This avoids MuJoCo rendering which causes segfaults in Streamlit
            try:
                import matplotlib
                matplotlib.use('Agg')  # Non-interactive backend
                import matplotlib.pyplot as plt
                from matplotlib.patches import Circle, Rectangle
                
                fig, ax = plt.subplots(figsize=(self.render_width/100, self.render_height/100), dpi=100)
                ax.set_xlim(xmin - 0.5, xmax + 0.5)
                ax.set_ylim(ymin - 0.5, ymax + 0.5)
                ax.set_aspect('equal')
                ax.grid(True, alpha=0.3)
                ax.set_facecolor('white')
                
                # Draw room boundaries
                room_rect = Rectangle(
                    (xmin, ymin), xmax - xmin, ymax - ymin,
                    linewidth=2, edgecolor='black', facecolor='lightgray', alpha=0.3
                )
                ax.add_patch(room_rect)
                
                # Draw goals (green circles)
                for goal in goals:
                    circle = Circle(goal, 0.2, color='green', alpha=0.5, label='Goal')
                    ax.add_patch(circle)
                
                # Draw robots (blue circles with numbers)
                for i, (pos, goal) in enumerate(zip(positions, goals)):
                    # Robot position
                    circle = Circle(pos, 0.15, color='blue', alpha=0.7)
                    ax.add_patch(circle)
                    ax.text(pos[0], pos[1], str(i), ha='center', va='center', 
                           fontsize=8, color='white', weight='bold')
                    
                    # Line from robot to goal
                    ax.plot([pos[0], goal[0]], [pos[1], goal[1]], 
                           'r--', alpha=0.3, linewidth=1)
                
                ax.set_xlabel('X (m)')
                ax.set_ylabel('Y (m)')
                ax.set_title('Multi-Robot Formation (Top View)')
                
                # Convert to numpy array
                fig.canvas.draw()
                # Use buffer_rgba() for newer matplotlib versions, fallback to tostring_rgb()
                try:
                    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
                    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (4,))
                    buf = buf[:, :, :3]  # Remove alpha channel
                except AttributeError:
                    # Fallback for older matplotlib
                    buf = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
                    buf = buf.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                plt.close(fig)
                
                # Resize to match expected dimensions
                if HAS_CV2:
                    buf = cv2.resize(buf, (self.render_width, self.render_height))
                else:
                    # Simple resize using PIL
                    from PIL import Image
                    img = Image.fromarray(buf)
                    img = img.resize((self.render_width, self.render_height))
                    buf = np.array(img)
                
                return buf
                
            except ImportError:
                # If matplotlib not available, return a simple colored frame
                frame = np.zeros((self.render_height, self.render_width, 3), dtype=np.uint8)
                frame[:, :] = [240, 240, 240]  # Light gray background
                return frame
                
        except Exception as e:
            # Fallback: return a simple visualization with error indicator
            print(f"Error in get_frame(): {e}")
            frame = np.zeros((self.render_height, self.render_width, 3), dtype=np.uint8)
            frame[:, :] = [240, 240, 240]  # Light gray
            # Add a colored border to indicate fallback mode
            h, w = self.render_height, self.render_width
            frame[0:10, :] = [200, 0, 0]  # Red top border
            frame[h-10:h, :] = [200, 0, 0]  # Red bottom border
            return frame
    
    def start(self, record: bool = False):
        """Start the simulation.
        
        Args:
            record: If True, record frames for video export
        """
        self.is_running = True
        self.record_frames = record
        if record:
            self.recorded_frames = []
    
    def stop(self):
        """Stop the simulation."""
        self.is_running = False
        self.record_frames = False
    
    def export_video(self, output_path: str, fps: int = 20) -> bool:
        """
        Export recorded frames as a video file.
        
        Args:
            output_path: Path to save the video (e.g., 'output.mp4')
            fps: Frames per second for the video
            
        Returns:
            True if successful, False otherwise
        """
        if not self.recorded_frames:
            return False
        
        try:
            if HAS_CV2:
                # Use OpenCV to create video
                height, width = self.recorded_frames[0].shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
                
                for frame in self.recorded_frames:
                    # Convert RGB to BGR for OpenCV
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    out.write(frame_bgr)
                
                out.release()
                return True
            else:
                # Fallback: use imageio if available
                try:
                    import imageio
                    imageio.mimwrite(output_path, self.recorded_frames, fps=fps)
                    return True
                except ImportError:
                    print("Error: Need cv2 or imageio to export video")
                    return False
        except Exception as e:
            print(f"Error exporting video: {e}")
            return False
    
    def get_recorded_frames_count(self) -> int:
        """Get the number of recorded frames."""
        return len(self.recorded_frames) if self.recorded_frames else 0
    
    def get_room_bounds(self) -> Tuple[float, float, float, float]:
        """Get room boundaries."""
        return self.env.get_room_bounds()
    
    def get_robot_positions(self) -> List[Tuple[float, float]]:
        """Get current positions of all robots."""
        positions = []
        for name in self.env.agent_names:
            pos = self.env._get_body_xy(name)
            positions.append((float(pos[0]), float(pos[1])))
        return positions
    
    def get_robot_distances(self) -> List[float]:
        """Get distances from each robot to its target."""
        distances = []
        for name in self.env.agent_names:
            if name not in self.env.goal_xy:
                distances.append(float('inf'))
                continue
            pos = np.array(self.env._get_body_xy(name))
            goal = np.array(self.env.goal_xy[name])
            dist = np.linalg.norm(pos - goal)
            distances.append(float(dist))
        return distances

