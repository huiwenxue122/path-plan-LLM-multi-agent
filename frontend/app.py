#!/usr/bin/env python3
"""
Streamlit UI for multi-robot formation control.

This is the main entry point for the interactive formation demo.
Run with: streamlit run frontend/app.py
"""

import os
import sys
import time
import numpy as np
from typing import Optional, List, Tuple
import streamlit as st
from PIL import Image

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from backend.controller import FormationController
from backend.llm import parse_user_command, parse_user_command_offline
from backend.formations import generate_formation

# Page configuration
st.set_page_config(
    page_title="Multi-Robot Formation Control",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better UI
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        font-weight: bold;
        color: #1f77b4;
        text-align: center;
        margin-bottom: 2rem;
    }
    .info-box {
        background-color: #f0f2f6;
        padding: 1rem;
        border-radius: 0.5rem;
        margin: 1rem 0;
    }
</style>
""", unsafe_allow_html=True)


@st.cache_resource
def get_controller(num_robots: int = 20):
    """Get or create the formation controller (cached)."""
    xml_path = os.path.join(project_root, "nav_world", "room_formation.xml")
    return FormationController(
        xml_path=xml_path,
        num_robots=num_robots,
        render_width=800,
        render_height=600
    )


def main():
    """Main Streamlit app."""
    st.markdown('<div class="main-header">🤖 Multi-Robot Formation Control</div>', unsafe_allow_html=True)
    
    # Sidebar for controls
    with st.sidebar:
        st.header("⚙️ Configuration")
        
        # Number of robots
        num_robots = st.slider(
            "Number of Robots",
            min_value=5,
            max_value=30,
            value=20,
            step=1
        )
        
        # Formation type
        formation_type = st.selectbox(
            "Formation Type",
            options=["B", "circle", "grid", "heart", "line", "random"],
            index=0,
            help="Select the formation shape"
        )
        
        # Region
        region = st.selectbox(
            "Region",
            options=["right_side", "left_side", "center"],
            index=0,
            help="Where to place the formation"
        )
        
        st.divider()
        
        # Manual controls
        st.header("🎮 Manual Controls")
        
        if st.button("🔄 Reset Simulation", use_container_width=True):
            if 'controller' in st.session_state:
                st.session_state.controller.reset()
                st.session_state.controller.stop()
                st.session_state.step_count = 0
                st.session_state.recorded_frames = []
                st.rerun()
        
        # Video export option
        st.divider()
        st.header("🎥 Video Export")
        record_video = st.checkbox("Record video while simulating", value=False, help="Record frames for video export")
        if 'record_video' not in st.session_state:
            st.session_state.record_video = False
        st.session_state.record_video = record_video
        
        if st.button("⏹️ Stop Simulation", use_container_width=True):
            if 'controller' in st.session_state:
                st.session_state.controller.stop()
                st.rerun()
        
        st.divider()
        
        # Info
        st.header("ℹ️ Info")
        if 'controller' in st.session_state:
            st.metric("Step Count", st.session_state.get('step_count', 0))
            if 'all_reached' in st.session_state:
                st.metric("All Reached", "✅" if st.session_state.all_reached else "⏳")
            if st.session_state.get('record_video', False):
                frame_count = st.session_state.controller.get_recorded_frames_count()
                st.metric("Recorded Frames", frame_count)
                
                # Export video button
                if frame_count > 0:
                    if st.button("💾 Export Video", use_container_width=True):
                        import tempfile
                        import os
                        with tempfile.NamedTemporaryFile(delete=False, suffix='.mp4') as tmp_file:
                            tmp_path = tmp_file.name
                        
                        if st.session_state.controller.export_video(tmp_path, fps=20):
                            with open(tmp_path, 'rb') as video_file:
                                st.download_button(
                                    label="📥 Download Video",
                                    data=video_file.read(),
                                    file_name="robot_formation.mp4",
                                    mime="video/mp4",
                                    use_container_width=True
                                )
                            os.unlink(tmp_path)
                        else:
                            st.error("Failed to export video")
    
    # Main content area
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.header("📝 Natural Language Command")
        
        # Text input for natural language
        user_command = st.text_input(
            "Enter your command:",
            placeholder="e.g., 'form a letter B on the right side'",
            key="user_command"
        )
        
        # LLM parsing options
        use_llm = st.checkbox("Use LLM (OpenAI API)", value=True, help="Uncheck to use rule-based parser")
        
        col_parse, col_generate = st.columns(2)
        
        with col_parse:
            parse_button = st.button("🔍 Parse Command", use_container_width=True)
        
        with col_generate:
            generate_button = st.button("🎯 Generate Targets", use_container_width=True)
        
        # Parse command
        if parse_button and user_command:
            with st.spinner("Parsing command with LLM..."):
                try:
                    if use_llm:
                        parsed = parse_user_command(user_command)
                    else:
                        parsed = parse_user_command_offline(user_command)
                    
                    st.session_state.parsed_command = parsed
                    st.success(f"✅ Parsed: {parsed['formation_type']} formation, {parsed['region']}, {parsed['num_robots']} robots")
                    
                    # Update UI with parsed values
                    st.info(f"""
                    **Formation Type:** {parsed['formation_type']}  
                    **Region:** {parsed['region']}  
                    **Number of Robots:** {parsed['num_robots']}
                    """)
                except Exception as e:
                    st.error(f"❌ Error parsing command: {e}")
        
        # Generate targets
        if generate_button:
            # Use parsed command if available, otherwise use UI selections
            if 'parsed_command' in st.session_state:
                formation = st.session_state.parsed_command['formation_type']
                region_parsed = st.session_state.parsed_command['region']
                num_robots_parsed = st.session_state.parsed_command['num_robots']
            else:
                formation = formation_type
                region_parsed = region
                num_robots_parsed = num_robots
            
            with st.spinner(f"Generating {formation} formation targets..."):
                try:
                    # Get or create controller
                    # Check if controller exists and if num_robots matches
                    need_new_controller = False
                    if 'controller' not in st.session_state:
                        need_new_controller = True
                    elif st.session_state.controller is None:
                        need_new_controller = True
                    elif st.session_state.controller.num_robots != num_robots_parsed:
                        need_new_controller = True
                    
                    if need_new_controller:
                        st.session_state.controller = get_controller(num_robots=num_robots_parsed)
                    
                    controller = st.session_state.controller
                    if controller is None:
                        raise ValueError("Failed to create controller")
                    
                    room_bounds = controller.get_room_bounds()
                    
                    # Generate targets
                    targets = generate_formation(
                        formation_type=formation,
                        room_bounds=room_bounds,
                        num_agents=num_robots_parsed,
                        region=region_parsed
                    )
                    
                    # Set targets (this resets the environment)
                    # CRITICAL: Do NOT call get_frame() or any rendering here
                    # This prevents segmentation faults in Streamlit
                    controller.set_targets(targets)
                    st.session_state.targets = targets
                    st.session_state.step_count = 0
                    st.session_state.simulation_running = False  # Reset simulation state
                    
                    st.success(f"✅ Generated {len(targets)} target positions!")
                    st.info("💡 Click 'Start Simulation' or 'Step Once' to see the simulation")
                    
                except Exception as e:
                    st.error(f"❌ Error generating targets: {e}")
                    import traceback
                    st.code(traceback.format_exc())
        
        st.divider()
        
        # Simulation controls (only one section - use empty container to prevent duplicates on rerun)
        if 'sim_control_placeholder' not in st.session_state:
            st.session_state.sim_control_placeholder = st.empty()
        
        with st.session_state.sim_control_placeholder.container():
            st.header("🎬 Simulation Control")
            
            col_start, col_step = st.columns(2)
            
            with col_start:
                start_button = st.button("▶️ Start Simulation", use_container_width=True, type="primary", key="start_sim_btn")
            
            with col_step:
                step_button = st.button("⏭️ Step Once", use_container_width=True, key="step_once_btn")
        
        # Start simulation
        if start_button:
            if 'controller' not in st.session_state or st.session_state.controller is None:
                st.warning("⚠️ Please generate targets first!")
            elif 'targets' not in st.session_state or st.session_state.targets is None:
                st.warning("⚠️ Please generate targets first!")
            else:
                try:
                    # Start with video recording if enabled
                    record = st.session_state.get('record_video', False)
                    st.session_state.controller.start(record=record)
                    st.session_state.simulation_running = True
                    # Step once to update the frame (but don't render yet)
                    # Rendering will happen in the display section after rerun
                    done, info = st.session_state.controller.step()
                    st.session_state.step_count = info.get('step', 0)
                    st.session_state.all_reached = info.get('all_reached', False)
                    if done:
                        st.session_state.controller.stop()
                        st.session_state.simulation_running = False
                        # If recording, show export option
                        if record and st.session_state.controller.get_recorded_frames_count() > 0:
                            st.success(f"✅ Simulation complete! Recorded {st.session_state.controller.get_recorded_frames_count()} frames")
                    st.rerun()
                except Exception as e:
                    st.error(f"❌ Error starting simulation: {e}")
                    import traceback
                    st.code(traceback.format_exc())
                    st.session_state.simulation_running = False
                    if 'controller' in st.session_state:
                        try:
                            st.session_state.controller.stop()
                        except:
                            pass
        
        # Step once
        if step_button:
            if 'controller' not in st.session_state or st.session_state.controller is None:
                st.warning("⚠️ Please generate targets first!")
            elif 'targets' not in st.session_state or st.session_state.targets is None:
                st.warning("⚠️ Please generate targets first!")
            else:
                try:
                    controller = st.session_state.controller
                    controller.start()
                    done, info = controller.step()
                    st.session_state.step_count = info.get('step', 0)
                    st.session_state.all_reached = info.get('all_reached', False)
                    if done:
                        controller.stop()
                        st.session_state.simulation_running = False
                    st.rerun()
                except Exception as e:
                    st.error(f"❌ Error stepping simulation: {e}")
                    import traceback
                    st.code(traceback.format_exc())
                    st.session_state.simulation_running = False
                    if 'controller' in st.session_state:
                        try:
                            st.session_state.controller.stop()
                        except:
                            pass
    
    with col2:
        st.header("📊 Simulation View")
        
        # Display current frame
        # IMPORTANT: Only render if simulation is running or user explicitly requests it
        # This prevents segmentation faults from MuJoCo renderer in Streamlit
        if 'controller' in st.session_state and st.session_state.controller is not None:
            controller = st.session_state.controller
            
            # Only render if:
            # 1. Targets are set (environment initialized)
            # 2. Simulation is running OR user clicked step button
            should_render = (
                'targets' in st.session_state and 
                st.session_state.targets is not None and
                (st.session_state.get('simulation_running', False) or 
                 st.session_state.get('step_count', 0) > 0)
            )
            
            if should_render:
                # Get and display frame
                # IMPORTANT: Wrap in try-except to catch segfaults and other rendering errors
                try:
                    # Use a placeholder first to avoid blocking
                    with st.spinner("Rendering..."):
                        frame = controller.get_frame()
                    
                    # Ensure frame is valid
                    if frame is None:
                        st.warning("⚠️ Frame rendering returned None")
                    elif len(frame.shape) != 3 or frame.shape[2] != 3:
                        st.warning(f"⚠️ Invalid frame shape: {frame.shape}")
                    else:
                        # Ensure frame is uint8
                        if frame.dtype != np.uint8:
                            frame = np.clip(frame, 0, 255).astype(np.uint8)
                        
                        # Convert to PIL Image for Streamlit
                        img = Image.fromarray(frame)
                        st.image(img, caption="Current Simulation State", use_container_width=True)
                    
                    # Robot positions info
                    with st.expander("📍 Robot Positions"):
                        try:
                            positions = controller.get_robot_positions()
                            distances = controller.get_robot_distances()
                            
                            for i, (pos, dist) in enumerate(zip(positions[:10], distances[:10])):  # Show first 10
                                st.text(f"Robot {i}: ({pos[0]:.2f}, {pos[1]:.2f}), dist={dist:.3f}m")
                            if len(positions) > 10:
                                st.text(f"... and {len(positions) - 10} more robots")
                        except Exception as e:
                            st.warning(f"Could not get robot positions: {e}")
                    
                except SystemExit:
                    # Streamlit might exit on segfault, catch it
                    st.error("❌ Rendering caused a system error. Please restart the app.")
                    st.session_state.simulation_running = False
                except Exception as e:
                    st.error(f"❌ Error rendering frame: {e}")
                    import traceback
                    st.code(traceback.format_exc())
                    # Stop simulation on rendering error
                    st.session_state.simulation_running = False
                    if 'controller' in st.session_state and st.session_state.controller is not None:
                        try:
                            st.session_state.controller.stop()
                        except:
                            pass
            elif 'targets' in st.session_state and st.session_state.targets is not None:
                st.info("👈 Click 'Start Simulation' or 'Step Once' to see the simulation")
            else:
                st.info("👈 Generate targets to start simulation")
        else:
            st.info("👈 Generate targets to start simulation")
    
    # Auto-run simulation if running
    # Only auto-run if targets are set and controller is ready
    if (st.session_state.get('simulation_running', False) and 
        'controller' in st.session_state and 
        st.session_state.controller is not None and
        'targets' in st.session_state and 
        st.session_state.targets is not None):
        controller = st.session_state.controller
        if controller.is_running:
            # Step the simulation
            done, info = controller.step()
            st.session_state.step_count = info.get('step', 0)
            st.session_state.all_reached = info.get('all_reached', False)
            
            if done:
                controller.stop()
                st.session_state.simulation_running = False
                st.success("✅ Simulation completed! All robots reached their targets.")
            else:
                # Continue running - refresh after a short delay
                time.sleep(0.05)  # 50ms delay for smooth animation (~20 FPS)
                st.rerun()


if __name__ == "__main__":
    # Initialize session state
    if 'controller' not in st.session_state:
        st.session_state.controller = None
    if 'step_count' not in st.session_state:
        st.session_state.step_count = 0
    if 'simulation_running' not in st.session_state:
        st.session_state.simulation_running = False
    
    main()

