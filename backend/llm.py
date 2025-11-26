#!/usr/bin/env python3
"""
LLM integration for parsing natural language formation commands.
"""

import os
import sys
from typing import Dict, Optional

# Add project root to path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from nav_world.llm_formation_controller import LLMFormationController
from nav_world.formation_task import FormationTask


def parse_user_command(text: str) -> Dict[str, any]:
    """
    Parse natural language command into formation parameters.
    
    Args:
        text: Natural language command (e.g., "form a letter B on the right side")
        
    Returns:
        Dictionary with keys:
        - formation_type: str ("B", "circle", "grid", "heart", "line", "random")
        - region: str ("left_side", "right_side", "center")
        - num_robots: int
        - shape: str (same as formation_type, for compatibility)
    """
    controller = LLMFormationController(model_name="gpt-4o-mini")
    task = controller.parse_command(text)
    
    # Map shape to formation_type (handle "line" and "random" which aren't in FormationTask yet)
    formation_type = task.shape
    
    # For now, "line" and "random" are not in FormationTask, so we'll use rule-based parsing
    text_lower = text.lower()
    if "line" in text_lower or "straight" in text_lower:
        formation_type = "line"
    elif "random" in text_lower or "scatter" in text_lower:
        formation_type = "random"
    
    return {
        "formation_type": formation_type,
        "shape": formation_type,  # Alias for compatibility
        "region": task.region,
        "num_robots": task.num_agents,
        "num_agents": task.num_agents,  # Alias
    }


def parse_user_command_offline(text: str) -> Dict[str, any]:
    """
    Parse command using rule-based parser (no LLM call).
    
    Args:
        text: Natural language command
        
    Returns:
        Dictionary with formation parameters
    """
    from nav_world.llm_formation_controller import rule_based_parse
    
    task = rule_based_parse(text)
    
    # Map shape to formation_type
    formation_type = task.shape
    text_lower = text.lower()
    if "line" in text_lower or "straight" in text_lower:
        formation_type = "line"
    elif "random" in text_lower or "scatter" in text_lower:
        formation_type = "random"
    
    return {
        "formation_type": formation_type,
        "shape": formation_type,
        "region": task.region,
        "num_robots": task.num_agents,
        "num_agents": task.num_agents,
    }


