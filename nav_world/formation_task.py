#!/usr/bin/env python3
"""
Formation Task Data Structure

Defines the unified Pydantic model for team-level formation tasks parsed from natural language.
This is the single source of truth for formation task schemas used by all LLM parsers.
"""

from typing import Literal, Optional
from pydantic import BaseModel, Field


class FormationTask(BaseModel):
    """
    Unified team-level formation task specification.
    
    This is the canonical schema used by all LLM parsers for formation tasks.
    All LLM outputs for formation commands must validate against this model.
    
    Attributes:
        shape: Shape to form ("B", "circle", "grid", or "heart")
        region: Where to place the formation ("left_side", "right_side", or "center")
        num_agents: Number of robots to use (required field)
    
    Example:
        >>> task = FormationTask(shape="B", region="left_side", num_agents=20)
        >>> task.region
        'left_side'
        >>> task = FormationTask(shape="circle", region="center", num_agents=12)
        >>> task.shape
        'circle'
    """
    shape: Literal["B", "circle", "grid", "heart"] = Field(
        default="B",
        description="Formation shape: 'B' (letter B), 'circle' (circular formation), 'grid' (rectangular grid), or 'heart' (heart shape)"
    )
    region: Literal["left_side", "right_side", "center"] = Field(
        default="right_side",
        description="Where to place the formation in the room"
    )
    num_agents: int = Field(
        ...,
        description="Number of robots/agents to use for the formation",
        gt=0
    )
    
    class Config:
        """Pydantic configuration."""
        # Allow extra fields to be ignored (for backward compatibility)
        extra = "ignore"
    
    def __repr__(self) -> str:
        """String representation for debugging."""
        return (
            f"FormationTask(shape='{self.shape}', "
            f"region='{self.region}', num_agents={self.num_agents})"
        )


# Default task for fallback (uses default num_agents=20)
DEFAULT_TASK = FormationTask(num_agents=20)

