"""
LLM Interface for Natural Language Control of Multi-Agent Navigation

This module provides a clean interface to parse natural language instructions
into structured navigation plans using LLM (OpenAI GPT models).
"""

from .llm_controller import (
    AgentSpec,
    TaskPlan,
    llm_parse_instruction,
    llm_parse_instruction_offline,
    _fallback_plan
)

__all__ = [
    'AgentSpec',
    'TaskPlan',
    'llm_parse_instruction',
    'llm_parse_instruction_offline',
    '_fallback_plan'
]

