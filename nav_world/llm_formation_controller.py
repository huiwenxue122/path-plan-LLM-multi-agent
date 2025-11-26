#!/usr/bin/env python3
"""
LLM Formation Controller

Parses natural language commands into FormationTask objects using OpenAI API
or a simple rule-based fallback parser.
"""

import os
import json
import logging
from typing import Optional

from .formation_task import FormationTask, DEFAULT_TASK

# Try to load OpenAI API key (reuse existing pattern from llm_interface)
OPENAI_API_KEY = None

# Method 1: Try loading from .env file
if os.path.exists(".env"):
    try:
        from dotenv import load_dotenv
        load_dotenv()
        OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
    except ImportError:
        pass

# Method 2: Try loading from openai_key.json (legacy support)
if not OPENAI_API_KEY and os.path.exists("openai_key.json"):
    try:
        OPENAI_API_KEY = str(json.load(open("openai_key.json")))
    except Exception:
        pass

# Method 3: Try environment variable
if not OPENAI_API_KEY:
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def rule_based_parse(text: str) -> FormationTask:
    """
    Simple rule-based parser as fallback when LLM is unavailable.
    
    Args:
        text: Natural language instruction
        
    Returns:
        FormationTask object with extracted parameters (Pydantic validated)
    """
    t = text.lower()
    
    # Extract shape (supports "B", "circle", "grid", "heart")
    shape: Literal["B", "circle", "grid", "heart"] = "B"
    if "b" in t or "letter b" in t:
        shape = "B"
    elif "circle" in t or "circular" in t:
        shape = "circle"
    elif "grid" in t or "rectangular" in t or "matrix" in t:
        shape = "grid"
    elif "heart" in t:
        shape = "heart"
    
    # Extract region
    region: Literal["left_side", "right_side", "center"] = "right_side"
    if "left" in t:
        region = "left_side"
    elif "center" in t or "middle" in t or "centre" in t:
        region = "center"
    elif "right" in t:
        region = "right_side"
    
    # Extract number of robots (simple pattern matching)
    num_agents = 20  # Default
    words = t.split()
    for i, word in enumerate(words):
        if word in ["robot", "robots", "agent", "agents"] and i + 1 < len(words):
            try:
                num_agents = int(words[i + 1])
                if num_agents <= 0:
                    num_agents = 20
                break
            except ValueError:
                pass
    
    # Return Pydantic-validated FormationTask
    return FormationTask(shape=shape, region=region, num_agents=num_agents)


class LLMFormationController:
    """
    LLM-based formation task parser.
    
    Uses OpenAI API to parse natural language into FormationTask objects.
    Falls back to rule-based parsing if API is unavailable.
    """
    
    def __init__(self, model_name: str = "gpt-4o-mini"):
        """
        Initialize LLM controller.
        
        Args:
            model_name: OpenAI model to use (default: "gpt-4o-mini")
        """
        self.model_name = model_name
        self.api_key = OPENAI_API_KEY
        
        if not self.api_key:
            logger.warning(
                "⚠️  OPENAI_API_KEY not found. "
                "LLMFormationController will use rule-based fallback parser."
            )
    
    def parse_command(self, text: str) -> FormationTask:
        """
        Parse natural language command into FormationTask.
        
        First attempts to use LLM API. If that fails, falls back to
        rule-based parsing.
        
        Args:
            text: Natural language instruction
            
        Returns:
            FormationTask object
        """
        # Try LLM first if API key is available
        if self.api_key:
            try:
                return self._parse_with_llm(text)
            except Exception as e:
                logger.warning(
                    f"⚠️  LLM parsing failed: {e}. "
                    "Falling back to rule-based parser."
                )
        
        # Fallback to rule-based parser
        logger.info("Using rule-based parser (LLM unavailable or failed)")
        return rule_based_parse(text)
    
    def _parse_with_llm(self, text: str) -> FormationTask:
        """
        Parse command using OpenAI API.
        
        Args:
            text: Natural language instruction
            
        Returns:
            FormationTask object
            
        Raises:
            Exception: If API call or parsing fails
        """
        try:
            import openai
            client = openai.OpenAI(api_key=self.api_key)
        except ImportError:
            raise ImportError("openai package not installed")
        
        # Build system prompt
        # REFACTORED: Updated to match new Pydantic FormationTask schema
        # Schema uses: shape (Literal["B"]), region (Literal["left_side", "right_side", "center"]), num_agents (int)
        system_prompt = """You are a formation task planner for multi-robot systems.

Your task is to parse natural language instructions and output a JSON object that validates against the FormationTask schema:

{
    "shape": "B",
    "region": "right_side",
    "num_agents": 20
}

Schema Rules (STRICT):
1. shape: MUST be one of these exact strings (case-sensitive):
   - "B": Letter B formation
   - "circle": Circular formation (robots arranged in a circle)
   - "grid": Rectangular grid formation (robots in rows and columns)
   - "heart": Heart shape formation
2. region: MUST be one of these exact strings:
   - "left_side": Place formation on the left side of the room
   - "right_side": Place formation on the right side of the room
   - "center": Place formation in the center of the room
3. num_agents: MUST be a positive integer (number of robots/agents to use)
   - If not specified in instruction, use 20 as default
   - DO NOT use null or omit this field

Examples:
- "form a letter B on the right side" → {"shape": "B", "region": "right_side", "num_agents": 20}
- "arrange robots in a circle in the center" → {"shape": "circle", "region": "center", "num_agents": 20}
- "form a grid on the left side with 16 robots" → {"shape": "grid", "region": "left_side", "num_agents": 16}
- "make a heart shape on the right" → {"shape": "heart", "region": "right_side", "num_agents": 20}
- "form letter B" → {"shape": "B", "region": "right_side", "num_agents": 20}

IMPORTANT:
- Output ONLY valid JSON that matches the schema exactly
- Do NOT include "mode" or "scale" fields (they are not in the schema)
- Do NOT use "num_robots" (use "num_agents" instead)
- If region is not specified, default to "right_side"
- If shape is not specified, default to "B"
- num_agents is REQUIRED (use 20 if not specified)
"""

        user_prompt = f"""Parse this formation instruction and output the JSON plan:

"{text}"

Output ONLY the JSON object, no additional text."""

        logger.info(f"🤖 Calling OpenAI API (model: {self.model_name})...")
        
        # Call OpenAI API
        response = client.chat.completions.create(
            model=self.model_name,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
            max_tokens=256,
            response_format={"type": "json_object"}  # Enforce JSON output
        )
        
        logger.info("✅ OpenAI API call successful")
        
        # Extract and log token usage
        if hasattr(response, 'usage') and response.usage:
            usage = response.usage
            prompt_tokens = getattr(usage, 'prompt_tokens', 0)
            completion_tokens = getattr(usage, 'completion_tokens', 0)
            total_tokens = getattr(usage, 'total_tokens', 0)
            logger.info(
                f"📊 Token usage: prompt={prompt_tokens}, "
                f"completion={completion_tokens}, total={total_tokens}"
            )
            # Also print to console for visibility
            print(
                f"📊 Token usage: prompt={prompt_tokens}, "
                f"completion={completion_tokens}, total={total_tokens}"
            )
        
        # Extract response text
        response_text = response.choices[0].message.content
        logger.debug(f"📥 Raw response: {response_text[:200]}...")
        
        # Parse JSON
        try:
            json_data = json.loads(response_text)
        except json.JSONDecodeError as e:
            raise ValueError(f"Failed to parse JSON from LLM response: {e}")
        
        logger.debug(f"📋 Extracted JSON: {json_data}")
        
        # Validate and create FormationTask using Pydantic
        # This ensures type safety and validates Literal constraints
        try:
            from pydantic import ValidationError
            
            # Map old field names to new schema if needed
            # Handle backward compatibility: "num_robots" -> "num_agents"
            if "num_robots" in json_data and "num_agents" not in json_data:
                json_data["num_agents"] = json_data.pop("num_robots")
            
            # Ensure num_agents is provided (required field)
            if "num_agents" not in json_data or json_data["num_agents"] is None:
                # Use default if not specified
                json_data["num_agents"] = 20
            
            # Remove fields not in the new schema (mode, scale)
            json_data.pop("mode", None)
            json_data.pop("scale", None)
            
            # Validate using Pydantic BaseModel
            task = FormationTask(**json_data)
            
            logger.info(f"✅ Successfully parsed instruction: {task}")
            return task
            
        except ValidationError as e:
            logger.error(f"❌ Pydantic validation failed: {e}")
            logger.error(f"   Received JSON: {json_data}")
            # Fallback: try to construct with defaults
            try:
                num_agents = json_data.get("num_robots") or json_data.get("num_agents", 20)
                region = json_data.get("region", "right_side")
                if region not in ["left_side", "right_side", "center"]:
                    region = "right_side"
                task = FormationTask(shape="B", region=region, num_agents=num_agents)
                logger.warning(f"⚠️  Using fallback FormationTask: {task}")
                return task
            except Exception as e2:
                logger.error(f"❌ Fallback also failed: {e2}")
                raise ValueError(f"Failed to create FormationTask from LLM response: {e}")

