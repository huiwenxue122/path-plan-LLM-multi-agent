#!/usr/bin/env python3
"""
LLM Controller for Natural Language Navigation Control

This module provides functions to parse natural language instructions into
structured navigation plans using OpenAI's API or offline deterministic parsing.

Key Features:
- Pydantic models for type-safe data structures
- OpenAI API integration with JSON-only output enforcement
- Offline mode for testing without API calls
- Automatic fallback to default plan on parsing errors
"""

import os
import json
import re
from typing import List, Optional
from pydantic import BaseModel, Field, ValidationError

# Try to load OpenAI API key from .env file or openai_key.json
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


class AgentSpec(BaseModel):
    """
    Agent specification for navigation task
    
    Attributes:
        id: Agent identifier, typically "A" or "B" (or "alice"/"bob")
        goal: Target position in world coordinates [x, y] in meters
    """
    id: str = Field(..., description="Agent identifier (e.g., 'A', 'B', 'alice', 'bob')")
    goal: List[float] = Field(..., description="Target position [x, y] in meters")
    
    def __init__(self, **data):
        """Custom init to validate goal length"""
        # Validate goal before calling super().__init__
        if 'goal' in data:
            goal = data['goal']
            if not isinstance(goal, list):
                raise ValueError("goal must be a list")
            if len(goal) != 2:
                raise ValueError(f"goal must have exactly 2 elements [x, y], got {len(goal)}")
            if not all(isinstance(x, (int, float)) for x in goal):
                raise ValueError("goal must contain only numbers")
            data['goal'] = [float(x) for x in goal]
        super().__init__(**data)


class TaskPlan(BaseModel):
    """
    Task plan structure for navigation
    
    Attributes:
        task: Task type, must be "navigation", "nav", or "pathfinding"
        agents: List of agent specifications with their goals
        priority: Priority order list (e.g., ["A", "B"] means A plans first)
    """
    task: str = Field(
        default="navigation",
        pattern=r"^(navigation|nav|pathfinding)$",
        description="Task type"
    )
    agents: List[AgentSpec] = Field(..., description="List of agents with their goals")
    priority: List[str] = Field(..., description="Priority order for path planning")
    
    def __init__(self, **data):
        """Custom init to validate agents and priority lists"""
        # Validate agents list
        if 'agents' in data:
            agents = data['agents']
            if not isinstance(agents, list):
                raise ValueError("agents must be a list")
            if len(agents) < 1:
                raise ValueError("agents must have at least 1 agent")
        
        # Validate priority list
        if 'priority' in data:
            priority = data['priority']
            if not isinstance(priority, list):
                raise ValueError("priority must be a list")
            if len(priority) < 1:
                raise ValueError("priority must have at least 1 element")
        
        super().__init__(**data)


def _fallback_plan() -> TaskPlan:
    """
    Default fallback plan when LLM parsing fails
    
    Returns:
        TaskPlan with default agent goals and priority order
    """
    return TaskPlan(
        task="navigation",
        agents=[
            AgentSpec(id="A", goal=[3.0, 2.0]),
            AgentSpec(id="B", goal=[-2.0, 2.0])
        ],
        priority=["A", "B"]
    )


def _extract_json_from_response(response_text: str) -> Optional[dict]:
    """
    Extract JSON from LLM response text
    
    Handles cases where LLM wraps JSON in markdown code blocks or adds extra text
    
    Args:
        response_text: Raw response text from LLM
        
    Returns:
        Parsed JSON dictionary or None if extraction fails
    """
    # Try to find JSON in markdown code blocks
    json_match = re.search(r'```(?:json)?\s*(\{.*?\})\s*```', response_text, re.DOTALL)
    if json_match:
        try:
            return json.loads(json_match.group(1))
        except json.JSONDecodeError:
            pass
    
    # Try to find JSON object directly
    json_match = re.search(r'\{.*\}', response_text, re.DOTALL)
    if json_match:
        try:
            return json.loads(json_match.group(0))
        except json.JSONDecodeError:
            pass
    
    # Try parsing the entire response as JSON
    try:
        return json.loads(response_text)
    except json.JSONDecodeError:
        pass
    
    return None


def llm_parse_instruction(user_text: str) -> TaskPlan:
    """
    Parse natural language instruction using OpenAI API
    
    This function:
    1. Sends user instruction to OpenAI GPT model
    2. Enforces JSON-only output format
    3. Validates response using Pydantic models
    4. Returns fallback plan if parsing/validation fails
    
    Args:
        user_text: Natural language instruction from user
        
    Returns:
        TaskPlan object with parsed agent goals and priority order
        
    Example:
        >>> plan = llm_parse_instruction("Robot A go to (3, 2), Robot B go to (-2, 2), A has priority")
        >>> print(plan.agents[0].goal)  # [3.0, 2.0]
        >>> print(plan.priority)  # ["A", "B"]
    """
    # Check if API key is available
    if not OPENAI_API_KEY:
        print("⚠️  Warning: OPENAI_API_KEY not found. Using offline parser.")
        return llm_parse_instruction_offline(user_text)
    
    try:
        import openai
        client = openai.OpenAI(api_key=OPENAI_API_KEY)
    except ImportError:
        print("⚠️  Warning: openai package not installed. Using offline parser.")
        return llm_parse_instruction_offline(user_text)
    
    # Build system prompt with strict JSON format requirement
    system_prompt = """You are a navigation task planner for multi-robot systems.

Your task is to parse natural language instructions and output a JSON object with the following structure:
{
    "task": "navigation",
    "agents": [
        {"id": "A", "goal": [x, y]},
        {"id": "B", "goal": [x, y]}
    ],
    "priority": ["A", "B"]
}

Rules:
1. Extract agent goals from the instruction (coordinates in meters)
2. Determine priority order from the instruction (e.g., "A first" means ["A", "B"])
3. Agent IDs can be "A", "B", "alice", "bob", etc.
4. Goals must be [x, y] coordinates in world coordinates (meters)
5. Priority list determines planning order (first agent plans first)
6. Output ONLY valid JSON, no additional text or markdown

IMPORTANT - Environment Constraints:
- Environment bounds: x ∈ [-4.0, 4.0], y ∈ [-3.0, 3.0]
- AVOID left side (x < 0) - many obstacles there, goals may be unreachable
- PREFER right side (x > 2.0) for goals - fewer obstacles, more reachable
- If instruction doesn't specify coordinates, use safe default positions on the right side

Example inputs and outputs:
- "Robot A go to (3, 2), Robot B go to (3.2, -1)" 
  → {"task":"navigation","agents":[{"id":"A","goal":[3.0,2.0]},{"id":"B","goal":[3.2,-1.0]}],"priority":["A","B"]}

- "Alice goes to position 3, 1.6. Bob goes to 3.2, -1. Alice has priority"
  → {"task":"navigation","agents":[{"id":"alice","goal":[3.0,1.6]},{"id":"bob","goal":[3.2,-1.0]}],"priority":["alice","bob"]}

- "both robots arrive their goals, A has priority" (no specific coordinates)
  → {"task":"navigation","agents":[{"id":"A","goal":[3.0,1.6]},{"id":"B","goal":[3.2,-1.0]}],"priority":["A","B"]}
"""

    user_prompt = f"""Parse this navigation instruction and output the JSON plan:

"{user_text}"

Output ONLY the JSON object, no additional text."""

    try:
        # Call OpenAI API
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1,
            max_tokens=256,
            response_format={"type": "json_object"}  # Enforce JSON output
        )
        
        # Extract response text
        response_text = response.choices[0].message.content
        
        # Extract JSON from response
        json_data = _extract_json_from_response(response_text)
        
        if json_data is None:
            print("⚠️  Warning: Failed to extract JSON from LLM response. Using fallback plan.")
            return _fallback_plan()
        
        # Validate and parse using Pydantic
        try:
            task_plan = TaskPlan(**json_data)
            print(f"✅ Successfully parsed instruction: {user_text}")
            return task_plan
        except ValidationError as e:
            print(f"⚠️  Warning: Validation error: {e}. Using fallback plan.")
            return _fallback_plan()
            
    except Exception as e:
        print(f"⚠️  Warning: OpenAI API error: {e}. Using fallback plan.")
        return _fallback_plan()


def llm_parse_instruction_offline(user_text: str) -> TaskPlan:
    """
    Offline parser for natural language instructions (deterministic, no API calls)
    
    This function provides a simple rule-based parser for testing and demos
    without requiring OpenAI API access.
    
    Args:
        user_text: Natural language instruction from user
        
    Returns:
        TaskPlan object with parsed agent goals and priority order
        
    Example:
        >>> plan = llm_parse_instruction_offline("A to (3, 2), B to (-2, 2)")
        >>> print(plan.agents[0].goal)  # [3.0, 2.0]
    """
    # Simple pattern matching for offline parsing
    # This is a basic implementation - can be extended with more patterns
    
    agents = []
    priority = []
    
    # Try to extract coordinates and agent IDs
    # Pattern: "A to (3, 2)" or "Robot A go to 3, 2" or "alice goes to 3.0, 1.6"
    patterns = [
        r'([Aa]lice|[Bb]ob|[AB])\s+(?:goes?|to|go\s+to)\s*[\(]?([-]?\d+\.?\d*)\s*[,，]\s*([-]?\d+\.?\d*)',
        r'([Aa]lice|[Bb]ob|[AB])\s*[:：]\s*\(([-]?\d+\.?\d*)\s*[,，]\s*([-]?\d+\.?\d*)\)',
        r'([Aa]lice|[Bb]ob|[AB])\s+([-]?\d+\.?\d*)\s*[,，]\s*([-]?\d+\.?\d*)',
    ]
    
    for pattern in patterns:
        matches = re.finditer(pattern, user_text, re.IGNORECASE)
        for match in matches:
            agent_id = match.group(1).upper() if len(match.group(1)) == 1 else match.group(1).lower()
            x = float(match.group(2))
            y = float(match.group(3))
            
            # Normalize agent IDs
            if agent_id.lower() in ['a', 'alice']:
                agent_id = 'A'
            elif agent_id.lower() in ['b', 'bob']:
                agent_id = 'B'
            
            agents.append(AgentSpec(id=agent_id, goal=[x, y]))
    
    # Extract priority order
    priority_patterns = [
        r'([Aa]lice|[Bb]ob|[AB])\s+(?:first|优先|has priority|priority)',
        r'priority[:\s]+([AB])',
    ]
    
    for pattern in priority_patterns:
        matches = re.finditer(pattern, user_text, re.IGNORECASE)
        for match in matches:
            agent_id = match.group(1).upper() if len(match.group(1)) == 1 else match.group(1).lower()
            if agent_id.lower() in ['a', 'alice']:
                priority.append('A')
            elif agent_id.lower() in ['b', 'bob']:
                priority.append('B')
    
    # If no agents found, check if instruction mentions goals without coordinates
    if not agents:
        # Check if instruction mentions "goals" or "arrive" without specific coordinates
        if any(keyword in user_text.lower() for keyword in ['goal', 'arrive', 'reach', 'target', 'destination']):
            # Use safe default positions on the right side (x > 2.0) to avoid obstacles
            print("⚠️  Warning: No specific coordinates found. Using safe default positions (right side).")
            agents = [
                AgentSpec(id='A', goal=[3.0, 1.6]),  # Alice default (right side, upper)
                AgentSpec(id='B', goal=[3.2, -1.0])   # Bob default (right side, lower)
            ]
        else:
            print("⚠️  Warning: Could not parse agents from instruction. Using fallback plan.")
            return _fallback_plan()
    
    # If no priority specified, use default order
    if not priority:
        # Extract unique agent IDs from agents list
        agent_ids = [agent.id for agent in agents]
        priority = list(dict.fromkeys(agent_ids))  # Preserve order, remove duplicates
    
    # Ensure all agents are in priority list
    agent_ids = [agent.id for agent in agents]
    for agent_id in agent_ids:
        if agent_id not in priority:
            priority.append(agent_id)
    
    try:
        task_plan = TaskPlan(
            task="navigation",
            agents=agents,
            priority=priority
        )
        print(f"✅ Offline parser parsed instruction: {user_text}")
        return task_plan
    except ValidationError as e:
        print(f"⚠️  Warning: Validation error in offline parser: {e}. Using fallback plan.")
        return _fallback_plan()


# Example usage and testing
if __name__ == "__main__":
    print("=" * 60)
    print("LLM Controller - Test Suite")
    print("=" * 60)
    
    # Test offline parser
    test_cases = [
        "Robot A go to (3, 2), Robot B go to (-2, 2)",
        "Alice goes to 3.0, 1.6. Bob goes to 3.2, -1.0",
        "A: (3, 2), B: (-2, 2), A first",
    ]
    
    print("\n📝 Testing Offline Parser:")
    print("-" * 60)
    for test_case in test_cases:
        print(f"\nInput: {test_case}")
        plan = llm_parse_instruction_offline(test_case)
        print(f"  Task: {plan.task}")
        print(f"  Agents: {[(a.id, a.goal) for a in plan.agents]}")
        print(f"  Priority: {plan.priority}")
    
    # Test online parser (if API key available)
    if OPENAI_API_KEY:
        print("\n\n📝 Testing Online Parser (OpenAI API):")
        print("-" * 60)
        test_case = "Robot A go to position 3, 2. Robot B go to -2, 2. A has priority."
        print(f"\nInput: {test_case}")
        plan = llm_parse_instruction(test_case)
        print(f"  Task: {plan.task}")
        print(f"  Agents: {[(a.id, a.goal) for a in plan.agents]}")
        print(f"  Priority: {plan.priority}")
    else:
        print("\n\n⚠️  Skipping online parser test (no API key found)")

