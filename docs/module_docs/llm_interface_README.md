# 🎯 End-to-End Natural Language Navigation Control

This module provides a complete pipeline for controlling multi-robot navigation using natural language:

**User Command → LLM Parsing → MAPF Planning → MuJoCo Execution**

## 📁 Files

- **`llm_controller.py`**: LLM parser with Pydantic models for type-safe data structures
- **`end_to_end_navigation.py`**: Complete integration pipeline
- **`__init__.py`**: Module exports

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install pydantic openai python-dotenv
```

### 2. Set Up API Key

**Option A: Use .env file (recommended)**
```bash
echo "OPENAI_API_KEY=your-api-key-here" > .env
```

**Option B: Use openai_key.json (legacy)**
```bash
echo '"your-api-key-here"' > openai_key.json
```

**Option C: Environment variable**
```bash
export OPENAI_API_KEY=your-api-key-here
```

### 3. Run End-to-End Navigation

```bash
cd /Users/claire/co-robot-pathfinding
python llm_interface/end_to_end_navigation.py
```

Then type your instruction, for example:
```
Robot A go to (3, 2), Robot B go to (-2, 2), A has priority
```

## 📝 Usage Examples

### Example 1: Basic Navigation

```
Input: "Robot A go to (3, 2), Robot B go to (-2, 2)"
Output:
  ✅ Parsed plan:
     Task: navigation
     Agents: [('A', [3.0, 2.0]), ('B', [-2.0, 2.0])]
     Priority: ['A', 'B']
```

### Example 2: With Priority

```
Input: "Alice goes to 3.0, 1.6. Bob goes to 3.2, -1.0. Alice first"
Output:
  ✅ Parsed plan:
     Task: navigation
     Agents: [('alice', [3.0, 1.6]), ('bob', [3.2, -1.0])]
     Priority: ['alice', 'bob']
```

### Example 3: Using Offline Parser

```python
from llm_interface.llm_controller import llm_parse_instruction_offline

plan = llm_parse_instruction_offline("A to (3, 2), B to (-2, 2)")
print(plan.agents[0].goal)  # [3.0, 2.0]
```

## 🔧 API Reference

### `llm_parse_instruction(user_text: str) -> TaskPlan`

Parse natural language using OpenAI API.

**Args:**
- `user_text`: Natural language instruction

**Returns:**
- `TaskPlan` object with parsed goals and priority

**Example:**
```python
from llm_interface.llm_controller import llm_parse_instruction

plan = llm_parse_instruction("Robot A go to (3, 2), Robot B go to (-2, 2)")
```

### `llm_parse_instruction_offline(user_text: str) -> TaskPlan`

Parse natural language using rule-based parser (no API calls).

**Args:**
- `user_text`: Natural language instruction

**Returns:**
- `TaskPlan` object with parsed goals and priority

**Example:**
```python
from llm_interface.llm_controller import llm_parse_instruction_offline

plan = llm_parse_instruction_offline("A to (3, 2), B to (-2, 2)")
```

### `EndToEndNavigationController`

Complete controller class for end-to-end navigation.

**Usage:**
```python
from llm_interface.end_to_end_navigation import EndToEndNavigationController

controller = EndToEndNavigationController(xml_path="room.xml")
task_plan = controller.parse_user_instruction("Robot A go to (3, 2)")
controller.set_goals_from_plan(task_plan)
controller.plan_paths_with_mapf(task_plan)
controller.execute_in_mujoco(use_viewer=True)
```

## 📊 Data Models

### `AgentSpec`

```python
class AgentSpec(BaseModel):
    id: str                    # "A", "B", "alice", "bob"
    goal: List[float]          # [x, y] in world coordinates (meters)
```

### `TaskPlan`

```python
class TaskPlan(BaseModel):
    task: str                  # "navigation", "nav", or "pathfinding"
    agents: List[AgentSpec]     # List of agents with goals
    priority: List[str]         # Priority order (e.g., ["A", "B"])
```

## 🔄 Complete Pipeline

```
1. User Input
   ↓
2. LLM Parser (llm_parse_instruction)
   - Extracts agent goals from natural language
   - Determines priority order
   - Returns TaskPlan object
   ↓
3. Goal Setting (set_goals_from_plan)
   - Maps LLM agent IDs to NavEnv agent names
   - Sets goal positions in environment
   ↓
4. MAPF Planning (plan_paths_with_mapf)
   - Creates MAPF agent specifications
   - Plans collision-free paths using priority planning
   - Converts paths to world coordinates
   ↓
5. MuJoCo Execution (execute_in_mujoco)
   - Runs simulation with 3D viewer or headless
   - Displays real-time progress
   - Shows completion status
```

## 🛠️ Troubleshooting

### Issue: API Key Not Found

**Solution:**
- Check `.env` file exists and contains `OPENAI_API_KEY`
- Or use `openai_key.json` with format: `"your-key"`
- Or set environment variable: `export OPENAI_API_KEY=your-key`

### Issue: Import Error

**Solution:**
```bash
pip install pydantic openai python-dotenv
```

### Issue: MuJoCo Viewer Not Working

**Solution:**
- The script automatically falls back to headless mode
- Or use `use_viewer=False` in `execute_in_mujoco()`

## 📚 Related Documentation

- **MAPF Implementation**: `MAPF_IMPLEMENTATION_EXPLAINED.md`
- **MAPF Usage**: `real_world/nav_world/HOW_TO_RUN_MAPF.md`
- **MuJoCo Viewer**: `RUN_MAPF_3D.md`

