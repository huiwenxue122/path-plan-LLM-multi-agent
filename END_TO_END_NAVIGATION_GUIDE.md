# 🎯 End-to-End Natural Language Navigation - Complete Guide

## ✅ Implementation Summary

I've successfully implemented a complete end-to-end natural language navigation control system with the following components:

### 📁 Created Files

1. **`llm_interface/llm_controller.py`** (NEW)
   - Pydantic models: `AgentSpec`, `TaskPlan`
   - `llm_parse_instruction()`: OpenAI API parser
   - `llm_parse_instruction_offline()`: Offline rule-based parser
   - `_fallback_plan()`: Default plan on parsing errors

2. **`llm_interface/end_to_end_navigation.py`** (NEW)
   - `EndToEndNavigationController`: Complete pipeline integration
   - Connects LLM parsing → MAPF planning → MuJoCo execution

3. **`llm_interface/__init__.py`** (NEW)
   - Module exports

4. **`llm_interface/README.md`** (NEW)
   - Complete usage documentation

---

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install pydantic openai python-dotenv
```

### 2. Set Up API Key

**Option A: .env file (recommended)**
```bash
echo "OPENAI_API_KEY=your-api-key-here" > .env
```

**Option B: openai_key.json (legacy support)**
```bash
echo '"your-api-key-here"' > openai_key.json
```

### 3. Run End-to-End Navigation

```bash
cd /Users/claire/co-robot-pathfinding
python llm_interface/end_to_end_navigation.py
```

**Example input:**
```
Robot A go to (3, 2), Robot B go to (-2, 2), A has priority
```

---

## 🔄 Complete Pipeline Flow

```
┌─────────────────────────────────────────────────────────────┐
│ 1. User Types Natural Language Command                     │
│    "Robot A go to (3, 2), Robot B go to (-2, 2)"          │
└────────────────────┬──────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. LLM Parser (llm_parse_instruction)                      │
│    - Extracts agent goals: A→(3,2), B→(-2,2)              │
│    - Determines priority: ["A", "B"]                        │
│    - Returns TaskPlan object                                │
└────────────────────┬──────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Goal Setting (set_goals_from_plan)                     │
│    - Maps LLM IDs (A/B) to NavEnv IDs (alice/bob)         │
│    - Sets goal positions in environment                    │
└────────────────────┬──────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. MAPF Planning (plan_paths_with_mapf)                   │
│    - Creates MAPF agent specs from current positions       │
│    - Plans collision-free paths using priority planning    │
│    - Converts grid paths to world coordinates              │
└────────────────────┬──────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. MuJoCo Execution (execute_in_mujoco)                    │
│    - Runs simulation with 3D viewer                        │
│    - Displays real-time robot movement                     │
│    - Shows completion status                                │
└─────────────────────────────────────────────────────────────┘
```

---

## 📝 Code Structure

### Pydantic Models

```python
class AgentSpec(BaseModel):
    id: str                    # "A", "B", "alice", "bob"
    goal: List[float]          # [x, y] in world coordinates (meters)

class TaskPlan(BaseModel):
    task: str                  # "navigation", "nav", or "pathfinding"
    agents: List[AgentSpec]     # List of agents with goals
    priority: List[str]         # Priority order (e.g., ["A", "B"])
```

### Key Functions

**1. LLM Parsing**
```python
from llm_interface.llm_controller import llm_parse_instruction

plan = llm_parse_instruction("Robot A go to (3, 2), Robot B go to (-2, 2)")
# Returns: TaskPlan with parsed goals and priority
```

**2. Offline Parsing (No API)**
```python
from llm_interface.llm_controller import llm_parse_instruction_offline

plan = llm_parse_instruction_offline("A to (3, 2), B to (-2, 2)")
# Returns: TaskPlan using rule-based parser
```

**3. End-to-End Controller**
```python
from llm_interface.end_to_end_navigation import EndToEndNavigationController

controller = EndToEndNavigationController(xml_path="room.xml")
task_plan = controller.parse_user_instruction("Robot A go to (3, 2)")
controller.set_goals_from_plan(task_plan)
controller.plan_paths_with_mapf(task_plan)
controller.execute_in_mujoco(use_viewer=True)
```

---

## 🎯 Features

✅ **Type-Safe**: Pydantic models ensure data validation  
✅ **Flexible API Key Loading**: Supports .env, openai_key.json, or environment variables  
✅ **Offline Mode**: Rule-based parser for testing without API calls  
✅ **Error Handling**: Automatic fallback to default plan on parsing errors  
✅ **Complete Integration**: Seamless connection between LLM, MAPF, and MuJoCo  
✅ **Clean Code**: Modular design with clear comments  

---

## 📊 Example Usage

### Example 1: Basic Navigation

```python
from llm_interface.end_to_end_navigation import EndToEndNavigationController

controller = EndToEndNavigationController(
    xml_path="real_world/nav_world/room.xml",
    use_offline_parser=False  # Use OpenAI API
)

# Parse instruction
task_plan = controller.parse_user_instruction(
    "Robot A go to (3, 2), Robot B go to (-2, 2), A has priority"
)

# Set goals
controller.set_goals_from_plan(task_plan)

# Plan paths
mapf_paths = controller.plan_paths_with_mapf(task_plan)

# Execute
controller.execute_in_mujoco(use_viewer=True)
```

### Example 2: Offline Mode

```python
controller = EndToEndNavigationController(
    xml_path="real_world/nav_world/room.xml",
    use_offline_parser=True  # No API calls
)

task_plan = controller.parse_user_instruction("A to (3, 2), B to (-2, 2)")
# ... rest of the pipeline
```

---

## 🔧 Configuration

### API Key Setup

The system tries to load API key in this order:
1. `.env` file: `OPENAI_API_KEY=your-key`
2. `openai_key.json`: `"your-key"`
3. Environment variable: `export OPENAI_API_KEY=your-key`

### Parser Selection

- **Online Parser**: Uses OpenAI GPT-4o, requires API key
- **Offline Parser**: Rule-based, no API calls, good for testing

---

## 🛠️ Troubleshooting

### Issue: "OPENAI_API_KEY not found"

**Solution:**
- Create `.env` file with `OPENAI_API_KEY=your-key`
- Or use `openai_key.json` with format: `"your-key"`
- Or set environment variable

### Issue: "Validation error"

**Solution:**
- Check that instruction contains valid coordinates
- Ensure agent IDs are recognized (A, B, alice, bob)
- System will automatically use fallback plan

### Issue: "MAPF planning failed"

**Solution:**
- Check that goal positions are within environment bounds
- Try adjusting priority order
- Ensure obstacles don't block all paths

---

## 📚 Related Documentation

- **LLM Interface**: `llm_interface/README.md`
- **MAPF Implementation**: `MAPF_IMPLEMENTATION_EXPLAINED.md`
- **MAPF Usage**: `real_world/nav_world/HOW_TO_RUN_MAPF.md`
- **MuJoCo Viewer**: `RUN_MAPF_3D.md`

---

## ✅ Testing

Test the offline parser:
```bash
python -c "from llm_interface.llm_controller import llm_parse_instruction_offline; plan = llm_parse_instruction_offline('A to (3, 2), B to (-2, 2)'); print(plan)"
```

Test the complete pipeline:
```bash
python llm_interface/end_to_end_navigation.py
```

---

## 🎉 Summary

You now have a complete end-to-end system that:
1. ✅ Parses natural language using LLM (with offline fallback)
2. ✅ Plans collision-free paths using MAPF
3. ✅ Executes paths in MuJoCo 3D environment
4. ✅ Provides type-safe data structures with Pydantic
5. ✅ Handles errors gracefully with fallback plans

The system is modular, well-documented, and ready to use! 🚀

