# RoCo: Dialectic Multi-Robot Collaboration with Large Language Models

Codebase for paper: RoCo: Dialectic Multi-Robot Collaboration with Large Language Models

[Mandi Zhao](https://mandizhao.github.io), [Shreeya Jain](https://www.linkedin.com), [Shuran Song](https://www.cs.columbia.edu/~shurans/) 

[Arxiv](https://arxiv.org/abs/2307.04738) | [Project Website](https://project-roco.github.io) 

<img src="method.jpeg" alt="method" width="800"/>

## 🎯 Enhanced Dual-Robot Navigation System

### Overview
This repository contains an enhanced dual-robot navigation system built on top of the original RoCo project. The system features:
- **Multi-Agent Path Finding (MAPF)**: Priority-based collision-free path planning
- **Natural Language Control**: LLM-powered command parsing for robot control
- **Real-time 3D Visualization**: MuJoCo 3D viewer with interactive controls
- **Challenging Obstacle Course**: 6 strategically placed obstacles requiring complex navigation

### Key Features

#### 1. Multi-Agent Path Finding (MAPF)
- **Priority Planning**: Higher-priority robots plan first, avoiding collisions
- **Vertex & Edge Conflict Prevention**: Prevents same-position and position-swapping conflicts
- **Time-Extended A***: Time-aware path planning with wait actions
- **Reservation Table**: Tracks occupied positions and edges across time

#### 2. Natural Language Control
- **LLM Integration**: GPT-4o powered command parsing
- **Structured Task Plans**: Pydantic models for robust validation
- **Offline Mode**: Deterministic parsing for demos without API calls
- **Example Commands**: 
  - `"Robot A go to (3, 2), Robot B go to (-2, 2), A has priority"`
  - `"Alice 先到达目标，Bob 等 2 秒再出发"` (Chinese supported)

#### 3. Visualization
- **3D MuJoCo Viewer**: Real-time interactive visualization
- **2D Matplotlib Animation**: Cross-platform trajectory visualization
- **Path Visualization**: Visual representation of planned paths

### Quick Start

#### Method 1: End-to-End Natural Language Navigation (Recommended)
```bash
cd /Users/claire/co-robot-pathfinding
./llm_interface/run_with_viewer.sh
# Or directly:
python llm_interface/end_to_end_navigation.py
```
Then type a natural language command like:
```
Robot A go to (3, 2), Robot B go to (-2, 2), A has priority
```

#### Method 2: MAPF Path Planning Demo
```bash
cd /Users/claire/co-robot-pathfinding
python nav_world/run_mapf_demo.py
```

#### Method 3: 2D Matplotlib Animation
```bash
cd /Users/claire/co-robot-pathfinding
python my_demos/robot_navigation_demo.py
```

### Project Structure
```
co-robot-pathfinding/
├── llm_interface/                    # Natural language control
│   ├── end_to_end_navigation.py     # Main end-to-end controller ⭐
│   ├── llm_controller.py            # LLM parsing (Pydantic models)
│   ├── find_valid_commands.py       # Utility to find valid goals
│   └── run_with_viewer.sh          # Launch script ⭐
│
├── nav_world/                       # Core navigation system
│   ├── nav_env_mapf.py             # MAPF-integrated environment
│   ├── multi_agent_planner.py      # MAPF algorithm (Priority Planning)
│   ├── nav_env.py                  # Base navigation environment
│   ├── room.xml                    # MuJoCo 3D scene
│   ├── run_mapf_demo.py            # MAPF demo script
│   └── test_mapf.py                # MAPF test script
│
├── my_demos/                        # 2D visualization demos
│   └── robot_navigation_demo.py    # Matplotlib animation
│
├── results/                         # Generated visualization files
│
└── Documentation/
    ├── README.md                    # This file
    ├── MAPF_IMPLEMENTATION_EXPLAINED.md
    ├── END_TO_END_NAVIGATION_GUIDE.md
    └── RUN_MAPF_3D.md

⭐ = Main entry points
```

### Technical Details

#### Robot Configuration
- **Alice**: Blue robot (default start: (-3.2, 2.0))
- **Bob**: Green robot (default start: (-3.2, -2.3))
- Goals can be set via natural language commands

#### Path Planning Algorithms
- **MAPF (Priority Planning)**: Multi-agent collision-free path planning
  - Time-extended A* search
  - Reservation table for conflict prevention
  - Supports wait actions
- **A* (Fallback)**: Single-agent path planning for independent navigation

#### Environment
- **Grid Resolution**: 0.1m for precise navigation
- **Obstacle Layout**: 6 strategically placed obstacles
- **Room Size**: 8m × 6m with walls

### Requirements
- Python 3.8+
- MuJoCo 2.3.0+
- NumPy, Matplotlib, ImageIO
- macOS users need `mjpython` for 3D viewer

### Installation
```bash
# Clone the repository
git clone https://github.com/huiwenxue122/co-robot-pathfinding.git
cd co-robot-pathfinding

# Install dependencies
pip install -r requirements.txt

# For macOS users, install MuJoCo viewer
pip install mujoco
```


---

## Original Project Setup

### Setup conda environment and package installation

```bash
conda create -n roco python=3.8 
conda activate roco
```

### Install MuJoCo and dm_control

```bash
pip install mujoco==2.3.0
pip install dm_control==1.0.8 
```

**If you have M1 Macbook and would like to visualize the task scenes locally:**

Download the macOS-compatible `.dmg` file from MuJoCo release page, inside it should have a `MuJoCo.app` file that you can drag into your /Application folder, so it becomes just like other apps in your Mac. You could then open up the app and drag xml files in it. Find more information in the official documentation.

### Install other packages

```bash
pip install -r requirements.txt
```

### Acquire OpenAI/Claude API Keys

This is required for prompting GPTs or Claude LLMs. You don't necessarily need both of them. Put your key string somewhere safely in your local repo, and provide a file path (something like `./roco/openai_key.json`) and load them in the scripts. Example code snippet:

```python
import openai  
openai.api_key = YOUR_OPENAI_KEY

import anthropic
client = anthropic.Client(api_key=YOUR_CLAUDE_KEY)
streamed = client.completion_stream(...)  
```

## Usage

### Run multi-robot dialog on the PackGrocery Task using the latest GPT-4 model

```bash
$ conda activate roco
(roco) $ python run_dialog.py --task pack -llm gpt-4
```

## Contact

Please direct to Mandi Zhao. If you are interested in contributing or collaborating, please feel free to reach out! I'm more than happy to chat and brainstorm together.

## Cite

```bibtex
@misc{mandi2023roco,
      title={RoCo: Dialectic Multi-Robot Collaboration with Large Language Models}, 
      author={Zhao Mandi and Shreeya Jain and Shuran Song},
      year={2023},
      eprint={2307.04738},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```