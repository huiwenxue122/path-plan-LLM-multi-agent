# 🤖 Multi-Robot Formation Control System

A comprehensive multi-robot formation control system with natural language interface, featuring collision-free path planning using ORCA (Optimal Reciprocal Collision Avoidance) and an interactive Streamlit web UI.

![Python Version](https://img.shields.io/badge/python-3.11+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

## ✨ Features

### 🎯 Core Capabilities
- **Multi-Robot Formation Control**: Control 5-30 robots to form various shapes (B, circle, grid, heart, line, random)
- **Natural Language Interface**: Use LLM (GPT-4o) or rule-based parser to control robots via natural language commands
- **Collision-Free Navigation**: ORCA algorithm ensures robots never collide during movement
- **Interactive Web UI**: Streamlit-based interface for real-time visualization and control
- **Video Export**: Record and export robot formation animations as MP4 videos

### 🛡️ Collision Avoidance
- **ORCA Algorithm**: Optimal Reciprocal Collision Avoidance for smooth, collision-free multi-robot navigation
- **Safe Spacing**: Robots maintain minimum 0.5m distance from each other
- **Dynamic Speed Control**: Automatic speed adjustment based on proximity to other robots

### 🎨 Formation Shapes
- **Letter B**: Custom-designed letter B formation with 20 robots
- **Circle**: Circular formations
- **Grid**: Rectangular grid patterns
- **Heart**: Heart-shaped formations
- **Line**: Linear formations
- **Random**: Random scattered positions

## 🚀 Quick Start

### Prerequisites
- **Python 3.11+** (required for rvo2/ORCA support)
- Conda or pyenv for environment management
- OpenAI API key (optional, for LLM features)

### Installation

#### 1. Clone the Repository
```bash
git clone https://github.com/huiwenxue122/co-robot-pathfinding.git
cd co-robot-pathfinding
```

#### 2. Create Python 3.11 Environment

**Using conda (recommended):**
```bash
conda create -n co-robot-py311 python=3.11 -y
conda activate co-robot-py311
```

**Using pyenv:**
```bash
pyenv install 3.11.6
pyenv virtualenv 3.11.6 co-robot-py311
pyenv activate co-robot-py311
```

#### 3. Install Dependencies
```bash
# Install base dependencies
pip install -r requirements.txt

# Install rvo2 for ORCA collision avoidance
bash install_rvo2.sh
```

**Note:** The `rvo2` library requires Python 3.11+ and must be compiled from source. The `install_rvo2.sh` script automates this process.

#### 4. Set Up OpenAI API Key (Optional)
```bash
# Option 1: Using .env file (recommended)
echo "OPENAI_API_KEY=your-api-key-here" > .env

# Option 2: Using openai_key.json
echo '"your-api-key-here"' > openai_key.json

# Option 3: Environment variable
export OPENAI_API_KEY=your-api-key-here
```

**Note:** The system works in offline mode without an API key, using rule-based parsing.

## 🎮 Usage

### Method 1: Streamlit Web UI (Recommended)

Launch the interactive web interface:

```bash
./run_app.sh
```

Or directly:
```bash
streamlit run frontend/app.py
```

Then open your browser to `http://localhost:8501` and:
1. Enter a natural language command (e.g., "form a letter B on the right side")
2. Click "Parse Command" → "Generate Targets"
3. Click "Start Simulation" to watch robots form the shape
4. Optionally enable "Record video" to export the animation

### Method 2: Command Line - Letter B Formation

Run the letter B formation demo with 3D viewer:

```bash
python nav_world/run_formation_B.py --viewer
```

Watch 20 robots travel from the left side to form a letter **B** on the right side.

### Method 3: Command Line - LLM Formation Control

Use natural language to control formations:

```bash
python nav_world/run_formation_llm.py --viewer --prompt "form a letter B on the right side"
```

Or run interactively:
```bash
python nav_world/run_formation_llm.py --viewer
# Then enter your command when prompted
```

## 📁 Project Structure

```
co-robot-pathfinding/
├── frontend/                    # Streamlit web UI
│   └── app.py                  # Main UI application
│
├── backend/                     # Backend controllers
│   ├── controller.py           # Formation controller
│   ├── formations.py           # Formation generators
│   └── llm.py                  # LLM integration
│
├── nav_world/                   # Core navigation system
│   ├── nav_env_formation_orca.py  # ORCA-based formation environment
│   ├── nav_env.py              # Base navigation environment
│   ├── orca_controller.py      # ORCA collision avoidance wrapper
│   ├── formations.py           # Formation pattern generators
│   ├── formation_task.py       # Formation task data models
│   ├── llm_formation_controller.py  # LLM formation controller
│   ├── room_formation.xml      # MuJoCo scene (16m × 14m room)
│   ├── run_formation_B.py      # Letter B demo script
│   └── run_formation_llm.py    # LLM-controlled formation script
│
├── llm_interface/              # LLM integration (legacy)
│   ├── end_to_end_formation.py
│   └── llm_controller.py
│
├── docs/                        # Documentation
│   ├── LEARNING_GUIDE.md       # Complete learning guide
│   └── user_guides/            # User-facing guides
│
├── results/                     # Generated visualizations
│   ├── letter_b_20_robots.png
│   └── ...
│
├── run_app.sh                   # Streamlit launcher script
├── install_rvo2.sh             # rvo2 installation script
├── requirements.txt            # Python dependencies
└── README.md                   # This file
```

## 🎯 Key Components

### ORCA Collision Avoidance
- **File**: `nav_world/orca_controller.py`
- **Algorithm**: Optimal Reciprocal Collision Avoidance (ORCA)
- **Library**: `rvo2` (Python-RVO2)
- **Features**:
  - Robot radius: 0.25m (ensures 0.5m minimum spacing)
  - Neighbor distance: 4.0m
  - Time horizon: 4.0s
  - Maximum speed: 1.5 m/s

### Formation Generators
- **File**: `nav_world/formations.py`
- **Supported Shapes**: B, circle, grid, heart, line, random
- **Features**:
  - Minimum spacing: 0.5m between robots
  - Region support: left_side, right_side, center
  - Customizable margins and scaling

### Streamlit UI
- **File**: `frontend/app.py`
- **Features**:
  - Natural language command input
  - Real-time 2D visualization (matplotlib)
  - Video recording and export
  - Interactive controls (start, stop, step, reset)

## 🔧 Configuration

### Room Size
- **Current**: 16m × 14m (x: [-8, 8], y: [-7, 7])
- **File**: `nav_world/room_formation.xml`
- **Code**: `nav_world/nav_env.py` (lines 81-82)

### Robot Configuration
- **Default Count**: 20 robots
- **Robot Radius**: 0.25m (ORCA)
- **Minimum Spacing**: 0.5m (2 × radius)
- **Maximum Speed**: 1.5 m/s

### Formation Parameters
- **B Formation**: 5.0m width × 6.0m height
- **Minimum Spacing**: 0.5m between target points
- **Margin**: 0.5m from room edges

## 📊 Example Commands

### Natural Language Commands
```
"form a letter B on the right side"
"move all robots to form a circle in the center"
"create a heart shape on the left side"
"form a grid with 20 robots on the right"
"scatter robots randomly"
```

### Supported Formation Types
- `B` - Letter B formation
- `circle` - Circular formation
- `grid` - Rectangular grid
- `heart` - Heart shape
- `line` - Linear formation
- `random` - Random scattered positions

### Supported Regions
- `right_side` - Right side of the room
- `left_side` - Left side of the room
- `center` - Center of the room

## 🐛 Troubleshooting

### ORCA Not Working
If robots are colliding, ensure:
1. Python 3.11+ environment is active
2. `rvo2` is installed: `python -c "import rvo2; print('OK')"`
3. Streamlit is running in the correct environment

### Streamlit Port Already in Use
```bash
# Kill existing Streamlit processes
pkill -f "streamlit run frontend/app.py"
# Or use a different port
streamlit run frontend/app.py --server.port 8502
```

### Import Errors
```bash
# Ensure project root is in PYTHONPATH
export PYTHONPATH="$(pwd):$PYTHONPATH"
```

## 📚 Documentation

- **[HOW_TO_RUN.md](HOW_TO_RUN.md)** - Detailed running guide
- **[docs/LEARNING_GUIDE.md](docs/LEARNING_GUIDE.md)** - Complete learning path
- **[QUICK_START_STREAMLIT.md](QUICK_START_STREAMLIT.md)** - Quick start for Streamlit UI

## 🎬 Demo Videos

The system can export formation animations as MP4 videos. Enable "Record video while simulating" in the Streamlit UI, then click "Export Video" after the simulation completes.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

See [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ORCA Algorithm**: Based on the Optimal Reciprocal Collision Avoidance algorithm
- **rvo2 Library**: Python-RVO2 implementation
- **MuJoCo**: Physics simulation engine
- **Streamlit**: Web UI framework

## 📧 Contact

For questions or issues, please open an issue on GitHub.

---

**Made with ❤️ for multi-robot coordination research**
