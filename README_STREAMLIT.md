# Streamlit UI for Multi-Robot Formation Control

## Overview

This is an interactive web-based UI for controlling multi-robot formations using natural language commands and LLM integration.

## Features

- 🤖 **Natural Language Control**: Input commands like "form a letter B on the right side"
- 🎯 **Multiple Formations**: Support for B, circle, grid, heart, line, and random formations
- 🎮 **Interactive Controls**: Start, stop, step, and reset simulation
- 📊 **Real-time Visualization**: Live 3D MuJoCo rendering in the browser
- 🔍 **LLM Integration**: Uses OpenAI GPT-4o-mini to parse natural language commands

## Installation

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Set up OpenAI API key (optional, for LLM features):**
   ```bash
   # Option 1: Create .env file
   echo "OPENAI_API_KEY=your-api-key-here" > .env
   
   # Option 2: Use openai_key.json
   echo '"your-api-key-here"' > openai_key.json
   ```

## Running the App

### Method 1: Using the script
```bash
./run_app.sh
```

### Method 2: Direct Streamlit command
```bash
streamlit run frontend/app.py
```

The app will open in your browser at `http://localhost:8501`

## Usage

### 1. Natural Language Command

Enter a command in the text box:
- "form a letter B on the right side"
- "arrange robots in a circle in the center"
- "make a grid formation on the left"
- "scatter robots randomly"

### 2. Parse Command

Click "🔍 Parse Command" to use LLM to extract:
- Formation type
- Region (left_side, right_side, center)
- Number of robots

### 3. Generate Targets

Click "🎯 Generate Targets" to create target positions based on the parsed command or manual selections.

### 4. Start Simulation

Click "▶️ Start Simulation" to begin the formation process. Robots will move toward their targets in real-time.

### 5. Manual Controls

- **⏭️ Step Once**: Advance simulation by one step
- **🔄 Reset**: Reset simulation to initial state
- **⏹️ Stop**: Stop the running simulation

## Supported Formations

- **B**: Letter B shape (uses existing `generate_letter_b_targets`)
- **circle**: Circular formation
- **grid**: Rectangular grid
- **heart**: Heart shape
- **line**: Straight line formation
- **random**: Random scattered positions

## Architecture

```
frontend/app.py          # Streamlit UI
backend/
  ├── controller.py      # MuJoCo environment controller
  ├── llm.py            # LLM command parsing
  └── formations.py     # Formation generators
```

## Troubleshooting

### "Module not found" errors
Make sure you're running from the project root directory.

### "OpenAI API key not found"
- Check `.env` file or `openai_key.json`
- Or uncheck "Use LLM" to use rule-based parser

### Simulation not starting
- Make sure you've generated targets first
- Check that the XML file exists at `nav_world/room_formation.xml`

### Rendering issues
- Ensure MuJoCo is properly installed
- Check that OpenGL drivers are up to date

## Example Workflow

1. Enter: "form a letter B on the right side with 20 robots"
2. Click "🔍 Parse Command" → See parsed parameters
3. Click "🎯 Generate Targets" → Targets generated
4. Click "▶️ Start Simulation" → Watch robots form the B shape
5. Wait for completion or click "⏹️ Stop" to stop early

## Notes

- The simulation runs at ~20 FPS (50ms per step)
- Maximum simulation steps: 10,000
- Success threshold: 0.15m distance to target
- All formations support left_side, right_side, and center regions

