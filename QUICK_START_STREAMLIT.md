# Quick Start Guide: Streamlit Formation Control UI

## 🚀 Quick Start (3 Steps)

### 1. Install Dependencies
```bash
pip install streamlit Pillow
# Or install all requirements:
pip install -r requirements.txt
```

### 2. Run the App
```bash
./run_app.sh
# Or directly:
streamlit run frontend/app.py
```

### 3. Use the UI
1. Enter command: "form a letter B on the right side"
2. Click "🔍 Parse Command"
3. Click "🎯 Generate Targets"
4. Click "▶️ Start Simulation"
5. Watch robots form the B shape!

## 📋 Example Commands

Try these natural language commands:

- "form a letter B on the right side"
- "arrange 20 robots in a circle in the center"
- "make a grid formation on the left"
- "scatter 15 robots randomly"
- "form a heart shape on the right"
- "create a line formation in the center"

## 🎮 Manual Controls

You can also use the sidebar to:
- Select formation type (B, circle, grid, heart, line, random)
- Choose region (left_side, right_side, center)
- Set number of robots (5-30)
- Step through simulation manually

## ⚙️ Configuration

### OpenAI API Key (Optional)
For LLM parsing, set up your API key:
```bash
# Option 1: .env file
echo "OPENAI_API_KEY=your-key" > .env

# Option 2: openai_key.json
echo '"your-key"' > openai_key.json
```

If no API key is set, the app will use a rule-based parser.

## 🐛 Troubleshooting

**"Module not found"**
- Make sure you're in the project root directory
- Run: `pip install -r requirements.txt`

**"OpenAI API key not found"**
- Set up API key (see above)
- Or uncheck "Use LLM" checkbox

**Simulation not starting**
- Make sure you clicked "Generate Targets" first
- Check that `nav_world/room_formation.xml` exists

**Rendering issues**
- Ensure MuJoCo is installed: `pip install mujoco`
- Check OpenGL drivers are up to date

## 📁 Project Structure

```
project/
├── frontend/
│   └── app.py              # Streamlit UI
├── backend/
│   ├── controller.py       # MuJoCo controller
│   ├── llm.py             # LLM integration
│   └── formations.py      # Formation generators
├── nav_world/
│   ├── nav_env_formation_orca.py
│   └── room_formation.xml
└── run_app.sh             # Launch script
```

## 🎯 Features

✅ Natural language command parsing  
✅ Multiple formation types (B, circle, grid, heart, line, random)  
✅ Real-time 3D visualization  
✅ Interactive controls (start, stop, step, reset)  
✅ LLM integration (OpenAI GPT-4o-mini)  
✅ Manual formation selection  
✅ Robot position tracking  

Enjoy controlling your robot formations! 🤖


