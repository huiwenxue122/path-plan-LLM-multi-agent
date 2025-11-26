#!/bin/bash
# Script to run the Streamlit formation control app

echo "🚀 Starting Multi-Robot Formation Control UI..."
echo ""
echo "Make sure you have:"
echo "  1. Installed all dependencies: pip install -r requirements.txt"
echo "  2. Set up OpenAI API key (optional, for LLM features)"
echo ""

# Activate Python 3.11 environment if available (for rvo2/ORCA support)
# This is CRITICAL for collision avoidance - ORCA requires Python 3.11+
if command -v conda &> /dev/null; then
    if conda env list | grep -q "co-robot-py311"; then
        echo "✅ Activating co-robot-py311 environment for ORCA collision avoidance..."
        echo "   This ensures robots won't collide (ORCA algorithm)"
        eval "$(conda shell.bash hook)"
        conda activate co-robot-py311
        # Verify rvo2 is available
        python -c "import rvo2; print('   ✅ rvo2 (ORCA) is available')" 2>/dev/null || echo "   ⚠️  Warning: rvo2 not found in this environment"
    else
        echo "⚠️  Warning: co-robot-py311 environment not found"
        echo "   Robots may collide - install rvo2 for proper collision avoidance"
    fi
fi

# Check if streamlit is installed
if ! command -v streamlit &> /dev/null; then
    echo "❌ Streamlit not found. Installing..."
    pip install streamlit Pillow
fi

# Ensure the project root is in PYTHONPATH
export PYTHONPATH="$(dirname "$0"):$PYTHONPATH"

# Run the app
echo ""
echo "🌐 Starting Streamlit server..."
echo "   URL: http://localhost:8501"
echo ""
streamlit run frontend/app.py --server.port 8501 --server.address localhost


