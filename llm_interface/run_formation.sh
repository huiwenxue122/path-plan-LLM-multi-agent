#!/bin/bash
# 使用mjpython运行端到端编队任务系统（macOS推荐方式）

echo "🎯 End-to-End Natural Language Formation Control with 3D Viewer"
echo "============================================================"
echo ""
echo "This script runs the formation system with MuJoCo 3D Viewer"
echo "using mjpython (required on macOS for proper viewer support)"
echo ""

# Check if mjpython is available
if ! command -v mjpython &> /dev/null; then
    echo "❌ mjpython not found!"
    echo ""
    echo "💡 Solutions:"
    echo "   1. Install MuJoCo: pip install mujoco"
    echo "   2. Or use regular python (may not show 3D viewer on macOS):"
    echo "      python llm_interface/end_to_end_formation.py"
    echo ""
    exit 1
fi

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"

# Change to project root
cd "$PROJECT_ROOT"

echo "✅ Using mjpython for 3D viewer support"
echo "📁 Project directory: $PROJECT_ROOT"
echo ""
echo "🚀 Starting formation system..."
echo ""

# Run with mjpython
mjpython llm_interface/end_to_end_formation.py

