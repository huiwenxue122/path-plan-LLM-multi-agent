#!/bin/bash
# 运行MuJoCo 3D Viewer的简单脚本

echo "🎮 启动MuJoCo 3D Viewer"
echo "=========================="

# 检查mjpython是否可用
if ! command -v mjpython &> /dev/null; then
    echo "❌ mjpython未找到，请先安装MuJoCo"
    echo "💡 安装方法: pip install mujoco"
    exit 1
fi

# 运行3D viewer
echo "🚀 启动3D Viewer..."
mjpython fixed_mujoco_viewer.py
