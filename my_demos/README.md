# 我的机器人导航演示

## 📁 文件说明

### 🎬 主要演示文件
- **`robot_navigation_demo.py`** - 双机器人导航演示脚本
  - 基于原项目的NavEnv
  - 生成动画视频和轨迹图
  - 支持避开障碍物

### 📊 生成的结果文件
- **`robot_navigation_animation.gif`** - 机器人运动动画
- **`robot_navigation_trajectory.png`** - 静态轨迹图

## 🚀 使用方法

### 运行演示
```bash
cd /Users/claire/co-robot-pathfinding
python my_demos/robot_navigation_demo.py
```

### 查看结果
```bash
# 查看动画
open my_demos/robot_navigation_animation.gif

# 查看轨迹图
open my_demos/robot_navigation_trajectory.png
```

## 🎯 功能特点

1. **双机器人导航**：Alice和Bob同时导航
2. **避开障碍物**：使用A*路径规划
3. **动画可视化**：生成GIF动画显示运动过程
4. **轨迹记录**：保存完整的运动轨迹

## 🔧 技术实现

- **环境**：基于原项目的NavEnv
- **路径规划**：A*算法
- **可视化**：matplotlib动画
- **物理仿真**：MuJoCo
