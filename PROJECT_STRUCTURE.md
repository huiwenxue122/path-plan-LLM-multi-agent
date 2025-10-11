# 项目结构说明

## 📁 项目整体结构

### 🎯 我的演示项目
- **`my_demos/`** - 我的双机器人导航演示
  - `robot_navigation_demo.py` - 主要演示脚本
  - `robot_navigation_animation.gif` - 机器人运动动画
  - `robot_navigation_trajectory.png` - 静态轨迹图
  - `README.md` - 演示说明文档

### 📦 原项目代码（完整保留）
- **`real_world/`** - 原项目核心代码
  - `nav_world/` - 导航世界环境
  - `runners/` - 运行器
  - `utils/` - 工具函数
- **`rocobench/`** - 原项目基准测试
- **`prompting/`** - 原项目提示模块
- **`data/`** - 原项目数据
- **`runs/`** - 原项目运行结果

### 📄 项目配置文件
- `README.md` - 项目说明（已更新，包含我的演示说明）
- `requirements.txt` - Python依赖
- `environment.yml` - Conda环境配置
- `setup.py` - 安装脚本
- `LICENSE` - 许可证

## 🎮 使用方法

### 运行我的演示
```bash
cd /Users/claire/co-robot-pathfinding
python my_demos/robot_navigation_demo.py
```

### 查看结果
```bash
open my_demos/robot_navigation_animation.gif
open my_demos/robot_navigation_trajectory.png
```

### 使用原项目功能
原项目的所有功能都完整保留，可以继续使用原项目的所有功能。

## ✅ 清理完成
- 删除了所有重复文件
- 保留了原项目完整代码
- 我的演示代码整理在 `my_demos/` 文件夹中
- 项目结构清晰，没有冗余文件
