# 🚀 项目运行指南

## 📋 前置要求

### 1. 安装依赖
```bash
cd /Users/claire/co-robot-pathfinding
pip install -r requirements.txt
```

### 2. 安装 MuJoCo（如果还没有）
```bash
pip install mujoco
```

### 3. macOS 用户：安装 mjpython（用于3D可视化）
```bash
# mjpython 通常随 MuJoCo 一起安装
# 如果没有，可以尝试：
pip install mujoco
```

### 4. 配置 OpenAI API Key（可选，用于LLM功能）
```bash
# 方式1: 使用 .env 文件（推荐）
echo "OPENAI_API_KEY=your-api-key-here" > .env

# 方式2: 使用 openai_key.json（已存在）
# 编辑 openai_key.json，填入你的API key
```

---

## 🎯 三种运行方式

### 方式1: 端到端自然语言导航（推荐）⭐

**功能：** 使用自然语言命令控制两个机器人，支持LLM解析、MAPF路径规划和3D可视化

**运行命令：**
```bash
cd /Users/claire/co-robot-pathfinding
./llm_interface/run_with_viewer.sh
```

**或者直接使用 Python：**
```bash
python llm_interface/end_to_end_navigation.py
```

**使用步骤：**
1. 运行脚本后，会提示输入自然语言命令
2. 输入命令，例如：
   ```
   Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
   ```
3. 系统会自动：
   - 解析命令（LLM或离线解析）
   - 规划碰撞避免路径（MAPF）
   - 在MuJoCo 3D环境中执行

**示例命令（已验证可用）：**
- `Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority`
- `Robot A go to (2.0, 1.0), Robot B go to (2.0, -1.0), A has priority`
- `Robot A go to (2.0, 0.3), Robot B go to (2.0, -2.5), A has priority`

**更多命令示例：** 查看 `llm_interface/VALID_COMMANDS.md`

---

### 方式2: MAPF路径规划演示

**功能：** 演示多智能体路径规划（MAPF）算法，生成2D可视化轨迹图

**运行命令：**
```bash
cd /Users/claire/co-robot-pathfinding
python nav_world/run_mapf_demo.py
```

**输出：**
- 在终端显示规划过程（包括每个智能体的路径长度和规划时间）
- 显示仿真进度（每30步显示一次距离目标的位置）
- 生成可视化图片：`mapf_navigation_result.png`（显示两个机器人的碰撞避免路径）

**运行示例输出：**
```
============================================================
MAPF多智能体路径规划演示
============================================================

📦 初始化NavEnv with MAPF...
✅ 环境初始化完成
   网格大小: (81, 61)
   智能体: ['alice', 'bob']
   优先级顺序: ['alice', 'bob']

🔄 使用MAPF规划重置环境...
[Priority Planning] Planning path for agent 'alice' (1/2)... ✅ 92 steps
[Priority Planning] Planning path for agent 'bob' (2/2)... ✅ 94 steps
✅ MAPF规划完成

📊 MAPF路径规划结果:
   alice: 路径长度: 92 步
   bob: 路径长度: 94 步

🏃 运行仿真...
   步骤   0: Alice距离=6.57m, Bob距离=6.44m
   ...
✅ 任务完成！在步骤 370 完成

🎨 生成可视化...
✅ 可视化已保存: mapf_navigation_result.png
```

**特点：**
- 使用优先级规划（Priority Planning）
- 自动避免顶点冲突和边冲突
- 支持等待动作
- 显示完整的规划过程和仿真进度

---

### 方式3: 2D Matplotlib 动画演示

**功能：** 使用Matplotlib生成2D动画，展示机器人导航轨迹

**运行命令：**
```bash
cd /Users/claire/co-robot-pathfinding
python my_demos/robot_navigation_demo.py
```

**输出：**
- 生成 `results/robot_navigation_animation.gif` - GIF动画
- 生成 `results/robot_navigation_trajectory.png` - 静态轨迹图

**特点：**
- 跨平台兼容（不需要mjpython）
- 适合生成演示视频
- 显示完整的运动轨迹

---

## 🔧 故障排除

### 问题1: mjpython 未找到（macOS）
```bash
# 解决方案1: 使用普通python
python llm_interface/end_to_end_navigation.py

# 解决方案2: 安装mujoco
pip install mujoco
```

### 问题2: 导入错误
```bash
# 确保在项目根目录运行
cd /Users/claire/co-robot-pathfinding

# 检查Python路径
python -c "import sys; print(sys.path)"
```

### 问题3: MuJoCo 3D Viewer 无法显示
- macOS: 使用 `mjpython` 而不是 `python`
- Linux/Windows: 直接使用 `python` 即可

### 问题4: LLM API 调用失败
- 检查 `openai_key.json` 或 `.env` 文件中的API key
- 或者使用离线模式（自动使用离线解析器）

---

## 📊 查看结果

### 可视化文件位置
所有生成的可视化文件都在 `results/` 目录：
```bash
cd /Users/claire/co-robot-pathfinding/results
ls -la
```

### 查看图片
```bash
# macOS
open results/robot_navigation_trajectory.png
open results/mapf_navigation_result.png

# Linux
xdg-open results/robot_navigation_trajectory.png
```

---

## 🎓 快速开始示例

### 第一次运行（推荐流程）

1. **测试基础功能（2D可视化）**
   ```bash
   python my_demos/robot_navigation_demo.py
   ```
   查看生成的GIF和轨迹图

2. **测试MAPF路径规划**
   ```bash
   python nav_world/run_mapf_demo.py
   ```
   查看碰撞避免路径规划

3. **体验完整功能（自然语言控制）**
   ```bash
   ./llm_interface/run_with_viewer.sh
   ```
   输入命令：`Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority`

---

## 📚 更多文档

- **LLM接口文档**: `llm_interface/README.md`
- **MAPF实现说明**: `MAPF_IMPLEMENTATION_EXPLAINED.md`
- **端到端导航指南**: `END_TO_END_NAVIGATION_GUIDE.md`
- **可用命令列表**: `llm_interface/VALID_COMMANDS.md`

---

## ✅ 运行检查清单

- [ ] 已安装依赖：`pip install -r requirements.txt`
- [ ] 已安装MuJoCo：`pip install mujoco`
- [ ] 在项目根目录：`cd /Users/claire/co-robot-pathfinding`
- [ ] （可选）已配置OpenAI API key

现在你可以开始运行项目了！🎉

