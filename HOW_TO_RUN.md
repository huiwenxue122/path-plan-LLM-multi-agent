# 🚀 项目运行指南

## 📋 前置要求

### 1. 环境设置

```bash
# 进入项目目录
cd /Users/claire/co-robot-pathfinding

# 使用 conda 环境（推荐）
conda activate co-robot-py311  # 或你的环境名称

# 或者使用 Python 3.11+
python --version  # 应该显示 Python 3.11 或更高
```

### 2. 安装依赖

```bash
# 安装基础依赖
pip install -r requirements.txt

# 安装 MuJoCo（如果还没有）
pip install mujoco

# 安装 rvo2（用于 ORCA 碰撞避免）
# 如果 install_rvo2.sh 存在，可以运行：
bash install_rvo2.sh
# 或者手动安装（见 README）
```

### 3. 配置 OpenAI API Key（可选，用于 LLM 功能）

```bash
# 方式1: 使用 .env 文件（推荐）
echo "OPENAI_API_KEY=your-api-key-here" > .env

# 方式2: 使用 openai_key.json
# 编辑 openai_key.json，填入你的 API key

# 方式3: 环境变量
export OPENAI_API_KEY=your-api-key-here
```

**注意：** 如果没有 API key，系统会自动使用规则解析器（rule-based parser），功能仍然可用。

---

## 🎯 运行方式

### 方式1: 字母 B 编队演示（最简单）⭐

**功能：** 20 个机器人从左侧移动到右侧，排成字母 B 的形状

```bash
# 带 3D 可视化
python nav_world/run_formation_B.py --viewer

# 无可视化（更快）
python nav_world/run_formation_B.py

# 自定义参数
python nav_world/run_formation_B.py --viewer \
  --num-robots 20 \
  --sim-time 35.0 \
  --dt 0.05
```

**参数说明：**
- `--viewer`: 启动 MuJoCo 3D 可视化
- `--num-robots`: 机器人数量（默认：20）
- `--sim-time`: 最大仿真时间（秒，默认：35.0）
- `--dt`: 仿真时间步（秒，默认：0.05）

---

### 方式2: LLM 控制的编队（自然语言）⭐

**功能：** 使用自然语言命令控制机器人编队

```bash
# 使用命令行参数
python nav_world/run_formation_llm.py --viewer \
  --prompt "form a letter B on the right side"

# 交互式输入命令
python nav_world/run_formation_llm.py --viewer

# 更多示例命令
python nav_world/run_formation_llm.py --viewer \
  --prompt "move all robots to form B on the left"

python nav_world/run_formation_llm.py --viewer \
  --prompt "arrange robots in a B shape in the center"
```

**支持的命令示例：**
- `"form a letter B on the right side"`
- `"move all robots to form B on the left"`
- `"arrange robots in a B shape in the center"`
- `"form letter B with 20 robots on the right"`

**参数说明：**
- `--prompt`: 自然语言命令（如果不提供，会交互式输入）
- `--viewer`: 启动 MuJoCo 3D 可视化
- `--num-robots`: 机器人数量（默认：20）
- `--sim-time`: 最大仿真时间（秒，默认：35.0）
- `--dt`: 仿真时间步（秒，默认：0.05）

---

### 方式3: 双机器人 MAPF 导航（自然语言控制）

**功能：** 使用自然语言控制两个机器人（Alice 和 Bob）进行碰撞避免导航

```bash
# 使用脚本（推荐）
./llm_interface/run_with_viewer.sh

# 或直接运行
python llm_interface/end_to_end_navigation.py
```

**使用步骤：**
1. 运行脚本后，会提示输入自然语言命令
2. 输入命令，例如：
   ```
   Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
   ```
3. 系统会自动解析命令、规划路径并执行

**示例命令：**
- `Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority`
- `Alice goes to 3.0, 1.6. Bob goes to 3.2, -1.0. Alice first`
- `Robot A go to (2.0, 1.0), Robot B go to (2.0, -1.0), A has priority`

---

### 方式4: MAPF 路径规划演示

**功能：** 演示多智能体路径规划算法（2D 可视化）

```bash
python nav_world/run_mapf_demo.py
```

**输出：**
- 终端显示规划过程
- 生成轨迹可视化图片：`results/mapf_navigation_result.png`

---

### 方式5: 编队端到端（LLM 控制）

**功能：** 使用 LLM 控制编队任务

```bash
python llm_interface/end_to_end_formation.py
```

**示例命令：**
- `"form the letter B with 20 robots"`
- `"let all robots form a letter B on the right side"`

---

## 🎮 快速开始示例

### 第一次运行（推荐流程）

1. **测试字母 B 编队（最简单）**
   ```bash
   python nav_world/run_formation_B.py --viewer
   ```
   观察 20 个机器人从左侧移动到右侧，形成字母 B

2. **尝试 LLM 控制编队**
   ```bash
   python nav_world/run_formation_llm.py --viewer \
     --prompt "form a letter B on the right side"
   ```

3. **体验双机器人导航**
   ```bash
   python llm_interface/end_to_end_navigation.py
   ```
   输入：`Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority`

---

## 📊 查看结果

### 可视化文件位置

所有生成的可视化文件都在 `results/` 目录：

```bash
cd results
ls -la
```

**常见文件：**
- `letter_b_20_robots.png` - 20 机器人 B 形状预览
- `letter_b_preview.png` - B 形状预览图
- `mapf_navigation_result.png` - MAPF 导航轨迹图

### 查看图片

```bash
# macOS
open results/letter_b_20_robots.png

# Linux
xdg-open results/letter_b_20_robots.png
```

---

## ⚙️ 常见问题

### 问题1: 模块导入错误

```bash
# 确保在项目根目录运行
cd /Users/claire/co-robot-pathfinding

# 检查 Python 路径
python -c "import sys; print('\n'.join(sys.path))"
```

### 问题2: MuJoCo 相关错误

```bash
# 重新安装 MuJoCo
pip install --upgrade mujoco

# 检查 MuJoCo 版本
python -c "import mujoco; print(mujoco.__version__)"
```

### 问题3: rvo2 安装失败

```bash
# 使用提供的安装脚本
bash install_rvo2.sh

# 或查看 README 中的手动安装说明
```

### 问题4: LLM API 调用失败

- 系统会自动使用规则解析器（rule-based parser）
- 检查日志中的警告信息
- 功能仍然可用，只是不使用 LLM

### 问题5: 机器人翻倒

- 这个问题已经修复，机器人会始终保持竖直
- 如果仍然出现，检查 `nav_world/nav_env.py` 中的 `_reset_body_orientation` 方法

---

## 📚 更多文档

- **项目 README**: `README.md`
- **Formation 运行指南**: `docs/user_guides/HOW_TO_RUN_FORMATION.md`
- **MAPF 运行指南**: `docs/technical/HOW_TO_RUN_MAPF.md`
- **3D Viewer 使用**: `docs/user_guides/HOW_TO_USE_3D_VIEWER.md`

---

## ✅ 运行检查清单

- [ ] 已安装依赖：`pip install -r requirements.txt`
- [ ] 已安装 MuJoCo：`pip install mujoco`
- [ ] 在项目根目录：`cd /Users/claire/co-robot-pathfinding`
- [ ] （可选）已配置 OpenAI API key
- [ ] （可选）已安装 rvo2（用于 ORCA）

---

## 🎉 开始运行

最简单的开始方式：

```bash
cd /Users/claire/co-robot-pathfinding
python nav_world/run_formation_B.py --viewer
```

享受多机器人编队演示！🚀

