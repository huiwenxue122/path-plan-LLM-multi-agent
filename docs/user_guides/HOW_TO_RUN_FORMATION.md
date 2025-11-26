# 🎯 如何运行编队任务（Formation Tasks）

## 快速开始

> ⚙️ 依赖：请确保已安装 `pip install rvo2`（ORCA 控制器使用该库）。

### 方法1：直接运行 B 阵型 Demo（ORCA，推荐）

```bash
python nav_world/run_formation_B.py --viewer
```

该脚本会直接启动 ORCA 编队环境，让 12 个机器人从房间左侧移动到右侧并形成字母 **B**。

### 方法2：使用启动脚本（推荐，macOS）

```bash
./llm_interface/run_formation.sh
```

### 方法3：直接运行 Python 端到端脚本

```bash
python llm_interface/end_to_end_formation.py
```

### 方法4：使用 mjpython（macOS，确保3D查看器正常工作）

```bash
mjpython llm_interface/end_to_end_formation.py
```

---

## 使用流程

### 1. 启动程序

运行上述任一命令后，程序会显示：

```
🎯 End-to-End Natural Language Formation Control
======================================================================

This system:
  1. Parses natural language formation instructions using LLM
  2. Generates formation target positions
  3. Uses ORCA (rvo2) to compute collision-free velocities
  4. Executes formation in MuJoCo 3D environment
======================================================================

📝 Enter your formation instruction:
======================================================================
Examples:
  - "form the letter B"
  - "让机器人形成字母B"
  - "form letter B with 12 robots on the right side"
  - "让12个机器人形成字母B在右侧"
======================================================================
```

### 2. 输入自然语言命令

在提示符后输入你的命令，例如：

**英文命令：**
- `form the letter B`
- `form letter B with 12 robots on the right side`
- `form letter B with 10 robots`

**中文命令：**
- `让机器人形成字母B`
- `让12个机器人形成字母B在右侧`
- `形成字母B`

### 3. 系统自动执行

系统会：
1. **LLM解析**：使用 GPT-4o 解析你的命令
2. **生成编队**：计算字母 B 的目标位置
3. **ORCA 控制**：通过 rvo2 计算无碰撞速度
4. **3D 可视化**：在 MuJoCo 3D Viewer 中自动运行

### 4. 查看结果

- MuJoCo 3D Viewer会自动打开
- 你可以看到机器人从左侧出发，移动到右侧形成字母B
- 所有机器人到达目标位置后，程序会显示完成状态

---

## 示例命令

### 基本命令

```
form the letter B
```

**效果：**
- 使用12个机器人（默认）
- 在右侧形成字母B
- 自动规划路径并执行

### 指定机器人数量

```
form letter B with 10 robots
```

**效果：**
- 使用10个机器人
- 在右侧形成字母B

### 指定位置

```
form letter B on the right side
```

**效果：**
- 在右侧形成字母B（默认就是右侧）

### 中文命令

```
让12个机器人形成字母B在右侧
```

**效果：**
- 使用12个机器人
- 在右侧形成字母B

---

## 系统工作流程

```
用户输入自然语言命令
    ↓
[LLM解析] llm_formation_controller.py
    ↓
FormationTaskPlan {formation_type, num_robots, region}
    ↓
[生成编队目标] NavEnvFormationORCA.generate_letter_B
    ↓
字母B目标位置列表 [(x, y), ...]
    ↓
[初始化环境] nav_env_formation_orca.py
    ↓
N个机器人的环境（使用 ORCA 控制器）
    ↓
[ORCA 计算速度] nav_world/orca_controller.py
    ↓
每个机器人获得无碰撞速度
    ↓
[MuJoCo执行] 3D Viewer
    ↓
机器人形成字母B
```

---

## 故障排除

### 问题1：MuJoCo 3D Viewer无法打开

**解决方案：**
- macOS用户：使用 `mjpython` 或 `./llm_interface/run_formation.sh`
- 如果仍然失败，程序会自动切换到headless模式

### 问题2：LLM API调用失败

**解决方案：**
- 检查 `OPENAI_API_KEY` 是否设置
- 或者使用离线模式（修改代码中的 `use_offline_parser=True`）

### 问题3：机器人无法到达目标

**解决方案：**
- 检查目标位置是否在房间边界内
- 检查是否有足够的机器人数量（推荐10或12个）

---

## 技术细节

### 支持的任务类型

- ✅ **letter_b**: 形成字母 B（已实现）
- ⏳ **line**: 形成直线（待实现）
- ⏳ **circle**: 形成圆形（待实现）

### 机器人数量

- **推荐**：10~14 个机器人（字母 B 模板优化）
- **支持**：最多 20 个机器人（取决于 XML 中的数量）
- **默认**：12 个机器人

### 区域选项

- **right_side**: 右侧（默认）
- **left_side**: 左侧
- **center**: 中心

---

## 相关文件

- **ORCA 控制器**: `nav_world/orca_controller.py`
- **编队环境**: `nav_world/nav_env_formation_orca.py`
- **Demo 脚本**: `nav_world/run_formation_B.py`
- **LLM 端到端脚本**: `llm_interface/end_to_end_formation.py`
- **LLM 控制器**: `llm_interface/llm_formation_controller.py`
- **XML 文件**: `nav_world/room_formation.xml`

---

## 下一步

想要扩展功能？可以：
1. 添加新的编队类型（圆形、直线等）
2. 支持更多机器人数量
3. 添加更复杂的编队模式

