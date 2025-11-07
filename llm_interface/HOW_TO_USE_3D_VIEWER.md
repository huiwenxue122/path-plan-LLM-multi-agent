# 🎮 如何在MuJoCo 3D Viewer中查看LLM控制效果

## 🚀 快速开始（推荐方式）

### macOS用户（自动使用mjpython）

```bash
cd /Users/claire/co-robot-pathfinding
./llm_interface/run_with_viewer.sh
```

这个脚本会自动：
- ✅ 检测mjpython是否可用
- ✅ 使用mjpython运行（macOS必需）
- ✅ 自动打开3D Viewer

---

### 手动运行（如果脚本不工作）

```bash
cd /Users/claire/co-robot-pathfinding
mjpython llm_interface/end_to_end_navigation.py
```

然后输入命令，例如：
```
Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
```

---

## 📋 完整流程

1. **运行脚本**
   ```bash
   ./llm_interface/run_with_viewer.sh
   ```

2. **输入自然语言命令**
   ```
   Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
   ```

3. **系统会自动：**
   - ✅ 解析LLM命令
   - ✅ MAPF规划无碰撞路径
   - ✅ **自动打开MuJoCo 3D Viewer**
   - ✅ 实时显示机器人移动

4. **在3D Viewer中：**
   - 🖱️ **鼠标拖动**: 旋转视角
   - 🔍 **滚轮**: 缩放
   - ⌨️ **ESC键**: 退出

---

## 🔧 如果3D Viewer没有自动打开

### 问题1: 使用普通python运行

**错误信息：**
```
Viewer error: `launch_passive` requires that the Python script be run under `mjpython` on macOS
```

**解决方案：**
```bash
# 使用mjpython而不是python
mjpython llm_interface/end_to_end_navigation.py
```

### 问题2: mjpython未安装

**检查：**
```bash
which mjpython
```

**安装：**
```bash
pip install mujoco
```

如果还是没有，可能需要：
```bash
# 检查MuJoCo安装
python -c "import mujoco; print(mujoco.__version__)"

# 如果版本 < 2.3.0，更新
pip install --upgrade mujoco
```

### 问题3: 自动回退到headless模式

如果viewer无法启动，程序会自动：
- 使用headless模式运行
- 在终端显示进度
- 完成仿真

---

## 💡 推荐的工作流程

### 方式1: 使用便捷脚本（最简单）

```bash
./llm_interface/run_with_viewer.sh
```

### 方式2: 直接使用mjpython

```bash
mjpython llm_interface/end_to_end_navigation.py
```

### 方式3: 如果mjpython不可用

程序会自动检测并回退到headless模式，你仍然可以看到：
- LLM解析结果
- MAPF规划进度
- 仿真执行进度

---

## 🎯 验证3D Viewer是否工作

运行后，你应该看到：

1. **初始化信息**
   ```
   🎬 Starting MuJoCo 3D Viewer...
   ✅ 3D Viewer started!
   ```

2. **3D窗口打开**
   - 显示房间、障碍物、两个机器人
   - 可以鼠标拖动旋转视角

3. **实时执行**
   - 机器人按照MAPF规划的路径移动
   - 终端显示进度信息

---

## 📝 示例命令（已验证可用）

```
Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
```

更多命令示例：查看 `llm_interface/VALID_COMMANDS.md`

---

## 🛠️ 故障排除

### 问题: Viewer窗口打开但立即关闭

**可能原因：**
- MuJoCo版本问题
- 图形驱动问题

**解决方案：**
```bash
# 更新MuJoCo
pip install --upgrade mujoco

# 检查系统图形支持
python -c "import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type=\"plane\"/></worldbody></mujoco>'); print('OK')"
```

### 问题: 程序卡在"Starting MuJoCo 3D Viewer..."

**解决方案：**
- 等待5-10秒（viewer需要时间加载）
- 如果超过30秒，按Ctrl+C退出
- 尝试使用headless模式：`use_viewer=False`

---

## 📚 相关文档

- **查找可用命令**: `python llm_interface/find_valid_commands.py`
- **完整使用指南**: `llm_interface/README.md`
- **MAPF说明**: `MAPF_IMPLEMENTATION_EXPLAINED.md`

