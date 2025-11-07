# 🎮 在MuJoCo 3D环境中查看MAPF路径规划

## 快速开始

### macOS用户（推荐）

```bash
cd /Users/claire/co-robot-pathfinding
mjpython mapf_mujoco_viewer.py
```

### Linux/Windows用户

```bash
cd /Users/claire/co-robot-pathfinding
python mapf_mujoco_viewer.py
```

---

## 功能说明

✅ **MAPF路径规划**: 使用优先级规划算法，确保无碰撞路径  
✅ **3D实时可视化**: 在MuJoCo 3D viewer中实时查看机器人移动  
✅ **交互式控制**: 
   - ESC键: 退出
   - 空格键: 暂停/继续（如果支持）
   - 鼠标: 旋转视角
   - 滚轮: 缩放

---

## 运行效果

运行后会看到：

1. **初始化阶段**:
   ```
   📦 初始化NavEnv with MAPF...
   ✅ 环境初始化完成
   🔄 使用MAPF规划路径...
   📊 MAPF路径规划结果:
      alice: 92步, 从(15,10)到(70,46), 时长91时间步
      bob: 94步, 从(15,50)到(72,20), 时长93时间步
   ```

2. **3D Viewer启动**:
   - 会打开一个3D窗口
   - 显示房间、障碍物、两个机器人（Alice和Bob）
   - 实时显示机器人按照MAPF规划的路径移动

3. **实时进度**:
   ```
   步骤   0/450: Alice距离=6.57m, Bob距离=6.44m
   步骤  30/450: Alice距离=6.17m, Bob距离=6.13m
   ...
   ✅ 任务完成！用时 12.3秒
   ```

---

## 如果3D Viewer无法启动

如果遇到问题，脚本会自动使用**离屏渲染**生成视频：

- 输出文件: `mapf_navigation_3d.mp4`
- 可以在视频播放器中查看

---

## 修改优先级顺序

如果想改变优先级（例如让Bob先规划），可以编辑 `mapf_mujoco_viewer.py`:

```python
env = NavEnvMAPF(
    xml_path=xml_path,
    grid_res=0.1,
    priority_order=['bob', 'alice']  # 改为Bob优先
)
```

---

## 故障排除

### 问题1: `mjpython` 命令不存在

**解决方案**:
```bash
# 安装MuJoCo viewer
pip install mujoco

# 或者使用普通python
python mapf_mujoco_viewer.py
```

### 问题2: Viewer闪退

**解决方案**:
- 确保MuJoCo版本 >= 2.3.0
- 尝试使用 `mjpython` 而不是 `python`
- 检查系统是否有足够的图形支持

### 问题3: 导入错误

**解决方案**:
```bash
# 确保在项目根目录
cd /Users/claire/co-robot-pathfinding

# 检查Python路径
python -c "import sys; print(sys.path)"
```

---

## 与2D可视化的区别

| 特性 | 2D可视化 (run_mapf_demo.py) | 3D可视化 (mapf_mujoco_viewer.py) |
|------|------------------------------|----------------------------------|
| 视角 | 俯视图 | 3D视角，可旋转 |
| 交互性 | 静态图片 | 实时交互 |
| 输出 | PNG图片 | 实时3D窗口（或MP4视频） |
| 适用场景 | 分析轨迹 | 实时观察和演示 |

---

## 下一步

- 查看2D轨迹图: `python real_world/nav_world/run_mapf_demo.py`
- 运行测试: `python real_world/nav_world/test_mapf.py`
- 查看文档: `cat real_world/nav_world/MAPF_README.md`

