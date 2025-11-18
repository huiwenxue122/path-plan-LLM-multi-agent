# 🚀 MAPF路径规划运行指南

## 三种运行方式

### 方式1: 运行测试脚本（最简单）⭐

测试MAPF规划器的基本功能，包含可视化：

```bash
cd /Users/claire/co-robot-pathfinding/real_world/nav_world
python test_mapf.py
```

**输出：**
- ✅ 运行2个测试场景
- ✅ 验证路径无碰撞
- ✅ 生成可视化图片：`mapf_test_case1.png` 和 `mapf_test_case2.png`

---

### 方式2: 运行完整演示（推荐）⭐

运行完整的导航演示，使用MAPF规划：

```bash
cd /Users/claire/co-robot-pathfinding/real_world/nav_world
python run_mapf_demo.py
```

**功能：**
- ✅ 使用MAPF规划器规划路径
- ✅ 运行完整仿真
- ✅ 生成轨迹可视化：`mapf_navigation_result.png`
- ✅ 显示规划结果和进度

---

### 方式3: 运行NavEnv集成版本

直接运行NavEnvMAPF的demo：

```bash
cd /Users/claire/co-robot-pathfinding/real_world/nav_world
python nav_env_mapf.py
```

**功能：**
- ✅ 展示NavEnv与MAPF的集成
- ✅ 显示时间戳路径信息
- ✅ 运行仿真并显示进度

---

## 📝 在代码中使用MAPF

### 基本使用

```python
from real_world.nav_world.multi_agent_planner import plan_priority, AgentSpec
import numpy as np

# 定义网格
grid = np.array([
    [0, 0, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 0, 0]
])

# 定义智能体
agents = [
    AgentSpec(id='A', start=(0, 0), goal=(2, 3)),
    AgentSpec(id='B', start=(2, 0), goal=(0, 3)),
]

# 规划路径（A优先）
paths = plan_priority(grid, agents, order=['A', 'B'], max_time=50)

# 结果：paths['A'] 和 paths['B'] 都是时间戳路径
```

### 与NavEnv集成

```python
from real_world.nav_world.nav_env_mapf import NavEnvMAPF

# 创建环境
env = NavEnvMAPF(
    xml_path="room.xml",
    grid_res=0.1,
    priority_order=['alice', 'bob']  # Alice优先
)

# 使用MAPF规划重置
env.reset(use_mapf=True)

# 获取时间戳路径
mapf_paths = env.get_mapf_paths()

# 运行仿真
for step in range(100):
    obs, done = env.step(dt=0.02)
    if done:
        break
```

---

## 🎯 修改优先级顺序

```python
# 方法1: 创建时指定
env = NavEnvMAPF(xml_path="room.xml", priority_order=['bob', 'alice'])

# 方法2: 运行时修改
env.set_priority_order(['bob', 'alice'])
env.reset(use_mapf=True)
```

---

## 📊 查看结果

### 测试结果图片
```bash
# 查看测试可视化
open real_world/nav_world/mapf_test_case1.png
open real_world/nav_world/mapf_test_case2.png
```

### 演示结果图片
```bash
# 查看演示可视化
open mapf_navigation_result.png
```

---

## ⚙️ 参数说明

### `plan_priority()` 参数
- `grid`: 2D数组，0=可通行，1=障碍物
- `agents`: AgentSpec列表
- `order`: 优先级顺序（列表）
- `max_time`: 最大时间步数（默认200）

### `NavEnvMAPF` 参数
- `xml_path`: MuJoCo XML文件路径
- `grid_res`: 网格分辨率（米，默认0.1）
- `priority_order`: 优先级顺序（默认使用agent_names顺序）

---

## 🔍 故障排除

### 问题1: 找不到路径
```
RuntimeError: Failed to find path for agent 'X'
```

**解决方案：**
- 检查起点和终点是否可达
- 尝试调整优先级顺序
- 增加 `max_time` 参数

### 问题2: 导入错误
```
ModuleNotFoundError: No module named 'multi_agent_planner'
```

**解决方案：**
```bash
# 确保在正确的目录
cd /Users/claire/co-robot-pathfinding/real_world/nav_world

# 或者添加路径
export PYTHONPATH=/Users/claire/co-robot-pathfinding:$PYTHONPATH
```

---

## 📚 更多信息

查看详细文档：
```bash
cat real_world/nav_world/MAPF_README.md
```

