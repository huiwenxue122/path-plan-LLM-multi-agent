# 🤖 LLM 如何控制路径规划 - 完整流程解析

## 📋 概述

这个项目使用 **GPT-4o** 将自然语言命令转换为结构化的路径规划任务，然后通过 **MAPF (Multi-Agent Path Finding)** 算法生成无碰撞路径。

---

## 🔄 完整流程

```
用户输入自然语言
    ↓
[步骤1] LLM 解析 (GPT-4o)
    ↓
[步骤2] 结构化任务计划 (TaskPlan)
    ↓
[步骤3] 设置目标位置和优先级
    ↓
[步骤4] MAPF 路径规划
    ↓
[步骤5] 执行路径 (MuJoCo 仿真)
```

---

## 📝 详细步骤

### 步骤 1: 自然语言输入 → LLM 解析

**用户输入示例：**
```
"Robot A go to (3, 2), Robot B go to (3.2, -1), A has priority"
```

**代码位置：** `llm_interface/llm_controller.py` → `llm_parse_instruction()`

**LLM 调用过程：**

1. **构建 System Prompt**（告诉 GPT-4o 如何解析）：
```python
system_prompt = """You are a navigation task planner for multi-robot systems.

Your task is to parse natural language instructions and output a JSON object:
{
    "task": "navigation",
    "agents": [
        {"id": "A", "goal": [x, y], "delay": 0.0},
        {"id": "B", "goal": [x, y], "delay": 0.0}
    ],
    "priority": ["A", "B"]
}

Rules:
1. Extract agent goals from the instruction (coordinates in meters)
2. Determine priority order from the instruction
3. Extract delay/wait times from the instruction
...
"""
```

2. **调用 GPT-4o API**：
```python
response = client.chat.completions.create(
    model="gpt-4o",
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": user_prompt}
    ],
    temperature=0.1,
    response_format={"type": "json_object"}  # 强制 JSON 输出
)
```

3. **GPT-4o 返回 JSON**：
```json
{
    "task": "navigation",
    "agents": [
        {"id": "A", "goal": [3.0, 2.0], "delay": 0.0},
        {"id": "B", "goal": [3.2, -1.0], "delay": 0.0}
    ],
    "priority": ["A", "B"]
}
```

4. **Pydantic 验证**：
```python
task_plan = TaskPlan(**json_data)  # 自动验证数据结构
```

**关键点：**
- ✅ LLM 理解自然语言（支持中英文）
- ✅ 提取坐标、优先级、延迟时间
- ✅ 输出结构化 JSON
- ✅ Pydantic 确保数据有效性

---

### 步骤 2: 结构化任务计划 (TaskPlan)

**代码位置：** `llm_interface/llm_controller.py` → `TaskPlan` 类

**数据结构：**
```python
class TaskPlan:
    task: str = "navigation"
    agents: List[AgentSpec]  # 每个智能体的目标和延迟
    priority: List[str]      # 优先级顺序，如 ["A", "B"]

class AgentSpec:
    id: str           # "A", "B", "alice", "bob"
    goal: List[float] # [x, y] 坐标（米）
    delay: float      # 延迟时间（秒）
```

**示例输出：**
```python
TaskPlan(
    task="navigation",
    agents=[
        AgentSpec(id="A", goal=[3.0, 2.0], delay=0.0),
        AgentSpec(id="B", goal=[3.2, -1.0], delay=0.0)
    ],
    priority=["A", "B"]
)
```

---

### 步骤 3: 设置目标位置和优先级

**代码位置：** `llm_interface/end_to_end_navigation.py` → `set_goals_from_plan()`

**过程：**

1. **映射智能体 ID**：
```python
# LLM 使用 "A"/"B"，NavEnv 使用 "alice"/"bob"
agent_id_map = {
    'A': 'alice',
    'B': 'bob',
    'alice': 'alice',
    'bob': 'bob'
}
```

2. **设置目标位置**：
```python
for agent_spec in task_plan.agents:
    nav_env_id = agent_id_map[agent_spec.id]
    goal_mapping[nav_env_id] = tuple(agent_spec.goal)  # (3.0, 2.0)
    agent_delays[nav_env_id] = agent_spec.delay         # 0.0
```

3. **更新环境**：
```python
self.env.goal_xy['alice'] = (3.0, 2.0)
self.env.goal_xy['bob'] = (3.2, -1.0)
```

---

### 步骤 4: MAPF 路径规划

**代码位置：** `llm_interface/end_to_end_navigation.py` → `plan_paths_with_mapf()`

**过程：**

1. **创建 MAPF 智能体规格**：
```python
mapf_agents = []
for agent_spec in task_plan.agents:
    nav_env_id = agent_id_map[agent_spec.id]
    
    # 获取当前位置（网格坐标）
    current_pos = self.env._get_body_xy(nav_env_id)
    start_grid = self.env._world2grid(current_pos)  # 世界坐标 → 网格坐标
    
    # 获取目标位置（网格坐标）
    goal_world = goal_mapping[nav_env_id]
    goal_grid = self.env._world2grid(goal_world)
    
    mapf_agents.append(MAPFAgentSpec(
        id=nav_env_id,
        start=start_grid,  # (x_grid, y_grid)
        goal=goal_grid     # (x_grid, y_grid)
    ))
```

2. **调用 MAPF 规划器**（使用优先级顺序）：
```python
# priority_order = ["A", "B"] → ["alice", "bob"]
paths = plan_priority(
    grid=self.env.occupancy_grid,
    agents=mapf_agents,
    order=priority_order,  # 高优先级先规划
    max_time=200
)
```

3. **MAPF 算法**（`nav_world/multi_agent_planner.py`）：
   - 按优先级顺序规划每个智能体
   - 使用时间扩展 A* 搜索
   - 检查预留表避免冲突
   - 返回时间戳路径：`[(x, y, t), (x, y, t+1), ...]`

**返回结果：**
```python
paths = {
    'alice': [(10, 20, 0), (11, 20, 1), (12, 20, 2), ...],
    'bob': [(10, 15, 0), (10, 16, 1), (11, 16, 2), ...]
}
```

---

### 步骤 5: 执行路径 (MuJoCo 仿真)

**代码位置：** `llm_interface/end_to_end_navigation.py` → `execute_in_mujoco()`

**过程：**

1. **将网格路径转换为世界坐标**：
```python
for agent_id, path_grid in paths.items():
    path_world = [self.env._grid2world(x, y) for x, y, t in path_grid]
    self.env.agents[agent_id].path_world = path_world
```

2. **执行仿真循环**：
```python
current_time = 0.0
for step in range(steps):
    # 检查延迟：如果 current_time < delay，智能体不移动
    obs, done = self.env.step(
        dt=dt,
        current_time=current_time,
        agent_delays=self.agent_delays
    )
    current_time += dt
    viewer.sync()  # 更新 3D 可视化
```

3. **智能体移动逻辑**（`nav_world/nav_env.py`）：
```python
def step(self, dt, current_time, agent_delays):
    for name in self.agent_names:
        delay = agent_delays.get(name, 0.0)
        
        # 如果还在延迟期间，不移动
        if current_time < delay:
            continue  # 智能体保持原地
        
        # 否则按照路径移动
        st = self.agents[name]
        if st.path_world:
            # 移动到路径的下一个点
            next_pos = st.path_world[st.path_ptr]
            # ... 移动逻辑
```

---

## 🎯 关键设计点

### 1. **LLM 的作用**
- **不是**直接规划路径
- **而是**将自然语言转换为结构化任务
- 提取：目标坐标、优先级、延迟时间

### 2. **MAPF 的作用**
- 接收结构化的任务（目标、优先级）
- 使用算法生成无碰撞路径
- 考虑空间和时间冲突

### 3. **数据流**
```
自然语言 → JSON → TaskPlan → MAPF输入 → 路径 → 执行
```

### 4. **错误处理**
- LLM 解析失败 → 使用离线解析器
- MAPF 规划失败 → 回退到独立 A* 规划
- 确保系统始终可用

---

## 📊 完整示例

### 输入：
```
"Robot A go to (3, 2), Robot B wait 5 minutes then go to (3.2, -1), A has priority"
```

### 步骤 1: LLM 解析
```json
{
    "task": "navigation",
    "agents": [
        {"id": "A", "goal": [3.0, 2.0], "delay": 0.0},
        {"id": "B", "goal": [3.2, -1.0], "delay": 300.0}
    ],
    "priority": ["A", "B"]
}
```

### 步骤 2: 设置目标
```python
goal_mapping = {
    'alice': (3.0, 2.0),
    'bob': (3.2, -1.0)
}
agent_delays = {
    'alice': 0.0,   # 立即出发
    'bob': 300.0   # 等待 5 分钟
}
```

### 步骤 3: MAPF 规划
```python
# Alice 先规划（优先级高）
alice_path = plan_path(start=(10, 20), goal=(30, 20), reservation_table=empty)

# Bob 后规划（避开 Alice 的路径）
bob_path = plan_path(start=(10, 15), goal=(32, -10), reservation_table=alice_reserved)
```

### 步骤 4: 执行
- t=0s: Alice 开始移动，Bob 等待
- t=1s: Alice 继续移动，Bob 等待
- ...
- t=300s: Bob 开始移动
- 两个智能体都到达目标

---

## 🔑 总结

**LLM 控制路径规划的核心思想：**

1. **LLM = 自然语言理解器**
   - 理解用户意图
   - 提取结构化信息（目标、优先级、延迟）

2. **MAPF = 路径规划算法**
   - 接收结构化输入
   - 生成无碰撞路径

3. **分离关注点**
   - LLM 负责"理解"
   - 算法负责"规划"
   - 各司其职，职责清晰

这种设计使得：
- ✅ 用户可以用自然语言控制
- ✅ 算法保证路径质量
- ✅ 系统模块化，易于维护


