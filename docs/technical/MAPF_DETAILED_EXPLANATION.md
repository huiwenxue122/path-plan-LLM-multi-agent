# 🧠 MAPF实现详细讲解

## 📋 目录
1. [整体架构](#整体架构)
2. [核心概念](#核心概念)
3. [算法流程](#算法流程)
4. [关键代码解析](#关键代码解析)
5. [实际运行示例](#实际运行示例)
6. [算法复杂度分析](#算法复杂度分析)
7. [优势与局限](#优势与局限)

---

## 🏗️ 整体架构

### 文件结构

```
nav_world/
├── multi_agent_planner.py    # MAPF核心算法实现
├── nav_env_mapf.py           # MAPF与NavEnv的集成层
└── nav_env.py                # 基础导航环境（独立A*规划）
```

### 模块关系

```
┌─────────────────────────────────────────┐
│   llm_interface/end_to_end_navigation.py │
│   (用户命令 → LLM解析 → 调用MAPF)        │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│      nav_world/nav_env_mapf.py          │
│      (NavEnvMAPF类，集成MAPF规划)        │
└─────────────────┬───────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────┐
│   nav_world/multi_agent_planner.py      │
│   (ReservationTable + Time-Extended A*)│
└─────────────────────────────────────────┘
```

---

## 🎯 核心概念

### 1. 优先级规划 (Priority Planning)

**基本思想：**
- 多个智能体按照**优先级顺序**依次规划路径
- 高优先级智能体先规划，路径被记录到**预留表**中
- 低优先级智能体规划时，必须避开预留表中已占用的位置和时间

**为什么有效？**
- ✅ **简单高效**：时间复杂度低，适合实时应用
- ✅ **保证无碰撞**：如果所有智能体都能找到路径，保证无碰撞
- ✅ **支持等待**：智能体可以原地等待以避免冲突

**示例：**
```
优先级顺序：['alice', 'bob']

步骤1：Alice先规划
  - Alice找到路径：[(0,0,0), (0,1,1), (1,1,2), ...]
  - 路径被加入预留表

步骤2：Bob后规划
  - Bob规划时检查预留表
  - 发现(0,0,0)被Alice占用 → 跳过
  - 发现(0,1,1)被Alice占用 → 跳过
  - Bob选择其他路径或等待：[(2,0,0), (1,0,1), (0,0,2), ...]
```

---

### 2. 预留表 (Reservation Table) 🔑

**这是MAPF的核心机制！**

#### 数据结构

```python
class ReservationTable:
    def __init__(self):
        # 顶点预留：记录位置-时间占用 (x, y, t)
        self.vertices: Set[Tuple[int, int, int]] = set()
        
        # 边预留：记录移动边占用 (x1, y1, x2, y2, t)
        self.edges: Set[Tuple[int, int, int, int, int]] = set()
```

#### 工作原理

**顶点预留 (Vertex Reservation)：**
- 记录智能体在特定时间占据的位置
- 格式：`(x, y, t)` - 在时间t，位置(x,y)被占用
- 用于检测**顶点冲突**：两个智能体在同一时间占据同一位置

**边预留 (Edge Reservation)：**
- 记录智能体在特定时间的移动
- 格式：`(x1, y1, x2, y2, t)` - 在时间t，从(x1,y1)移动到(x2,y2)
- 用于检测**边冲突**：两个智能体在连续时间步交换位置

**示例：**
```python
# Alice的路径：[(0,0,0), (0,1,1), (1,1,2)]

# 预留表记录：
vertices = {
    (0, 0, 0),  # Alice在t=0时在(0,0)
    (0, 1, 1),  # Alice在t=1时在(0,1)
    (1, 1, 2),  # Alice在t=2时在(1,1)
}

edges = {
    (0, 0, 0, 1, 0),  # Alice在t=0时从(0,0)移动到(0,1)
    (0, 1, 1, 1, 1),  # Alice在t=1时从(0,1)移动到(1,1)
    (0, 1, 0, 0, 0),  # 反向边：用于检测交换冲突
    (1, 1, 0, 1, 1),  # 反向边
}
```

---

### 3. 时间扩展A* (Time-Extended A*)

#### 与标准A*的区别

| 特性 | 标准A* | 时间扩展A* |
|------|--------|------------|
| **节点格式** | `(x, y)` | `(x, y, t)` |
| **搜索空间** | 2D空间 | 3D时空 |
| **冲突检查** | 无 | 检查预留表 |
| **等待动作** | 无 | 支持原地等待 |
| **目标判断** | `(x, y) == goal` | `(x, y) == goal` (忽略时间) |

#### 关键特性

**1. 时间维度：**
- 节点从`(x, y)`扩展到`(x, y, t)`
- 搜索空间从2D变为3D：空间 × 时间

**2. 冲突检查：**
- 在扩展邻居节点时，检查预留表
- 如果位置或边被占用，跳过该节点

**3. 等待动作：**
- 智能体可以选择原地等待：`(x, y, t) → (x, y, t+1)`
- 等待代价较低（0.5），鼓励移动但允许等待

---

### 4. 冲突类型

#### 顶点冲突 (Vertex Conflict)

**定义：** 两个智能体在同一时间占据同一位置

**示例：**
```
Agent A: t=5 在 (3, 4)
Agent B: t=5 在 (3, 4)  ❌ 冲突！
```

**检测方法：**
```python
def has_vertex_conflict(self, x, y, t):
    return (x, y, t) in self.vertices  # 检查预留表
```

#### 边冲突 (Edge Conflict)

**定义：** 两个智能体在连续时间步交换位置

**示例：**
```
Agent A: t=5 在 (3,4), t=6 在 (4,4)
Agent B: t=5 在 (4,4), t=6 在 (3,4)  ❌ 冲突！
```

**为什么是冲突？**
- 如果两个智能体同时交换位置，它们会在中间位置相撞
- 即使它们不会在同一时间占据同一位置，这种交换也是不安全的

**检测方法：**
```python
def has_edge_conflict(self, x1, y1, x2, y2, t):
    # 检查反向边是否被占用
    return (x2, y2, x1, y1, t) in self.edges
```

---

## 🔄 算法流程

### 主函数：`plan_priority()`

```python
def plan_priority(grid, agents, order, max_time):
    """
    优先级规划主函数
    
    流程：
    1. 初始化预留表
    2. 按优先级顺序规划每个智能体
    3. 每个智能体规划后，将路径加入预留表
    4. 返回所有路径
    """
    # 1. 初始化
    reservation_table = ReservationTable()
    paths = {}
    
    # 2. 按优先级顺序规划
    for agent_id in order:  # ['alice', 'bob']
        agent = agent_dict[agent_id]
        
        # 3. 为当前智能体规划路径（避开已规划路径）
        path = astar_time_extended(
            grid=grid,
            start=agent.start,
            goal=agent.goal,
            reservation_table=reservation_table,  # 🔑 传入预留表
            max_time=max_time
        )
        
        # 4. 存储路径
        paths[agent_id] = path
        
        # 5. 🔑 关键：将路径加入预留表
        reservation_table.add_path(path)
    
    return paths
```

### 详细步骤示例

**场景：** 两个智能体，Alice优先级高于Bob

```
┌─────────────────────────────────────────────────────────┐
│ 步骤1：规划Alice的路径                                    │
├─────────────────────────────────────────────────────────┤
│ 1. 调用 astar_time_extended()                            │
│ 2. 预留表为空，Alice可以自由规划                          │
│ 3. 得到路径：[(0,0,0), (0,1,1), (1,1,2), (2,1,3), ...] │
│ 4. 将路径加入预留表：                                     │
│    vertices: {(0,0,0), (0,1,1), (1,1,2), (2,1,3), ...} │
│    edges: {(0,0,0,1,0), (0,1,1,1,1), ...}              │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ 步骤2：规划Bob的路径                                      │
├─────────────────────────────────────────────────────────┤
│ 1. 调用 astar_time_extended()                            │
│ 2. 预留表包含Alice的路径                                 │
│ 3. 检查每个候选位置：                                     │
│    * (0,0,0) → 检查预留表 → 被Alice占用 → 跳过          │
│    * (0,1,1) → 检查预留表 → 被Alice占用 → 跳过          │
│    * (1,0,0) → 检查预留表 → 未被占用 → 可以使用          │
│ 4. Bob会绕开Alice的路径，或等待Alice通过                  │
│ 5. 得到路径：[(2,0,0), (1,0,1), (0,0,2), ...]           │
│    (Bob在t=0和t=1时等待，t=2时Alice已通过(0,0))          │
│ 6. 将路径加入预留表                                       │
└─────────────────────────────────────────────────────────┘
```

---

## 💻 关键代码解析

### 1. ReservationTable 类

```python
class ReservationTable:
    """预留表：记录已占用的位置和时间"""
    
    def add_path(self, path):
        """
        将完整路径加入预留表
        
        关键点：
        1. 预留每个时间步的顶点
        2. 预留每个移动的边（正向和反向）
        3. 反向边用于检测交换冲突
        """
        for i, (x, y, t) in enumerate(path):
            # 预留顶点
            self.reserve_vertex(x, y, t)
            
            # 预留边（移动）
            if i < len(path) - 1:
                x_next, y_next, t_next = path[i + 1]
                # 正向边：从(x,y)到(x_next,y_next)在时间t
                self.reserve_edge(x, y, x_next, y_next, t)
                # 反向边：用于检测交换冲突
                self.reserve_edge(x_next, y_next, x, y, t)
```

**关键点：**
- ✅ 同时预留正向和反向边，用于检测交换冲突
- ✅ 使用Set数据结构，查找效率O(1)
- ✅ 路径一旦加入，后续智能体必须避开

---

### 2. astar_time_extended() 函数

```python
def astar_time_extended(grid, start, goal, reservation_table, max_time):
    """
    时间扩展A*搜索
    
    关键特性：
    1. 节点格式：(x, y, t) 而不是 (x, y)
    2. 在扩展邻居时检查冲突
    3. 支持等待动作
    """
    
    def get_neighbors(x, y, t):
        """
        获取有效邻居节点（包括等待动作）
        
        关键：在添加邻居前检查冲突
        """
        neighbors = []
        
        # 4方向移动
        for dx, dy in [(1,0), (-1,0), (0,1), (0,-1)]:
            nx, ny = x + dx, y + dy
            nt = t + 1
            
            # 🔑 关键：检查冲突
            if in_bounds((nx, ny)) and grid[nx, ny] == 0:
                # 检查顶点冲突
                if not reservation_table.has_vertex_conflict(nx, ny, nt):
                    # 检查边冲突
                    if not reservation_table.has_edge_conflict(x, y, nx, ny, t):
                        neighbors.append((nx, ny, nt))
        
        # 等待动作：原地停留
        if t + 1 < max_time:
            if not reservation_table.has_vertex_conflict(x, y, t + 1):
                neighbors.append((x, y, t + 1))
        
        return neighbors
    
    # A*搜索循环
    g = {(start[0], start[1], 0): 0}  # g-cost
    pq = [(heuristic(start[0], start[1]), start[0], start[1], 0)]  # 优先队列
    
    while pq:
        f_cost, x, y, t = heapq.heappop(pq)
        
        # 到达目标（忽略时间）
        if (x, y) == goal:
            return reconstruct_path()
        
        # 扩展邻居（已经过冲突检查）
        for nx, ny, nt in get_neighbors(x, y, t):
            # 计算代价
            move_cost = 1.0 if (nx, ny) != (x, y) else 0.5  # 等待代价更低
            alt_g = g[(x, y, t)] + move_cost
            
            # 更新路径
            if (nx, ny, nt) not in g or alt_g < g[(nx, ny, nt)]:
                g[(nx, ny, nt)] = alt_g
                f_new = alt_g + heuristic(nx, ny)
                heapq.heappush(pq, (f_new, nx, ny, nt))
```

**关键点：**
- ✅ 在`get_neighbors()`中检查冲突，而不是在扩展时检查
- ✅ 支持等待动作（原地停留），代价较低（0.5）
- ✅ 目标判断忽略时间：`(x, y) == goal`

---

### 3. plan_priority() 函数

```python
def plan_priority(grid, agents, order, max_time):
    """
    优先级规划主函数
    
    流程：
    1. 初始化预留表
    2. 按优先级顺序规划每个智能体
    3. 每个智能体规划后，将路径加入预留表
    4. 返回所有路径
    """
    reservation_table = ReservationTable()
    paths = {}
    
    # 按优先级顺序规划
    for idx, agent_id in enumerate(order):
        agent = agent_dict[agent_id]
        
        # 规划路径（避开已规划路径）
        path = astar_time_extended(
            grid=grid,
            start=agent.start,
            goal=agent.goal,
            reservation_table=reservation_table,  # 🔑 传入预留表
            max_time=max_time
        )
        
        if path is None:
            raise RuntimeError(f"Failed to find path for {agent_id}")
        
        # 存储路径
        paths[agent_id] = path
        
        # 🔑 关键：更新预留表
        reservation_table.add_path(path)
    
    return paths
```

**关键点：**
- ✅ 顺序规划：高优先级先规划
- ✅ 预留表累积：每个智能体的路径都会加入预留表
- ✅ 保证无碰撞：后续智能体必须避开已规划路径

---

## 🎬 实际运行示例

### 示例1：简单场景

**场景设置：**
- 网格大小：4×4
- 障碍物：无
- 智能体A：从(0,0)到(3,3)
- 智能体B：从(3,0)到(0,3)
- 优先级：A > B

**规划结果：**
```
Agent A路径（优先级1）：
  t=0: (0,0)
  t=1: (0,1)
  t=2: (0,2)
  t=3: (0,3)
  t=4: (1,3)
  t=5: (2,3)
  t=6: (3,3) ✅

Agent B路径（优先级2）：
  t=0: (3,0)
  t=1: (2,0)
  t=2: (1,0)
  t=3: (0,0)  # 等待A通过
  t=4: (0,1)  # 等待A通过
  t=5: (0,2)  # 等待A通过
  t=6: (0,3) ✅
```

**冲突检查：**
- ✅ 无顶点冲突：A和B从未在同一时间占据同一位置
- ✅ 无边冲突：A和B从未交换位置

---

### 示例2：复杂场景（有障碍物）

**场景设置：**
- 网格大小：5×5
- 障碍物：中间一行 (2,1), (2,2), (2,3)
- 智能体A：从(0,0)到(4,4)
- 智能体B：从(4,0)到(0,4)
- 优先级：A > B

**规划结果：**
```
Agent A路径：
  t=0: (0,0)
  t=1: (0,1)
  t=2: (1,1)
  t=3: (1,2)
  t=4: (1,3)
  t=5: (2,3)  # 绕过障碍物
  t=6: (3,3)
  t=7: (4,3)
  t=8: (4,4) ✅

Agent B路径：
  t=0: (4,0)
  t=1: (3,0)
  t=2: (2,0)
  t=3: (1,0)
  t=4: (0,0)  # 等待A通过
  t=5: (0,1)  # 等待A通过
  t=6: (0,2)
  t=7: (0,3)
  t=8: (0,4) ✅
```

---

## 📊 算法复杂度分析

### 时间复杂度

**单智能体规划：** O(n × m × T)
- n = 网格行数
- m = 网格列数
- T = 最大时间步数

**多智能体规划：** O(k × n × m × T)
- k = 智能体数量

**实际性能：**
- 对于100×100网格，T=200，k=2：约 2 × 100 × 100 × 200 = 4,000,000 次操作
- 使用优先队列（堆），实际运行时间通常远小于最坏情况

### 空间复杂度

**预留表：** O(k × T)
- 每个智能体的路径长度约为T
- k个智能体，总空间为O(k × T)

**A*搜索：** O(n × m × T)
- 最坏情况下需要存储所有节点

---

## ✅ 优势与局限

### 优势

1. **✅ 简单高效**
   - 实现简单，易于理解和维护
   - 运行速度快，适合实时应用

2. **✅ 保证无碰撞**
   - 如果所有智能体都能找到路径，保证无碰撞
   - 通过预留表机制，严格避免顶点和边冲突

3. **✅ 支持等待**
   - 智能体可以原地等待以避免冲突
   - 等待代价较低，鼓励移动但允许等待

4. **✅ 实时性**
   - 时间复杂度低，适合实时应用
   - 可以动态调整优先级

### 局限

1. **❌ 非最优**
   - 优先级顺序影响解的质量
   - 不同的优先级顺序可能得到不同的结果

2. **❌ 可能失败**
   - 如果低优先级智能体无法避开高优先级路径，规划失败
   - 需要调整优先级顺序或增加max_time

3. **❌ 顺序敏感**
   - 不同的优先级顺序可能得到不同的结果
   - 需要合理选择优先级顺序（可以由LLM决定）

---

## 🔍 调试技巧

### 查看预留表内容

```python
# 在plan_priority()中添加调试输出
print(f"Reservation table after {agent_id}:")
print(f"  Vertices: {len(reservation_table.vertices)}")
print(f"  Edges: {len(reservation_table.edges)}")
```

### 可视化路径

```python
# 使用run_mapf_demo.py可视化
python nav_world/run_mapf_demo.py
```

### 验证路径

```python
# 使用validate_paths()函数
is_valid, errors = validate_paths(paths, grid)
if not is_valid:
    for error in errors:
        print(error)
```

---

## 🚀 扩展方向

1. **动态优先级调整**
   - 根据情况动态改变优先级
   - 例如：距离目标更近的智能体优先级更高

2. **冲突解决策略**
   - 当规划失败时的回退机制
   - 例如：降低优先级或增加等待时间

3. **最优性保证**
   - 使用CBS (Conflict-Based Search) 等算法
   - 保证找到最优解（但计算成本更高）

4. **大规模优化**
   - 针对大量智能体的优化
   - 例如：分组规划、分层规划

---

## 💡 总结

MAPF路径规划的核心是：

1. **预留表机制**：记录已占用的位置和时间
2. **时间扩展A***：在时空空间中搜索
3. **优先级规划**：按顺序规划，后续智能体避开已规划路径

这种方法的优势是简单高效，适合实时应用，虽然不能保证最优解，但能保证无碰撞。

---

## 📚 相关文件

- **核心实现：** `nav_world/multi_agent_planner.py`
- **集成层：** `nav_world/nav_env_mapf.py`
- **使用示例：** `nav_world/run_mapf_demo.py`
- **LLM集成：** `llm_interface/end_to_end_navigation.py`

