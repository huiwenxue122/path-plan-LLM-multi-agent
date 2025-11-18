# 📝 可用的LLM导航命令

## 🎯 推荐命令（已验证可用）

### 命令1: 使用默认目标位置（最推荐）⭐

```
Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
```

**说明：**
- Alice目标: (3.0, 1.6) - 右上角
- Bob目标: (3.2, -1.0) - 右下角
- 已验证：两个路径都可以找到

---

### 命令2: 简单目标（靠近起点）

```
Robot A go to (2.0, 1.0), Robot B go to (2.0, -1.0), A has priority
```

**说明：**
- 两个机器人都向右移动
- Alice向上，Bob向下
- 路径较短，规划更快

---

### 命令3: 不同高度的目标

```
Robot A go to (2.0, 0.3), Robot B go to (2.0, -2.5), A has priority
```

**说明：**
- Alice在中间偏上
- Bob在底部
- 避免路径冲突

---

### 命令4: 交换目标（有趣场景）

```
Robot A go to (3.2, -1.0), Robot B go to (3.0, 1.6), A has priority
```

**说明：**
- Alice和Bob交换默认目标
- 测试MAPF如何处理交叉路径

---

## 📊 环境信息

### 机器人初始位置
- **Alice**: (-2.5, -2.0) - 左下角
- **Bob**: (-2.5, 2.0) - 左上角

### 环境边界
- **X范围**: [-4.0, 4.0] 米
- **Y范围**: [-3.0, 3.0] 米

### 默认目标位置
- **Alice默认目标**: (3.0, 1.6) - 右上角
- **Bob默认目标**: (3.2, -1.0) - 右下角

---

## 🔍 如何找到更多可用命令

运行工具自动查找：

```bash
cd /Users/claire/co-robot-pathfinding
python llm_interface/find_valid_commands.py
```

这个工具会：
1. 测试环境中的各种位置
2. 找到可达的目标位置
3. 生成可用的命令示例

---

## 💡 命令格式

### 基本格式
```
Robot A go to (x, y), Robot B go to (x, y), A has priority
```

### 支持的变体
```
Alice goes to x, y. Bob goes to x, y. Alice first
A to (x, y), B to (x, y), A priority
Robot A: (x, y), Robot B: (x, y), A first
```

### 坐标格式
- 支持整数和小数：`(3, 2)` 或 `(3.0, 2.0)`
- 支持负坐标：`(-2, -1)`
- 空格可选：`(3,2)` 或 `(3, 2)`

---

## ⚠️ 注意事项

1. **坐标范围**: 确保目标在 [-4, 4] × [-3, 3] 范围内
2. **避开障碍物**: 不要使用被障碍物阻挡的位置
3. **优先级**: 第一个提到的机器人通常优先级更高
4. **距离**: 太远的目标可能需要更长的规划时间

---

## 🚀 快速测试

直接复制粘贴这些命令：

```bash
# 运行端到端系统
python llm_interface/end_to_end_navigation.py

# 然后输入（推荐）：
Robot A go to (3.0, 1.6), Robot B go to (3.2, -1.0), A has priority
```

---

## 📚 更多信息

- **查找工具**: `python llm_interface/find_valid_commands.py`
- **完整文档**: `llm_interface/README.md`
- **MAPF说明**: `../technical/MAPF_IMPLEMENTATION_EXPLAINED.md`

