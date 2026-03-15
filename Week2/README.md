# Week2: 机器人轨迹控制

本目录包含ROS包 `turtle_control`，实现了多种 turtlesim 机器人的轨迹控制方法。

## 项目结构

```
catkin_ws/
└── src/
    └── turtle_control/
        ├── src/
        │   ├── circle.py              # 开环圆形轨迹控制
        │   ├── hexagon.py             # 开环正六边形轨迹控制
        │   └── closed_loop_control.py # 闭环正六边形轨迹控制
        ├── CMakeLists.txt             # 构建配置
        └── package.xml                # 包依赖配置
```

## 功能说明

### 1. 开环圆形轨迹控制 (`circle.py`)

同时设置线速度和角速度，实现圆形轨迹。

**参数：**
- 线速度: 1.0 m/s
- 角速度: 1.0 rad/s
- 运行时间: 10 秒

**运行方式：**
```bash
rosrun turtle_control circle.py
```

### 2. 开环正六边形轨迹控制 (`hexagon.py`)

交替设置线速度和角速度，绘制正六边形。

**参数：**
- 线速度: 2.0 m/s
- 角速度: 1.0 rad/s
- 边长: 2.0 m
- 转角: 60° (π/3 弧度)

**运行方式：**
```bash
rosrun turtle_control hexagon.py
```

**局限性：**
- 累积误差：依赖预设时间参数，实际速度可能不一致
- 无法应对扰动：外部干扰会影响轨迹精度
- 缺乏实时反馈：无法根据当前状态调整控制

### 3. 闭环正六边形轨迹控制 (`closed_loop_control.py`)

基于实时位姿反馈的PID闭环控制，精确绘制正六边形。

**核心组件：**
- 位姿订阅者：订阅 `/turtle1/pose` 获取当前位姿
- 位置PID控制器：控制线速度，根据距离误差调整
- 角度PID控制器：控制角速度，根据角度误差调整
- 数据记录器：将控制数据记录到CSV文件

**PID参数：**
- 位置控制器：kp=1.0, ki=0.01, kd=0.1，线速度限制 2.0 m/s
- 角度控制器：kp=2.0, ki=0.05, kd=0.2，角速度限制 1.5 rad/s
- 目标到达阈值：0.05 m

**运行方式：**
```bash
rosrun turtle_control closed_loop_control.py
```

**数据记录：**
- 文件路径：`closed_loop_data.csv`
- 记录内容：时间戳、当前位姿、目标位姿、误差、控制输出、顶点索引

## 快速开始

### 1. 环境设置

```bash
cd /home/sunzj/Robot/Robot_test/Week2/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动 turtlesim

```bash
# 终端1
roscore

# 终端2
rosrun turtlesim turtlesim_node
```

### 3. 运行控制程序

根据需要选择以下任一程序：

```bash
# 圆形轨迹（开环）
rosrun turtle_control circle.py

# 正六边形轨迹（开环）
rosrun turtle_control hexagon.py

# 正六边形轨迹（闭环）
rosrun turtle_control closed_loop_control.py
```

## 开环 vs 闭环控制对比

| 维度 | 开环控制 | 闭环控制 |
|------|---------|---------|
| 轨迹精度 | 低，存在累积误差 | 高，实时反馈修正 |
| 抗干扰能力 | 无 | 强，自动补偿干扰 |
| 参数敏感性 | 高，依赖精确的时间参数 | 低，自动适应 |
| 实现复杂度 | 简单 | 较复杂 |
| 适用场景 | 简单运动，精度要求低 | 精确轨迹，复杂环境 |

## 依赖项

- ROS Noetic
- `geometry_msgs`
- `rospy`
- `std_msgs`
- `turtlesim`

## 控制策略详解

### 闭环控制算法

1. **计算目标顶点**：以(5.5, 5.5)为中心，计算6个六边形顶点
2. **计算误差**：
   - 位置误差 = 当前位置到目标顶点的距离
   - 角度误差 = 目标朝向角度 - 当前角度
3. **PID控制输出**：
   - 线速度 = 位置PID(位置误差)
   - 角速度 = 角度PID(角度误差)
4. **到达判断**：距离误差 < 0.05m 时切换到下一个顶点
5. **完成条件**：返回起点且满足阈值

### 数据分析建议

运行闭环控制后，可以使用以下Python代码分析CSV数据：

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('closed_loop_data.csv')

# 绘制轨迹
plt.figure(figsize=(10, 8))
plt.plot(df['current_x'], df['current_y'], 'b-', label='Actual Trajectory')
plt.scatter(df['target_x'], df['target_y'], c='r', label='Target Vertices')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('Closed-Loop Hexagon Trajectory')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()

# 误差分析
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.plot(df['position_error'])
plt.xlabel('Time Step')
plt.ylabel('Position Error (m)')
plt.title('Position Error over Time')

plt.subplot(1, 2, 2)
plt.plot(df['angle_error'])
plt.xlabel('Time Step')
plt.ylabel('Angle Error (rad)')
plt.title('Angle Error over Time')
plt.tight_layout()
plt.show()
```

## 扩展建议

1. **参数调优**：根据实际效果调整PID参数
2. **轨迹优化**：添加速度规划实现平滑运动
3. **多点导航**：扩展为任意路径规划
4. **障碍避障**：集成激光雷达数据和避障算法
5. **参数配置**：使用动态参数配置ROS dynamic_reconfigure

## 许可证

TODO