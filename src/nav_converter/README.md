# 无人机路径跟踪控制器

这个包提供了基于路径点的无人机位置控制器，用于跟踪move_base生成的全局路径规划结果。

## 功能特性

### 基础路径跟踪控制器 (`path_following_controller`)
- 订阅全局路径规划结果 (`/move_base1/NavfnROS/plan`)
- 基于前瞻距离的路径点跟踪
- 平滑轨迹生成
- 速度和加速度限制
- 安全检查和紧急停止

### 高级路径跟踪控制器 (`advanced_path_controller`)
- 包含基础控制器的所有功能
- PID控制器的位置和速度控制
- 路径平滑和插值
- 自适应前瞻距离
- 障碍物避免集成
- 紧急停止和安全功能

## 话题接口

### 订阅话题
- `global_path` (nav_msgs/Path): 全局路径规划结果
- `odom` (nav_msgs/Odometry): 无人机里程计信息
- `cmd_vel` (geometry_msgs/Twist): 速度命令（用于紧急停止检测）

### 发布话题
- `cmd` (nav_converter/PositionCommand): 位置控制命令
- `current_path` (nav_msgs/Path): 当前路径可视化
- `debug_info` (geometry_msgs/Twist): 调试信息

## 使用方法

### 1. 编译包
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动基础控制器
```bash
roslaunch nav_converter path_following_controller.launch
```

### 3. 启动高级控制器
```bash
roslaunch nav_converter tunnel_path_controller.launch
```

### 4. 与move_base集成
确保move_base正在运行并发布路径规划结果：
```bash
roslaunch sentry_nav sentry_movebase_tunnel.launch
```

## 参数配置

### 基础参数
- `lookahead_distance`: 前瞻距离 (默认: 2.0m)
- `waypoint_tolerance`: 路径点容差 (默认: 0.5m)
- `max_velocity`: 最大速度 (默认: 2.0m/s)
- `max_acceleration`: 最大加速度 (默认: 1.0m/s²)
- `control_frequency`: 控制频率 (默认: 50Hz)
- `fixed_height`: 固定飞行高度 (默认: 1.5m)

### 控制增益
- `yaw_gain`: 偏航控制增益 (默认: 1.0)
- `position_gain`: 位置控制增益 (默认: 1.0)
- `velocity_gain`: 速度控制增益 (默认: 0.5)

### 安全参数
- `emergency_stop_timeout`: 紧急停止超时 (默认: 1.0s)
- `max_path_age`: 路径最大年龄 (默认: 5.0s)

### 高级参数（仅高级控制器）
- `enable_path_smoothing`: 启用路径平滑 (默认: true)
- `smoothing_weight`: 平滑权重 (默认: 0.1)
- `smoothing_iterations`: 平滑迭代次数 (默认: 10)
- `adaptive_lookahead`: 自适应前瞻距离 (默认: true)
- `min_lookahead`: 最小前瞻距离 (默认: 1.0m)
- `max_lookahead`: 最大前瞻距离 (默认: 5.0m)

## 控制算法

### 路径跟踪算法
1. **前瞻点选择**: 基于前瞻距离选择目标路径点
2. **位置控制**: 计算当前位置到目标位置的方向和距离
3. **速度规划**: 根据距离和最大速度限制计算目标速度
4. **轨迹平滑**: 应用平滑因子减少轨迹抖动

### PID控制（高级控制器）
- **位置PID**: 控制X、Y、Z轴位置误差
- **速度PID**: 控制X、Y、Z轴速度误差
- **偏航PID**: 控制偏航角误差

### 安全机制
- **路径年龄检查**: 如果路径太旧则停止跟踪
- **紧急停止**: 检测零速度命令并执行紧急停止
- **速度和加速度限制**: 确保命令在安全范围内

## 调试和可视化

### RViz可视化
在RViz中添加以下显示：
- Path: 订阅 `/current_path` 查看当前路径
- Pose: 订阅 `/position_cmd` 查看目标位置

### 调试信息
- 订阅 `/debug_info` 话题获取调试信息：
  - `linear.x`: 当前前瞻距离
  - `linear.y`: 当前路径点索引
  - `linear.z`: 路径跟踪状态 (1=活跃, 0=停止)
  - `angular.x/y/z`: 目标位置

## 故障排除

### 常见问题
1. **控制器不响应**: 检查是否接收到路径数据
2. **轨迹不平滑**: 调整 `smoothing_factor` 参数
3. **跟踪精度差**: 调整PID增益参数
4. **速度过快**: 降低 `max_velocity` 参数

### 日志信息
控制器会输出以下日志：
- 参数加载信息
- 路径接收确认
- 路径跟踪状态
- 错误和警告信息

## 扩展功能

### 自定义控制器
可以通过继承基础控制器类来实现自定义功能：
- 添加障碍物避免
- 实现更复杂的路径平滑算法
- 集成其他传感器数据

### 参数动态调整
使用 `dynamic_reconfigure` 包实现运行时参数调整：
```bash
rosrun dynamic_reconfigure reconfigure_gui
``` 