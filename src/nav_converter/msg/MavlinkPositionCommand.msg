# MAVLink兼容的位置命令消息
# 基于MAVLink的SET_POSITION_TARGET_LOCAL_NED消息格式

Header header

# 位置信息 (米)
geometry_msgs/Point position

# 速度信息 (米/秒)
geometry_msgs/Vector3 velocity

# 加速度信息 (米/秒²)
geometry_msgs/Vector3 acceleration

# 加加速度信息 (米/秒³)
geometry_msgs/Vector3 jerk

# 偏航角 (弧度)
float64 yaw

# 偏航角速度 (弧度/秒)
float64 yaw_dot

# 位置控制增益 [x, y, z]
float64[3] kx

# 速度控制增益 [x, y, z]
float64[3] kv

# 轨迹ID
uint32 trajectory_id

# 轨迹状态标志
uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTORY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7
uint8 trajectory_flag

# MAVLink兼容字段
# 坐标系类型 (参考MAVLink FRAME_* 常量)
uint8 coordinate_frame

# 位置目标类型掩码 (参考MAVLink POSITION_TARGET_TYPEMASK_* 常量)
uint16 type_mask

# 时间戳 (微秒)
uint64 time_usec

# 目标系统ID
uint8 target_system

# 目标组件ID
uint8 target_component

# 有效性标志
bool position_valid
bool velocity_valid
bool acceleration_valid
bool yaw_valid
bool yaw_rate_valid 