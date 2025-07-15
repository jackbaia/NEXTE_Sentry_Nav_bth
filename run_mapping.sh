#!/bin/bash

# 用法: ./run_mapping.sh [地图保存路径] [rviz配置文件]
# 示例: ./run_mapping.sh ~/my_maps/map1 /path/to/your_mapping.rviz

source "$(cd "$(dirname "$0")"; pwd)/devel/setup.bash"

MAP_PATH=${1:-~/map}
RVIZ_CFG=${2:-""}

set -e

echo "==== 1. 启动建图流程 ===="
# 启动LIVOX驱动
roslaunch src/livox_ros_driver2/launch_ROS1/msg_MID360.launch &
LIVOX_PID=$!
sleep 2
# 启动FAST_LIO建图
roslaunch src/FAST_LIO_LOCALIZATION/launch/sentry_build_map.launch &
FASTLIO_PID=$!
sleep 2
# 处理rviz配置文件参数，未指定则用默认配置
if [ -z "$RVIZ_CFG" ]; then
  RVIZ_CFG="src/FAST_LIO/rviz_cfg/loam_livox.rviz"
fi
# 启动rviz
rviz -d "$RVIZ_CFG" &
RVIZ_PID=$!
# 等待rviz关闭
wait $RVIZ_PID
# rviz关闭后立即保存地图
echo "检测到rviz已关闭，自动保存地图..."
rosrun map_server map_saver -f "$MAP_PATH"
# 保存后kill建图相关节点
kill $LIVOX_PID $FASTLIO_PID

echo "建图流程完成，地图已保存到: $MAP_PATH" 