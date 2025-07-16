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
# 启动FAST_LIO建图（禁用内置RViz）
roslaunch src/FAST_LIO_LOCALIZATION/launch/sentry_build_map.launch rviz:=false &
FASTLIO_PID=$!
sleep 2
# 启动FAST_LIO三维建图（禁用内置RViz）
roslaunch src/FAST_LIO/launch/mapping_mid360.launch rviz:=false &
FASTLIO_3D_PID=$!
sleep 2

# 手动启动两个独立的RViz窗口
echo "启动第一个RViz窗口（二维建图）..."
gnome-terminal --tab --title="2D Mapping RViz" -- bash -c "rviz -d src/FAST_LIO_LOCALIZATION/rviz_cfg/sentry_build_map.rviz; exec bash" &
RVIZ_2D_PID=$!
sleep 1

echo "启动第二个RViz窗口（三维建图）..."
gnome-terminal --tab --title="3D Mapping RViz" -- bash -c "rviz -d src/FAST_LIO/rviz_cfg/loam_livox.rviz; exec bash" &
RVIZ_3D_PID=$!

echo "建图流程已启动，请手动保存地图"
echo "使用命令: rosrun map_server map_saver -f $MAP_PATH"
echo "按 Ctrl+C 停止所有节点"

# 等待用户中断
trap 'echo "正在停止所有节点..."; kill $LIVOX_PID $FASTLIO_PID $FASTLIO_3D_PID $RVIZ_2D_PID $RVIZ_3D_PID 2>/dev/null; exit' INT
wait 