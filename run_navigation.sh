#!/bin/bash

# 用法: ./run_navigation.sh
# 示例: ./run_navigation.sh

source "$(cd "$(dirname "$0")"; pwd)/devel/setup.bash"

echo "==== 2. 启动导航（重定位）流程 ===="

# 检查roscore是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "错误: roscore未运行，请先启动roscore"
    echo "运行命令: roscore &"
    exit 1
fi

echo "启动 livox_ros_driver2/msg_MID360.launch..."
gnome-terminal --tab --title="Livox Driver" -- bash -c "roslaunch src/livox_ros_driver2/launch_ROS1/msg_MID360.launch; exec bash" &
LIVOX_PID=$!
sleep 3

echo "启动 FAST_LIO_LOCALIZATION/sentry_localize.launch..."
gnome-terminal --tab --title="FAST-LIO Localization" -- bash -c "roslaunch src/FAST_LIO_LOCALIZATION/launch/sentry_localize.launch; exec bash" &
LOCALIZATION_PID=$!
sleep 3

echo "启动 sentry_nav/sentry_movebase_tunnel.launch..."
gnome-terminal --tab --title="Move Base" -- bash -c "roslaunch src/sentry_nav/launch/sentry_movebase_tunnel.launch; exec bash" &
MOVE_BASE_PID=$!
sleep 3

echo "启动 nav_converter/advanced_path_controller.launch..."
gnome-terminal --tab --title="Path Controller" -- bash -c "roslaunch src/nav_converter/launch/advanced_path_controller.launch; exec bash" &
PATH_CONTROLLER_PID=$!
sleep 3





echo "==== 导航流程已全部启动 ===="
echo "已启动的节点PID:"
echo "  livox_ros_driver2: $LIVOX_PID"
echo "  FAST_LIO_LOCALIZATION: $LOCALIZATION_PID"
echo "  sentry_nav: $MOVE_BASE_PID"
echo "  nav_converter: $PATH_CONTROLLER_PID"
echo ""
echo "==== 请用rviz发布目标点 ===="
echo "按 Ctrl+C 停止所有节点"

# 等待用户中断
trap 'echo "正在停止所有节点..."; kill $LIVOX_PID $LOCALIZATION_PID $MOVE_BASE_PID $PATH_CONTROLLER_PID 2>/dev/null; exit' INT
wait 