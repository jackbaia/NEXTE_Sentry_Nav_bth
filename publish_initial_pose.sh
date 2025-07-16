#!/bin/bash

# 用法: ./publish_initial_pose.sh [x] [y] [z] [roll] [pitch] [yaw]
# 示例: ./publish_initial_pose.sh 0 0 0 0 0 0
# 示例: ./publish_initial_pose.sh 1.5 2.0 0.0 0.0 0.0 1.57

# 设置ROS环境
source devel/setup.bash

# 设置默认值
X=${1:-0}
Y=${2:-0}
Z=${3:-0}
ROLL=${4:-0}
PITCH=${5:-0}
YAW=${6:-0}

# 检查roscore是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "错误: roscore未运行，请先启动roscore"
    echo "运行命令: roscore &"
    exit 1
fi

echo "==== 发布初始位姿 ===="
echo "位置: x=$X, y=$Y, z=$Z"
echo "姿态: roll=$ROLL, pitch=$PITCH, yaw=$YAW"
echo ""

# 运行publish_initial_pose.py
echo "执行: rosrun fast_lio_localization publish_initial_pose.py $X $Y $Z $ROLL $PITCH $YAW"
rosrun fast_lio_localization publish_initial_pose.py $X $Y $Z $ROLL $PITCH $YAW

echo ""
echo "初始位姿发布完成" 