#!/usr/bin/env bash

# 1. 启动Livox激光雷达驱动
echo "启动 livox_ros_driver2..."
roslaunch livox_ros_driver2 msg_MID360.launch &
LIVOX_PID=$!
sleep 5

# 2. 启动FAST-LIO定位
echo "启动 fast_lio_localization..."
roslaunch fast_lio_localization sentry_localize.launch &
LIO_PID=$!
sleep 5

# 3. 发布初始位姿
echo "发布初始位姿..."
rosrun fast_lio_localization publish_initial_pose.py 0 0 0 0 0 0
sleep 2

# 4. 启动move_base导航
echo "启动 sentry_nav..."
roslaunch sentry_nav sentry_movebase.launch &
NAV_PID=$!
sleep 5

# 5. 启动串口通信
echo "启动 sentry_serial..."
roslaunch sentry_serial sentry_serial.launch &
SERIAL_PID=$!
sleep 3

# 6. 检查TF树和topic
echo "检查TF树和topic发布情况..."
rosrun tf view_frames
sleep 2
if [ -f frames.pdf ]; then
    echo "TF树已生成(frames.pdf)，请用PDF查看器打开检查。"
else
    echo "TF树未生成，可能有TF发布问题。"
fi

echo "检查/map, /odom, /cmd_vel等topic是否正常发布："
rostopic list | grep -E "/map|/odom|/cmd_vel|/move_base"

# 7. 结束测试后，关闭所有节点
read -p "按回车键结束测试并关闭所有节点..."
kill $LIVOX_PID $LIO_PID $NAV_PID $SERIAL_PID

echo "测试结束。"