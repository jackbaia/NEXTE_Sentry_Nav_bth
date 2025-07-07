#!/bin/bash

# 设置 ROS 环境
source /opt/ros/noetic/setup.bash
# 如果你使用的是自己的工作空间（例如 ~/catkin_ws），可以修改为：
# source ~/catkin_ws/devel/setup.bash

# 启动 map_server 保存地图的命令
rosrun map_server map_saver map:=/projected_map -f "/home/bob/mid_ros/src/FAST_LIO/PCD/scans"

