#!/bin/bash

# 彩色输出
info() { echo -e "\033[1;36m$*\033[0m"; }
warn() { echo -e "\033[1;33m$*\033[0m"; }
err()  { echo -e "\033[1;31m$*\033[0m"; }
succ() { echo -e "\033[1;32m$*\033[0m"; }

# 默认参数
device="/dev/ttyACM0"
pcd_dir="/home/bob/mid_ros/src/FAST_LIO/PCD"
lidar_ip="192.168.1.183"

# 参数解析
temp_args=$(getopt -o '' --long port:,pcd-dir:,lidar-ip: -- "$@")
eval set -- "$temp_args"
while true; do
  case "$1" in
    --port) device="$2"; shift 2;;
    --pcd-dir) pcd_dir="$2"; shift 2;;
    --lidar-ip) lidar_ip="$2"; shift 2;;
    --) shift; break;;
    *) break;;
  esac
done

cmd="$1"

check_device() {
  if [ ! -e "$device" ]; then
    err "串口设备 $device 不存在！"
    exit 1
  fi
  sudo chmod 777 "$device"
}

check_lidar() {
  info "正在检测雷达IP连通性: $lidar_ip ..."
  if ping -c 2 -W 1 "$lidar_ip" >/dev/null 2>&1; then
    succ "雷达 $lidar_ip 可达。"
  else
    err "雷达 $lidar_ip 不可达，请检查网络连接和IP设置！"
    exit 1
  fi
}

confirm() {
  read -p "$1 [Y/n]: " yn
  case $yn in
    [Nn]*) return 1;;
    *) return 0;;
  esac
}

build_map() {
  info "=== 一键建图流程 ==="
  check_lidar
  confirm "是否启动mavros？" && roslaunch mavros px4.launch fcu_url:=$device
  confirm "是否启动飞控控制节点？" && roslaunch px4ctrl run_ctrl.launch
  confirm "是否启动雷达驱动？" && roslaunch livox_ros_driver2 launch_ROS1/msg_MID360.launch
  # 检查是否第一次建图
  mkdir -p "$pcd_dir"
  if ls "$pcd_dir"/scans* 1>/dev/null 2>&1; then
    warn "检测到已有地图数据，将自动备份旧数据..."
    for f in "$pcd_dir"/scans*; do
      mv "$f" "$f.bak"
    done
  fi
  read -p "请在rviz中确认点云，准备好后按回车继续..."
  roslaunch FAST_LIO mapping_mid360.launch
  roslaunch FAST_LIO_LOCALIZATION sentry_build_map.launch
  if confirm "建图完成后，是否保存地图？"; then
    read -p "请输入保存地图的路径 [默认: $pcd_dir/projected_map]: " map_path
    map_path=${map_path:-$pcd_dir/projected_map}
    rosrun map_server map_saver map:=$map_path -f "$pcd_dir/scans"
  fi
  succ "建图流程结束！"
}

localize() {
  info "=== 一键定位流程 ==="
  check_device
  roslaunch mavros px4.launch fcu_url:=$device
  roslaunch px4ctrl run_ctrl.launch
  roslaunch livox_ros_driver2 launch_ROS1/msg_MID360.launch
  roslaunch FAST_LIO_LOCALIZATION sentry_localize.launch
  confirm "是否执行起飞脚本？" && sh shfiles/takeoff.sh
  read -p "请确认无人机已进入board模式，准备好后按回车继续..."
  info "开始导航模块..."
  roslaunch FAST_LIO_LOCALIZATION sentry_movebase.launch
  roslaunch nav_converter advanced_path_controller.launch
  succ "定位流程结束！"
}

navigate() {
  info "=== 一键导航流程 ==="
  warn "导航流程请参考定位流程后续步骤..."
}

usage() {
  echo "用法: $0 [--port=串口] [--pcd-dir=地图目录] [--lidar-ip=雷达IP] <build-map|localize|navigate>"
  exit 1
}

case "$cmd" in
  build-map) build_map;;
  localize) localize;;
  navigate) navigate;;
  *) usage;;
esac 