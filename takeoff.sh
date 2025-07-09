#!/bin/bash

#floder=`ls ~/231021/log  -l| grep ^-|wc -l`
#sh ~/231021/nvidiajingjian/Fast-Drone-250_bkp/shfiles/pix_r.sh ~/231021/log/$floder.bag

rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
