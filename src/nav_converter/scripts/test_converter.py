#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
nav_converter test script
This script publishes test velocity commands and monitors output
"""

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_converter.msg import PositionCommand

class ConverterTester:
    def __init__(self):
        rospy.init_node('converter_tester', anonymous=True)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Subscribers
        self.cmd_sub = rospy.Subscriber('/cmd', PositionCommand, self.cmd_callback)
        
        # Test parameters
        self.test_duration = 10.0  # seconds
        self.cmd_frequency = 10.0  # Hz
        self.odom_frequency = 50.0  # Hz
        
        # Test state
        self.start_time = None
        self.cmd_count = 0
        self.last_cmd_time = None
        
        rospy.loginfo("Converter tester initialized")
    
    def cmd_callback(self, msg):
        """Position command message callback function"""
        if self.last_cmd_time is None:
            self.last_cmd_time = rospy.Time.now()
        
        current_time = rospy.Time.now()
        dt = (current_time - self.last_cmd_time).to_sec()
        
        rospy.loginfo("Received position command:")
        rospy.loginfo("  Position: (%.2f, %.2f, %.2f)", 
                     msg.position.x, msg.position.y, msg.position.z)
        rospy.loginfo("  Velocity: (%.2f, %.2f, %.2f)", 
                     msg.velocity.x, msg.velocity.y, msg.velocity.z)
        rospy.loginfo("  Yaw: %.2f, Yaw rate: %.2f", msg.yaw, msg.yaw_dot)
        rospy.loginfo("  Time since last command: %.3f s", dt)
        
        self.last_cmd_time = current_time
    
    def publish_odometry(self):
        """Publish fake odometry data"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        
        # Set position (circular motion)
        t = (rospy.Time.now() - self.start_time).to_sec()
        radius = 2.0
        angular_vel = 0.5
        
        odom.pose.pose.position.x = radius * math.cos(angular_vel * t)
        odom.pose.pose.position.y = radius * math.sin(angular_vel * t)
        odom.pose.pose.position.z = 1.0
        
        # Set orientation (yaw follows circle)
        yaw = angular_vel * t + math.pi/2
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw/2)
        odom.pose.pose.orientation.w = math.cos(yaw/2)
        
        # Set velocity
        odom.twist.twist.linear.x = -radius * angular_vel * math.sin(angular_vel * t)
        odom.twist.twist.linear.y = radius * angular_vel * math.cos(angular_vel * t)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)
    
    def publish_test_commands(self):
        """Publish test velocity commands"""
        if self.start_time is None:
            self.start_time = rospy.Time.now()
            return
        
        t = (rospy.Time.now() - self.start_time).to_sec()
        
        # 创建不同的测试模式
        if t < 3.0:
            # 前进运动
            cmd = Twist()
            cmd.linear.x = 1.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
        elif t < 6.0:
            # 转向运动
            cmd = Twist()
            cmd.linear.x = 0.5
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.5
        elif t < 9.0:
            # 横向运动
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.5
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
        else:
            # 停止
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
        self.cmd_count += 1
        
        rospy.loginfo("发布速度命令 %d: (%.2f, %.2f, %.2f, %.2f)", 
                     self.cmd_count, cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z)
    
    def run_test(self):
        """运行完整测试"""
        rospy.loginfo("开始转换器测试...")
        
        # 等待转换器准备就绪
        rospy.sleep(2.0)
        
        # 设置定时器
        cmd_timer = rospy.Timer(rospy.Duration(1.0/self.cmd_frequency), 
                               lambda event: self.publish_test_commands())
        odom_timer = rospy.Timer(rospy.Duration(1.0/self.odom_frequency), 
                                lambda event: self.publish_odometry())
        
        # 运行测试持续时间
        start_time = rospy.Time.now()
        rate = rospy.Rate(1.0)  # 1 Hz监控
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            
            if elapsed > self.test_duration:
                rospy.loginfo("测试完成。发布了 %d 个命令。", self.cmd_count)
                break
            
            rospy.loginfo("测试运行中... %.1f 秒已过", elapsed)
            rate.sleep()
        
        # 清理
        cmd_timer.shutdown()
        odom_timer.shutdown()
        
        # 发布停止命令
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("测试结束。发布了停止命令。")

if __name__ == '__main__':
    try:
        tester = ConverterTester()
        tester.run_test()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
    except Exception as e:
        rospy.logerr("测试失败: %s", str(e)) 