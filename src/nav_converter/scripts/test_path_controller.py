#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Path following controller test script
Generate test paths and verify controller response
"""

import rospy
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from nav_converter.msg import PositionCommand
import tf.transformations as tf_trans

class PathControllerTester:
    def __init__(self):
        rospy.init_node('path_controller_tester', anonymous=True)
        
        # Publishers
        self.path_pub = rospy.Publisher('/move_base1/NavfnROS/plan', Path, queue_size=10)
        self.odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribers
        self.position_cmd_sub = rospy.Subscriber('/position_cmd', PositionCommand, self.position_cmd_callback)
        
        # State variables
        self.current_position = [0.0, 0.0, 1.5]  # Initial position
        self.current_yaw = 0.0
        self.test_path = None
        self.test_started = False
        self.last_cmd_time = rospy.Time.now()
        
        # Test parameters
        self.test_radius = 5.0
        self.test_height = 1.5
        self.path_update_rate = 1.0  # Hz
        
        rospy.loginfo("Path controller tester initialized")
        
    def generate_circular_path(self, center_x=0.0, center_y=0.0, radius=5.0, num_points=20):
        """Generate circular test path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        for i in range(num_points):
            angle = 2.0 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = self.test_height
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Calculate orientation (pointing to next point)
            next_angle = 2.0 * math.pi * (i + 1) / num_points
            yaw = next_angle + math.pi / 2.0  # Tangent direction
            
            quat = tf_trans.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path.poses.append(pose)
        
        return path
    
    def generate_square_path(self, center_x=0.0, center_y=0.0, size=4.0):
        """Generate square test path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        # Define square vertices
        corners = [
            (center_x - size/2, center_y - size/2),
            (center_x + size/2, center_y - size/2),
            (center_x + size/2, center_y + size/2),
            (center_x - size/2, center_y + size/2)
        ]
        
        for i, (x, y) in enumerate(corners):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.test_height
            
            # Calculate orientation (pointing to next corner)
            next_x, next_y = corners[(i + 1) % len(corners)]
            yaw = math.atan2(next_y - y, next_x - x)
            
            quat = tf_trans.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path.poses.append(pose)
        
        return path
    
    def generate_8_figure_path(self, center_x=0.0, center_y=0.0, radius=3.0, num_points=40):
        """Generate figure-8 test path"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        
        for i in range(num_points):
            t = 2.0 * math.pi * i / num_points
            x = center_x + radius * math.sin(t)
            y = center_y + radius * math.sin(t) * math.cos(t)
            z = self.test_height
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Calculate tangent direction
            dx = radius * math.cos(t)
            dy = radius * (math.cos(t)**2 - math.sin(t)**2)
            yaw = math.atan2(dy, dx)
            
            quat = tf_trans.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            path.poses.append(pose)
        
        return path
    
    def publish_odometry(self):
        """Publish simulated odometry data"""
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = rospy.Time.now()
        odom.child_frame_id = "body"
        
        # Set position
        odom.pose.pose.position.x = self.current_position[0]
        odom.pose.pose.position.y = self.current_position[1]
        odom.pose.pose.position.z = self.current_position[2]
        
        # Set orientation
        quat = tf_trans.quaternion_from_euler(0, 0, self.current_yaw)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Set velocity (simulated)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)
    
    def position_cmd_callback(self, msg):
        """Handle position command callback"""
        self.last_cmd_time = rospy.Time.now()
        
        # Update current position (simulated)
        self.current_position[0] = msg.position.x
        self.current_position[1] = msg.position.y
        self.current_position[2] = msg.position.z
        self.current_yaw = msg.yaw
        
        # Print command info
        rospy.loginfo_throttle(1.0, 
            f"Target position: ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}) "
            f"Yaw: {math.degrees(msg.yaw):.1f}°")
    
    def run_test(self, test_type="circle"):
        """Run test"""
        rospy.loginfo(f"Starting {test_type} path test")
        
        # Generate test path
        if test_type == "circle":
            self.test_path = self.generate_circular_path()
        elif test_type == "square":
            self.test_path = self.generate_square_path()
        elif test_type == "figure8":
            self.test_path = self.generate_8_figure_path()
        else:
            rospy.logerr(f"Unknown test type: {test_type}")
            return
        
        # 发布路径
        self.path_pub.publish(self.test_path)
        rospy.loginfo(f"已发布 {len(self.test_path.poses)} 个路径点")
        
        # 开始测试循环
        rate = rospy.Rate(10)  # 10Hz
        test_duration = rospy.Duration(60.0)  # 60秒测试
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < test_duration:
            # 发布里程计数据
            self.publish_odometry()
            
            # 定期重新发布路径
            if (rospy.Time.now() - start_time).to_sec() % 10 < 0.1:
                self.test_path.header.stamp = rospy.Time.now()
                self.path_pub.publish(self.test_path)
            
            # 检查控制器响应
            if (rospy.Time.now() - self.last_cmd_time).to_sec() > 5.0:
                rospy.logwarn("控制器未响应位置命令")
            
            rate.sleep()
        
        rospy.loginfo("测试完成")
    
    def emergency_stop_test(self):
        """测试紧急停止功能"""
        rospy.loginfo("开始紧急停止测试")
        
        # 生成简单路径
        self.test_path = self.generate_circular_path(radius=2.0, num_points=8)
        self.path_pub.publish(self.test_path)
        
        # 运行5秒后发送紧急停止命令
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time) < rospy.Duration(10.0):
            self.publish_odometry()
            
            # 5秒后发送紧急停止
            if (rospy.Time.now() - start_time).to_sec() > 5.0:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.linear.y = 0.0
                cmd_vel.linear.z = 0.0
                cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd_vel)
                rospy.loginfo("发送紧急停止命令")
            
            rate.sleep()
        
        rospy.loginfo("紧急停止测试完成")

def main():
    tester = PathControllerTester()
    
    # 等待控制器启动
    rospy.sleep(2.0)
    
    # 运行不同的测试
    tests = ["circle", "square", "figure8"]
    
    for test in tests:
        if rospy.is_shutdown():
            break
        
        rospy.loginfo(f"准备运行 {test} 测试...")
        rospy.sleep(2.0)
        
        tester.run_test(test)
        
        if not rospy.is_shutdown():
            rospy.loginfo("等待5秒后开始下一个测试...")
            rospy.sleep(5.0)
    
    # 运行紧急停止测试
    if not rospy.is_shutdown():
        rospy.loginfo("准备运行紧急停止测试...")
        rospy.sleep(2.0)
        tester.emergency_stop_test()
    
    rospy.loginfo("所有测试完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被中断")
    except Exception as e:
        rospy.logerr(f"测试出错: {e}") 