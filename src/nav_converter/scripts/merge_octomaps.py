#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Octomap地图合并工具
用于将分段建图的多个.bt文件合并成一个完整的地图
"""

import rospy
import sys
import os
import numpy as np
from octomap_msgs.msg import Octomap
from octomap_msgs.srv import GetOctomap
from std_srvs.srv import Empty
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import threading
import time

class OctomapMerger:
    def __init__(self):
        rospy.init_node('octomap_merger', anonymous=True)
        
        # 参数
        self.segment_files = rospy.get_param('~segment_files', [])
        self.output_file = rospy.get_param('~output_file', '/home/bob/mid_ros/maps/tunnel_complete.bt')
        self.resolution = rospy.get_param('~resolution', 0.3)
        self.overlap_threshold = rospy.get_param('~overlap_threshold', 0.1)
        
        # 服务客户端
        self.get_octomap_client = rospy.ServiceProxy('/octomap_server/get_octomap', GetOctomap)
        self.save_map_client = rospy.ServiceProxy('/octomap_server/save_map', Empty)
        
        # 发布器
        self.merged_map_pub = rospy.Publisher('/merged_octomap', Octomap, queue_size=1)
        
        rospy.loginfo("Octomap合并器初始化完成")
    
    def load_octomap_from_file(self, file_path):
        """从文件加载Octomap"""
        try:
            # 使用octomap_server加载地图
            os.system(f"rosrun octomap_server octomap_server_node {file_path}")
            time.sleep(2)  # 等待加载完成
            
            # 获取Octomap数据
            response = self.get_octomap_client()
            return response.map
        except Exception as e:
            rospy.logerr(f"加载地图文件 {file_path} 失败: {e}")
            return None
    
    def merge_octomaps(self):
        """合并多个Octomap"""
        rospy.loginfo("开始合并Octomap地图...")
        
        if not self.segment_files:
            rospy.logerr("没有指定要合并的地图文件")
            return False
        
        merged_octomap = None
        
        for i, segment_file in enumerate(self.segment_files):
            rospy.loginfo(f"正在处理第 {i+1} 个地图文件: {segment_file}")
            
            # 加载当前段地图
            current_octomap = self.load_octomap_from_file(segment_file)
            if current_octomap is None:
                continue
            
            # 合并地图
            if merged_octomap is None:
                merged_octomap = current_octomap
                rospy.loginfo("使用第一个地图作为基础")
            else:
                # 简单的体素合并（实际应用中可能需要更复杂的配准）
                merged_octomap = self.merge_two_octomaps(merged_octomap, current_octomap)
                rospy.loginfo(f"已合并第 {i+1} 个地图")
        
        if merged_octomap is not None:
            # 保存合并后的地图
            self.save_merged_map(merged_octomap)
            rospy.loginfo(f"地图合并完成，已保存到: {self.output_file}")
            return True
        else:
            rospy.logerr("地图合并失败")
            return False
    
    def merge_two_octomaps(self, octomap1, octomap2):
        """合并两个Octomap（简化版本）"""
        # 这里实现简单的体素合并逻辑
        # 实际应用中可能需要更复杂的空间配准
        
        # 创建新的合并Octomap
        merged = Octomap()
        merged.header = octomap1.header
        merged.binary = octomap1.binary
        merged.id = octomap1.id
        merged.resolution = self.resolution
        merged.data = octomap1.data + octomap2.data  # 简单的数据合并
        
        return merged
    
    def save_merged_map(self, merged_octomap):
        """保存合并后的地图"""
        try:
            # 发布合并后的地图
            self.merged_map_pub.publish(merged_octomap)
            
            # 保存到文件
            os.system(f"rosservice call /octomap_server/save_map 'filename: {self.output_file}'")
            
            rospy.loginfo("合并地图已保存")
        except Exception as e:
            rospy.logerr(f"保存合并地图失败: {e}")

def main():
    if len(sys.argv) < 3:
        print("用法: rosrun nav_converter merge_octomaps.py <segment1.bt> <segment2.bt> [segment3.bt ...] <output.bt>")
        return
    
    # 解析命令行参数
    segment_files = sys.argv[1:-1]
    output_file = sys.argv[-1]
    
    # 设置ROS参数
    rospy.set_param('~segment_files', segment_files)
    rospy.set_param('~output_file', output_file)
    
    # 创建合并器
    merger = OctomapMerger()
    
    # 执行合并
    success = merger.merge_octomaps()
    
    if success:
        print("地图合并成功！")
    else:
        print("地图合并失败！")

if __name__ == '__main__':
    main() 