#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云地图合并工具
用于将分段建图的多个.pcd文件合并成一个完整的地图
"""

import rospy
import sys
import os
import numpy as np
import pcl
import pcl_helper
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import threading
import time

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('pointcloud_merger', anonymous=True)
        
        # 参数
        self.segment_files = rospy.get_param('~segment_files', [])
        self.output_file = rospy.get_param('~output_file', '/home/bob/mid_ros/maps/tunnel_complete.pcd')
        self.voxel_size = rospy.get_param('~voxel_size', 0.3)
        self.overlap_threshold = rospy.get_param('~overlap_threshold', 0.1)
        
        # 发布器
        self.merged_cloud_pub = rospy.Publisher('/merged_pointcloud', PointCloud2, queue_size=1)
        
        rospy.loginfo("点云合并器初始化完成")
    
    def load_pointcloud_from_file(self, file_path):
        """从文件加载点云"""
        try:
            # 使用PCL加载点云
            cloud = pcl.load(file_path)
            rospy.loginfo(f"成功加载点云文件: {file_path}, 点数: {cloud.size}")
            return cloud
        except Exception as e:
            rospy.logerr(f"加载点云文件 {file_path} 失败: {e}")
            return None
    
    def filter_pointcloud(self, cloud):
        """过滤点云（体素滤波）"""
        try:
            # 体素滤波
            voxel_filter = cloud.make_voxel_grid_filter()
            voxel_filter.set_leaf_size(self.voxel_size, self.voxel_size, self.voxel_size)
            filtered_cloud = voxel_filter.filter()
            
            # 统计离群点滤波
            outlier_filter = filtered_cloud.make_statistical_outlier_filter()
            outlier_filter.set_mean_k(50)
            outlier_filter.set_std_dev_mul_thresh(1.0)
            filtered_cloud = outlier_filter.filter()
            
            rospy.loginfo(f"过滤后点数: {filtered_cloud.size}")
            return filtered_cloud
        except Exception as e:
            rospy.logerr(f"点云过滤失败: {e}")
            return cloud
    
    def merge_pointclouds(self):
        """合并多个点云"""
        rospy.loginfo("开始合并点云地图...")
        
        if not self.segment_files:
            rospy.logerr("没有指定要合并的点云文件")
            return False
        
        merged_cloud = None
        
        for i, segment_file in enumerate(self.segment_files):
            rospy.loginfo(f"正在处理第 {i+1} 个点云文件: {segment_file}")
            
            # 加载当前段点云
            current_cloud = self.load_pointcloud_from_file(segment_file)
            if current_cloud is None:
                continue
            
            # 过滤点云
            filtered_cloud = self.filter_pointcloud(current_cloud)
            
            # 合并点云
            if merged_cloud is None:
                merged_cloud = filtered_cloud
                rospy.loginfo("使用第一个点云作为基础")
            else:
                # 合并点云
                merged_cloud = self.merge_two_pointclouds(merged_cloud, filtered_cloud)
                rospy.loginfo(f"已合并第 {i+1} 个点云")
        
        if merged_cloud is not None:
            # 最终过滤
            final_cloud = self.filter_pointcloud(merged_cloud)
            
            # 保存合并后的点云
            self.save_merged_pointcloud(final_cloud)
            rospy.loginfo(f"点云合并完成，已保存到: {self.output_file}")
            return True
        else:
            rospy.logerr("点云合并失败")
            return False
    
    def merge_two_pointclouds(self, cloud1, cloud2):
        """合并两个点云"""
        try:
            # 简单的点云合并
            merged_points = np.vstack([cloud1.to_array(), cloud2.to_array()])
            merged_cloud = pcl.PointCloud()
            merged_cloud.from_array(merged_points)
            
            rospy.loginfo(f"合并后总点数: {merged_cloud.size}")
            return merged_cloud
        except Exception as e:
            rospy.logerr(f"点云合并失败: {e}")
            return cloud1
    
    def save_merged_pointcloud(self, merged_cloud):
        """保存合并后的点云"""
        try:
            # 保存为PCD文件
            pcl.save(merged_cloud, self.output_file)
            
            # 发布合并后的点云
            cloud_msg = pcl_helper.pcl_to_ros(merged_cloud)
            cloud_msg.header.frame_id = "map"
            cloud_msg.header.stamp = rospy.Time.now()
            self.merged_cloud_pub.publish(cloud_msg)
            
            rospy.loginfo("合并点云已保存并发布")
        except Exception as e:
            rospy.logerr(f"保存合并点云失败: {e}")

def main():
    if len(sys.argv) < 3:
        print("用法: rosrun nav_converter merge_pointclouds.py <segment1.pcd> <segment2.pcd> [segment3.pcd ...] <output.pcd>")
        return
    
    # 解析命令行参数
    segment_files = sys.argv[1:-1]
    output_file = sys.argv[-1]
    
    # 设置ROS参数
    rospy.set_param('~segment_files', segment_files)
    rospy.set_param('~output_file', output_file)
    
    # 创建合并器
    merger = PointCloudMerger()
    
    # 执行合并
    success = merger.merge_pointclouds()
    
    if success:
        print("点云合并成功！")
    else:
        print("点云合并失败！")

if __name__ == '__main__':
    main() 