#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import copy
import _thread
import time
import numpy as np
from scipy.spatial import KDTree
import rospy
import struct
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import tf
import tf.transformations as tf_trans

# Global variables
global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None

# Parameter configuration
MAP_VOXEL_SIZE = 0.4
SCAN_VOXEL_SIZE = 0.1
FREQ_LOCALIZATION = 0.5
LOCALIZATION_TH = 0.85
FOV = 6.28  # Field of view (radians)
FOV_FAR = 30  # Maximum distance (meters)

def pose_to_mat(pose_msg):
    """Convert ROS Pose message to 4x4 transformation matrix"""
    trans = tf_trans.translation_matrix([
        pose_msg.pose.pose.position.x,
        pose_msg.pose.pose.position.y,
        pose_msg.pose.pose.position.z
    ])
    rot = tf_trans.quaternion_matrix([
        pose_msg.pose.pose.orientation.x,
        pose_msg.pose.pose.orientation.y,
        pose_msg.pose.pose.orientation.z,
        pose_msg.pose.pose.orientation.w
    ])
    return np.dot(trans, rot)

def msg_to_array(pc_msg):
    """Convert PointCloud2 message to numpy array"""
    # 手动解析PointCloud2消息
    points = []
    for i in range(len(pc_msg.data)):
        if i % pc_msg.point_step == 0:
            point_data = pc_msg.data[i:i+pc_msg.point_step]
            x = struct.unpack('f', point_data[0:4])[0]
            y = struct.unpack('f', point_data[4:8])[0]
            z = struct.unpack('f', point_data[8:12])[0]
            intensity = struct.unpack('f', point_data[12:16])[0]
            points.append([x, y, z, intensity])
    
    if len(points) == 0:
        # 如果没有解析到点，返回一个小的占位符数组
        return np.zeros([100, 4])
    
    return np.array(points)

def voxel_down_sample(points, voxel_size):
    """Voxel downsampling to replace Open3D functionality"""
    if len(points) == 0:
        return points
    
    # Calculate which voxel grid each point belongs to
    voxel_coords = np.floor(points / voxel_size)
    
    # Find unique voxel grids
    unique_voxels, indices = np.unique(voxel_coords, axis=0, return_index=True)
    
    # Return the first point of each voxel grid
    return points[indices]

def icp_registration(source, target, initial_guess, max_iterations=20, tolerance=0.001):
    """简化版ICP配准算法"""
    prev_error = 0
    transformation = np.copy(initial_guess)
    
    # 构建KD树加速最近邻搜索
    kdtree = KDTree(target)
    
    for i in range(max_iterations):
        # 转换源点云
        transformed_source = np.dot(source, transformation[:3, :3].T) + transformation[:3, 3]
        
        # 查找最近邻
        distances, indices = kdtree.query(transformed_source)
        
        # 计算对应点对
        correspondences = target[indices]
        
        # 计算变换矩阵 (使用SVD分解)
        H = np.dot((transformed_source - transformed_source.mean(0)).T, 
                  (correspondences - correspondences.mean(0)))
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        
        # 处理反射情况
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        
        t = correspondences.mean(0) - np.dot(R, transformed_source.mean(0))
        
        # 更新变换矩阵
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = t
        
        # 计算误差
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    
    # 计算匹配分数 (1 / (1 + 平均误差))
    fitness = 1.0 / (1.0 + prev_error)
    return transformation, fitness

def registration_at_scale(pc_scan, pc_map, initial, scale):
    """多尺度配准"""
    # 下采样
    scan_down = voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale)
    map_down = voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale)
    
    # 执行ICP配准
    return icp_registration(scan_down, map_down, initial)

def inverse_se3(trans):
    """计算SE(3)变换的逆"""
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.dot(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse

def publish_point_cloud(publisher, header, pc):
    """发布点云消息"""
    # 创建PointCloud2消息
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pc)
    
    # 设置字段
    from sensor_msgs.msg import PointField
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    ]
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    
    # 填充数据
    msg.data = []
    for point in pc:
        msg.data.extend(struct.pack('ffff', point[0], point[1], point[2], point[3] if len(point) > 3 else 0.0))
    
    publisher.publish(msg)

def crop_global_map_in_FOV(global_map_points, pose_estimation, cur_odom):
    """裁剪视野范围内的全局地图"""
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.dot(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.column_stack([global_map_points, np.ones(len(global_map_points))])
    global_map_in_base_link = np.dot(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )[0]
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )[0]
    
    global_map_in_FOV = global_map_in_map[indices, :3]

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, global_map_in_FOV[::10])

    return global_map_in_FOV

def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom)

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: %.2f seconds' % (toc - tic))
    rospy.loginfo('Fitness score: %.4f' % fitness)

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf_trans.translation_from_matrix(T_map_to_odom)
        quat = tf_trans.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = cur_odom.header.stamp
        map_to_odom.header.frame_id = 'map'
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('Transformation:\n%s' % transformation)
        rospy.logwarn('Fitness score: %.4f' % fitness)
        return False

def initialize_global_map(pc_msg):
    """初始化全局地图"""
    global global_map
    global_map = msg_to_array(pc_msg)[:, :3]
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)
    rospy.loginfo('Global map received.')

def cb_save_cur_odom(odom_msg):
    """保存当前里程计"""
    global cur_odom
    cur_odom = odom_msg

def cb_save_cur_scan(pc_msg):
    """保存当前扫描"""
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为点云数组
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)
    cur_scan = pc[:, :3]

def thread_localization():
    """定期执行全局定位的线程"""
    global T_map_to_odom
    while not rospy.is_shutdown():
        # 每隔一段时间进行全局定位
        time.sleep(1.0 / FREQ_LOCALIZATION)
        global_localization(T_map_to_odom)

if __name__ == '__main__':
    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # 发布器
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    # 订阅器
    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))

    # 等待初始位姿
    while not initialized and not rospy.is_shutdown():
        rospy.logwarn('Waiting for initial pose....')
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan is not None:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('Initialize successfully!!!!!!')
    
    # 启动定期全局定位线程
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()
