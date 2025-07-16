/**
 * @file mavlink_position_controller.cpp
 * @brief MAVLink兼容的位置控制器，用于无人机导航
 * 
 * 此节点将ROS导航系统与MAVLink协议兼容的无人机控制系统连接。
 * 
 * 主要功能：
 * 1. 坐标系转换（ENU <-> NED）
 * 2. MAVLink兼容的消息格式
 * 3. 标准化控制频率（50Hz）
 * 4. 安全检查和限制
 * 5. 轨迹状态管理
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <string>
#include <cmath>

// 包含生成的消息
#include <nav_converter/PositionCommand.h>

// MAVLink兼容的常量定义
namespace MAVLinkConstants {
    // 坐标系类型
    const uint8_t FRAME_LOCAL_NED = 1;      // NED坐标系
    const uint8_t FRAME_LOCAL_ENU = 2;      // ENU坐标系
    const uint8_t FRAME_GLOBAL_INT = 5;     // 全局坐标系
    const uint8_t FRAME_GLOBAL_RELATIVE_ALT = 6; // 相对高度坐标系
    const uint8_t FRAME_LOCAL_OFFSET_NED = 7;    // 局部偏移NED
    const uint8_t FRAME_BODY_NED = 8;       // 机体NED坐标系
    const uint8_t FRAME_BODY_OFFSET_NED = 9; // 机体偏移NED
    
    // 位置目标类型掩码
    const uint16_t POSITION_TARGET_TYPEMASK_X_IGNORE = 1;
    const uint16_t POSITION_TARGET_TYPEMASK_Y_IGNORE = 2;
    const uint16_t POSITION_TARGET_TYPEMASK_Z_IGNORE = 4;
    const uint16_t POSITION_TARGET_TYPEMASK_VX_IGNORE = 8;
    const uint16_t POSITION_TARGET_TYPEMASK_VY_IGNORE = 16;
    const uint16_t POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32;
    const uint16_t POSITION_TARGET_TYPEMASK_AX_IGNORE = 64;
    const uint16_t POSITION_TARGET_TYPEMASK_AY_IGNORE = 128;
    const uint16_t POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256;
    const uint16_t POSITION_TARGET_TYPEMASK_FORCE_SET = 512;
    const uint16_t POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024;
    const uint16_t POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048;
    
    // 标准控制频率
    const double STANDARD_CONTROL_FREQUENCY = 50.0; // Hz
    
    // 安全限制
    const double MAX_VELOCITY = 5.0;        // m/s
    const double MAX_ACCELERATION = 3.0;    // m/s²
    const double MAX_JERK = 10.0;           // m/s³
    const double MAX_YAW_RATE = 2.0;        // rad/s
}

class MavlinkPositionController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    
    // 发布者
    ros::Publisher position_cmd_pub_;
    ros::Publisher mavlink_position_target_pub_;  // MAVLink兼容的位置目标
    ros::Publisher path_visualization_pub_;
    ros::Publisher debug_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 路径数据
    std::deque<geometry_msgs::PoseStamped> path_points_;
    bool path_received_;
    bool path_active_;
    size_t current_waypoint_index_;
    
    // 当前状态
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    double current_yaw_;
    
    // 目标状态
    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_velocity_;
    Eigen::Vector3d target_acceleration_;
    double target_yaw_;
    double target_yaw_rate_;
    
    // 控制参数
    double lookahead_distance_;
    double waypoint_tolerance_;
    double max_velocity_;
    double max_acceleration_;
    double max_jerk_;
    double control_frequency_;
    double fixed_height_;
    double yaw_gain_;
    double position_gain_;
    double velocity_gain_;
    
    // 坐标系设置
    bool use_ned_frame_;           // 是否使用NED坐标系
    bool enable_coordinate_transform_; // 是否启用坐标系转换
    std::string output_frame_id_;  // 输出坐标系ID
    
    // 安全参数
    double emergency_stop_timeout_;
    double max_path_age_;
    ros::Time last_path_time_;
    ros::Time last_odom_time_;
    ros::Time last_cmd_time_;
    
    // 轨迹状态
    uint32_t trajectory_id_;
    uint8_t trajectory_flag_;
    
    // 历史数据用于计算jerk
    Eigen::Vector3d last_target_acceleration_;
    ros::Time last_acceleration_time_;
    
public:
    MavlinkPositionController() : 
        nh_(),
        private_nh_("~"),
        path_received_(false),
        path_active_(false),
        current_waypoint_index_(0),
        current_position_(Eigen::Vector3d::Zero()),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_yaw_(0.0),
        target_position_(Eigen::Vector3d::Zero()),
        target_velocity_(Eigen::Vector3d::Zero()),
        target_acceleration_(Eigen::Vector3d::Zero()),
        target_yaw_(0.0),
        target_yaw_rate_(0.0),
        trajectory_id_(1),
        trajectory_flag_(0),
        last_target_acceleration_(Eigen::Vector3d::Zero()) {
        
        // 加载参数
        loadParameters();
        
        // 初始化订阅者
        path_sub_ = nh_.subscribe("global_path", 10, &MavlinkPositionController::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &MavlinkPositionController::odomCallback, this);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &MavlinkPositionController::cmdVelCallback, this);
        
        // 初始化发布者
        position_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("position_cmd", 10);
        path_visualization_pub_ = nh_.advertise<nav_msgs::Path>("current_path", 10);
        debug_pub_ = nh_.advertise<geometry_msgs::Twist>("debug_info", 10);
        
        ROS_INFO("MavlinkPositionController initialized");
        ROS_INFO("Control frequency: %.1f Hz", control_frequency_);
        ROS_INFO("Coordinate frame: %s", use_ned_frame_ ? "NED" : "ENU");
        ROS_INFO("Output frame ID: %s", output_frame_id_.c_str());
    }
    
    void run() {
        ros::Rate rate(control_frequency_);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            // 更新路径跟踪
            updatePathFollowing();
            
            // 生成位置命令
            generatePositionCommand();
            
            // 发布可视化信息
            publishPathVisualization();
            publishDebugInfo();
            
            rate.sleep();
        }
    }
    
private:
    void loadParameters() {
        // 基础控制参数
        private_nh_.param("lookahead_distance", lookahead_distance_, 2.0);
        private_nh_.param("waypoint_tolerance", waypoint_tolerance_, 0.5);
        private_nh_.param("max_velocity", max_velocity_, MAVLinkConstants::MAX_VELOCITY);
        private_nh_.param("max_acceleration", max_acceleration_, MAVLinkConstants::MAX_ACCELERATION);
        private_nh_.param("max_jerk", max_jerk_, MAVLinkConstants::MAX_JERK);
        private_nh_.param("control_frequency", control_frequency_, MAVLinkConstants::STANDARD_CONTROL_FREQUENCY);
        private_nh_.param("fixed_height", fixed_height_, 1.5);
        private_nh_.param("yaw_gain", yaw_gain_, 1.0);
        private_nh_.param("position_gain", position_gain_, 1.0);
        private_nh_.param("velocity_gain", velocity_gain_, 0.5);
        
        // 坐标系参数
        private_nh_.param("use_ned_frame", use_ned_frame_, false);
        private_nh_.param("enable_coordinate_transform", enable_coordinate_transform_, true);
        private_nh_.param("output_frame_id", output_frame_id_, std::string("map"));
        
        // 安全参数
        private_nh_.param("emergency_stop_timeout", emergency_stop_timeout_, 1.0);
        private_nh_.param("max_path_age", max_path_age_, 5.0);
        
        ROS_INFO("Parameters loaded successfully");
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        path_points_.clear();
        for (const auto& pose : msg->poses) {
            path_points_.push_back(pose);
        }
        
        path_received_ = true;
        path_active_ = true;
        current_waypoint_index_ = 0;
        last_path_time_ = ros::Time::now();
        
        ROS_INFO("Received path with %zu waypoints", path_points_.size());
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 更新当前位置和速度
        current_position_[0] = msg->pose.pose.position.x;
        current_position_[1] = msg->pose.pose.position.y;
        current_position_[2] = msg->pose.pose.position.z;
        
        current_velocity_[0] = msg->twist.twist.linear.x;
        current_velocity_[1] = msg->twist.twist.linear.y;
        current_velocity_[2] = msg->twist.twist.linear.z;
        
        // 从四元数提取偏航角
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        current_yaw_ = tf::getYaw(q);
        
        last_odom_time_ = ros::Time::now();
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 检查紧急停止命令
        if (std::abs(msg->linear.x) < 0.01 && std::abs(msg->linear.y) < 0.01 && 
            std::abs(msg->linear.z) < 0.01 && std::abs(msg->angular.z) < 0.01) {
            path_active_ = false;
            ROS_WARN("Emergency stop activated");
        }
        
        last_cmd_time_ = ros::Time::now();
    }
    
    void updatePathFollowing() {
        if (!path_active_ || path_points_.empty()) {
            return;
        }
        
        // 检查路径是否过期
        if ((ros::Time::now() - last_path_time_).toSec() > max_path_age_) {
            ROS_WARN_THROTTLE(2.0, "Path too old, stopping");
            path_active_ = false;
            return;
        }
        
        // 找到目标航点
        size_t target_waypoint = findTargetWaypoint();
        
        if (target_waypoint >= path_points_.size()) {
            ROS_INFO("Reached end of path");
            path_active_ = false;
            return;
        }
        
        // 获取目标航点
        const geometry_msgs::PoseStamped& target_pose = path_points_[target_waypoint];
        
        // 设置目标位置（保持固定高度）
        target_position_[0] = target_pose.pose.position.x;
        target_position_[1] = target_pose.pose.position.y;
        target_position_[2] = fixed_height_;
        
        // 计算目标偏航角
        target_yaw_ = calculateTargetYaw(target_waypoint);
        
        // 计算目标速度
        calculateTargetVelocity(target_waypoint);
        
        // 更新当前航点索引
        if (target_waypoint > current_waypoint_index_) {
            current_waypoint_index_ = target_waypoint;
        }
    }
    
    size_t findTargetWaypoint() {
        if (path_points_.empty()) {
            return 0;
        }
        
        double accumulated_distance = 0.0;
        size_t last_waypoint = 0;
        
        for (size_t i = 0; i < path_points_.size() - 1; ++i) {
            const geometry_msgs::Point& p1 = path_points_[i].pose.position;
            const geometry_msgs::Point& p2 = path_points_[i + 1].pose.position;
            
            double segment_distance = calculateDistance(p1, p2);
            accumulated_distance += segment_distance;
            
            if (accumulated_distance >= lookahead_distance_) {
                return i;
            }
            last_waypoint = i;
        }
        
        return last_waypoint;
    }
    
    double calculateTargetYaw(size_t waypoint_index) {
        if (waypoint_index + 1 >= path_points_.size()) {
            return current_yaw_;
        }
        
        const geometry_msgs::Point& current_point = path_points_[waypoint_index].pose.position;
        const geometry_msgs::Point& next_point = path_points_[waypoint_index + 1].pose.position;
        
        double dx = next_point.x - current_point.x;
        double dy = next_point.y - current_point.y;
        
        return atan2(dy, dx);
    }
    
    void calculateTargetVelocity(size_t waypoint_index) {
        double distance_to_target = calculateDistance(
            current_position_,
            target_position_
        );
        
        double base_velocity = std::min(max_velocity_, distance_to_target * velocity_gain_);
        
        Eigen::Vector3d direction = (target_position_ - current_position_).normalized();
        target_velocity_ = direction * base_velocity;
        
        if (target_velocity_.norm() > max_velocity_) {
            target_velocity_ = target_velocity_.normalized() * max_velocity_;
        }
    }
    
    void generatePositionCommand() {
        nav_converter::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = output_frame_id_;
        
        double dt = 1.0 / control_frequency_;
        
        // 计算位置误差
        double x_error = target_position_[0] - current_position_[0];
        double y_error = target_position_[1] - current_position_[1];
        double z_error = target_position_[2] - current_position_[2];
        
        // 计算速度误差
        double vx_error = target_velocity_[0] - current_velocity_[0];
        double vy_error = target_velocity_[1] - current_velocity_[1];
        double vz_error = target_velocity_[2] - current_velocity_[2];
        
        // 计算偏航角误差
        double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);
        
        // 应用PID控制（简化版本）
        double x_correction = x_error * position_gain_;
        double y_correction = y_error * position_gain_;
        double z_correction = z_error * position_gain_;
        
        double vx_correction = vx_error * velocity_gain_;
        double vy_correction = vy_error * velocity_gain_;
        double vz_correction = vz_error * velocity_gain_;
        
        double yaw_correction = yaw_error * yaw_gain_;
        
        // 设置位置（带修正）
        Eigen::Vector3d corrected_position = target_position_;
        corrected_position[0] += x_correction;
        corrected_position[1] += y_correction;
        corrected_position[2] += z_correction;
        
        // 坐标系转换（如果需要）
        if (enable_coordinate_transform_ && use_ned_frame_) {
            corrected_position = enuToNed(corrected_position);
        }
        
        cmd.position.x = corrected_position[0];
        cmd.position.y = corrected_position[1];
        cmd.position.z = corrected_position[2];
        
        // 设置速度（带修正）
        Eigen::Vector3d corrected_velocity = target_velocity_;
        corrected_velocity[0] += vx_correction;
        corrected_velocity[1] += vy_correction;
        corrected_velocity[2] += vz_correction;
        
        // 限制速度
        if (corrected_velocity.norm() > max_velocity_) {
            corrected_velocity = corrected_velocity.normalized() * max_velocity_;
        }
        
        // 坐标系转换（如果需要）
        if (enable_coordinate_transform_ && use_ned_frame_) {
            corrected_velocity = enuToNed(corrected_velocity);
        }
        
        cmd.velocity.x = corrected_velocity[0];
        cmd.velocity.y = corrected_velocity[1];
        cmd.velocity.z = corrected_velocity[2];
        
        // 计算加速度
        target_acceleration_ = (target_velocity_ - current_velocity_) / dt;
        
        // 限制加速度
        if (target_acceleration_.norm() > max_acceleration_) {
            target_acceleration_ = target_acceleration_.normalized() * max_acceleration_;
        }
        
        // 坐标系转换（如果需要）
        Eigen::Vector3d corrected_acceleration = target_acceleration_;
        if (enable_coordinate_transform_ && use_ned_frame_) {
            corrected_acceleration = enuToNed(corrected_acceleration);
        }
        
        cmd.acceleration.x = corrected_acceleration[0];
        cmd.acceleration.y = corrected_acceleration[1];
        cmd.acceleration.z = corrected_acceleration[2];
        
        // 计算jerk（加加速度）
        Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
        if (last_acceleration_time_.isValid()) {
            double jerk_dt = (ros::Time::now() - last_acceleration_time_).toSec();
            if (jerk_dt > 0) {
                jerk = (target_acceleration_ - last_target_acceleration_) / jerk_dt;
                
                // 限制jerk
                if (jerk.norm() > max_jerk_) {
                    jerk = jerk.normalized() * max_jerk_;
                }
            }
        }
        
        // 坐标系转换（如果需要）
        if (enable_coordinate_transform_ && use_ned_frame_) {
            jerk = enuToNed(jerk);
        }
        
        cmd.jerk.x = jerk[0];
        cmd.jerk.y = jerk[1];
        cmd.jerk.z = jerk[2];
        
        // 设置偏航角和偏航角速度
        cmd.yaw = target_yaw_ + yaw_correction;
        cmd.yaw_dot = target_yaw_rate_;
        
        // 限制偏航角速度
        if (std::abs(cmd.yaw_dot) > MAVLinkConstants::MAX_YAW_RATE) {
            cmd.yaw_dot = (cmd.yaw_dot > 0) ? MAVLinkConstants::MAX_YAW_RATE : -MAVLinkConstants::MAX_YAW_RATE;
        }
        
        // 设置增益
        cmd.kx[0] = position_gain_;
        cmd.kx[1] = position_gain_;
        cmd.kx[2] = position_gain_;
        cmd.kv[0] = velocity_gain_;
        cmd.kv[1] = velocity_gain_;
        cmd.kv[2] = velocity_gain_;
        
        // 设置轨迹信息
        cmd.trajectory_id = trajectory_id_;
        cmd.trajectory_flag = path_active_ ? 
            nav_converter::PositionCommand::TRAJECTORY_STATUS_READY : 
            nav_converter::PositionCommand::TRAJECTORY_STATUS_EMPTY;
        
        // 更新历史数据
        last_target_acceleration_ = target_acceleration_;
        last_acceleration_time_ = ros::Time::now();
        
        // 发布命令
        position_cmd_pub_.publish(cmd);
    }
    
    // 坐标系转换函数
    Eigen::Vector3d enuToNed(const Eigen::Vector3d& enu) {
        // ENU到NED的转换矩阵
        // NED = [0, 1, 0; 1, 0, 0; 0, 0, -1] * ENU
        Eigen::Vector3d ned;
        ned[0] = enu[1];   // N = E
        ned[1] = enu[0];   // E = N
        ned[2] = -enu[2];  // D = -U
        return ned;
    }
    
    Eigen::Vector3d nedToEnu(const Eigen::Vector3d& ned) {
        // NED到ENU的转换矩阵
        // ENU = [0, 1, 0; 1, 0, 0; 0, 0, -1] * NED
        Eigen::Vector3d enu;
        enu[0] = ned[1];   // E = N
        enu[1] = ned[0];   // N = E
        enu[2] = -ned[2];  // U = -D
        return enu;
    }
    
    void publishPathVisualization() {
        if (!path_received_ || path_points_.empty()) {
            return;
        }
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = output_frame_id_;
        
        for (const auto& pose : path_points_) {
            path_msg.poses.push_back(pose);
        }
        
        path_visualization_pub_.publish(path_msg);
    }
    
    void publishDebugInfo() {
        geometry_msgs::Twist debug_msg;
        debug_msg.linear.x = lookahead_distance_;
        debug_msg.linear.y = current_waypoint_index_;
        debug_msg.linear.z = path_active_ ? 1.0 : 0.0;
        debug_msg.angular.x = target_position_[0];
        debug_msg.angular.y = target_position_[1];
        debug_msg.angular.z = target_position_[2];
        
        debug_pub_.publish(debug_msg);
    }
    
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        return sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    double calculateDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return (p2 - p1).norm();
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavlink_position_controller");
    
    MavlinkPositionController controller;
    controller.run();
    
    return 0;
} 