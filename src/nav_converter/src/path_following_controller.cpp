/**
 * @file path_following_controller.cpp
 * @brief 无人机导航的路径跟踪控制器
 * 
 * 此节点订阅来自move_base的全局路径规划结果
 * 并生成平滑的位置命令用于无人机控制。
 * 
 * 功能特性：
 * 1. 具有前瞻距离的路径点跟踪
 * 2. 平滑轨迹生成
 * 3. 速度和加速度限制
 * 4. 安全检查和紧急停止
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <string>
#include <cmath>

// 包含生成的消息
#include <nav_converter/PositionCommand.h>

class PathFollowingController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber tunnel_path_sub_;      // 自定义隧道路径
    ros::Subscriber planner_status_sub_;   // 规划器状态
    
    // 发布者
    ros::Publisher position_cmd_pub_;
    ros::Publisher path_visualization_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 路径数据
    std::deque<geometry_msgs::PoseStamped> path_points_;
    std::deque<geometry_msgs::PoseStamped> tunnel_path_;      // 自定义隧道路径
    bool path_received_;
    bool path_active_;
    size_t current_waypoint_index_;
    bool use_custom_planner_;     // 是否使用自定义规划器
    std::string planner_status_;  // 规划器状态
    
    // 当前状态
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    double current_yaw_;
    
    // 目标状态
    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_velocity_;
    Eigen::Vector3d target_acceleration_;
    double target_yaw_;
    
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
    
    // 安全参数
    double emergency_stop_timeout_;
    double max_path_age_;
    ros::Time last_path_time_;
    ros::Time last_odom_time_;
    
    // 轨迹平滑
    Eigen::Vector3d last_target_position_;
    Eigen::Vector3d last_target_velocity_;
    double smoothing_factor_;
    
public:
    PathFollowingController() : 
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
        last_target_position_(Eigen::Vector3d::Zero()),
        last_target_velocity_(Eigen::Vector3d::Zero()),
        use_custom_planner_(true),
        planner_status_("idle") {
        
        // 加载参数
        loadParameters();
        
        // 初始化订阅者
        path_sub_ = nh_.subscribe("global_path", 10, &PathFollowingController::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &PathFollowingController::odomCallback, this);
        
        // 增强版规划器集成订阅者
        tunnel_path_sub_ = nh_.subscribe("/tunnel_path", 10, &PathFollowingController::tunnelPathCallback, this);
        planner_status_sub_ = nh_.subscribe("/tunnel_planner_status", 10, &PathFollowingController::plannerStatusCallback, this);
        
        // 初始化发布者
        position_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("cmd", 10);
        path_visualization_pub_ = nh_.advertise<nav_msgs::Path>("current_path", 10);
        
        ROS_INFO("PathFollowingController initialized");
    }
    
    void run() {
        ros::Rate rate(control_frequency_);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            // Check if necessary data is received
            if (!path_received_) {
                ROS_WARN_THROTTLE(5.0, "Waiting for path data...");
                rate.sleep();
                continue;
            }
            
            // 更新路径跟踪
            updatePathFollowing();
            
            // 生成并发布位置命令
            generatePositionCommand();
            
            // 发布可视化
            publishPathVisualization();
            
            rate.sleep();
        }
    }
    
private:
    void loadParameters() {
        // 加载参数，使用默认值
        private_nh_.param("lookahead_distance", lookahead_distance_, 2.0);
        private_nh_.param("waypoint_tolerance", waypoint_tolerance_, 0.5);
        private_nh_.param("max_velocity", max_velocity_, 2.0);
        private_nh_.param("max_acceleration", max_acceleration_, 1.0);
        private_nh_.param("max_jerk", max_jerk_, 2.0);
        private_nh_.param("control_frequency", control_frequency_, 50.0);
        private_nh_.param("fixed_height", fixed_height_, 1.5);
        private_nh_.param("yaw_gain", yaw_gain_, 1.0);
        private_nh_.param("position_gain", position_gain_, 1.0);
        private_nh_.param("velocity_gain", velocity_gain_, 0.5);
        private_nh_.param("emergency_stop_timeout", emergency_stop_timeout_, 1.0);
        private_nh_.param("max_path_age", max_path_age_, 5.0);
        private_nh_.param("smoothing_factor", smoothing_factor_, 0.1);
        
        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Lookahead distance: %.2f m", lookahead_distance_);
        ROS_INFO("  Waypoint tolerance: %.2f m", waypoint_tolerance_);
        ROS_INFO("  Max velocity: %.2f m/s", max_velocity_);
        ROS_INFO("  Max acceleration: %.2f m/s²", max_acceleration_);
        ROS_INFO("  Control frequency: %.1f Hz", control_frequency_);
        ROS_INFO("  Fixed height: %.2f m", fixed_height_);
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 存储接收到的路径
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
        // 从里程计更新当前状态
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
    
    void updatePathFollowing() {
        if (!path_active_ || path_points_.empty()) {
            return;
        }
        
        // Check if path is too old
        if ((ros::Time::now() - last_path_time_).toSec() > max_path_age_) {
            ROS_WARN_THROTTLE(2.0, "Path too old, stopping");
            path_active_ = false;
            return;
        }
        
        // 找到要跟踪的最佳航点
        size_t target_waypoint = findTargetWaypoint();
        
        if (target_waypoint >= path_points_.size()) {
            // Reached end of path
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
        
        // 计算目标偏航角（朝向下一航点）
        target_yaw_ = calculateTargetYaw(target_waypoint);
        
        // 根据到目标的距离计算目标速度
        calculateTargetVelocity(target_waypoint);
        
        // 应用平滑
        smoothTargets();
        
        // 更新当前航点索引
        if (target_waypoint > current_waypoint_index_) {
            current_waypoint_index_ = target_waypoint;
        }
    }
    
    size_t findTargetWaypoint() {
        if (path_points_.empty()) {
            return 0;
        }
        
        // 找到距离当前位置前瞻距离的航点
        double accumulated_distance = 0.0;
        size_t last_waypoint = 0;
        
        for (size_t i = 0; i < path_points_.size(); ++i) {
            if (i > 0) {
                double segment_distance = calculateDistance(
                    path_points_[i-1].pose.position,
                    path_points_[i].pose.position
                );
                accumulated_distance += segment_distance;
            }
            
            if (accumulated_distance >= lookahead_distance_) {
                return i;
            }
            last_waypoint = i;
        }
        
        return last_waypoint;
    }
    
    double calculateTargetYaw(size_t waypoint_index) {
        if (waypoint_index + 1 >= path_points_.size()) {
            // 最后一个航点，保持当前偏航角
            return current_yaw_;
        }
        
        // 计算朝向下一航点的偏航角
        const geometry_msgs::Point& current_point = path_points_[waypoint_index].pose.position;
        const geometry_msgs::Point& next_point = path_points_[waypoint_index + 1].pose.position;
        
        double dx = next_point.x - current_point.x;
        double dy = next_point.y - current_point.y;
        
        return atan2(dy, dx);
    }
    
    void calculateTargetVelocity(size_t waypoint_index) {
        // 计算到目标的距离
        double distance_to_target = calculateDistance(
            current_position_,
            target_position_
        );
        
        // 基于距离的基础速度
        double base_velocity = std::min(max_velocity_, distance_to_target * velocity_gain_);
        
        // 计算方向向量
        Eigen::Vector3d direction = (target_position_ - current_position_).normalized();
        
        // 设置目标速度
        target_velocity_ = direction * base_velocity;
        
        // 限制速度
        if (target_velocity_.norm() > max_velocity_) {
            target_velocity_ = target_velocity_.normalized() * max_velocity_;
        }
    }
    
    void smoothTargets() {
        // 平滑位置
        target_position_ = smoothing_factor_ * target_position_ + 
                          (1.0 - smoothing_factor_) * last_target_position_;
        
        // 平滑速度
        target_velocity_ = smoothing_factor_ * target_velocity_ + 
                          (1.0 - smoothing_factor_) * last_target_velocity_;
        
        // 存储用于下次迭代
        last_target_position_ = target_position_;
        last_target_velocity_ = target_velocity_;
    }
    
    void generatePositionCommand() {
        nav_converter::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        
        // 设置位置
        cmd.position.x = target_position_[0];
        cmd.position.y = target_position_[1];
        cmd.position.z = target_position_[2];
        
        // 设置速度
        cmd.velocity.x = target_velocity_[0];
        cmd.velocity.y = target_velocity_[1];
        cmd.velocity.z = target_velocity_[2];
        
        // 计算加速度（简单有限差分）
        double dt = 1.0 / control_frequency_;
        target_acceleration_ = (target_velocity_ - last_target_velocity_) / dt;
        
        // 限制加速度
        if (target_acceleration_.norm() > max_acceleration_) {
            target_acceleration_ = target_acceleration_.normalized() * max_acceleration_;
        }
        
        cmd.acceleration.x = target_acceleration_[0];
        cmd.acceleration.y = target_acceleration_[1];
        cmd.acceleration.z = target_acceleration_[2];
        
        // 设置加加速度（目前为零，可以从加速度计算）
        cmd.jerk.x = 0.0;
        cmd.jerk.y = 0.0;
        cmd.jerk.z = 0.0;
        
        // 设置偏航角
        cmd.yaw = target_yaw_;
        cmd.yaw_dot = 0.0; // 可以从偏航角速度计算
        
        // 设置增益
        cmd.kx[0] = position_gain_;
        cmd.kx[1] = position_gain_;
        cmd.kx[2] = position_gain_;
        cmd.kv[0] = velocity_gain_;
        cmd.kv[1] = velocity_gain_;
        cmd.kv[2] = velocity_gain_;
        
        // 设置轨迹信息
        cmd.trajectory_id = path_active_ ? 1 : 0;
        cmd.trajectory_flag = path_active_ ? 
            nav_converter::PositionCommand::TRAJECTORY_STATUS_READY : 
            nav_converter::PositionCommand::TRAJECTORY_STATUS_EMPTY;
        
        // 发布命令
        position_cmd_pub_.publish(cmd);
    }
    
    void publishPathVisualization() {
        if (!path_received_ || path_points_.empty()) {
            return;
        }
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        
        // 添加当前路径点
        for (const auto& pose : path_points_) {
            path_msg.poses.push_back(pose);
        }
        
        path_visualization_pub_.publish(path_msg);
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
    
    void tunnelPathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // 处理自定义隧道路径
        tunnel_path_.clear();
        for (const auto& pose : msg->poses) {
            tunnel_path_.push_back(pose);
        }
        
        // 如果启用自定义规划器，使用隧道路径
        if (use_custom_planner_ && !tunnel_path_.empty()) {
            path_points_ = tunnel_path_;
            path_received_ = true;
            path_active_ = true;
            current_waypoint_index_ = 0;
            last_path_time_ = ros::Time::now();
            
            ROS_INFO("PathFollowingController: Received tunnel path with %zu waypoints", tunnel_path_.size());
        }
    }
    
    void plannerStatusCallback(const std_msgs::Header::ConstPtr& msg) {
        // 处理规划器状态
        planner_status_ = msg->frame_id;
        
        // 根据状态决定使用哪个规划器
        if (planner_status_ == "error" || planner_status_ == "avoiding") {
            use_custom_planner_ = false;
            ROS_WARN("PathFollowingController: Switching to move_base planner due to custom planner status: %s", planner_status_.c_str());
        } else if (planner_status_ == "following" || planner_status_ == "planning") {
            use_custom_planner_ = true;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_following_controller");
    
    PathFollowingController controller;
    controller.run();
    
    return 0;
} 