/**
 * @file advanced_path_controller.cpp
 * @brief 具有PID控制和路径平滑的高级路径跟踪控制器
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <algorithm>

// 包含生成的消息
#include <nav_converter/PositionCommand.h>

class PIDController {
private:
    double kp_, ki_, kd_;
    double integral_;
    double last_error_;
    double integral_limit_;
    double output_limit_;
    bool first_run_;
    
public:
    PIDController(double kp, double ki, double kd, double integral_limit, double output_limit) :
        kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), output_limit_(output_limit),
        integral_(0.0), last_error_(0.0), first_run_(true) {}
    
    double compute(double error, double dt) {
        if (first_run_) {
            last_error_ = error;
            first_run_ = false;
        }
        
        // 比例项
        double p_term = kp_ * error;
        
        // 积分项
        integral_ += error * dt;
        integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));
        double i_term = ki_ * integral_;
        
        // 微分项
        double derivative = (error - last_error_) / dt;
        double d_term = kd_ * derivative;
        
        // 总输出
        double output = p_term + i_term + d_term;
        output = std::max(-output_limit_, std::min(output_limit_, output));
        
        last_error_ = error;
        return output;
    }
    
    void reset() {
        integral_ = 0.0;
        last_error_ = 0.0;
        first_run_ = true;
    }
};

class AdvancedPathController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    
    // 发布者
    ros::Publisher position_cmd_pub_;
    ros::Publisher path_visualization_pub_;
    ros::Publisher debug_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 路径数据
    std::deque<geometry_msgs::PoseStamped> original_path_;
    std::deque<geometry_msgs::PoseStamped> smoothed_path_;
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
    
    // PID控制器
    PIDController x_pid_, y_pid_, z_pid_;
    PIDController vx_pid_, vy_pid_, vz_pid_;
    PIDController yaw_pid_;
    
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
    ros::Time last_cmd_time_;
    
    // 路径平滑
    bool enable_path_smoothing_;
    double smoothing_weight_;
    int smoothing_iterations_;
    
    // 自适应参数
    bool adaptive_lookahead_;
    double min_lookahead_;
    double max_lookahead_;
    double velocity_lookahead_factor_;
    
    // 紧急停止
    bool emergency_stop_;
    Eigen::Vector3d emergency_position_;
    
public:
    AdvancedPathController() : 
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
        // 初始化PID控制器
        x_pid_(1.0, 0.1, 0.05, 1.0, 2.0),
        y_pid_(1.0, 0.1, 0.05, 1.0, 2.0),
        z_pid_(1.0, 0.1, 0.05, 1.0, 2.0),
        vx_pid_(0.5, 0.0, 0.1, 0.5, 1.0),
        vy_pid_(0.5, 0.0, 0.1, 0.5, 1.0),
        vz_pid_(0.5, 0.0, 0.1, 0.5, 1.0),
        yaw_pid_(1.0, 0.0, 0.1, 0.5, 1.0),
        emergency_stop_(false),
        emergency_position_(Eigen::Vector3d::Zero()) {
        
        // 加载参数
        loadParameters();
        
        // 初始化订阅者
        path_sub_ = nh_.subscribe("global_path", 10, &AdvancedPathController::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &AdvancedPathController::odomCallback, this);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &AdvancedPathController::cmdVelCallback, this);
        
        // 初始化发布者
        position_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("cmd", 10);
        path_visualization_pub_ = nh_.advertise<nav_msgs::Path>("current_path", 10);
        debug_pub_ = nh_.advertise<geometry_msgs::Twist>("debug_info", 10);
        
        ROS_INFO("AdvancedPathController initialized");
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
            
            // 发布调试信息
            publishDebugInfo();
            
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
        private_nh_.param("enable_path_smoothing", enable_path_smoothing_, true);
        private_nh_.param("smoothing_weight", smoothing_weight_, 0.1);
        private_nh_.param("smoothing_iterations", smoothing_iterations_, 10);
        private_nh_.param("adaptive_lookahead", adaptive_lookahead_, true);
        private_nh_.param("min_lookahead", min_lookahead_, 1.0);
        private_nh_.param("max_lookahead", max_lookahead_, 5.0);
        private_nh_.param("velocity_lookahead_factor", velocity_lookahead_factor_, 0.5);
        
        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Lookahead distance: %.2f m", lookahead_distance_);
        ROS_INFO("  Max velocity: %.2f m/s", max_velocity_);
        ROS_INFO("  Control frequency: %.1f Hz", control_frequency_);
        ROS_INFO("  Path smoothing: %s", enable_path_smoothing_ ? "enabled" : "disabled");
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        // Store original path
        original_path_.clear();
        for (const auto& pose : msg->poses) {
            original_path_.push_back(pose);
        }
        
        // Apply path smoothing if enabled
        if (enable_path_smoothing_ && original_path_.size() > 2) {
            smoothPath();
        } else {
            smoothed_path_ = original_path_;
        }
        
        path_received_ = true;
        path_active_ = true;
        current_waypoint_index_ = 0;
        last_path_time_ = ros::Time::now();
        
        // Reset PID controllers
        x_pid_.reset();
        y_pid_.reset();
        z_pid_.reset();
        vx_pid_.reset();
        vy_pid_.reset();
        vz_pid_.reset();
        yaw_pid_.reset();
        
        ROS_INFO("Received path with %zu waypoints", original_path_.size());
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Update current state from odometry
        current_position_[0] = msg->pose.pose.position.x;
        current_position_[1] = msg->pose.pose.position.y;
        current_position_[2] = msg->pose.pose.position.z;
        
        current_velocity_[0] = msg->twist.twist.linear.x;
        current_velocity_[1] = msg->twist.twist.linear.y;
        current_velocity_[2] = msg->twist.twist.linear.z;
        
        // Extract yaw angle from quaternion
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        current_yaw_ = tf::getYaw(q);
        
        last_odom_time_ = ros::Time::now();
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Check emergency stop command
        if (std::abs(msg->linear.x) < 0.01 && std::abs(msg->linear.y) < 0.01 && 
            std::abs(msg->linear.z) < 0.01 && std::abs(msg->angular.z) < 0.01) {
            emergency_stop_ = true;
            emergency_position_ = current_position_;
            ROS_WARN("Emergency stop activated");
        } else {
            emergency_stop_ = false;
        }
        
        last_cmd_time_ = ros::Time::now();
    }
    
    void smoothPath() {
        smoothed_path_ = original_path_;
        
        for (int iter = 0; iter < smoothing_iterations_; ++iter) {
            std::deque<geometry_msgs::PoseStamped> temp_path = smoothed_path_;
            
            for (size_t i = 1; i < smoothed_path_.size() - 1; ++i) {
                // Smooth position
                smoothed_path_[i].pose.position.x = 
                    (1.0 - smoothing_weight_) * temp_path[i].pose.position.x +
                    smoothing_weight_ * 0.5 * (temp_path[i-1].pose.position.x + temp_path[i+1].pose.position.x);
                
                smoothed_path_[i].pose.position.y = 
                    (1.0 - smoothing_weight_) * temp_path[i].pose.position.y +
                    smoothing_weight_ * 0.5 * (temp_path[i-1].pose.position.y + temp_path[i+1].pose.position.y);
                
                smoothed_path_[i].pose.position.z = 
                    (1.0 - smoothing_weight_) * temp_path[i].pose.position.z +
                    smoothing_weight_ * 0.5 * (temp_path[i-1].pose.position.z + temp_path[i+1].pose.position.z);
            }
        }
    }
    
    void updatePathFollowing() {
        if (!path_active_ || smoothed_path_.empty()) {
            return;
        }
        
        // Check if path is too old
        if ((ros::Time::now() - last_path_time_).toSec() > max_path_age_) {
            ROS_WARN_THROTTLE(2.0, "Path too old, stopping");
            path_active_ = false;
            return;
        }
        
        // Handle emergency stop
        if (emergency_stop_) {
            target_position_ = emergency_position_;
            target_velocity_ = Eigen::Vector3d::Zero();
            target_acceleration_ = Eigen::Vector3d::Zero();
            return;
        }
        
        // Update adaptive lookahead distance
        if (adaptive_lookahead_) {
            updateLookaheadDistance();
        }
        
        // Find the best waypoint to track
        size_t target_waypoint = findTargetWaypoint();
        
        if (target_waypoint >= smoothed_path_.size()) {
            // Reached end of path
            ROS_INFO("Reached end of path");
            path_active_ = false;
            return;
        }
        
        // Get target waypoint
        const geometry_msgs::PoseStamped& target_pose = smoothed_path_[target_waypoint];
        
        // Set target position (maintain fixed height)
        target_position_[0] = target_pose.pose.position.x;
        target_position_[1] = target_pose.pose.position.y;
        target_position_[2] = fixed_height_;
        
        // Calculate target yaw (towards next waypoint)
        target_yaw_ = calculateTargetYaw(target_waypoint);
        
        // Calculate target velocity based on distance to target
        calculateTargetVelocity(target_waypoint);
        
        // Update current waypoint index
        if (target_waypoint > current_waypoint_index_) {
            current_waypoint_index_ = target_waypoint;
        }
    }
    
    void updateLookaheadDistance() {
        double current_speed = current_velocity_.norm();
        lookahead_distance_ = min_lookahead_ + 
                             velocity_lookahead_factor_ * current_speed * (max_lookahead_ - min_lookahead_) / max_velocity_;
        lookahead_distance_ = std::max(min_lookahead_, std::min(max_lookahead_, lookahead_distance_));
    }
    
    size_t findTargetWaypoint() {
        if (smoothed_path_.empty()) {
            return 0;
        }
        
        // 找到距离当前位置前瞻距离的航点
        double accumulated_distance = 0.0;
        size_t last_waypoint = 0;
        
        for (size_t i = 0; i < smoothed_path_.size(); ++i) {
            if (i > 0) {
                double segment_distance = calculateDistance(
                    smoothed_path_[i-1].pose.position,
                    smoothed_path_[i].pose.position
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
        if (waypoint_index + 1 >= smoothed_path_.size()) {
            // 最后一个航点，保持当前偏航角
            return current_yaw_;
        }
        
        // 计算朝向下一航点的偏航角
        const geometry_msgs::Point& current_point = smoothed_path_[waypoint_index].pose.position;
        const geometry_msgs::Point& next_point = smoothed_path_[waypoint_index + 1].pose.position;
        
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
    
    void generatePositionCommand() {
        nav_converter::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        
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
        
        // 应用PID控制
        double x_correction = x_pid_.compute(x_error, dt);
        double y_correction = y_pid_.compute(y_error, dt);
        double z_correction = z_pid_.compute(z_error, dt);
        
        double vx_correction = vx_pid_.compute(vx_error, dt);
        double vy_correction = vy_pid_.compute(vy_error, dt);
        double vz_correction = vz_pid_.compute(vz_error, dt);
        
        double yaw_correction = yaw_pid_.compute(yaw_error, dt);
        
        // 设置带修正的位置
        cmd.position.x = target_position_[0] + x_correction;
        cmd.position.y = target_position_[1] + y_correction;
        cmd.position.z = target_position_[2] + z_correction;
        
        // 设置带修正的速度
        cmd.velocity.x = target_velocity_[0] + vx_correction;
        cmd.velocity.y = target_velocity_[1] + vy_correction;
        cmd.velocity.z = target_velocity_[2] + vz_correction;
        
        // 计算加速度
        target_acceleration_ = (target_velocity_ - current_velocity_) / dt;
        
        // 限制加速度
        if (target_acceleration_.norm() > max_acceleration_) {
            target_acceleration_ = target_acceleration_.normalized() * max_acceleration_;
        }
        
        cmd.acceleration.x = target_acceleration_[0];
        cmd.acceleration.y = target_acceleration_[1];
        cmd.acceleration.z = target_acceleration_[2];
        
        // 设置加加速度（目前为零）
        cmd.jerk.x = 0.0;
        cmd.jerk.y = 0.0;
        cmd.jerk.z = 0.0;
        
        // 设置带修正的偏航角
        cmd.yaw = target_yaw_ + yaw_correction;
        cmd.yaw_dot = 0.0;
        
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
        if (!path_received_ || smoothed_path_.empty()) {
            return;
        }
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        
        // 添加平滑路径点
        for (const auto& pose : smoothed_path_) {
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
    ros::init(argc, argv, "advanced_path_controller");
    
    AdvancedPathController controller;
    controller.run();
    
    return 0;
} 