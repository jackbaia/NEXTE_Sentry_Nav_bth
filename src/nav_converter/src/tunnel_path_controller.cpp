/**
 * @file tunnel_path_controller.cpp
 * @brief 针对隧道环境优化的高级路径跟踪控制器
 */

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

#include <nav_converter/PositionCommand.h>

// 隧道环境PID控制器 - 更保守的参数
class TunnelPIDController {
private:
    double kp_, ki_, kd_;
    double integral_;
    double last_error_;
    double integral_limit_;
    double output_limit_;
    bool first_run_;
    double error_threshold_;  // 误差阈值，防止积分饱和
    double derivative_limit_; // 微分项限幅
    
public:
    TunnelPIDController(double kp, double ki, double kd, double integral_limit, double output_limit) :
        kp_(kp), ki_(ki), kd_(kd), integral_limit_(integral_limit), output_limit_(output_limit),
        integral_(0.0), last_error_(0.0), first_run_(true), error_threshold_(0.1), derivative_limit_(2.0) {}
    
    double compute(double error, double dt) {
        if (first_run_) {
            last_error_ = error;
            first_run_ = false;
        }
        
        // 隧道环境：只有当误差超过阈值时才进行积分
        if (std::abs(error) > error_threshold_) {
            integral_ += error * dt;
            integral_ = std::max(-integral_limit_, std::min(integral_limit_, integral_));
        }
        
        // 比例项
        double p_term = kp_ * error;
        
        // 积分项
        double i_term = ki_ * integral_;
        
        // 微分项 - 隧道环境需要更平滑的微分
        double derivative = (error - last_error_) / dt;
        derivative = std::max(-derivative_limit_, std::min(derivative_limit_, derivative));
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
    
    void setErrorThreshold(double threshold) {
        error_threshold_ = threshold;
    }
    
    void setDerivativeLimit(double limit) {
        derivative_limit_ = limit;
    }
};

// 隧道安全监控器
class TunnelSafetyMonitor {
private:
    double min_safe_distance_;
    double max_safe_distance_;
    double emergency_stop_distance_;
    std::vector<double> laser_ranges_;
    bool laser_data_valid_;
    
public:
    TunnelSafetyMonitor() : 
        min_safe_distance_(0.5),
        max_safe_distance_(3.0),
        emergency_stop_distance_(0.3),
        laser_data_valid_(false) {}
    
    void updateLaserData(const sensor_msgs::LaserScan::ConstPtr& scan) {
        laser_ranges_.clear();
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max) {
                laser_ranges_.push_back(scan->ranges[i]);
            }
        }
        laser_data_valid_ = true;
    }
    
    bool isSafeToMove() {
        if (!laser_data_valid_ || laser_ranges_.empty()) {
            return true;
        }
        
        for (double range : laser_ranges_) {
            if (range < emergency_stop_distance_) {
                return false;
            }
        }
        return true;
    }
    
    double getMinDistance() {
        if (laser_ranges_.empty()) return std::numeric_limits<double>::max();
        return *std::min_element(laser_ranges_.begin(), laser_ranges_.end());
    }
    
    void setSafetyDistances(double min_dist, double max_dist, double emergency_dist) {
        min_safe_distance_ = min_dist;
        max_safe_distance_ = max_dist;
        emergency_stop_distance_ = emergency_dist;
    }
};

// 隧道路径平滑器 - 专门针对隧道环境
class TunnelPathSmoother {
private:
    double smoothing_weight_;
    int smoothing_iterations_;
    double max_curvature_;         // 最大曲率限制
    double tunnel_width_;          // 隧道宽度估计
    
public:
    TunnelPathSmoother() : 
        smoothing_weight_(0.05),   // 隧道环境：更小的平滑权重
        smoothing_iterations_(20), // 更多迭代次数
        max_curvature_(0.5),       // 最大曲率0.5
        tunnel_width_(2.0) {}      // 假设隧道宽度2米
    
    void smoothPath(std::deque<geometry_msgs::PoseStamped>& path) {
        if (path.size() < 3) return;
        
        std::deque<geometry_msgs::PoseStamped> original_path = path;
        
        for (int iter = 0; iter < smoothing_iterations_; ++iter) {
            std::deque<geometry_msgs::PoseStamped> temp_path = path;
            
            for (size_t i = 1; i < path.size() - 1; ++i) {
                // 计算当前点的曲率
                double curvature = calculateCurvature(temp_path, i);
                
                // 如果曲率太大，减少平滑权重
                double adaptive_weight = smoothing_weight_;
                if (curvature > max_curvature_) {
                    adaptive_weight *= 0.5;
                }
                
                // 平滑位置
                path[i].pose.position.x = 
                    (1.0 - adaptive_weight) * temp_path[i].pose.position.x +
                    adaptive_weight * 0.5 * (temp_path[i-1].pose.position.x + temp_path[i+1].pose.position.x);
                
                path[i].pose.position.y = 
                    (1.0 - adaptive_weight) * temp_path[i].pose.position.y +
                    adaptive_weight * 0.5 * (temp_path[i-1].pose.position.y + temp_path[i+1].pose.position.y);
                
                path[i].pose.position.z = 
                    (1.0 - adaptive_weight) * temp_path[i].pose.position.z +
                    adaptive_weight * 0.5 * (temp_path[i-1].pose.position.z + temp_path[i+1].pose.position.z);
                
                // 隧道环境：保持路径在隧道中心
                centerPathInTunnel(path[i]);
            }
        }
    }
    
    void setTunnelWidth(double width) {
        tunnel_width_ = width;
    }
    
private:
    double calculateCurvature(const std::deque<geometry_msgs::PoseStamped>& path, size_t index) {
        if (index == 0 || index >= path.size() - 1) return 0.0;
        
        // 计算三点形成的曲率
        const auto& p1 = path[index-1].pose.position;
        const auto& p2 = path[index].pose.position;
        const auto& p3 = path[index+1].pose.position;
        
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;
        
        double cross_product = dx1 * dy2 - dy1 * dx2;
        double d1 = sqrt(dx1*dx1 + dy1*dy1);
        double d2 = sqrt(dx2*dx2 + dy2*dy2);
        
        if (d1 < 1e-6 || d2 < 1e-6) return 0.0;
        
        return std::abs(cross_product) / (d1 * d2);
    }
    
    void centerPathInTunnel(geometry_msgs::PoseStamped& pose) {
        // 简单的隧道中心化：假设隧道中心在y=0
        // 实际应用中需要根据激光数据动态调整
        if (std::abs(pose.pose.position.y) > tunnel_width_ * 0.3) {
            pose.pose.position.y *= 0.8; // 向中心拉回
        }
    }
};

// 隧道环境路径控制器
class TunnelPathController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber laser_sub_;
    
    // 发布者
    ros::Publisher position_cmd_pub_;
    ros::Publisher path_visualization_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher safety_status_pub_;
    
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
    
    // 隧道环境PID控制器
    TunnelPIDController x_pid_, y_pid_, z_pid_;
    TunnelPIDController vx_pid_, vy_pid_, vz_pid_;
    TunnelPIDController yaw_pid_;
    
    // 隧道安全监控器
    TunnelSafetyMonitor safety_monitor_;
    
    // 隧道路径平滑器
    TunnelPathSmoother path_smoother_;
    
    // 隧道环境控制参数
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
    
    // 隧道环境安全参数
    double emergency_stop_timeout_;
    double max_path_age_;
    ros::Time last_path_time_;
    ros::Time last_odom_time_;
    ros::Time last_cmd_time_;
    
    // 隧道环境自适应参数
    bool adaptive_lookahead_;
    double min_lookahead_;
    double max_lookahead_;
    double velocity_lookahead_factor_;
    
    // 隧道环境紧急停止
    bool emergency_stop_;
    Eigen::Vector3d emergency_position_;
    
    // 隧道环境特殊参数
    double tunnel_width_estimate_;
    double lateral_safety_margin_;
    double vertical_safety_margin_;
    bool enable_tunnel_centering_;
    
public:
    TunnelPathController() : 
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
        // 隧道环境：更保守的PID参数
        x_pid_(0.8, 0.05, 0.1, 0.5, 1.5),
        y_pid_(0.8, 0.05, 0.1, 0.5, 1.5),
        z_pid_(0.8, 0.05, 0.1, 0.5, 1.5),
        vx_pid_(0.3, 0.0, 0.15, 0.3, 0.8),
        vy_pid_(0.3, 0.0, 0.15, 0.3, 0.8),
        vz_pid_(0.3, 0.0, 0.15, 0.3, 0.8),
        yaw_pid_(0.6, 0.0, 0.15, 0.3, 0.8),
        emergency_stop_(false),
        emergency_position_(Eigen::Vector3d::Zero()),
        tunnel_width_estimate_(2.0),
        lateral_safety_margin_(0.3),
        vertical_safety_margin_(0.2),
        enable_tunnel_centering_(true) {
        
        loadTunnelParameters();
        
        // 初始化订阅者
        path_sub_ = nh_.subscribe("global_path", 10, &TunnelPathController::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &TunnelPathController::odomCallback, this);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &TunnelPathController::cmdVelCallback, this);
        laser_sub_ = nh_.subscribe("laser_scan", 10, &TunnelPathController::laserCallback, this);
        
        // 初始化发布者
        position_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("cmd", 10);
        path_visualization_pub_ = nh_.advertise<nav_msgs::Path>("current_path", 10);
        debug_pub_ = nh_.advertise<geometry_msgs::Twist>("debug_info", 10);
        safety_status_pub_ = nh_.advertise<geometry_msgs::Twist>("safety_status", 10);
        
        ROS_INFO("TunnelPathController initialized for tunnel environment");
    }
    
    void run() {
        ros::Rate rate(control_frequency_);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            // 检查安全状态
            if (!safety_monitor_.isSafeToMove()) {
                ROS_WARN_THROTTLE(1.0, "Safety violation detected! Emergency stop.");
                emergency_stop_ = true;
                emergency_position_ = current_position_;
            }
            
            if (!path_received_) {
                ROS_WARN_THROTTLE(5.0, "Waiting for path data...");
                rate.sleep();
                continue;
            }
            
            updatePathFollowing();
            generatePositionCommand();
            publishPathVisualization();
            publishDebugInfo();
            publishSafetyStatus();
            
            rate.sleep();
        }
    }
    
private:
    void loadTunnelParameters() {
        // 隧道环境：更保守的参数
        private_nh_.param("lookahead_distance", lookahead_distance_, 1.5);
        private_nh_.param("waypoint_tolerance", waypoint_tolerance_, 0.3);
        private_nh_.param("max_velocity", max_velocity_, 1.5);
        private_nh_.param("max_acceleration", max_acceleration_, 0.8);
        private_nh_.param("max_jerk", max_jerk_, 1.5);
        private_nh_.param("control_frequency", control_frequency_, 50.0);
        private_nh_.param("fixed_height", fixed_height_, 1.2);
        private_nh_.param("yaw_gain", yaw_gain_, 0.8);
        private_nh_.param("position_gain", position_gain_, 0.8);
        private_nh_.param("velocity_gain", velocity_gain_, 0.4);
        
        // 隧道环境安全参数
        private_nh_.param("emergency_stop_timeout", emergency_stop_timeout_, 0.5);
        private_nh_.param("max_path_age", max_path_age_, 3.0);
        private_nh_.param("adaptive_lookahead", adaptive_lookahead_, true);
        private_nh_.param("min_lookahead", min_lookahead_, 0.8);
        private_nh_.param("max_lookahead", max_lookahead_, 3.0);
        private_nh_.param("velocity_lookahead_factor", velocity_lookahead_factor_, 0.3);
        
        // 隧道环境特殊参数
        private_nh_.param("tunnel_width_estimate", tunnel_width_estimate_, 2.0);
        private_nh_.param("lateral_safety_margin", lateral_safety_margin_, 0.3);
        private_nh_.param("vertical_safety_margin", vertical_safety_margin_, 0.2);
        private_nh_.param("enable_tunnel_centering", enable_tunnel_centering_, true);
        
        // PID参数优化
        double error_threshold, derivative_limit;
        private_nh_.param("error_threshold", error_threshold, 0.1);
        private_nh_.param("derivative_limit", derivative_limit, 2.0);
        
        // 设置PID参数
        x_pid_.setErrorThreshold(error_threshold);
        y_pid_.setErrorThreshold(error_threshold);
        z_pid_.setErrorThreshold(error_threshold);
        vx_pid_.setErrorThreshold(error_threshold);
        vy_pid_.setErrorThreshold(error_threshold);
        vz_pid_.setErrorThreshold(error_threshold);
        yaw_pid_.setErrorThreshold(error_threshold);
        
        x_pid_.setDerivativeLimit(derivative_limit);
        y_pid_.setDerivativeLimit(derivative_limit);
        z_pid_.setDerivativeLimit(derivative_limit);
        vx_pid_.setDerivativeLimit(derivative_limit);
        vy_pid_.setDerivativeLimit(derivative_limit);
        vz_pid_.setDerivativeLimit(derivative_limit);
        yaw_pid_.setDerivativeLimit(derivative_limit);
        
        safety_monitor_.setSafetyDistances(0.5, 3.0, 0.3);
        
        ROS_INFO("Tunnel parameters loaded:");
        ROS_INFO("  Max velocity: %.2f m/s (reduced for tunnel)", max_velocity_);
        ROS_INFO("  Max acceleration: %.2f m/s² (reduced for tunnel)", max_acceleration_);
        ROS_INFO("  Lookahead distance: %.2f m (reduced for tunnel)", lookahead_distance_);
        ROS_INFO("  Tunnel width estimate: %.2f m", tunnel_width_estimate_);
        ROS_INFO("  Safety margins: lateral=%.2f m, vertical=%.2f m", 
                 lateral_safety_margin_, vertical_safety_margin_);
    }
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        safety_monitor_.updateLaserData(msg);
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        original_path_.clear();
        for (const auto& pose : msg->poses) {
            original_path_.push_back(pose);
        }
        
        // 隧道环境：应用专门的路径平滑
        smoothed_path_ = original_path_;
        if (original_path_.size() > 2) {
            path_smoother_.setTunnelWidth(tunnel_width_estimate_);
            path_smoother_.smoothPath(smoothed_path_);
        }
        
        path_received_ = true;
        path_active_ = true;
        current_waypoint_index_ = 0;
        last_path_time_ = ros::Time::now();
        
        // 重置PID控制器
        x_pid_.reset();
        y_pid_.reset();
        z_pid_.reset();
        vx_pid_.reset();
        vy_pid_.reset();
        vz_pid_.reset();
        yaw_pid_.reset();
        
        ROS_INFO("Received tunnel path with %zu waypoints", original_path_.size());
    }
    

    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_position_[0] = msg->pose.pose.position.x;
        current_position_[1] = msg->pose.pose.position.y;
        current_position_[2] = msg->pose.pose.position.z;
        
        current_velocity_[0] = msg->twist.twist.linear.x;
        current_velocity_[1] = msg->twist.twist.linear.y;
        current_velocity_[2] = msg->twist.twist.linear.z;
        
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
        current_yaw_ = tf::getYaw(q);
        
        last_odom_time_ = ros::Time::now();
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
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
    
    void updatePathFollowing() {
        if (!path_active_ || smoothed_path_.empty()) {
            return;
        }
        
        if ((ros::Time::now() - last_path_time_).toSec() > max_path_age_) {
            ROS_WARN_THROTTLE(2.0, "Path too old, stopping");
            path_active_ = false;
            return;
        }
        
        if (emergency_stop_) {
            target_position_ = emergency_position_;
            target_velocity_ = Eigen::Vector3d::Zero();
            target_acceleration_ = Eigen::Vector3d::Zero();
            return;
        }
        
        if (adaptive_lookahead_) {
            updateLookaheadDistance();
        }
        
        size_t target_waypoint = findTargetWaypoint();
        
        if (target_waypoint >= smoothed_path_.size()) {
            ROS_INFO("Reached end of tunnel path");
            path_active_ = false;
            return;
        }
        
        const geometry_msgs::PoseStamped& target_pose = smoothed_path_[target_waypoint];
        
        target_position_[0] = target_pose.pose.position.x;
        target_position_[1] = target_pose.pose.position.y;
        target_position_[2] = fixed_height_;
        
        applyTunnelSafetyMargins();
        
        target_yaw_ = calculateTargetYaw(target_waypoint);
        calculateTargetVelocity(target_waypoint);
        
        if (target_waypoint > current_waypoint_index_) {
            current_waypoint_index_ = target_waypoint;
        }
    }
    
    void applyTunnelSafetyMargins() {
        // 横向安全边距
        if (enable_tunnel_centering_) {
            if (std::abs(target_position_[1]) > tunnel_width_estimate_ * 0.3) {
                target_position_[1] *= 0.8;
            }
        }
        
        // 垂直安全边距
        double min_height = 0.5 + vertical_safety_margin_;
        double max_height = 2.5 - vertical_safety_margin_;
        target_position_[2] = std::max(min_height, std::min(max_height, target_position_[2]));
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
            return current_yaw_;
        }
        
        const geometry_msgs::Point& current_point = smoothed_path_[waypoint_index].pose.position;
        const geometry_msgs::Point& next_point = smoothed_path_[waypoint_index + 1].pose.position;
        
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
        
        // 根据安全距离调整速度
        double min_distance = safety_monitor_.getMinDistance();
        if (min_distance < 1.0) {
            base_velocity *= (min_distance / 1.0);
        }
        
        Eigen::Vector3d direction = (target_position_ - current_position_).normalized();
        target_velocity_ = direction * base_velocity;
        
        if (target_velocity_.norm() > max_velocity_) {
            target_velocity_ = target_velocity_.normalized() * max_velocity_;
        }
    }
    
    void generatePositionCommand() {
        nav_converter::PositionCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        
        double dt = 1.0 / control_frequency_;
        
        // 计算误差
        double x_error = target_position_[0] - current_position_[0];
        double y_error = target_position_[1] - current_position_[1];
        double z_error = target_position_[2] - current_position_[2];
        
        double vx_error = target_velocity_[0] - current_velocity_[0];
        double vy_error = target_velocity_[1] - current_velocity_[1];
        double vz_error = target_velocity_[2] - current_velocity_[2];
        
        double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);
        
        // 应用PID控制
        double x_correction = x_pid_.compute(x_error, dt);
        double y_correction = y_pid_.compute(y_error, dt);
        double z_correction = z_pid_.compute(z_error, dt);
        
        double vx_correction = vx_pid_.compute(vx_error, dt);
        double vy_correction = vy_pid_.compute(vy_error, dt);
        double vz_correction = vz_pid_.compute(vz_error, dt);
        
        double yaw_correction = yaw_pid_.compute(yaw_error, dt);
        
        // 设置命令
        cmd.position.x = target_position_[0] + x_correction;
        cmd.position.y = target_position_[1] + y_correction;
        cmd.position.z = target_position_[2] + z_correction;
        
        cmd.velocity.x = target_velocity_[0] + vx_correction;
        cmd.velocity.y = target_velocity_[1] + vy_correction;
        cmd.velocity.z = target_velocity_[2] + vz_correction;
        
        target_acceleration_ = (target_velocity_ - current_velocity_) / dt;
        
        if (target_acceleration_.norm() > max_acceleration_) {
            target_acceleration_ = target_acceleration_.normalized() * max_acceleration_;
        }
        
        cmd.acceleration.x = target_acceleration_[0];
        cmd.acceleration.y = target_acceleration_[1];
        cmd.acceleration.z = target_acceleration_[2];
        
        cmd.jerk.x = 0.0;
        cmd.jerk.y = 0.0;
        cmd.jerk.z = 0.0;
        
        cmd.yaw = target_yaw_ + yaw_correction;
        cmd.yaw_dot = 0.0;
        
        cmd.kx[0] = position_gain_;
        cmd.kx[1] = position_gain_;
        cmd.kx[2] = position_gain_;
        cmd.kv[0] = velocity_gain_;
        cmd.kv[1] = velocity_gain_;
        cmd.kv[2] = velocity_gain_;
        
        cmd.trajectory_id = path_active_ ? 1 : 0;
        cmd.trajectory_flag = path_active_ ? 
            nav_converter::PositionCommand::TRAJECTORY_STATUS_READY : 
            nav_converter::PositionCommand::TRAJECTORY_STATUS_EMPTY;
        
        position_cmd_pub_.publish(cmd);
    }
    
    void publishPathVisualization() {
        if (!path_received_ || smoothed_path_.empty()) {
            return;
        }
        
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        
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
    
    void publishSafetyStatus() {
        geometry_msgs::Twist safety_msg;
        safety_msg.linear.x = safety_monitor_.getMinDistance();
        safety_msg.linear.y = safety_monitor_.isSafeToMove() ? 1.0 : 0.0;
        safety_msg.linear.z = emergency_stop_ ? 1.0 : 0.0;
        safety_msg.angular.x = tunnel_width_estimate_;
        safety_msg.angular.y = lateral_safety_margin_;
        safety_msg.angular.z = vertical_safety_margin_;
        
        safety_status_pub_.publish(safety_msg);
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
    ros::init(argc, argv, "tunnel_path_controller");
    
    TunnelPathController controller;
    controller.run();
    
    return 0;
} 