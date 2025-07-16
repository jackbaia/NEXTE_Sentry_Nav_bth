/**
 * @file advanced_path_controller.cpp
 * @brief 具有PID控制、路径平滑和隧道中心化的高级路径跟踪控制器
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
    ros::Subscriber laser_scan_sub_;  // 新增：激光扫描订阅者
    
    // 发布者
    ros::Publisher position_cmd_pub_;
    ros::Publisher path_visualization_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher tunnel_center_pub_;  // 新增：隧道中心线可视化
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 路径数据
    std::deque<geometry_msgs::PoseStamped> original_path_;
    std::deque<geometry_msgs::PoseStamped> smoothed_path_;
    std::deque<geometry_msgs::PoseStamped> tunnel_centered_path_;  // 新增：隧道中心化路径
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
    
    // 新增：隧道中心化参数
    bool enable_tunnel_centering_;
    double tunnel_width_;
    double safety_margin_;
    double centering_gain_;
    double tunnel_detection_range_;
    double min_tunnel_width_;
    double max_tunnel_width_;
    std::vector<double> laser_ranges_;
    bool laser_data_received_;
    ros::Time last_laser_time_;
    
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
        emergency_position_(Eigen::Vector3d::Zero()),
        // 初始化隧道中心化参数
        enable_tunnel_centering_(true),
        tunnel_width_(2.0),
        safety_margin_(0.3),
        centering_gain_(1.0),
        tunnel_detection_range_(5.0),
        min_tunnel_width_(1.5),
        max_tunnel_width_(4.0),
        laser_data_received_(false) {
        
        // 加载参数
        loadParameters();
        
        // 初始化订阅者
        path_sub_ = nh_.subscribe("global_path", 10, &AdvancedPathController::pathCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &AdvancedPathController::odomCallback, this);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &AdvancedPathController::cmdVelCallback, this);
        laser_scan_sub_ = nh_.subscribe("laser_scan", 10, &AdvancedPathController::laserScanCallback, this);  // 新增
        
        // 初始化发布者
        position_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("cmd", 10);
        path_visualization_pub_ = nh_.advertise<nav_msgs::Path>("current_path", 10);
        debug_pub_ = nh_.advertise<geometry_msgs::Twist>("debug_info", 10);
        tunnel_center_pub_ = nh_.advertise<nav_msgs::Path>("tunnel_center_path", 10);  // 新增
        
        ROS_INFO("AdvancedPathController with tunnel centering initialized");
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

            // 发布隧道中心化路径可视化
            if (enable_tunnel_centering_ && laser_data_received_ && (ros::Time::now() - last_laser_time_).toSec() < 0.1) {
                publishTunnelCenterPath();
            }
            
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
        private_nh_.param("control_frequency", control_frequency_, 30.0);  // 降低到30Hz
        private_nh_.param("fixed_height", fixed_height_, 1.5);
        private_nh_.param("yaw_gain", yaw_gain_, 1.0);
        private_nh_.param("position_gain", position_gain_, 1.0);
        private_nh_.param("velocity_gain", velocity_gain_, 0.5);
        private_nh_.param("emergency_stop_timeout", emergency_stop_timeout_, 1.0);
        private_nh_.param("max_path_age", max_path_age_, 30.0);  // 增加到30秒
        private_nh_.param("enable_path_smoothing", enable_path_smoothing_, true);
        private_nh_.param("smoothing_weight", smoothing_weight_, 0.1);
        private_nh_.param("smoothing_iterations", smoothing_iterations_, 10);
        private_nh_.param("adaptive_lookahead", adaptive_lookahead_, true);
        private_nh_.param("min_lookahead", min_lookahead_, 1.0);
        private_nh_.param("max_lookahead", max_lookahead_, 5.0);
        private_nh_.param("velocity_lookahead_factor", velocity_lookahead_factor_, 0.5);
        
        // 隧道中心化参数
        private_nh_.param("enable_tunnel_centering", enable_tunnel_centering_, true);
        private_nh_.param("tunnel_width", tunnel_width_, 2.0);
        private_nh_.param("safety_margin", safety_margin_, 0.3);
        private_nh_.param("centering_gain", centering_gain_, 1.0);
        private_nh_.param("tunnel_detection_range", tunnel_detection_range_, 5.0);
        private_nh_.param("min_tunnel_width", min_tunnel_width_, 1.5);
        private_nh_.param("max_tunnel_width", max_tunnel_width_, 4.0);
        
        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Lookahead distance: %.2f m", lookahead_distance_);
        ROS_INFO("  Max velocity: %.2f m/s", max_velocity_);
        ROS_INFO("  Control frequency: %.1f Hz", control_frequency_);
        ROS_INFO("  Path smoothing: %s", enable_path_smoothing_ ? "enabled" : "disabled");
        ROS_INFO("  Tunnel centering: %s", enable_tunnel_centering_ ? "enabled" : "disabled");
        if (enable_tunnel_centering_) {
            ROS_INFO("  Tunnel width: %.2f m", tunnel_width_);
            ROS_INFO("  Safety margin: %.2f m", safety_margin_);
            ROS_INFO("  Centering gain: %.2f", centering_gain_);
        }
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

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        laser_ranges_.clear();
        laser_ranges_.assign(msg->ranges.begin(), msg->ranges.end());
        laser_data_received_ = true;
        last_laser_time_ = ros::Time::now();
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

        // 执行隧道中心化
        if (enable_tunnel_centering_ && laser_data_received_ && (ros::Time::now() - last_laser_time_).toSec() < 0.1) { // 假设激光数据每0.1秒更新一次
            correctPathForTunnelCentering();
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

    void correctPathForTunnelCentering() {
        if (smoothed_path_.empty() || laser_ranges_.empty()) {
            return;
        }

        // 检测隧道环境
        if (!detectTunnelEnvironment()) {
            return; // 不在隧道环境中，不需要中心化
        }

        // 计算隧道中心线
        Eigen::Vector2d tunnel_center_direction;
        double tunnel_width_measured;
        if (!calculateTunnelCenterline(tunnel_center_direction, tunnel_width_measured)) {
            return;
        }

        // 计算当前机器人相对于隧道中心线的偏移
        double lateral_offset = calculateLateralOffset(tunnel_center_direction);
        
        // 如果偏移超过安全边距，进行路径修正
        if (std::abs(lateral_offset) > safety_margin_) {
            applyTunnelCenteringCorrection(lateral_offset, tunnel_center_direction);
            ROS_INFO_THROTTLE(2.0, "Tunnel centering applied. Lateral offset: %.2f m", lateral_offset);
        }
    }

    bool detectTunnelEnvironment() {
        if (laser_ranges_.empty()) {
            return false;
        }

        // 统计有效激光点数量
        int valid_points = 0;
        double total_distance = 0.0;
        
        for (size_t i = 0; i < laser_ranges_.size(); ++i) {
            if (laser_ranges_[i] > 0.1 && laser_ranges_[i] < tunnel_detection_range_) {
                valid_points++;
                total_distance += laser_ranges_[i];
            }
        }

        // 如果有效点太少，可能不在隧道中
        if (valid_points < 10) {
            return false;
        }

        // 计算平均距离，判断是否在隧道中
        double avg_distance = total_distance / valid_points;
        return (avg_distance > min_tunnel_width_ / 2.0 && avg_distance < max_tunnel_width_ / 2.0);
    }

    bool calculateTunnelCenterline(Eigen::Vector2d& center_direction, double& tunnel_width) {
        if (laser_ranges_.empty()) {
            return false;
        }

        // 找到左右两侧的墙壁点
        std::vector<Eigen::Vector2d> left_wall_points, right_wall_points;
        
        // 假设激光扫描的角度范围是 -π 到 π
        double angle_increment = 2.0 * M_PI / laser_ranges_.size();
        
        for (size_t i = 0; i < laser_ranges_.size(); ++i) {
            double angle = -M_PI + i * angle_increment;
            double range = laser_ranges_[i];
            
            if (range > 0.1 && range < tunnel_detection_range_) {
                Eigen::Vector2d point;
                point[0] = range * cos(angle);
                point[1] = range * sin(angle);
                
                // 根据角度判断是左侧还是右侧墙壁
                if (angle > 0) {
                    right_wall_points.push_back(point);
                } else {
                    left_wall_points.push_back(point);
                }
            }
        }

        if (left_wall_points.size() < 3 || right_wall_points.size() < 3) {
            return false;
        }

        // 拟合左右墙壁的直线
        Eigen::Vector2d left_wall_normal, right_wall_normal;
        double left_wall_distance, right_wall_distance;
        
        if (!fitWallLine(left_wall_points, left_wall_normal, left_wall_distance) ||
            !fitWallLine(right_wall_points, right_wall_normal, right_wall_distance)) {
            return false;
        }

        // 计算隧道中心线方向（垂直于墙壁法向量的平均值）
        center_direction = (left_wall_normal + right_wall_normal).normalized();
        
        // 计算隧道宽度
        tunnel_width = left_wall_distance + right_wall_distance;
        
        return true;
    }

    bool fitWallLine(const std::vector<Eigen::Vector2d>& points, 
                     Eigen::Vector2d& normal, double& distance) {
        if (points.size() < 3) {
            return false;
        }

        // 使用最小二乘法拟合直线
        Eigen::MatrixXd A(points.size(), 2);
        Eigen::VectorXd b(points.size());
        
        for (size_t i = 0; i < points.size(); ++i) {
            A(i, 0) = points[i][0];
            A(i, 1) = points[i][1];
            b(i) = 0.0; // 假设直线通过原点
        }

        // 求解最小二乘问题
        Eigen::Vector2d solution = A.colPivHouseholderQr().solve(b);
        
        // 计算法向量和距离
        normal = Eigen::Vector2d(-solution[1], solution[0]).normalized();
        distance = std::abs(solution[0] * normal[0] + solution[1] * normal[1]);
        
        return true;
    }

    double calculateLateralOffset(const Eigen::Vector2d& tunnel_direction) {
        // 计算当前机器人位置相对于隧道中心线的横向偏移
        Eigen::Vector2d robot_pos(current_position_[0], current_position_[1]);
        
        // 假设隧道中心线通过原点，方向为tunnel_direction
        // 计算机器人到中心线的垂直距离
        Eigen::Vector2d perpendicular(-tunnel_direction[1], tunnel_direction[0]);
        double offset = robot_pos.dot(perpendicular);
        
        return offset;
    }

    void applyTunnelCenteringCorrection(double lateral_offset, const Eigen::Vector2d& tunnel_direction) {
        if (smoothed_path_.empty()) {
            return;
        }

        // 计算修正向量（指向隧道中心）
        Eigen::Vector2d correction_direction = (lateral_offset > 0) ? 
            Eigen::Vector2d(-tunnel_direction[1], tunnel_direction[0]) : 
            Eigen::Vector2d(tunnel_direction[1], -tunnel_direction[0]);
        
        // 计算修正量
        double correction_magnitude = std::min(std::abs(lateral_offset) * centering_gain_, 
                                              safety_margin_);
        Eigen::Vector2d correction = correction_direction * correction_magnitude;

        // 应用修正到路径点
        tunnel_centered_path_.clear();
        for (const auto& pose : smoothed_path_) {
            geometry_msgs::PoseStamped corrected_pose = pose;
            corrected_pose.pose.position.x += correction[0];
            corrected_pose.pose.position.y += correction[1];
            tunnel_centered_path_.push_back(corrected_pose);
        }

        // 更新当前使用的路径
        smoothed_path_ = tunnel_centered_path_;
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

    void publishTunnelCenterPath() {
        if (!path_received_ || smoothed_path_.empty()) {
            return;
        }

        nav_msgs::Path tunnel_center_msg;
        tunnel_center_msg.header.stamp = ros::Time::now();
        tunnel_center_msg.header.frame_id = "map";

        // 添加隧道中心化路径点
        for (const auto& pose : tunnel_centered_path_) {
            tunnel_center_msg.poses.push_back(pose);
        }

        tunnel_center_pub_.publish(tunnel_center_msg);
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