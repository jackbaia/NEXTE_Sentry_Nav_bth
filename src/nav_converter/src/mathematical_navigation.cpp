#include "nav_converter/mathematical_navigation.hpp"

namespace nav_converter {

MathematicalNavigator::MathematicalNavigator(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), is_goal_reached_(false), is_path_valid_(false), adaptation_confidence_(0.0), last_optimization_time_(0.0) {
    // 初始化参数
    loadParameters();
    // 初始化发布/订阅
    odom_sub_ = nh_.subscribe("odom", 1, &MathematicalNavigator::odomCallback, this);
    scan_sub_ = nh_.subscribe("scan", 1, &MathematicalNavigator::scanCallback, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &MathematicalNavigator::goalCallback, this);
    path_sub_ = nh_.subscribe("global_path", 1, &MathematicalNavigator::pathCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    optimized_path_pub_ = nh_.advertise<nav_msgs::Path>("optimized_path", 1);
    prediction_pub_ = nh_.advertise<nav_msgs::Path>("predicted_obstacles", 1);
    debug_pub_ = nh_.advertise<geometry_msgs::Twist>("debug_cmd_vel", 1);
    resetState();
}

MathematicalNavigator::~MathematicalNavigator() {}

void MathematicalNavigator::loadParameters() {
    // 优化权重参数
    private_nh_.param("goal_distance_weight", weights_.goal_distance_weight, 1.0);
    private_nh_.param("path_following_weight", weights_.path_following_weight, 0.8);
    private_nh_.param("obstacle_avoidance_weight", weights_.obstacle_avoidance_weight, 1.2);
    private_nh_.param("smoothness_weight", weights_.smoothness_weight, 0.6);
    private_nh_.param("energy_efficiency_weight", weights_.energy_efficiency_weight, 0.4);
    private_nh_.param("safety_weight", weights_.safety_weight, 1.5);
    private_nh_.param("time_efficiency_weight", weights_.time_efficiency_weight, 0.7);

    // 自适应控制参数
    private_nh_.param("learning_rate", adaptive_params_.learning_rate, 0.01);
    private_nh_.param("forgetting_factor", adaptive_params_.forgetting_factor, 0.95);
    private_nh_.param("adaptation_threshold", adaptive_params_.adaptation_threshold, 0.1);
    private_nh_.param("adaptation_window", adaptive_params_.adaptation_window, 50);
    private_nh_.param("min_confidence", adaptive_params_.min_confidence, 0.3);
    private_nh_.param("max_confidence", adaptive_params_.max_confidence, 0.9);

    // 动态障碍物预测参数
    private_nh_.param("prediction_horizon", obstacle_params_.prediction_horizon, 3.0);
    private_nh_.param("prediction_step", obstacle_params_.prediction_step, 0.1);
    private_nh_.param("velocity_uncertainty", obstacle_params_.velocity_uncertainty, 0.2);
    private_nh_.param("position_uncertainty", obstacle_params_.position_uncertainty, 0.1);
    private_nh_.param("collision_threshold", obstacle_params_.collision_threshold, 0.5);
    private_nh_.param("safety_margin", obstacle_params_.safety_margin, 0.3);

    // 路径优化参数
    private_nh_.param("smoothing_factor", path_params_.smoothing_factor, 0.8);
    private_nh_.param("curvature_weight", path_params_.curvature_weight, 0.5);
    private_nh_.param("length_weight", path_params_.length_weight, 0.3);
    private_nh_.param("optimization_iterations", path_params_.optimization_iterations, 10);
    private_nh_.param("convergence_threshold", path_params_.convergence_threshold, 1e-4);

    // 控制参数
    private_nh_.param("max_velocity", max_velocity_, 0.8);
    private_nh_.param("max_acceleration", max_acceleration_, 1.0);
    private_nh_.param("max_angular_velocity", max_angular_velocity_, 1.0);
    private_nh_.param("control_frequency", control_frequency_, 20.0);
    private_nh_.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh_.param("path_tolerance", path_tolerance_, 0.3);
}

void MathematicalNavigator::resetState() {
    current_pose_ = Eigen::Vector3d::Zero();
    current_velocity_ = Eigen::Vector3d::Zero();
    goal_pose_ = Eigen::Vector3d::Zero();
    global_path_.clear();
    obstacles_.clear();
    predicted_obstacles_.clear();
    performance_history_.clear();
    velocity_history_.clear();
    adaptation_confidence_ = 0.0;
    adaptive_weights_ = Eigen::VectorXd::Ones(7); // 7个目标
    is_goal_reached_ = false;
    is_path_valid_ = false;
    last_optimization_time_ = 0.0;
}

void MathematicalNavigator::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新当前位姿
    current_pose_(0) = msg->pose.pose.position.x;
    current_pose_(1) = msg->pose.pose.position.y;
    
    // 从四元数提取偏航角
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_pose_(2) = yaw;
    
    // 更新当前速度
    current_velocity_(0) = msg->twist.twist.linear.x;
    current_velocity_(1) = msg->twist.twist.linear.y;
    current_velocity_(2) = msg->twist.twist.angular.z;
}

void MathematicalNavigator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    obstacles_.clear();
    
    // 将激光数据转换为障碍物点
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
            continue;
        }
        
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];
        
        // 转换到机器人坐标系
        Eigen::Vector2d obstacle;
        obstacle(0) = range * cos(angle);
        obstacle(1) = range * sin(angle);
        
        // 转换到全局坐标系
        Eigen::Vector2d global_obstacle = rotatePoint(obstacle, current_pose_(2));
        global_obstacle += Eigen::Vector2d(current_pose_(0), current_pose_(1));
        
        obstacles_.push_back(global_obstacle);
    }
    
    // 预测障碍物轨迹
    predictObstacleTrajectories();
}

void MathematicalNavigator::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 更新目标位姿
    goal_pose_(0) = msg->pose.position.x;
    goal_pose_(1) = msg->pose.position.y;
    
    // 从四元数提取目标偏航角
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    goal_pose_(2) = yaw;
    
    // 重置状态
    is_goal_reached_ = false;
    resetState();
    
    ROS_INFO("New goal received: (%.2f, %.2f, %.2f)", goal_pose_(0), goal_pose_(1), goal_pose_(2));
}

void MathematicalNavigator::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    global_path_.clear();
    
    // 提取全局路径点
    for (const auto& pose : msg->poses) {
        Eigen::Vector2d path_point;
        path_point(0) = pose.pose.position.x;
        path_point(1) = pose.pose.position.y;
        global_path_.push_back(path_point);
    }
    
    // 优化路径
    if (!global_path_.empty()) {
        global_path_ = optimizePath(global_path_);
        is_path_valid_ = true;
    } else {
        is_path_valid_ = false;
    }
    
    // 发布优化后的路径
    publishOptimizedPath(global_path_);
}

std::vector<VelocitySample> MathematicalNavigator::generateVelocitySamples() {
    std::vector<VelocitySample> samples;
    
    // 速度采样范围
    double vx_min = -max_velocity_;
    double vx_max = max_velocity_;
    double vy_min = -max_velocity_ * 0.5;  // 侧向速度限制
    double vy_max = max_velocity_ * 0.5;
    double vth_min = -max_angular_velocity_;
    double vth_max = max_angular_velocity_;
    
    // 采样步长
    double vx_step = max_velocity_ / 8.0;
    double vy_step = max_velocity_ / 6.0;
    double vth_step = max_angular_velocity_ / 6.0;
    
    // 生成速度样本
    for (double vx = vx_min; vx <= vx_max; vx += vx_step) {
        for (double vy = vy_min; vy <= vy_max; vy += vy_step) {
            for (double vth = vth_min; vth <= vth_max; vth += vth_step) {
                VelocitySample sample;
                sample.vx = vx;
                sample.vy = vy;
                sample.vth = vth;
                
                // 预测轨迹
                sample.trajectory = predictTrajectory(vx, vy, vth);
                
                // 检查轨迹有效性
                if (isTrajectoryValid(sample.trajectory)) {
                    // 计算各项评价指标
                    sample.goal_distance = evaluateGoalDistance(sample);
                    sample.path_deviation = evaluatePathFollowing(sample);
                    sample.obstacle_clearance = evaluateObstacleAvoidance(sample);
                    sample.smoothness = evaluateSmoothness(sample);
                    sample.energy_cost = evaluateEnergyEfficiency(sample);
                    sample.safety_margin = evaluateSafety(sample);
                    sample.time_to_goal = evaluateTimeEfficiency(sample);
                    
                    // 计算综合评分
                    sample.score = evaluateVelocitySample(sample);
                    
                    samples.push_back(sample);
                }
            }
        }
    }
    
    return samples;
}

VelocitySample MathematicalNavigator::selectOptimalVelocity(const std::vector<VelocitySample>& samples) {
    if (samples.empty()) {
        return VelocitySample();
    }
    
    // 找到评分最高的样本
    VelocitySample best_sample = samples[0];
    double best_score = best_sample.score;
    
    for (const auto& sample : samples) {
        if (sample.score > best_score) {
            best_score = sample.score;
            best_sample = sample;
        }
    }
    
    return best_sample;
}

double MathematicalNavigator::evaluateVelocitySample(const VelocitySample& sample) {
    // 使用自适应权重进行多目标优化
    double score = 0.0;
    
    score += adaptive_weights_(0) * weights_.goal_distance_weight * sample.goal_distance;
    score += adaptive_weights_(1) * weights_.path_following_weight * sample.path_deviation;
    score += adaptive_weights_(2) * weights_.obstacle_avoidance_weight * sample.obstacle_clearance;
    score += adaptive_weights_(3) * weights_.smoothness_weight * sample.smoothness;
    score += adaptive_weights_(4) * weights_.energy_efficiency_weight * sample.energy_cost;
    score += adaptive_weights_(5) * weights_.safety_weight * sample.safety_margin;
    score += adaptive_weights_(6) * weights_.time_efficiency_weight * sample.time_to_goal;
    
    return score;
}

double MathematicalNavigator::evaluateGoalDistance(const VelocitySample& sample) {
    if (sample.trajectory.empty()) {
        return 0.0;
    }
    
    // 计算轨迹终点到目标的距离
    Eigen::Vector2d trajectory_end = sample.trajectory.back();
    Eigen::Vector2d goal_2d(goal_pose_(0), goal_pose_(1));
    double distance = calculateDistance(trajectory_end, goal_2d);
    
    // 距离越近评分越高，使用负指数函数
    return exp(-distance / 2.0);
}

double MathematicalNavigator::evaluatePathFollowing(const VelocitySample& sample) {
    if (global_path_.empty() || sample.trajectory.empty()) {
        return 0.0;
    }
    
    double total_deviation = 0.0;
    int valid_points = 0;
    
    // 计算轨迹点到路径的偏差
    for (const auto& traj_point : sample.trajectory) {
        double min_distance = std::numeric_limits<double>::max();
        
        // 找到最近的路径点
        for (const auto& path_point : global_path_) {
            double distance = calculateDistance(traj_point, path_point);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
        
        if (min_distance < path_tolerance_) {
            total_deviation += min_distance;
            valid_points++;
        }
    }
    
    if (valid_points == 0) {
        return 0.0;
    }
    
    double avg_deviation = total_deviation / valid_points;
    return exp(-avg_deviation / path_tolerance_);
}

double MathematicalNavigator::evaluateObstacleAvoidance(const VelocitySample& sample) {
    if (obstacles_.empty() || sample.trajectory.empty()) {
        return 1.0;  // 无障碍物时满分
    }
    
    double min_clearance = std::numeric_limits<double>::max();
    
    // 计算轨迹到所有障碍物的最小距离
    for (const auto& traj_point : sample.trajectory) {
        for (const auto& obstacle : obstacles_) {
            double distance = calculateDistance(traj_point, obstacle);
            if (distance < min_clearance) {
                min_clearance = distance;
            }
        }
    }
    
    // 距离越远评分越高
    return std::min(1.0, min_clearance / obstacle_params_.safety_margin);
}

double MathematicalNavigator::evaluateSmoothness(const VelocitySample& sample) {
    if (sample.trajectory.size() < 3) {
        return 1.0;
    }
    
    double total_curvature = 0.0;
    
    // 计算轨迹的曲率变化
    for (size_t i = 1; i < sample.trajectory.size() - 1; ++i) {
        Eigen::Vector2d prev = sample.trajectory[i-1];
        Eigen::Vector2d curr = sample.trajectory[i];
        Eigen::Vector2d next = sample.trajectory[i+1];
        
        // 计算三点形成的角度
        double angle1 = calculateAngle(prev, curr);
        double angle2 = calculateAngle(curr, next);
        double curvature = std::abs(normalizeAngle(angle2 - angle1));
        
        total_curvature += curvature;
    }
    
    double avg_curvature = total_curvature / (sample.trajectory.size() - 2);
    return exp(-avg_curvature / M_PI);
}

double MathematicalNavigator::evaluateEnergyEfficiency(const VelocitySample& sample) {
    // 计算能量消耗（基于速度和加速度）
    double velocity_magnitude = sqrt(sample.vx * sample.vx + sample.vy * sample.vy);
    double angular_velocity_magnitude = std::abs(sample.vth);
    
    // 能量消耗模型
    double energy_cost = velocity_magnitude * velocity_magnitude + 
                        0.5 * angular_velocity_magnitude * angular_velocity_magnitude;
    
    // 能量消耗越低评分越高
    return exp(-energy_cost / (max_velocity_ * max_velocity_));
}

double MathematicalNavigator::evaluateSafety(const VelocitySample& sample) {
    // 安全检查：速度限制、加速度限制等
    double safety_score = 1.0;
    
    // 速度限制检查
    double velocity_magnitude = sqrt(sample.vx * sample.vx + sample.vy * sample.vy);
    if (velocity_magnitude > max_velocity_) {
        safety_score *= 0.5;
    }
    
    // 角速度限制检查
    if (std::abs(sample.vth) > max_angular_velocity_) {
        safety_score *= 0.5;
    }
    
    // 轨迹碰撞检查
    if (checkCollision(sample.trajectory)) {
        safety_score *= 0.1;
    }
    
    return safety_score;
}

double MathematicalNavigator::evaluateTimeEfficiency(const VelocitySample& sample) {
    if (sample.trajectory.empty()) {
        return 0.0;
    }
    
    // 计算到目标的时间（基于当前速度）
    double velocity_magnitude = sqrt(sample.vx * sample.vx + sample.vy * sample.vy);
    if (velocity_magnitude < 0.1) {
        return 0.0;
    }
    
    Eigen::Vector2d trajectory_end = sample.trajectory.back();
    Eigen::Vector2d goal_2d(goal_pose_(0), goal_pose_(1));
    double distance_to_goal = calculateDistance(trajectory_end, goal_2d);
    
    double time_to_goal = distance_to_goal / velocity_magnitude;
    
    // 时间越短评分越高
    return exp(-time_to_goal / 10.0);
}

std::vector<Eigen::Vector2d> MathematicalNavigator::predictTrajectory(double vx, double vy, double vth) {
    std::vector<Eigen::Vector2d> trajectory;
    
    // 预测时间范围
    double prediction_time = 2.0;  // 2秒
    double dt = 0.1;  // 0.1秒步长
    
    Eigen::Vector3d current_state = current_pose_;
    
    for (double t = 0; t <= prediction_time; t += dt) {
        // 简单的运动学模型
        double x = current_state(0) + vx * t * cos(current_state(2)) - vy * t * sin(current_state(2));
        double y = current_state(1) + vx * t * sin(current_state(2)) + vy * t * cos(current_state(2));
        
        trajectory.push_back(Eigen::Vector2d(x, y));
    }
    
    return trajectory;
}

bool MathematicalNavigator::isTrajectoryValid(const std::vector<Eigen::Vector2d>& trajectory) {
    if (trajectory.empty()) {
        return false;
    }
    
    // 检查轨迹是否与障碍物碰撞
    return !checkCollision(trajectory);
}

void MathematicalNavigator::updateAdaptiveWeights() {
    if (performance_history_.size() < adaptive_params_.adaptation_window) {
        return;
    }
    
    // 计算性能趋势
    double recent_performance = 0.0;
    double old_performance = 0.0;
    
    int half_window = adaptive_params_.adaptation_window / 2;
    
    for (int i = 0; i < half_window; ++i) {
        recent_performance += performance_history_[performance_history_.size() - 1 - i];
        old_performance += performance_history_[performance_history_.size() - 1 - half_window - i];
    }
    
    recent_performance /= half_window;
    old_performance /= half_window;
    
    // 计算性能变化
    double performance_change = recent_performance - old_performance;
    
    // 根据性能变化调整权重
    if (std::abs(performance_change) > adaptive_params_.adaptation_threshold) {
        // 计算自适应增益
        double adaptation_gain = adaptive_params_.learning_rate * performance_change;
        
        // 更新权重（保持权重在合理范围内）
        for (int i = 0; i < adaptive_weights_.size(); ++i) {
            adaptive_weights_(i) += adaptation_gain;
            adaptive_weights_(i) = std::max(0.1, std::min(2.0, adaptive_weights_(i)));
        }
        
        // 归一化权重
        double weight_sum = adaptive_weights_.sum();
        if (weight_sum > 0) {
            adaptive_weights_ /= weight_sum;
        }
    }
    
    // 更新自适应置信度
    adaptation_confidence_ = calculateAdaptationConfidence();
}

void MathematicalNavigator::updatePerformanceHistory(double performance) {
    performance_history_.push_back(performance);
    
    // 保持历史记录在窗口大小内
    if (performance_history_.size() > adaptive_params_.adaptation_window) {
        performance_history_.pop_front();
    }
}

double MathematicalNavigator::calculateAdaptationConfidence() {
    if (performance_history_.size() < 10) {
        return adaptive_params_.min_confidence;
    }
    
    // 计算性能的稳定性
    double mean_performance = 0.0;
    for (double perf : performance_history_) {
        mean_performance += perf;
    }
    mean_performance /= performance_history_.size();
    
    double variance = 0.0;
    for (double perf : performance_history_) {
        variance += (perf - mean_performance) * (perf - mean_performance);
    }
    variance /= performance_history_.size();
    
    double std_dev = sqrt(variance);
    double stability = 1.0 / (1.0 + std_dev);
    
    // 计算置信度
    double confidence = adaptive_params_.min_confidence + 
                       (adaptive_params_.max_confidence - adaptive_params_.min_confidence) * stability;
    
    return std::min(adaptive_params_.max_confidence, confidence);
}

void MathematicalNavigator::adaptControlParameters() {
    // 根据自适应置信度调整控制参数
    if (adaptation_confidence_ > 0.7) {
        // 高置信度时，可以更激进地调整参数
        weights_.goal_distance_weight *= (1.0 + 0.1 * adaptation_confidence_);
        weights_.obstacle_avoidance_weight *= (1.0 - 0.05 * adaptation_confidence_);
    } else if (adaptation_confidence_ < 0.3) {
        // 低置信度时，更保守地调整参数
        weights_.safety_weight *= (1.0 + 0.2 * (1.0 - adaptation_confidence_));
        weights_.obstacle_avoidance_weight *= (1.0 + 0.1 * (1.0 - adaptation_confidence_));
    }
    
    // 确保参数在合理范围内
    weights_.goal_distance_weight = std::max(0.5, std::min(2.0, weights_.goal_distance_weight));
    weights_.path_following_weight = std::max(0.3, std::min(1.5, weights_.path_following_weight));
    weights_.obstacle_avoidance_weight = std::max(0.8, std::min(2.0, weights_.obstacle_avoidance_weight));
    weights_.smoothness_weight = std::max(0.3, std::min(1.2, weights_.smoothness_weight));
    weights_.energy_efficiency_weight = std::max(0.2, std::min(1.0, weights_.energy_efficiency_weight));
    weights_.safety_weight = std::max(1.0, std::min(3.0, weights_.safety_weight));
    weights_.time_efficiency_weight = std::max(0.4, std::min(1.5, weights_.time_efficiency_weight));
}

void MathematicalNavigator::predictObstacleTrajectories() {
    predicted_obstacles_.clear();
    
    if (obstacles_.empty()) {
        return;
    }
    
    // 为每个障碍物预测轨迹
    for (const auto& obstacle : obstacles_) {
        std::vector<Eigen::Vector2d> obstacle_trajectory;
        
        // 简单的线性预测模型
        for (double t = 0; t <= obstacle_params_.prediction_horizon; t += obstacle_params_.prediction_step) {
            // 假设障碍物保持当前速度（这里简化处理）
            Eigen::Vector2d predicted_position = obstacle;
            
            // 添加不确定性
            double uncertainty_x = obstacle_params_.position_uncertainty * t;
            double uncertainty_y = obstacle_params_.position_uncertainty * t;
            
            predicted_position(0) += (rand() % 100 - 50) / 100.0 * uncertainty_x;
            predicted_position(1) += (rand() % 100 - 50) / 100.0 * uncertainty_y;
            
            obstacle_trajectory.push_back(predicted_position);
        }
        
        predicted_obstacles_.insert(predicted_obstacles_.end(), 
                                   obstacle_trajectory.begin(), 
                                   obstacle_trajectory.end());
    }
}

std::vector<Eigen::Vector2d> MathematicalNavigator::predictObstaclePosition(double time) {
    std::vector<Eigen::Vector2d> positions;
    
    if (obstacles_.empty()) {
        return positions;
    }
    
    // 预测指定时间点的障碍物位置
    for (const auto& obstacle : obstacles_) {
        Eigen::Vector2d predicted_position = obstacle;
        
        // 添加时间相关的不确定性
        double uncertainty = obstacle_params_.position_uncertainty * time;
        predicted_position(0) += (rand() % 100 - 50) / 100.0 * uncertainty;
        predicted_position(1) += (rand() % 100 - 50) / 100.0 * uncertainty;
        
        positions.push_back(predicted_position);
    }
    
    return positions;
}

double MathematicalNavigator::calculateCollisionRisk(const Eigen::Vector2d& position, double time) {
    if (predicted_obstacles_.empty()) {
        return 0.0;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    // 计算到所有预测障碍物的最小距离
    for (const auto& predicted_obstacle : predicted_obstacles_) {
        double distance = calculateDistance(position, predicted_obstacle);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    
    // 计算碰撞风险（距离越近风险越高）
    if (min_distance < obstacle_params_.collision_threshold) {
        return 1.0 - (min_distance / obstacle_params_.collision_threshold);
    } else {
        return 0.0;
    }
}

bool MathematicalNavigator::checkCollision(const std::vector<Eigen::Vector2d>& trajectory) {
    if (obstacles_.empty() || trajectory.empty()) {
        return false;
    }
    
    // 检查轨迹是否与障碍物碰撞
    for (const auto& traj_point : trajectory) {
        for (const auto& obstacle : obstacles_) {
            double distance = calculateDistance(traj_point, obstacle);
            if (distance < obstacle_params_.safety_margin) {
                return true;  // 发生碰撞
            }
        }
    }
    
    return false;  // 无碰撞
}

std::vector<Eigen::Vector2d> MathematicalNavigator::optimizePath(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        return path;
    }
    
    std::vector<Eigen::Vector2d> optimized_path = path;
    
    // 多次迭代优化
    for (int iteration = 0; iteration < path_params_.optimization_iterations; ++iteration) {
        std::vector<Eigen::Vector2d> new_path = optimized_path;
        
        // 对每个路径点进行优化（跳过起点和终点）
        for (size_t i = 1; i < optimized_path.size() - 1; ++i) {
            Eigen::Vector2d optimized_point = optimizePathPoint(optimized_path, i);
            new_path[i] = optimized_point;
        }
        
        // 检查收敛性
        double total_change = 0.0;
        for (size_t i = 0; i < optimized_path.size(); ++i) {
            total_change += calculateDistance(optimized_path[i], new_path[i]);
        }
        
        optimized_path = new_path;
        
        // 如果变化很小，提前结束
        if (total_change < path_params_.convergence_threshold) {
            break;
        }
    }
    
    // 平滑处理
    optimized_path = smoothPath(optimized_path);
    
    return optimized_path;
}

Eigen::Vector2d MathematicalNavigator::optimizePathPoint(const std::vector<Eigen::Vector2d>& path, size_t index) {
    if (index == 0 || index >= path.size() - 1) {
        return path[index];
    }
    
    Eigen::Vector2d current_point = path[index];
    Eigen::Vector2d prev_point = path[index - 1];
    Eigen::Vector2d next_point = path[index + 1];
    
    // 计算当前曲率
    double current_curvature = calculatePointCurvature(prev_point, current_point, next_point);
    
    // 尝试不同的偏移量
    double best_score = std::numeric_limits<double>::max();
    Eigen::Vector2d best_point = current_point;
    
    for (double offset_x = -0.1; offset_x <= 0.1; offset_x += 0.02) {
        for (double offset_y = -0.1; offset_y <= 0.1; offset_y += 0.02) {
            Eigen::Vector2d test_point = current_point + Eigen::Vector2d(offset_x, offset_y);
            
            // 计算新曲率
            double new_curvature = calculatePointCurvature(prev_point, test_point, next_point);
            
            // 计算优化目标：曲率 + 路径长度 + 障碍物避让
            double curvature_cost = path_params_.curvature_weight * new_curvature;
            double length_cost = path_params_.length_weight * 
                               (calculateDistance(prev_point, test_point) + calculateDistance(test_point, next_point));
            double obstacle_cost = calculateObstacleCost(test_point);
            
            double total_cost = curvature_cost + length_cost + obstacle_cost;
            
            if (total_cost < best_score) {
                best_score = total_cost;
                best_point = test_point;
            }
        }
    }
    
    return best_point;
}

double MathematicalNavigator::calculatePointCurvature(const Eigen::Vector2d& prev, 
                                                     const Eigen::Vector2d& curr, 
                                                     const Eigen::Vector2d& next) {
    // 计算三点形成的曲率
    Eigen::Vector2d v1 = curr - prev;
    Eigen::Vector2d v2 = next - curr;
    
    double cross_product = v1(0) * v2(1) - v1(1) * v2(0);
    double dot_product = v1.dot(v2);
    
    double curvature = std::abs(cross_product) / (v1.norm() * v2.norm() + 1e-6);
    
    return curvature;
}

double MathematicalNavigator::calculateObstacleCost(const Eigen::Vector2d& point) {
    if (obstacles_.empty()) {
        return 0.0;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : obstacles_) {
        double distance = calculateDistance(point, obstacle);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    
    // 距离越近成本越高
    if (min_distance < obstacle_params_.safety_margin) {
        return 10.0 * (obstacle_params_.safety_margin - min_distance);
    } else {
        return 0.0;
    }
}

std::vector<Eigen::Vector2d> MathematicalNavigator::smoothPath(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        return path;
    }
    
    std::vector<Eigen::Vector2d> smoothed_path = path;
    
    // 使用移动平均平滑
    for (size_t i = 1; i < path.size() - 1; ++i) {
        Eigen::Vector2d smoothed_point = Eigen::Vector2d::Zero();
        int count = 0;
        
        // 计算周围点的平均值
        for (int j = -1; j <= 1; ++j) {
            if (i + j >= 0 && i + j < path.size()) {
                smoothed_point += path[i + j];
                count++;
            }
        }
        
        if (count > 0) {
            smoothed_point /= count;
            
            // 使用平滑因子进行插值
            smoothed_path[i] = path_params_.smoothing_factor * smoothed_point + 
                              (1.0 - path_params_.smoothing_factor) * path[i];
        }
    }
    
    return smoothed_path;
}

double MathematicalNavigator::calculatePathCurvature(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        return 0.0;
    }
    
    double total_curvature = 0.0;
    int valid_points = 0;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        double curvature = calculatePointCurvature(path[i-1], path[i], path[i+1]);
        total_curvature += curvature;
        valid_points++;
    }
    
    if (valid_points == 0) {
        return 0.0;
    }
    
    return total_curvature / valid_points;
}

bool MathematicalNavigator::isPathValid() {
    if (global_path_.empty()) {
        return false;
    }
    
    // 检查路径是否与障碍物碰撞
    for (const auto& path_point : global_path_) {
        for (const auto& obstacle : obstacles_) {
            double distance = calculateDistance(path_point, obstacle);
            if (distance < obstacle_params_.safety_margin) {
                return false;
            }
        }
    }
    
    // 检查路径连续性
    for (size_t i = 1; i < global_path_.size(); ++i) {
        double distance = calculateDistance(global_path_[i-1], global_path_[i]);
        if (distance > 1.0) {  // 路径点间距过大
            return false;
        }
    }
    
    return true;
}

bool MathematicalNavigator::initialize() {
    ROS_INFO("Initializing Mathematical Navigator...");
    
    // 检查参数有效性
    if (max_velocity_ <= 0 || max_angular_velocity_ <= 0) {
        ROS_ERROR("Invalid velocity parameters");
        return false;
    }
    
    if (control_frequency_ <= 0) {
        ROS_ERROR("Invalid control frequency");
        return false;
    }
    
    // 初始化自适应权重
    adaptive_weights_ = Eigen::VectorXd::Ones(7);
    adaptive_weights_ /= adaptive_weights_.sum();
    
    ROS_INFO("Mathematical Navigator initialized successfully");
    return true;
}

void MathematicalNavigator::run() {
    ros::Rate rate(control_frequency_);
    
    ROS_INFO("Starting Mathematical Navigator control loop...");
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // 更新控制循环
        updateControlLoop();
        
        rate.sleep();
    }
}

void MathematicalNavigator::stop() {
    // 发布停止指令
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_.publish(stop_cmd);
    
    ROS_INFO("Mathematical Navigator stopped");
}

void MathematicalNavigator::updateControlLoop() {
    // 检查是否到达目标
    if (isGoalReached()) {
        if (!is_goal_reached_) {
            ROS_INFO("Goal reached!");
            is_goal_reached_ = true;
        }
        return;
    }
    
    // 检查路径有效性
    if (!isPathValid()) {
        ROS_WARN("Path is not valid, waiting for new path...");
        return;
    }
    
    // 生成速度样本
    std::vector<VelocitySample> samples = generateVelocitySamples();
    
    if (samples.empty()) {
        ROS_WARN("No valid velocity samples generated");
        return;
    }
    
    // 选择最优速度
    VelocitySample optimal_sample = selectOptimalVelocity(samples);
    
    // 安全检查
    if (!isVelocitySafe(optimal_sample)) {
        ROS_WARN("Selected velocity is not safe, applying safety limits");
        applyVelocityLimits(optimal_sample);
    }
    
    // 发布速度指令
    publishVelocityCommand(optimal_sample);
    
    // 发布调试信息
    publishDebugInfo(optimal_sample);
    
    // 发布障碍物预测
    publishObstaclePrediction();
    
    // 更新性能历史
    double performance = optimal_sample.score;
    updatePerformanceHistory(performance);
    
    // 自适应控制更新
    static int adaptation_counter = 0;
    adaptation_counter++;
    if (adaptation_counter >= 10) {  // 每10个控制周期更新一次
        updateAdaptiveWeights();
        adaptControlParameters();
        adaptation_counter = 0;
    }
    
    // 保存速度历史
    velocity_history_.push_back(optimal_sample);
    if (velocity_history_.size() > 50) {
        velocity_history_.pop_front();
    }
}

void MathematicalNavigator::publishVelocityCommand(const VelocitySample& sample) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = sample.vx;
    cmd_vel.linear.y = sample.vy;
    cmd_vel.angular.z = sample.vth;
    
    cmd_vel_pub_.publish(cmd_vel);
}

void MathematicalNavigator::publishOptimizedPath(const std::vector<Eigen::Vector2d>& path) {
    nav_msgs::Path optimized_path_msg;
    optimized_path_msg.header.frame_id = "map";
    optimized_path_msg.header.stamp = ros::Time::now();
    
    for (const auto& point : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = point(0);
        pose.pose.position.y = point(1);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        optimized_path_msg.poses.push_back(pose);
    }
    
    optimized_path_pub_.publish(optimized_path_msg);
}

void MathematicalNavigator::publishObstaclePrediction() {
    nav_msgs::Path prediction_msg;
    prediction_msg.header.frame_id = "map";
    prediction_msg.header.stamp = ros::Time::now();
    
    for (const auto& obstacle : predicted_obstacles_) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = obstacle(0);
        pose.pose.position.y = obstacle(1);
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        prediction_msg.poses.push_back(pose);
    }
    
    prediction_pub_.publish(prediction_msg);
}

void MathematicalNavigator::publishDebugInfo(const VelocitySample& sample) {
    geometry_msgs::Twist debug_msg;
    debug_msg.linear.x = sample.goal_distance;
    debug_msg.linear.y = sample.path_deviation;
    debug_msg.angular.x = sample.obstacle_clearance;
    debug_msg.angular.y = sample.smoothness;
    debug_msg.angular.z = sample.score;
    
    debug_pub_.publish(debug_msg);
}

bool MathematicalNavigator::isVelocitySafe(const VelocitySample& sample) {
    // 检查速度限制
    double velocity_magnitude = sqrt(sample.vx * sample.vx + sample.vy * sample.vy);
    if (velocity_magnitude > max_velocity_) {
        return false;
    }
    
    if (std::abs(sample.vth) > max_angular_velocity_) {
        return false;
    }
    
    // 检查轨迹安全性
    if (checkCollision(sample.trajectory)) {
        return false;
    }
    
    // 检查加速度限制
    if (!velocity_history_.empty()) {
        const VelocitySample& last_sample = velocity_history_.back();
        double acc_x = (sample.vx - last_sample.vx) * control_frequency_;
        double acc_y = (sample.vy - last_sample.vy) * control_frequency_;
        double acc_th = (sample.vth - last_sample.vth) * control_frequency_;
        
        if (std::abs(acc_x) > max_acceleration_ || 
            std::abs(acc_y) > max_acceleration_ || 
            std::abs(acc_th) > max_acceleration_) {
            return false;
        }
    }
    
    return true;
}

void MathematicalNavigator::applyVelocityLimits(VelocitySample& sample) {
    // 限制线速度
    double velocity_magnitude = sqrt(sample.vx * sample.vx + sample.vy * sample.vy);
    if (velocity_magnitude > max_velocity_) {
        double scale = max_velocity_ / velocity_magnitude;
        sample.vx *= scale;
        sample.vy *= scale;
    }
    
    // 限制角速度
    if (std::abs(sample.vth) > max_angular_velocity_) {
        sample.vth = (sample.vth > 0) ? max_angular_velocity_ : -max_angular_velocity_;
    }
    
    // 重新计算轨迹和评分
    sample.trajectory = predictTrajectory(sample.vx, sample.vy, sample.vth);
    sample.score = evaluateVelocitySample(sample);
}

bool MathematicalNavigator::isGoalReached() {
    if (global_path_.empty()) {
        return false;
    }
    
    // 检查当前位置到目标的距离
    Eigen::Vector2d current_pos(current_pose_(0), current_pose_(1));
    Eigen::Vector2d goal_pos(goal_pose_(0), goal_pose_(1));
    double distance = calculateDistance(current_pos, goal_pos);
    
    return distance < goal_tolerance_;
}

double MathematicalNavigator::calculateDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    return (p2 - p1).norm();
}

double MathematicalNavigator::calculateAngle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
    return atan2(p2(1) - p1(1), p2(0) - p1(0));
}

double MathematicalNavigator::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

Eigen::Vector2d MathematicalNavigator::rotatePoint(const Eigen::Vector2d& point, double angle) {
    double cos_angle = cos(angle);
    double sin_angle = sin(angle);
    
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos_angle, -sin_angle,
                      sin_angle,  cos_angle;
    
    return rotation_matrix * point;
}

} // namespace nav_converter 