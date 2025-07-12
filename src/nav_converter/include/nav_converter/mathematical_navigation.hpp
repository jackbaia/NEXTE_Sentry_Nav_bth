/**
 * @file mathematical_navigation.hpp
 * @brief 基于数学优化的导航系统
 * 
 * 使用先进的数学方法优化move_base算法：
 * 1. 多目标优化算法
 * 2. 自适应控制理论
 * 3. 动态障碍物预测
 * 4. 路径平滑优化
 * 5. 实时性能优化
 */

#ifndef MATHEMATICAL_NAVIGATION_HPP
#define MATHEMATICAL_NAVIGATION_HPP

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <memory>
#include <functional>

namespace nav_converter {

/**
 * @brief 多目标优化权重结构
 */
struct OptimizationWeights {
    double goal_distance_weight = 1.0;      // 目标距离权重
    double path_following_weight = 0.8;     // 路径跟踪权重
    double obstacle_avoidance_weight = 1.2; // 障碍物避让权重
    double smoothness_weight = 0.6;         // 平滑度权重
    double energy_efficiency_weight = 0.4;  // 能量效率权重
    double safety_weight = 1.5;             // 安全性权重
    double time_efficiency_weight = 0.7;    // 时间效率权重
};

/**
 * @brief 自适应控制参数
 */
struct AdaptiveControlParams {
    double learning_rate = 0.01;            // 学习率
    double forgetting_factor = 0.95;        // 遗忘因子
    double adaptation_threshold = 0.1;      // 自适应阈值
    int adaptation_window = 50;             // 自适应窗口大小
    double min_confidence = 0.3;            // 最小置信度
    double max_confidence = 0.9;            // 最大置信度
};

/**
 * @brief 动态障碍物预测参数
 */
struct ObstaclePredictionParams {
    double prediction_horizon = 3.0;        // 预测时间范围
    double prediction_step = 0.1;           // 预测时间步长
    double velocity_uncertainty = 0.2;      // 速度不确定性
    double position_uncertainty = 0.1;      // 位置不确定性
    double collision_threshold = 0.5;       // 碰撞阈值
    double safety_margin = 0.3;             // 安全边距
};

/**
 * @brief 路径优化参数
 */
struct PathOptimizationParams {
    double smoothing_factor = 0.8;          // 平滑因子
    double curvature_weight = 0.5;          // 曲率权重
    double length_weight = 0.3;             // 长度权重
    int optimization_iterations = 10;       // 优化迭代次数
    double convergence_threshold = 1e-4;    // 收敛阈值
};

/**
 * @brief 速度样本结构
 */
struct VelocitySample {
    double vx = 0.0;                        // 线速度x
    double vy = 0.0;                        // 线速度y
    double vth = 0.0;                       // 角速度
    double score = 0.0;                     // 综合评分
    std::vector<Eigen::Vector2d> trajectory; // 预测轨迹
    double goal_distance = 0.0;             // 到目标距离
    double path_deviation = 0.0;            // 路径偏差
    double obstacle_clearance = 0.0;        // 障碍物清除距离
    double smoothness = 0.0;                // 平滑度
    double energy_cost = 0.0;               // 能量消耗
    double safety_margin = 0.0;             // 安全边距
    double time_to_goal = 0.0;              // 到目标时间
};

/**
 * @brief 数学优化导航器
 * 
 * 基于多目标优化和自适应控制的先进导航算法
 */
class MathematicalNavigator {
public:
    /**
     * @brief 构造函数
     */
    MathematicalNavigator(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    
    /**
     * @brief 析构函数
     */
    ~MathematicalNavigator();
    
    /**
     * @brief 初始化导航器
     */
    bool initialize();
    
    /**
     * @brief 主控制循环
     */
    void run();
    
    /**
     * @brief 停止导航
     */
    void stop();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber path_sub_;
    
    // 发布者
    ros::Publisher cmd_vel_pub_;
    ros::Publisher optimized_path_pub_;
    ros::Publisher prediction_pub_;
    ros::Publisher debug_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 参数
    OptimizationWeights weights_;
    AdaptiveControlParams adaptive_params_;
    ObstaclePredictionParams obstacle_params_;
    PathOptimizationParams path_params_;
    
    // 状态变量
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d goal_pose_;
    std::vector<Eigen::Vector2d> global_path_;
    std::vector<Eigen::Vector2d> obstacles_;
    std::vector<Eigen::Vector2d> predicted_obstacles_;
    
    // 控制参数
    double max_velocity_;
    double max_acceleration_;
    double max_angular_velocity_;
    double control_frequency_;
    double goal_tolerance_;
    double path_tolerance_;
    
    // 自适应控制状态
    std::deque<double> performance_history_;
    std::deque<VelocitySample> velocity_history_;
    double adaptation_confidence_;
    Eigen::VectorXd adaptive_weights_;
    
    // 优化状态
    bool is_goal_reached_;
    bool is_path_valid_;
    double last_optimization_time_;
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    
    // 核心算法函数
    std::vector<VelocitySample> generateVelocitySamples();
    VelocitySample selectOptimalVelocity(const std::vector<VelocitySample>& samples);
    double evaluateVelocitySample(const VelocitySample& sample);
    
    // 多目标优化函数
    double evaluateGoalDistance(const VelocitySample& sample);
    double evaluatePathFollowing(const VelocitySample& sample);
    double evaluateObstacleAvoidance(const VelocitySample& sample);
    double evaluateSmoothness(const VelocitySample& sample);
    double evaluateEnergyEfficiency(const VelocitySample& sample);
    double evaluateSafety(const VelocitySample& sample);
    double evaluateTimeEfficiency(const VelocitySample& sample);
    
    // 自适应控制函数
    void updateAdaptiveWeights();
    void updatePerformanceHistory(double performance);
    double calculateAdaptationConfidence();
    void adaptControlParameters();
    
    // 动态障碍物预测
    void predictObstacleTrajectories();
    std::vector<Eigen::Vector2d> predictObstaclePosition(double time);
    double calculateCollisionRisk(const Eigen::Vector2d& position, double time);
    
    // 路径优化
    std::vector<Eigen::Vector2d> optimizePath(const std::vector<Eigen::Vector2d>& path);
    Eigen::Vector2d optimizePathPoint(const std::vector<Eigen::Vector2d>& path, size_t index);
    double calculatePointCurvature(const Eigen::Vector2d& prev, const Eigen::Vector2d& curr, const Eigen::Vector2d& next);
    double calculateObstacleCost(const Eigen::Vector2d& point);
    std::vector<Eigen::Vector2d> smoothPath(const std::vector<Eigen::Vector2d>& path);
    double calculatePathCurvature(const std::vector<Eigen::Vector2d>& path);
    
    // 轨迹预测
    std::vector<Eigen::Vector2d> predictTrajectory(double vx, double vy, double vth);
    bool isTrajectoryValid(const std::vector<Eigen::Vector2d>& trajectory);
    
    // 数学工具函数
    double calculateDistance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double calculateAngle(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    double normalizeAngle(double angle);
    Eigen::Vector2d rotatePoint(const Eigen::Vector2d& point, double angle);
    
    // 安全检查和限制
    bool isVelocitySafe(const VelocitySample& sample);
    void applyVelocityLimits(VelocitySample& sample);
    bool checkCollision(const std::vector<Eigen::Vector2d>& trajectory);
    
    // 发布函数
    void publishVelocityCommand(const VelocitySample& sample);
    void publishOptimizedPath(const std::vector<Eigen::Vector2d>& path);
    void publishObstaclePrediction();
    void publishDebugInfo(const VelocitySample& sample);
    
    // 辅助函数
    void loadParameters();
    bool isGoalReached();
    bool isPathValid();
    void resetState();
    void updateControlLoop();
};

} // namespace nav_converter

#endif // MATHEMATICAL_NAVIGATION_HPP 