/**
 * @file cmd_vel_to_position.cpp
 * @brief 将速度命令转换为位置命令，用于无人机控制
 * 
 * 此节点订阅来自move_base的geometry_msgs::Twist (cmd_vel)
 * 并发布quadrotor_msgs::PositionCommand给px4ctrl。
 * 
 * 转换过程包括：
 * 1. 速度积分以获得位置
 * 2. 偏航角处理
 * 3. 安全检查和安全限制
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <string>
#include <cmath>

// 包含生成的消息
#include <nav_converter/PositionCommand.h>

class CmdVelToPositionConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;
    
    // 发布者
    ros::Publisher position_cmd_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 状态变量
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    double current_yaw_;
    double current_yaw_rate_;
    
    // 命令变量
    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_velocity_;
    double target_yaw_;
    double target_yaw_rate_;
    
    // 参数
    double max_velocity_;
    double max_acceleration_;
    double max_yaw_rate_;
    double position_tolerance_;
    double yaw_tolerance_;
    double control_frequency_;
    double integration_dt_;
    double fixed_height_;
    
    // 安全标志
    bool odom_received_;
    bool cmd_vel_received_;
    ros::Time last_cmd_time_;
    ros::Time last_odom_time_;
    
    // 积分变量
    Eigen::Vector3d integrated_position_;
    double integrated_yaw_;
    ros::Time last_integration_time_;
    
public:
    CmdVelToPositionConverter() : 
        nh_(),
        private_nh_("~"),
        current_position_(Eigen::Vector3d::Zero()),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_yaw_(0.0),
        current_yaw_rate_(0.0),
        target_position_(Eigen::Vector3d::Zero()),
        target_velocity_(Eigen::Vector3d::Zero()),
        target_yaw_(0.0),
        target_yaw_rate_(0.0),
        odom_received_(false),
        cmd_vel_received_(false),
        integrated_position_(Eigen::Vector3d::Zero()),
        integrated_yaw_(0.0) {
        
        // 加载参数
        loadParameters();
        
        // 初始化订阅者
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &CmdVelToPositionConverter::cmdVelCallback, this);
        odom_sub_ = nh_.subscribe("odom", 10, &CmdVelToPositionConverter::odomCallback, this);
        
        // 初始化发布者
        position_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("cmd", 10);
        
        // 初始化时间
        last_integration_time_ = ros::Time::now();
        
        ROS_INFO("CmdVelToPositionConverter initialized");
    }
    
    void run() {
        ros::Rate rate(control_frequency_);
        
        while (ros::ok()) {
            ros::spinOnce();
            
            // Check if necessary data is received
            if (!odom_received_) {
                ROS_WARN_THROTTLE(5.0, "Waiting for odometry data...");
                rate.sleep();
                continue;
            }
            
            // 处理速度积分
            integrateVelocity();
            
            // 发布位置命令
            publishPositionCommand();
            
            rate.sleep();
        }
    }
    
private:
    void loadParameters() {
        // 加载参数，使用默认值
        private_nh_.param("max_velocity", max_velocity_, 2.0);
        private_nh_.param("max_acceleration", max_acceleration_, 1.0);
        private_nh_.param("max_yaw_rate", max_yaw_rate_, 1.0);
        private_nh_.param("position_tolerance", position_tolerance_, 0.1);
        private_nh_.param("yaw_tolerance", yaw_tolerance_, 0.1);
        private_nh_.param("control_frequency", control_frequency_, 50.0);
        private_nh_.param("fixed_height", fixed_height_, 1.5);
        
        integration_dt_ = 1.0 / control_frequency_;
        
        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Max velocity: %.2f m/s", max_velocity_);
        ROS_INFO("  Max acceleration: %.2f m/s²", max_acceleration_);
        ROS_INFO("  Max yaw rate: %.2f rad/s", max_yaw_rate_);
        ROS_INFO("  Control frequency: %.1f Hz", control_frequency_);
        ROS_INFO("  Fixed height: %.2f m", fixed_height_);
    }
    
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 存储接收到的速度命令
        target_velocity_[0] = msg->linear.x;
        target_velocity_[1] = msg->linear.y;
        target_velocity_[2] = msg->linear.z;
        target_yaw_rate_ = msg->angular.z;
        
        // 应用速度限制
        limitVelocity(target_velocity_);
        limitYawRate(target_yaw_rate_);
        
        cmd_vel_received_ = true;
        last_cmd_time_ = ros::Time::now();
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
        current_yaw_rate_ = msg->twist.twist.angular.z;
        
        odom_received_ = true;
        last_odom_time_ = ros::Time::now();
    }
    
    void integrateVelocity() {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_integration_time_).toSec();
        
        // 限制dt以防止大幅跳跃
        if (dt > 0.1) dt = 0.1;
        
        // 只有在最近接收到速度命令时才进行积分
        if (cmd_vel_received_ && (current_time - last_cmd_time_).toSec() < 1.0) {
            // 积分位置
            integrated_position_ += target_velocity_ * dt;
            
            // 积分偏航角
            integrated_yaw_ += target_yaw_rate_ * dt;
            
            // 将偏航角标准化到[-pi, pi]
            integrated_yaw_ = normalizeAngle(integrated_yaw_);
        } else {
            // 如果没有最近的命令，保持当前位置
            integrated_position_ = current_position_;
            integrated_yaw_ = current_yaw_;
        }
        
        last_integration_time_ = current_time;
    }
    
    void publishPositionCommand() {
        // 创建位置命令消息
        nav_converter::PositionCommand position_cmd;
        
        // 设置头部
        position_cmd.header.stamp = ros::Time::now();
        position_cmd.header.frame_id = "map";
        
        // 设置位置
        position_cmd.position.x = integrated_position_[0];
        position_cmd.position.y = integrated_position_[1];
        position_cmd.position.z = fixed_height_;
        
        // 设置速度
        position_cmd.velocity.x = target_velocity_[0];
        position_cmd.velocity.y = target_velocity_[1];
        position_cmd.velocity.z = target_velocity_[2];
        
        // 设置加速度（目前为零，可以从速度差计算）
        position_cmd.acceleration.x = 0.0;
        position_cmd.acceleration.y = 0.0;
        position_cmd.acceleration.z = 0.0;
        
        // 设置加加速度（目前为零）
        position_cmd.jerk.x = 0.0;
        position_cmd.jerk.y = 0.0;
        position_cmd.jerk.z = 0.0;
        
        // 设置偏航角和偏航角速度
        position_cmd.yaw = integrated_yaw_;
        position_cmd.yaw_dot = target_yaw_rate_;
        
        // 设置增益（默认值）
        position_cmd.kx[0] = 1.0;
        position_cmd.kx[1] = 1.0;
        position_cmd.kx[2] = 1.0;
        position_cmd.kv[0] = 1.0;
        position_cmd.kv[1] = 1.0;
        position_cmd.kv[2] = 1.0;
        
        // 设置轨迹信息
        position_cmd.trajectory_id = 1;
        position_cmd.trajectory_flag = nav_converter::PositionCommand::TRAJECTORY_STATUS_READY;
        
        // 发布
        position_cmd_pub_.publish(position_cmd);
    }
    
    void limitVelocity(Eigen::Vector3d& velocity) {
        double vel_magnitude = velocity.norm();
        if (vel_magnitude > max_velocity_) {
            velocity = velocity.normalized() * max_velocity_;
        }
    }
    
    void limitYawRate(double& yaw_rate) {
        if (std::abs(yaw_rate) > max_yaw_rate_) {
            yaw_rate = (yaw_rate > 0) ? max_yaw_rate_ : -max_yaw_rate_;
        }
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_to_position");
    
    try {
        CmdVelToPositionConverter converter;
        converter.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main function: %s", e.what());
        return 1;
    }
    
    return 0;
} 