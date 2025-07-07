/**
 * @file cmd_vel_to_position.h
 * @brief Header file for CmdVelToPositionConverter class
 * 
 * This class converts velocity commands from move_base into position commands
 * suitable for drone control systems.
 */

#ifndef CMD_VEL_TO_POSITION_H
#define CMD_VEL_TO_POSITION_H

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

// Include generated message
#include <nav_converter/PositionCommand.h>

class CmdVelToPositionConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscribers
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber odom_sub_;
    
    // Publishers
    ros::Publisher position_cmd_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // State variables
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velocity_;
    double current_yaw_;
    double current_yaw_rate_;
    
    // Command variables
    Eigen::Vector3d target_position_;
    Eigen::Vector3d target_velocity_;
    double target_yaw_;
    double target_yaw_rate_;
    
    // Parameters
    double max_velocity_;
    double max_acceleration_;
    double max_yaw_rate_;
    double position_tolerance_;
    double yaw_tolerance_;
    double control_frequency_;
    double integration_dt_;
    double integration_timeout_;
    double max_integration_dt_;
    bool enable_safety_checks_;
    double emergency_stop_timeout_;
    
    // Fixed height for drone
    double fixed_height_; // meters
    
    // Safety flags
    bool odom_received_;
    bool cmd_vel_received_;
    ros::Time last_cmd_time_;
    ros::Time last_odom_time_;
    
    // Integration variables
    Eigen::Vector3d integrated_position_;
    double integrated_yaw_;
    ros::Time last_integration_time_;
    
public:
    CmdVelToPositionConverter();
    ~CmdVelToPositionConverter() = default;
    
    void run();
    
private:
    void loadParameters();
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void integrateVelocity();
    void publishPositionCommand();
    void limitVelocity(Eigen::Vector3d& velocity);
    void limitYawRate(double& yaw_rate);
    double normalizeAngle(double angle);
    bool checkSafety();
};

#endif // CMD_VEL_TO_POSITION_H 