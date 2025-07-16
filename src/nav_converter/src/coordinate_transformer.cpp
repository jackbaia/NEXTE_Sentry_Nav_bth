/**
 * @file coordinate_transformer.cpp
 * @brief 坐标系转换节点
 * 
 * 此节点提供ENU和NED坐标系之间的转换功能，
 * 确保位置命令与MAVLink协议兼容。
 */

#include <ros/ros.h>
#include <nav_converter/PositionCommand.h>
#include <nav_converter/MavlinkPositionCommand.h>
#include <Eigen/Dense>
#include <string>

class CoordinateTransformer {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber position_cmd_sub_;
    
    // 发布者
    ros::Publisher transformed_cmd_pub_;
    
    // 参数
    std::string input_frame_;
    std::string output_frame_;
    bool enable_transform_;
    
public:
    CoordinateTransformer() : 
        nh_(),
        private_nh_("~") {
        
        // 加载参数
        loadParameters();
        
        // 初始化订阅者和发布者
        position_cmd_sub_ = nh_.subscribe("input_position_cmd", 10, 
                                         &CoordinateTransformer::positionCmdCallback, this);
        transformed_cmd_pub_ = nh_.advertise<nav_converter::PositionCommand>("output_position_cmd", 10);
        
        ROS_INFO("CoordinateTransformer initialized");
        ROS_INFO("Input frame: %s", input_frame_.c_str());
        ROS_INFO("Output frame: %s", output_frame_.c_str());
        ROS_INFO("Transform enabled: %s", enable_transform_ ? "true" : "false");
    }
    
    void run() {
        ros::spin();
    }
    
private:
    void loadParameters() {
        private_nh_.param("input_frame", input_frame_, std::string("enu"));
        private_nh_.param("output_frame", output_frame_, std::string("ned"));
        private_nh_.param("enable_transform", enable_transform_, true);
    }
    
    void positionCmdCallback(const nav_converter::PositionCommand::ConstPtr& msg) {
        nav_converter::PositionCommand transformed_msg = *msg;
        
        if (enable_transform_) {
            // 执行坐标系转换
            transformCoordinates(transformed_msg);
        }
        
        // 发布转换后的消息
        transformed_cmd_pub_.publish(transformed_msg);
    }
    
    void transformCoordinates(nav_converter::PositionCommand& msg) {
        // ENU到NED的转换
        if (input_frame_ == "enu" && output_frame_ == "ned") {
            // 位置转换
            Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
            position = enuToNed(position);
            msg.position.x = position[0];
            msg.position.y = position[1];
            msg.position.z = position[2];
            
            // 速度转换
            Eigen::Vector3d velocity(msg.velocity.x, msg.velocity.y, msg.velocity.z);
            velocity = enuToNed(velocity);
            msg.velocity.x = velocity[0];
            msg.velocity.y = velocity[1];
            msg.velocity.z = velocity[2];
            
            // 加速度转换
            Eigen::Vector3d acceleration(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
            acceleration = enuToNed(acceleration);
            msg.acceleration.x = acceleration[0];
            msg.acceleration.y = acceleration[1];
            msg.acceleration.z = acceleration[2];
            
            // 加加速度转换
            Eigen::Vector3d jerk(msg.jerk.x, msg.jerk.y, msg.jerk.z);
            jerk = enuToNed(jerk);
            msg.jerk.x = jerk[0];
            msg.jerk.y = jerk[1];
            msg.jerk.z = jerk[2];
            
            // 偏航角转换（ENU到NED，偏航角需要加π/2）
            msg.yaw = normalizeAngle(msg.yaw + M_PI/2);
            
            ROS_DEBUG("Transformed ENU to NED coordinates");
        }
        // NED到ENU的转换
        else if (input_frame_ == "ned" && output_frame_ == "enu") {
            // 位置转换
            Eigen::Vector3d position(msg.position.x, msg.position.y, msg.position.z);
            position = nedToEnu(position);
            msg.position.x = position[0];
            msg.position.y = position[1];
            msg.position.z = position[2];
            
            // 速度转换
            Eigen::Vector3d velocity(msg.velocity.x, msg.velocity.y, msg.velocity.z);
            velocity = nedToEnu(velocity);
            msg.velocity.x = velocity[0];
            msg.velocity.y = velocity[1];
            msg.velocity.z = velocity[2];
            
            // 加速度转换
            Eigen::Vector3d acceleration(msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
            acceleration = nedToEnu(acceleration);
            msg.acceleration.x = acceleration[0];
            msg.acceleration.y = acceleration[1];
            msg.acceleration.z = acceleration[2];
            
            // 加加速度转换
            Eigen::Vector3d jerk(msg.jerk.x, msg.jerk.y, msg.jerk.z);
            jerk = nedToEnu(jerk);
            msg.jerk.x = jerk[0];
            msg.jerk.y = jerk[1];
            msg.jerk.z = jerk[2];
            
            // 偏航角转换（NED到ENU，偏航角需要减π/2）
            msg.yaw = normalizeAngle(msg.yaw - M_PI/2);
            
            ROS_DEBUG("Transformed NED to ENU coordinates");
        }
    }
    
    // ENU到NED的转换矩阵
    Eigen::Vector3d enuToNed(const Eigen::Vector3d& enu) {
        // ENU到NED的转换矩阵
        // NED = [0, 1, 0; 1, 0, 0; 0, 0, -1] * ENU
        Eigen::Vector3d ned;
        ned[0] = enu[1];   // N = E
        ned[1] = enu[0];   // E = N
        ned[2] = -enu[2];  // D = -U
        return ned;
    }
    
    // NED到ENU的转换矩阵
    Eigen::Vector3d nedToEnu(const Eigen::Vector3d& ned) {
        // NED到ENU的转换矩阵
        // ENU = [0, 1, 0; 1, 0, 0; 0, 0, -1] * NED
        Eigen::Vector3d enu;
        enu[0] = ned[1];   // E = N
        enu[1] = ned[0];   // N = E
        enu[2] = -ned[2];  // U = -D
        return enu;
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinate_transformer");
    
    CoordinateTransformer transformer;
    transformer.run();
    
    return 0;
} 