#include "nav_converter/mathematical_navigation.hpp"
#include <signal.h>

bool g_shutdown = false;

void signalHandler(int sig) {
    g_shutdown = true;
    ROS_INFO("Received shutdown signal, stopping mathematical navigation...");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mathematical_navigation_node", ros::init_options::NoSigintHandler);
    
    // 设置信号处理器
    signal(SIGINT, signalHandler);
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 创建数学导航器
    nav_converter::MathematicalNavigator navigator(nh, private_nh);
    
    // 初始化导航器
    if (!navigator.initialize()) {
        ROS_ERROR("Failed to initialize Mathematical Navigator");
        return -1;
    }
    
    ROS_INFO("Mathematical Navigation Node started successfully");
    ROS_INFO("Waiting for goal and path data...");
    
    // 主循环
    while (ros::ok() && !g_shutdown) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    
    // 停止导航器
    navigator.stop();
    
    ROS_INFO("Mathematical Navigation Node stopped");
    return 0;
} 