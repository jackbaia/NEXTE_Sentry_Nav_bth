#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include "geometry_msgs/TwistStamped.h" 

#include <Eigen/Dense>

#include "input.h"


ros::Subscriber odom_sub;

ros::Publisher pose_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
geometry_msgs::TwistStamped sub_vb;
Odom_Data_t odom;




double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

double fromQuaternion2yaw1(Eigen::Quaterniond q)
{
  double yaw1 = atan2(2 * (q.w()*q.z() + q.x()*q.y()), 
                   1 - 2 * (q.y()*q.y() + q.z()*q.z()));

  return yaw1;
}

void cmdCallback()
{
    Eigen::Vector3d pos(0,0,0);
    
    Eigen::Vector3d vel(Eigen::Vector3d::Zero());
    Eigen::Vector3d acc(Eigen::Vector3d::Zero());
    Eigen::Vector3d jerk(Eigen::Vector3d::Zero());
    ros::Time start_time;
    double last_yaw_, last_yaw_dot_;
    static double init_hgt = odom.p(2);
    static double init_px = odom.p(0);
    static double init_py = odom.p(1);

    Eigen::Vector3d des_v(0,0,0);//加个订阅  加个yaw?
    
    std::pair<double, double> yaw_yawdot(0, 0);

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time).toSec();
    // ROS_INFO("time = %f", t_cur);
    static ros::Time time_last = ros::Time::now();
    double yaw_odom = fromQuaternion2yaw(odom.q);
    double yaw_odom1 = fromQuaternion2yaw1(odom.q);
    ROS_INFO("yaw_odom,yaw_odom1 = %f,%f", yaw_odom,yaw_odom1);
    double des_yaw = yaw_odom;
    
    time_last = time_now;

    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";

    bool do_rec = 0;
    if((time_now - sub_vb.header.stamp).toSec() < 0.1){        
        des_v(0) = sub_vb.twist.linear.x;
        des_v(1) = sub_vb.twist.linear.y;
        des_v(2) = sub_vb.twist.linear.z;
        // des_yaw = yaw_odom+2.0/180.0*3.14159*sub_vb.twist.angular.z;
        des_yaw = yaw_odom+sub_vb.twist.angular.z;
        do_rec = 1;
    }
    else{
        // ROS_WARN("can not get des_vb");
        init_px = odom.p(0);
        init_py = odom.p(1);
        init_hgt = odom.p(2);
        do_rec = 0;
    }

    // pos = odom.p + odom.q*des_v;
    double des_vx = des_v(0)*cos(yaw_odom)-des_v(1)*sin(yaw_odom);
    double des_vy = des_v(0)*sin(yaw_odom)+des_v(1)*cos(yaw_odom);
    if(fabs(des_vx) > 0.05){
        init_px = odom.p(0);
    }  
     if(fabs(des_vy) > 0.05){
        init_py = odom.p(1);
    }
    if(fabs(des_v(2)) > 10e-6){
        init_hgt = odom.p(2);
    }

    pos(0) = init_px + des_vx;
    pos(1) = init_py + des_vy;
    pos(2) = init_hgt + des_v(2);
    ROS_INFO("des_v(2) = %f", des_v(2));
    // ROS_INFO("time = %f", yaw_odom);

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.velocity.x = des_v(0);
    cmd.velocity.y = des_v(1);
    cmd.velocity.z = des_v(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    cmd.jerk.x = jerk(0);
    cmd.jerk.y = jerk(1);
    cmd.jerk.z = jerk(2);

    // cmd.yaw = yaw_yawdot.first;
    cmd.yaw = des_yaw;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;


    if(do_rec){
        pose_cmd_pub.publish(cmd);
    }
    // static int loop=0;
    // if((des_v.norm()>0.01||des_yaw-yaw_odom>0.01||des_yaw-yaw_odom<0.01)&&do_rec==1){
    //     pose_cmd_pub.publish(cmd);
    // }
    // else{
    //     loop++;
    //     if(loop==100){
    //         ROS_WARN("the des_vb is too slow");
    //         loop = 0;
    //     }
    // }

}

void sub_vb_Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    sub_vb = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ctrl_vb");
    ros::NodeHandle nh("~");
    ros::Rate rate(20);

    ros::Subscriber sub_vb_nh = nh.subscribe("/px4_pub_vb", 100, sub_vb_Callback);

    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 100, 
                            boost::bind(&Odom_Data_t::feed, &odom, _1),
                            ros::VoidConstPtr(),
                            ros::TransportHints().tcpNoDelay());
    pose_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 100);
    
    // ROS_INFO("odomz = %f", odom.p(2));


    while (ros::ok())
    {
        if(fabs(odom.p(2))>10e-2)
            cmdCallback();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
