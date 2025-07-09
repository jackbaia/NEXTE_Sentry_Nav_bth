#include "controller.h"

using namespace std;

 
/*
作用    串级pid控制器
输入    东北天下的期望位置
输出    期望姿态角及期望推力
*/

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

float LinearControl::low_pass_filter_px(float value)
{
  float alpha = 0.8;
  static float out_last = 0; //上一次滤波值
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

float LinearControl::low_pass_filter_py(float value)
{
  float alpha = 0.8;
  static float out_last = 0; //上一次滤波值
  float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

Eigen::Vector3d LinearControl::limit_v3(Eigen::Vector3d v3,float limt_value){
  v3(0)=v3(0)>limt_value?limt_value:v3(0);
  v3(0)=v3(0)<-limt_value?-limt_value:v3(0);
  v3(1)=v3(1)>limt_value?limt_value:v3(1);
  v3(1)=v3(1)<-limt_value?-limt_value:v3(1);
  v3(2)=v3(2)>limt_value?limt_value:v3(2);
  v3(2)=v3(2)<-limt_value?-limt_value:v3(2);
  return v3;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
      static float init_yaw;
      static float init_flag = 1;

  /* WRITE YOUR CODE HERE */
      //compute disired acceleration
	    ros::Time now_time = ros::Time::now();
      static ros::Time last_time = now_time;
      float dt = (now_time-last_time).toSec()+0.0001;
      last_time = now_time;

      Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
      Eigen::Vector3d des_pos(des.p);
      Eigen::Vector3d des_vel(0.0, 0.0, 0.0);
      static Eigen::Vector3d vel_int(0.0, 0.0, 0.0);
      Eigen::Vector3d vel_dot(0.0, 0.0, 0.0);
      Eigen::Vector3d Kp,Kv,Kvi,Kvd;
      Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
      Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
      Kvi<< param_.gain.Kvi0, param_.gain.Kvi1, param_.gain.Kvi2;
      Kvd<< param_.gain.Kvd0, param_.gain.Kvd1, param_.gain.Kvd2;

      // double max_height = 0.0;

      // max_height=param_. takeoff_land.height;
      
      // des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);


      // des_pos(2) =  des_pos(2)>max_height?max_height:des_pos(2);
      des_vel=Kp.asDiagonal()*(des_pos-odom.p);//比例环节
      // ROS_INFO("odomz = %f", odom.p(2));
      des_vel(2) =  des_vel(2)>1.2?1.2: des_vel(2);
      // des_vel(2) =fabs (des.p(2)-odom.p(2))<0.05?0.0: des_vel(2);//siqu 

      Eigen::Vector3d enu_v(0.0, 0.0, 0.0);

      enu_v = odom.q*odom.v;
      static Eigen::Vector3d vel_last(enu_v);
      static Eigen::Vector3d des_vel_last(des_vel);

      vel_int+=(des_vel-enu_v)*dt;//积分环节
      vel_int = limit_v3(vel_int,10);
      vel_dot =((des_vel-des_vel_last)-(enu_v-vel_last))/dt;//微分环节
      vel_dot = limit_v3(vel_dot,3);
      vel_last = enu_v;
      des_vel_last = des_vel;
      des_acc =Kv.asDiagonal()*(des_vel-enu_v)+Kvi.asDiagonal()*vel_int+Kvd.asDiagonal()*vel_dot;


      des_acc += Eigen::Vector3d(0,0,param_.gra);

      u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
      if(u.thrust>0.8) u.thrust = 0.8;
      if(u.thrust<0.15) u.thrust = 0.15;
      double roll,pitch,yaw,yaw_imu;
      double yaw_odom = fromQuaternion2yaw(odom.q);
      double sin = std::sin(yaw_odom);
      double cos = std::cos(yaw_odom);
      
      roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
      pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
// 姿态角限幅
      roll =roll>0.17?0.17:roll;
      roll =roll<-0.17?-0.17:roll;
      pitch =pitch>0.17?0.17:pitch;
      pitch =pitch<-0.17?-0.17:pitch;
      // yaw = fromQuaternion2yaw(des.q);
      yaw_imu = fromQuaternion2yaw(imu.q);
      // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
      //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
      //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
      if(init_flag){
        init_yaw = yaw_odom;
        init_flag = 0;
      }
      // yaw = init_yaw;
      yaw = des.yaw;
      Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())//des.yaw
        * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
      u.q = imu.q * odom.q.inverse() * q;


  //used for debug
  debug_msg_.err_axisang_x = des_pos(0);
  debug_msg_.err_axisang_y = des_pos(1);
  debug_msg_.err_axisang_z = des_pos(2);
  
  debug_msg_.des_v_x = des_vel(0);
  debug_msg_.des_v_y = des_vel(1);
  debug_msg_.des_v_z = des_vel(2);

  debug_msg_.fb_a_x = roll*180/3.1416;
  debug_msg_.fb_a_y = pitch*180/3.1416;
  debug_msg_.fb_a_z = yaw*180/3.1416;
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();

  debug_msg_.fb_rate_x = enu_v(0);
  debug_msg_.fb_rate_y = enu_v(1);
  debug_msg_.fb_rate_z = enu_v(2);
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}






