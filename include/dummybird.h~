#include <ros/ros.h>
#include <mav_srvs/SetMotorsOnOff.h>
#include <mav_srvs/GetMotorsOnOff.h>
#include "std_msgs/Float64.h"

#include <pcl/point_cloud.h>
#include "vector_computation.h"

#include "math.h"
#include <vector>

class dummybird
{
 private:
  //direct control related
  //ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //control cmds publisher
  ros::Publisher pub_thrust;
  ros::Publisher pub_pitch;
  ros::Publisher pub_roll;
  ros::Publisher pub_yaw;
  //control msgs
  std_msgs::Float64 msg_thrust;
  std_msgs::Float64 msg_pitch;
  std_msgs::Float64 msg_roll;
  std_msgs::Float64 msg_yaw;
  //motor onoff services
  ros::ServiceClient client_SetMotorsOnOff;
  ros::ServiceClient client_GetMotorsOnOff;

  float thrust_in,pitch_in,roll_in,yaw_in;

  pcl::PointXYZ kd_hover,kp_hover,ki_hover,kd_path,kp_path;

  float g,heli_mass;

  void init_pid();
  void init_parameters();
  //take in desired acc and current psi angle, calculate control output to quadrotor
  void acc_des2control(double,double,double,double);
  double interpolate_thrust(double);

 public:
  std::vector<float> pitch_log,roll_log,thrust_log;

  dummybird(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~dummybird();

  void on();
  void off();
  //hover controller, takes in position error, velocity error, integrated error and psi angle
  void pid_hover_controller(pcl::PointXYZ,pcl::PointXYZ,pcl::PointXYZ,double);
  //path follower controller, takes in position error, velocity error, path point accelaration and psi angle
  void pid_path_controller(pcl::PointXYZ,pcl::PointXYZ,pcl::PointXYZ,double);
  //direct control, takes in desired accelaration in x,y,z and angle psi
  void direct_drive(double,double,double,double);
};
