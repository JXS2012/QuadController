#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>
#include <mav_srvs/SetMotorsOnOff.h>
#include <mav_srvs/GetMotorsOnOff.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "vector_computation.h"

#include "std_msgs/Float64.h"
#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>

class quadrotor
{
 private:
  std_msgs::Float64 msg_thrust;
  std_msgs::Float64 msg_pitch;
  std_msgs::Float64 msg_roll;
  std_msgs::Float64 msg_yaw;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vel;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_acc;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  pcl::PointXYZ currentPoint,prePoint,originPoint,targetPoint;
  pcl::PointXYZ e_p,e_v,acc_t,currentVel,intError;
  pcl::PointXYZ kd_hover, kp_hover, ki_hover, kd_path, kp_path;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  //control cmds publisher
  ros::Publisher pub_thrust;
  ros::Publisher pub_pitch;
  ros::Publisher pub_roll;
  ros::Publisher pub_yaw;
  //motor onoff services
  ros::ServiceClient client_SetMotorsOnOff;
  ros::ServiceClient client_GetMotorsOnOff;

  //ros::Timer timer;
  tf::TransformListener *listener;
  tf::StampedTransform *transform;

  double phi,psi,theta;
  
  float thrust_in,pitch_in,roll_in,yaw_in;
  std::vector<float> x_log, y_log, z_log, phi_log, theta_log, psi_log, pitch_log, roll_log, thrust_log;
  
  //constants
  float g;
  float freq; //control frequency
  float heli_mass;
  float flight_height; //unit mm
  float max_outter_radius;

  float root_x; //pid constants for hovering
  float root_y;
  
  float shift_no; //number of shifts performed during flight if in hovering mode
  float shift_time; //total time for each shift = shift_time/freq
  float shift_distance;
  float current_shift; //counting current shift steps,

  int control_mode; //0 for hover, 1 for path following
  int counter; //used in main loop for tracking time, current time = counter/freq
  
  bool fly; //turn motor on or not. If false, quadrotor won't fly. For testing purpose
  bool landing; //flag showing whether quadrotor is landing
  bool outRange;
  bool reachStartPoint;

  pcl::PointXYZ shiftedOrigin;
  
  //initiation
  void initiation();
  void init_position(); //init x,y,z,x_pre,.. 
  void init_parameters();
  void init_pid(); //defining pid constants
  void flightOrigin();
  void init_pointcloud(); //initiate path

  //in flight data updating
  void updateFlightStatus(); //update current position,velocity,etc
  void transformZYXtoZXY(double,double,double);

  //pid controller related
  void calculate_position_velocity_error(); //used for path controll
  double interpolate_thrust(double); //use pre found table for thrust cmd interpolating
  void pid_hover_control();  //hover controller
  void pid_path_control(); //path controller
  void acc_des2control(double,double,double); //convert desired accelaration to control input

  //flight behavior control
  void turnMotorOn(ros::ServiceClient, ros::ServiceClient);
  void turnMotorOff(ros::ServiceClient, ros::ServiceClient);

  //high level controllers
  void invokeController(); //deciding which controller to use
  void shiftTargetPoint(); //change target point in flight for hover controller
  void safeOutRange(); //if shifted away from point(0,500) farther than 1000, send outRange flag
  //land quadrotor once conditions are met(here is counter>1000 i.e. time constraint)
  void land(); 
  
  //flight log record
  void flightLog();

 public:
  quadrotor(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~quadrotor();

  //drive quadrotr
  void drive();
  //store flight data(x,y,z,roll,pitch,yaw,control inputs)
  void write_log();
  //return true when finished flying
  bool finish();
};
