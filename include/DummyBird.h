#include <ros/ros.h>
#include <mav_srvs/SetMotorsOnOff.h>
#include <mav_srvs/GetMotorsOnOff.h>
#include "std_msgs/Float64.h"

#include <pcl/point_cloud.h>
#include "VectorComputation.h"

#include "math.h"
#include <vector>


class DummyBird
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

  //constants
  double g,heli_mass,freq;
  std::string strThrustCmd,strActThrust,quadrotorType,sensor;
  std::vector<double> thrustCmd,actualThrust;
  pcl::PointXYZ kdHover,kpHover,kiHover,kdHoverVel,kpHoverVel,kiHoverVel,kdPath,kpPath;

  //variables
  std::vector<float> pitch_log,roll_log,thrust_log;

  //functions
  void initPid();
  void initParameters();
  //take in desired acc and current psi angle, calculate control output to quadrotor
  void acc_des2control(double,double,double,double);
  double interpolateThrust(double);
  std::vector<double> readTable(std::string);
  
 public:

  DummyBird(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~DummyBird();

  void on();
  void off();
  
  //hover controller, takes in position error, velocity error, integrated error and psi angle
  void pidHoverController(pcl::PointXYZ,pcl::PointXYZ,pcl::PointXYZ,double);
  void pidHoverVelController(pcl::PointXYZ,pcl::PointXYZ,pcl::PointXYZ,pcl::PointXYZ,double);
  
  //path follower controller, takes in position error, velocity error, path point accelaration and psi angle
  void pidPathController(pcl::PointXYZ,pcl::PointXYZ,pcl::PointXYZ,double);
  
  //direct control, takes in desired accelaration in x,y,z and angle psi
  void directDrive(double,double,double,double);

  //Navigation function controller
  void navController(pcl::PointXYZ accDes, pcl::PointXYZ e_p, pcl::PointXYZ e_v, pcl::PointXYZ e_int, double psi);

  void writeLog(int);

  inline float getLogPitch(int i)
  {
    return pitch_log[i];
  }

  inline float getLogRoll(int i)
  {
    return roll_log[i];
  }

  inline float getLogThrust(int i)
  {
    return thrust_log[i];
  }

};
