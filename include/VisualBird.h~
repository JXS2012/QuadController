#include <ros/ros.h>
#include "BirdEye.h"
#include "DummyBird.h"
#include "Potential.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "VectorComputation.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>

#include "math.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

#define _USE_MATH_DEFINES
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class VisualBird
{
 private:
  DummyBird *dummy;
  BirdEye *eye;
  Potential *nav;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber visual_pos_sub;
  ros::Subscriber visual_vel_sub;
  ros::Publisher target_pub;
  ros::Publisher pos_points_pub;

  double flight_height; //unit mm
  double max_outter_radius;
  double total_time;
  double freq;

  bool hover; //flag for hover controller
  bool fly; //turn motor on or not. If false, quadrotor won't fly. For testing purpose

  //variables
  pcl::PointXYZ targetPoint,targetBound;
  pcl::PointXYZ shiftOrigin;
  pcl::PointXYZ lastmid, lastQuad, zeroVector;
  bool landing; //flag showing whether quadrotor is landing
  bool outRange;
  bool reachStartPoint;
  bool VisualBird_finish;
  bool visual_feedback, visual_vel_feedback;
  bool start_visual;
  bool dropdown;
  bool climbup;

  int counter; //used in main loop for tracking time, current time = counter/freq
  std::string strTargets;
  std::string sensor;
  std::vector<pcl::PointXYZ> targets,visualTarget,visualBuffer;
  std::vector<double> timeTable,timeLog;
  int targetIndex;
  float jumpThreshold;

  //functions
  //initiation
  void initiation();
  void initParameters();
  void readTargets();

  //high level controllers
  void invokeController(); //deciding which controller to use
  void checkStartPoint();
  void safeOutRange(); //check shifted away from center of room
  void land();   //land quadrotor once conditions are met(here is counter>1000 i.e. time constraint)
  void switchHover();
  void hoverFlight();
  void switchNextTarget();
  void switchNavTarget(pcl::PointXYZ pos, pcl::PointXYZ vel);

  //Visual
  bool jumpDetector(pcl::PointXYZ now, pcl::PointXYZ last);
  pcl::PointXYZ rescaleVisual(pcl::PointXYZ input);
  void visualPosCallback(const std_msgs::Float32MultiArray msg); 
  void visualVelCallback(const std_msgs::Float32MultiArray msg); 
  
 public:
  VisualBird(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~VisualBird();

  //drive quadrotor to achieve goals
  void drive();
  //store flight data(x,y,z,roll,pitch,yaw,control inputs)
  void writeLog(int);
  //return true when finished flying
  bool finish();
};
