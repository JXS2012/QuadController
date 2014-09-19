#include <ros/ros.h>
#include "BirdEye.h"
#include "DummyBird.h"
#include "Potential.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "VectorComputation.h"
#include <std_msgs/Float32MultiArray.h>

#include "math.h"
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>

#define _USE_MATH_DEFINES

class VisualBird
{
 private:
  DummyBird *dummy;
  BirdEye *eye;
  Potential *nav;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber visual_sub;

  //constants
  double flight_height; //unit mm
  double max_outter_radius;
  double total_time;
  double freq;

  bool hover; //flag for hover controller
  bool fly; //turn motor on or not. If false, quadrotor won't fly. For testing purpose

  //variables
  pcl::PointXYZ targetPoint,targetBound;
  pcl::PointXYZ shiftOrigin;
  bool landing; //flag showing whether quadrotor is landing
  bool outRange;
  bool reachStartPoint;
  bool VisualBird_finish;
  bool visual_feedback;
  bool start_visual;

  int counter; //used in main loop for tracking time, current time = counter/freq
  std::string strTargets;
  std::string sensor;
  std::vector<pcl::PointXYZ> targets,visualTarget,visualBuffer;
  int targetIndex;

  //functions
  //initiation
  void initiation();
  void initParameters();
  void readTargets();
  void visualCallback(const std_msgs::Float32MultiArray msg);

  //high level controllers
  void invokeController(); //deciding which controller to use
  void checkStartPoint();
  void safeOutRange(); //check shifted away from center of room
  void land();   //land quadrotor once conditions are met(here is counter>1000 i.e. time constraint)
  void switchHover();
  void hoverFlight();
  void switchNextTarget();

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
