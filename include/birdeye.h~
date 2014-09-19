#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "vector_computation.h"

#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>

class birdeye
{
 private:
  //ros related
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener *listener;
  tf::StampedTransform *transform;

  //constants;
  float flight_radius,freq;
  pcl::PointXYZ shiftedOrigin,zeroVector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,cloud_vel,cloud_acc;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  //variables;
  double roll,pitch,yaw;
  pcl::PointXYZ currentPoint,prePoint,originPoint;
  pcl::PointXYZ e_p,e_v,acc_t,currentVel,intError;
  double phi,psi,theta;
  std::vector<float> x_log, y_log, z_log, phi_log, theta_log, psi_log;

  //functions
  //initiation
  void initiation();
  void init_position(); //init x,y,z,x_pre,.. 
  void init_parameters();
  void flightOrigin();
  void init_pointcloud(); //initiate path

  //in flight data updating
  void transformZYXtoZXY();

  //flight log record
  void flightLog();

 public:
  birdeye(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~birdeye();

  //in flight data updating
  void updateFlightStatus(); //update current position,velocity,etc
  void updateIntError(pcl::PointXYZ);
  void resetIntError();
  void calculate_position_velocity_error(); //used for path controll

  pcl::PointXYZ getShiftedOrigin();
  pcl::PointXYZ getStartPoint();
  pcl::PointXYZ getCurrentPoint();
  pcl::PointXYZ getOriginPoint();
  pcl::PointXYZ getE_p();
  pcl::PointXYZ getE_v();
  pcl::PointXYZ getAcc_t();
  pcl::PointXYZ getCurrentVel();
  pcl::PointXYZ getIntError();
  double getPhi();
  double getPsi();
  double getTheta();

  float getLog_x(int);
  float getLog_y(int);
  float getLog_z(int);
  float getLog_phi(int);
  float getLog_theta(int);
  float getLog_psi(int);

  int getLogSize();
};
