#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
//#include <LinearMath/btMatrix3x3.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "VectorComputation.h"

#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>
#include <std_msgs/Float32MultiArray.h>

#include <quadrotorTestControl/getTargetPosition.h>
#include <quadrotorTestControl/getTargetVelocity.h>

class BirdEye
{
 private:
  //ros related
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  tf::TransformListener *listener;
  tf::StampedTransform *transform;
  
  ros::ServiceClient targetPosClient;
  ros::ServiceClient targetVelClient;

  quadrotorTestControl::getTargetPosition srv_getTargetPos;
  quadrotorTestControl::getTargetVelocity srv_getTargetVel;
  
  //constants;
  double flight_radius,freq,flightHeight;
  std::string vicon;
  pcl::PointXYZ shiftedOrigin,zeroVector;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,cloud_vel,cloud_acc;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  int averageStep;

  //variables;
  pcl::PointXYZ currentPoint,prePoint,originPoint;
  pcl::PointXYZ e_p,e_v,acc_t,currentVel,currentAcc,intError;
  pcl::PointXYZ targetPos,targetVel;
  double phi,psi,theta;
  //std::vector<float> x_log, y_log, z_log, phi_log, theta_log, psi_log;
  std::vector<pcl::PointXYZ> pointLog,angleLog,velLog,accLog,targetLog;

  //handle vicon lost
  int freezeCounter,currentIndex;
  bool lostVicon;

  //functions
  //initiation
  void initiation();
  void initPosition(); //init x,y,z,x_pre,.. 
  void initParameters();
  void flightOrigin();
  void initPointcloud(); //initiate path

  //in flight data updating
  void transformZYXtoZXY(double,double,double);

  //flight log record
  void flightLog();

  //vicon lost handle
  void viconLostHandle();
  pcl::PointXYZ differentiate(const std::vector<pcl::PointXYZ>&);
  bool checkZeroVector(pcl::PointXYZ);

 public:
  BirdEye(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~BirdEye();

  //in flight data updating
  void updateFlightStatus(); //update current position,velocity,etc
  void updateIntError(pcl::PointXYZ); //update integrated error for hover
  void resetIntError(); //reset integrated error to zero
  void calculatePositionVelocityError(); //used for path control

  //getters
  inline pcl::PointXYZ getShiftedOrigin()
  {
    return shiftedOrigin;
  }

  inline pcl::PointXYZ getStartPoint()
  {
    return cloud->points[0];
  }

  inline pcl::PointXYZ getCurrentPoint()
  {
    return currentPoint;
  }
 
  inline pcl::PointXYZ getOriginPoint()
  {
    return originPoint;
  }

  inline pcl::PointXYZ getE_p()
  {
    return e_p;
  }
 
  inline pcl::PointXYZ getE_v()
  {
    return e_v;
  }
 
  inline pcl::PointXYZ getAcc_t()
  {
    return acc_t;
  }
 
  inline pcl::PointXYZ getCurrentVel()
  {
    return currentVel;
  }

  inline pcl::PointXYZ getCurrentAcc()
  {
    return currentAcc;
  }

  inline pcl::PointXYZ getIntError()
  {
    return intError;
  }

  inline pcl::PointXYZ getTargetPos()
  {
    return targetPos;
  }

  inline pcl::PointXYZ getTargetVel()
  {
    return targetVel;
  }

  inline double getPhi()
  {
    return phi;
  }
 
  inline double getPsi()
  {
    return psi;
  }
 
  inline double getTheta()
  {
    return theta;
  }

  inline pcl::PointXYZ getLogPos(int i)
  {
    return pointLog[i];
  }

  inline pcl::PointXYZ getLogAngle(int i)
  {
    return angleLog[i];
  }

  inline pcl::PointXYZ getLogVel(int i)
  {
    return velLog[i];
  }

  inline pcl::PointXYZ getLogAcc(int i)
  {
    return accLog[i];
  }

  inline pcl::PointXYZ getLogTarget(int i)
  {
    return targetLog[i];
  }

  inline int getLogSize()
  {
    return pointLog.size();
  }

};
