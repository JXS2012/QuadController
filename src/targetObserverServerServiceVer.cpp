#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include "VectorComputation.h"

#include "math.h"
#include <vector>

#include <quadrotorTestControl/getTargetPosition.h>
#include <quadrotorTestControl/getTargetVelocity.h>

double r = 0.15;
double pi = 3.14159265;
double omega = 2*pi/10;


bool pos(quadrotorTestControl::getTargetPosition::Request  &req,
         quadrotorTestControl::getTargetPosition::Response &res)
{
  //double secs = ros::Time::now().toSec();

  //res.x = r * cos(omega*secs);
  //res.y = r * sin(omega*secs);
  res.x = 0;
  res.y = 0;
  res.z = 0;

  return true;
}

bool vel(quadrotorTestControl::getTargetVelocity::Request  &req,
         quadrotorTestControl::getTargetVelocity::Response &res)
{
  //double secs = ros::Time::now().toSec();

  //res.x = -r * omega * sin(omega*secs);
  //res.y = r * omega* cos(omega*secs);
  res.x = 0;
  res.y = 0;
  res.z = 0;

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc,argv,"targetServer");
  ros::NodeHandle nh;

  ros::ServiceServer target_pos_pub = nh.advertiseService("/quad/getTargetPosition",pos);  
  ros::ServiceServer target_vel_pub = nh.advertiseService("/quad/getTargetVelocity",vel);

  while (nh.ok() )
    {
      ros::spinOnce();
    }


  return 0;
}

