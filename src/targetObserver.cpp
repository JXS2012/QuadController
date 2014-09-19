#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include "VectorComputation.h"

#include "math.h"
#include <vector>
#include <quadrotorTestControl/Point3D.h>

ros::Publisher target_pos_pub; 
ros::Publisher target_vel_pub; 

void timerCallback(const ros::TimerEvent& event)
{
  quadrotorTestControl::Point3D posMsg,velMsg;
  double r = 0.15;
  double pi = 3.14159265;
  double omega = 2*pi/10;
  double secs = event.current_real.toSec();

  posMsg.x = r * cos(omega*secs);
  posMsg.y = r * sin(omega*secs);
  posMsg.z = 0;

  velMsg.x = -r * omega * sin(omega*secs);
  velMsg.y = r * omega* cos(omega*secs);
  velMsg.z = 0;

  target_pos_pub.publish(posMsg);
  target_vel_pub.publish(velMsg);
}

int main(int argc, char** argv)
{
  double freq;

  ros::init(argc,argv,"target");
  ros::NodeHandle nh;

  if (!nh.getParam("freq",freq))
    freq = 30.0;
  ROS_INFO("call back freq %.3f",freq);


  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  ROS_INFO("create target timer");
  target_pos_pub = nh.advertise<quadrotorTestControl::Point3D>("/quad/TargetPos",1);  
  target_vel_pub = nh.advertise<quadrotorTestControl::Point3D>("/quad/TargetVel",1);

  while (nh.ok() )
    {
      ros::spinOnce();
    }


  return 0;
}

