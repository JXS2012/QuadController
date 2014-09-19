#include <ros/ros.h>
#include "HighBird.h"
#include<asctec_msgs/IMUCalcData.h>

int counter;

void timerCallback(const ros::TimerEvent&)
{
  counter ++;
  if (counter % 1000 == 0)
    ROS_INFO("%d\n",counter);
}

int main(int argc, char** argv)
{
  double freq;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/1000),timerCallback);

  counter = 0;
  while (nh.ok() )
    {
      ros::spinOnce();
    }


  return 0;
}
