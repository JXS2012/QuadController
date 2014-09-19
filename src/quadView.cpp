#include <ros/ros.h>
#include "BirdEye.h"

BirdEye *hummingbird;
int counter = 0;

void timerCallback(const ros::TimerEvent&)
{
  counter++;
  hummingbird->updateFlightStatus();  
}

int main(int argc, char** argv)
{
  double freq;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if (!nh_private.getParam("freq",freq))
    freq = 30.0;
  ROS_INFO("call back freq %.3f",freq);

  hummingbird = new BirdEye(nh,nh_private);
  ROS_INFO("create hummingbird eye");

  ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
  ROS_INFO("create timer");
  
  while (nh.ok() && counter <=20*freq)
    {
      ros::spinOnce();
    }

  std::FILE * pFile;
  
  pFile = fopen("/home/jianxin/log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<hummingbird->getLogSize();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f velX %.6f velY %.6f velZ %.6f accX %.6f accY %.6f accZ %.6f\n", hummingbird->getLogPos(i).x, hummingbird->getLogPos(i).y, hummingbird->getLogPos(i).z, hummingbird->getLogVel(i).x,hummingbird->getLogVel(i).y,hummingbird->getLogVel(i).z, hummingbird->getLogAcc(i).x,hummingbird->getLogAcc(i).y,hummingbird->getLogAcc(i).z);
    }

  delete hummingbird;
  return 0;
}
