#include <ros/ros.h>
#include "DummyBird.h"

DummyBird *bird;
int fileNo;

int main(int argc, char** argv)
{
  double freq;

  ros::init(argc,argv,"quadrotor");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  bird = new DummyBird(nh,nh_private);
  ROS_INFO("create bird");
  
  while (nh.ok() )
    {
	bird->on();

	sleep(5);

	bird->off();

	break;
    }


  return 0;
}
