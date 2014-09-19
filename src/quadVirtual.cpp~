#include <ros/ros.h>
#include "VisualBird.h"

VisualBird *bird;
int fileNo;

void timerCallback(const ros::TimerEvent& event)
{
  /*
  ros::Duration lag = event.current_real-event.current_expected;
  if ( abs(lag.toSec()) > 0.1 )
    ROS_INFO("Lag %f",lag.toSec());
  */
  bird->drive();  
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
  
  ROS_INFO("Start from test No?");
  std::cin >> fileNo;
  ROS_INFO("Start from test No.%d.", fileNo);
  ros::Rate r(freq);
  
  while (nh.ok() )
    {
      char flag;
      bird = new VisualBird(nh,nh_private);
      ROS_INFO("create bird");
      ros::Timer timer = nh.createTimer(ros::Duration(1.0/freq),timerCallback);
      ROS_INFO("create timer");
      
      while (!bird->finish())
	{
	  //bird->drive(); 
	  ros::spinOnce();
	  //r.sleep();
	}
      
      ROS_INFO("Writing to No.%d", fileNo);
      bird->writeLog(fileNo);
      delete bird;

      ROS_INFO("Continue? (y/n)");
      std::cin >> flag;
      if (flag == 'n')
	break;

      fileNo ++;
    }


  return 0;
}
