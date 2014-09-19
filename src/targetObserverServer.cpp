#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include "math.h"
#include <vector>

double r = 0.15;
double pi = 3.14159265;
double omega = 2*pi/10;

ros::Publisher pos_points_pub;
ros::Subscriber pos_points_sub;

double x,y,z,vx,vy,vz;

void send_pos(float x, float y, float z, float vx, float vy, float vz)	{
  std_msgs::Float32MultiArray pts;
  
  pts.data.push_back(x);
  pts.data.push_back(y);
  pts.data.push_back(z);
  
  pts.data.push_back(vx);
  pts.data.push_back(vy);
  pts.data.push_back(vz);
  
  pos_points_pub.publish(pts);
}

void pos_points_callback(const std_msgs::Float32MultiArray msg)
{

  x = msg.data[0];
  y = msg.data[1];
  z = msg.data[2];
  ROS_INFO("set to %.3f %.3f %.3f\n",x,y,z);
  vx = msg.data[3];
  vy = msg.data[4];
  vz = msg.data[5];
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"targetServer");
  ros::NodeHandle nh;
  ros::Rate myRate(30);
  pos_points_pub = nh.advertise<std_msgs::Float32MultiArray>("target_states", 10);
  pos_points_sub = nh.subscribe("/VisualBird/set_target_states",1000,pos_points_callback);

  z = 0;
  vz = 0;
  x = 0;
  y = 0;
  vx = 0;
  vy = 0;

  while (nh.ok() )
    {
      /*
      double secs = ros::Time::now().toSec();
      x = r * cos(omega*secs);
      y = r * sin(omega*secs);
      vx = -r * omega * sin(omega*secs);
      vy = r * omega* cos(omega*secs);
      */
      send_pos(x,y,z,vx,vy,vz);
      ros::spinOnce();
      myRate.sleep();
    }


  return 0;
}

