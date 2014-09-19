#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>
#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>

float x, y, z, x_origin, y_origin, z_origin;
double phi,psi,theta;
float vel_x, vel_y, vel_z;
std::vector<float> x_log, y_log, z_log, roll_log, pitch_log, yaw_log;

float freq = 30.0;

void  write_log();

int main(int argc, char** argv){
  ros::init(argc, argv, "angleTransform");

  ros::NodeHandle node;

  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Rate rate(freq);

  while (true)
    {
      try{
	listener.lookupTransform("/world", "/vicon/bird/bird",  
				 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	rate.sleep();
	continue;
      }
      x_origin = transform.getOrigin().x()*1000;
      y_origin = transform.getOrigin().y()*1000;
      z_origin = transform.getOrigin().z()*1000;
      break;
    }

  printf("origin at %.3f %.3f %.3f\n",x_origin,y_origin,z_origin);

  while (node.ok()){
    double roll,pitch,yaw;

    try{
      listener.lookupTransform("/world", "/vicon/bird/bird",  
			       ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    btMatrix3x3(transform.getRotation()).getEulerZYX(yaw,pitch,roll);
    //printf("ZYX roll %.3f pitch %.3f yaw %.3f\n",roll,pitch,yaw);
    
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
    printf("ZXY roll %.3f pitch %.3f yaw %.3f\n",phi/3.14*180,theta/3.14*180,psi/3.14*180);    

    btMatrix3x3(transform.getRotation()).getRPY(phi,theta,psi);

    x = transform.getOrigin().x()*1000;
    y = transform.getOrigin().y()*1000;
    z = transform.getOrigin().z()*1000;

    x_log.push_back(x);
    y_log.push_back(y);
    z_log.push_back(z);
    roll_log.push_back(phi);
    pitch_log.push_back(theta);
    yaw_log.push_back(psi);
    //printf("x %.3f y %.3f z %.3f\n",x,y,z);
    rate.sleep();

  }

  write_log();
  return 0;
};

void write_log()
{
  std::FILE * pFile;

  pFile = fopen("angleTransfrom.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<x_log.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f roll %.3f pitch %.3f yaw %.3f\n", x_log[i], y_log[i], z_log[i], roll_log[i], pitch_log[i], yaw_log[i]);
    }
  
}

