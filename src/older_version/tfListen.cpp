#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <LinearMath/btMatrix3x3.h>
#include "std_msgs/Float64.h"
#include "math.h"
#include <sstream>
#include <vector>
#include <stdio.h>
#include <mav_srvs/SetMotorsOnOff.h>
#include <mav_srvs/GetMotorsOnOff.h>

std_msgs::Float64 msg_thrust;
std_msgs::Float64 msg_pitch;
std_msgs::Float64 msg_roll;
std_msgs::Float64 msg_yaw;

float x, y, z, x_pre, y_pre, z_pre, x_origin, y_origin, z_origin;
float x_target,y_target,z_target;
double phi,psi,theta;
float vel_x, vel_y, vel_z;
float int_x, int_y, int_z;
float kd_x, kd_y, kd_z, kp_x, kp_y, kp_z, ki_x, ki_y, ki_z;
float thrust_in,pitch_in,roll_in,yaw_in;
std::vector<float> x_log, y_log, z_log, vel_x_log, vel_y_log, vel_z_log, pitch_log, roll_log, thrust_log;

float g = 9.81;
float freq = 30.0;
float pitch_roll_range = 0.14;
//float heli_mass = 615;
float heli_mass = 615;
float root_x = 0.003;
float root_y = 0.003;
float flight_height = 300;
float shift_no = 3;
float shift_time = 300;

void  init_value();
void  init_origin();
void  init_pid();
double  interpolate_thrust(double input);
void  get_ZXY();
void  get_loc_vel();
void  pid_control();
void  land_off();
void  write_log();

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  ros::NodeHandle n_thrust;
  ros::NodeHandle n_pitch;
  ros::NodeHandle n_roll;
  ros::NodeHandle n_yaw;

  ros::Publisher pub_thrust = n_thrust.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  ros::Publisher pub_pitch = n_pitch.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  ros::Publisher pub_roll = n_roll.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  ros::Publisher pub_yaw = n_yaw.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);
  ros::ServiceClient client_SetMotorsOnOff = node.serviceClient<mav_srvs::SetMotorsOnOff>("mav/setMotorsOnOff");
  ros::ServiceClient client_GetMotorsOnOff = node.serviceClient<mav_srvs::GetMotorsOnOff>("mav/getMotorsOnOff");

  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;

  srv_SetMotorsOnOff.request.on = true;
  client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
  client_GetMotorsOnOff.call(srv_GetMotorsOnOff);

  while (!srv_GetMotorsOnOff.response.on)
    {
      ROS_ERROR("Failed to turn motor on\n");
      client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
      client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
      usleep(100);
    }
  ROS_INFO("Motors on\n");

  init_value();
  init_pid();

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
  //x_target = x_origin+200;
  //y_target = y_origin-200;
  //x_target = -1750;
  //y_target = 1750;
  x_target = x_origin;
  y_target = y_origin;
  z_target = z_origin+flight_height;
  x_pre = x_origin;
  y_pre = y_origin;
  z_pre = z_origin;

  printf("origin at %.3f %.3f %.3f\n",x_origin,y_origin,z_origin);

  int counter = 0;
  bool landing = false;
  float current_shift = 0;
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
    //printf("ZXY roll %.3f pitch %.3f yaw %.3f\n",phi,theta,psi);

    //btMatrix3x3(transform.getRotation()).getRPY(phi,theta,psi);
    x = transform.getOrigin().x()*1000;
    y = transform.getOrigin().y()*1000;
    z = transform.getOrigin().z()*1000;

    int_x += x - x_target;
    int_y += y - y_target;
    int_z += z -z_target;

    vel_x = (x-x_pre)*freq;
    vel_y = (y-y_pre)*freq;
    vel_z = (z-z_pre)*freq;
    x_pre = x;
    y_pre = y;
    z_pre = z;
    x_log.push_back(x);
    y_log.push_back(y);
    z_log.push_back(z);
    vel_x_log.push_back(phi);
    vel_y_log.push_back(theta);
    vel_z_log.push_back(psi);
    //printf("x %.3f y %.3f z %.3f\n",x,y,z);
    
    if (vel_x == 0 && vel_y == 0 && vel_z == 0)
      {
	pitch_in = 0;
	roll_in = 0;
	thrust_in = interpolate_thrust(0);
      }
    else
      {
	pid_control();
      }
    counter ++;
    if (counter > shift_time*current_shift && current_shift<shift_no)
      {
	printf ("%.3f shift\n",current_shift);
	current_shift++;
	x_target += 150;
	y_target += 150;
      }
    if (z-z_origin < 100 && landing)
      {
	srv_SetMotorsOnOff.request.on = false;
	client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
	client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
	while (srv_GetMotorsOnOff.response.on)
	  {
	    ROS_ERROR("Failed to turn motor off\n");
	    client_SetMotorsOnOff.call(srv_SetMotorsOnOff);
	    client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
	    usleep(100);
	      }
	ROS_INFO("Motors off\n");	
	break;
      }
    
    if (counter > 1000 && !landing)
      {
	z_target -= flight_height;
	landing = true;
	printf("start landing off\n");
      }
    
    if (counter > freq*0.5)
      {
	msg_thrust.data = thrust_in;
	msg_pitch.data = pitch_in;
	msg_roll.data = roll_in;
	msg_yaw.data = yaw_in; 
	
	pub_pitch.publish(msg_pitch);
	pub_roll.publish(msg_roll);
	pub_yaw.publish(msg_yaw);
	pub_thrust.publish(msg_thrust);
      }
    else{
      msg_thrust.data = thrust_in;
      pub_thrust.publish(msg_thrust);
    }
    
    rate.sleep();

  }

  write_log();
  return 0;
};

void init_value()
{
  x = 0;
  y = 0;
  z = 0;
  x_pre = 0;
  y_pre = 0;
  z_pre = 0;
  vel_x = 0;
  vel_y = 0;
  vel_z = 0;
  int_x = 0;
  int_y = 0;
  int_z = 0;
  x_origin = 0;
  y_origin = 0;
  z_origin = 0;
  x_target = 0;
  y_target = 0;
  z_target = 0;
  phi = 0;
  psi = 0;
  theta = 0;
}

void init_pid()
{  
  kd_x = 1.2*root_x;
  kp_x = root_x;
  kd_y = 1.2*root_y;
  kp_y = root_y;
  ki_x = 0;
  ki_y = 0;
  //kd_z = 2*root_z;
  //kp_z = pow(root_z,2);

  kd_z = 3;
  kp_z = 2;
  ki_z = 0;
}

double interpolate_thrust(double input)
{
  double ros_output_table[11] = {0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5};
  double input_mass[11] = {57.8, 95.3, 138.3, 192.3, 265.3, 350.3, 430.3, 540.3, 665.3, 790.3, 932.3};
  int i = 4;
  while (input>input_mass[i]-heli_mass)
    i++;

  double output; 
  
  if (i >= 9)
    output = 0.4;

  if (i <= 4)
    output = 0.2;

  if ((i<9)&&(i>4))
    output  = (input + heli_mass - input_mass[i-1]) / (input_mass[i] - input_mass[i-1]) * 0.05 + ros_output_table[i-1];
  
  return output;
}

void pid_control()
{
  float acc_x, acc_y, acc_z;
  acc_x = -kp_x*(x-x_target) - kd_x*vel_x - ki_x*int_x;
  acc_y = -kp_y*(y-y_target) - kd_y*vel_y - ki_y*int_y;
  acc_z = -kp_z*(z-z_target) - kd_z*vel_z - ki_z*int_z;

  roll_in =  -(acc_x*sin(psi) - acc_y*cos(psi))/g;
  pitch_in = (acc_x*cos(psi) + acc_y*sin(psi))/g;
  thrust_in = interpolate_thrust(acc_z/1000.0*heli_mass/g);
  //printf("thrust %.3f\n",thrust_in);
  yaw_in = 0;
  roll_log.push_back(roll_in);
  pitch_log.push_back(pitch_in);
  thrust_log.push_back(thrust_in);
  //printf("Input roll %.3f pitch %.3f\n",roll_in,pitch_in);
}

void write_log()
{
  std::FILE * pFile;

  pFile = fopen("log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<x_log.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f vel_x %.3f vel_y %.3f vel_z %.3f roll %.6f pitch %.6f thrust %.6f\n", x_log[i], y_log[i], z_log[i], vel_x_log[i], vel_y_log[i], vel_z_log[i], roll_log[i], pitch_log[i], thrust_log[i]);
    }
  
}

void land_off()
{
  roll_in = 0;
  pitch_in = 0;
  thrust_in = interpolate_thrust(heli_mass-100.0);
}

/*
void init_origin()
{
  tf::TransformListener listener;
  tf::StampedTransform transform;

  while (true)
    {
      try{
	listener.lookupTransform("/world", "/vicon/hummingbird/hummingbird",  
				 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	rate.sleep();
	continue;
      }
      x_origin = transform.getOrigin().x()*1000;
      y_origin = transform.getOrigin().y()*1000;
      break;
    }
  printf("origin at %.3f %.3f\n",x_origin,y_origin);
}

void get_ZXY()
{
  double roll,pitch,yaw;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  try{
    listener.lookupTransform("/world", "/vicon/hummingbird/hummingbird",  
			     ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  btMatrix3x3(transform.getRotation()).getEulerZYX(yaw,pitch,roll);
  //printf("ZYX roll %.3f pitch %.3f yaw %.3f\n",roll,pitch,yaw);
  
  phi = asin(sin(roll)*cos(pitch));
  double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(pitch);
  double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
  psi = atan(-psi_1/psi_2);
  theta = atan(tan(pitch)/cos(roll));  
  //printf("ZXY roll %.3f pitch %.3f yaw %.3f\n",phi,theta,psi);
}

void get_loc_vel()
{
  tf::StampedTransform transform;

    x = transform.getOrigin().x()*1000-x_origin;
    y = transform.getOrigin().y()*1000-y_origin;
    vel_x = (x-x_pre)*freq;
    vel_y = (y-y_pre)*freq;
    x_pre = x;
    y_pre = y;
    x_log.push_back(x);
    y_log.push_back(y);
    //printf("x %.3f y %.3f vel_x %.3f vel_y %.3f\n",x,y,vel_x,vel_y);
}
*/
