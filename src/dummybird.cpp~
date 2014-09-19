#include "dummybird.h"

dummybird::dummybird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ros::NodeHandle node  (nh_);

  //control cmds publisher
  pub_thrust = node.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  pub_pitch = node.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  pub_roll = node.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  pub_yaw = node.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);
  //motor OnOff services
  client_SetMotorsOnOff = node.serviceClient<mav_srvs::SetMotorsOnOff>("mav/setMotorsOnOff");
  client_GetMotorsOnOff = node.serviceClient<mav_srvs::GetMotorsOnOff>("mav/getMotorsOnOff");

  init_pid();
  init_parameters();
}

dummybird::~dummybird()
{
  ROS_INFO("Destroying dummybird ");
}

void dummybird::init_pid()
{  
  float root_x = 3;
  float root_y = 3;
  //pid constants found through trial and error
  kd_hover.x = 1.4*root_x;
  kd_hover.y = 1.4*root_y;
  kd_hover.z = 4;

  kp_hover.x = root_x;
  kp_hover.y = root_y;
  kp_hover.z = 2;

  ki_hover.x = 0.05;
  ki_hover.y = 0.05;
  ki_hover.z = 0.05;

  kd_path.x = 4.2;
  kd_path.y = 4.2;
  kd_path.z = 4;

  kp_path.x = 3;
  kp_path.y = 3;
  kp_path.z = 2;

}

void dummybird::init_parameters()
{
  g = 9.81;
  heli_mass = 615;
}

double dummybird::interpolate_thrust(double input)
{
  double ros_output_table[11] = {0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5};
  double input_mass[11] = {57.8, 95.3, 138.3, 192.3, 265.3, 350.3, 430.3, 540.3, 665.3, 790.3, 932.3};
  int i = 4;
  while (input>input_mass[i]-heli_mass)
    i++;

  double output; 
  
  //lower bound on output is 0.2 and higher bound on output is 0.4
  if (i >= 9)
    output = 0.4;

  if (i <= 4)
    output = 0.2;

  if ((i<9)&&(i>4))
    output  = (input + heli_mass - input_mass[i-1]) / (input_mass[i] - input_mass[i-1]) * 0.05 + ros_output_table[i-1];
  
  return output;
}

void dummybird::pid_hover_controller(pcl::PointXYZ e_p,pcl::PointXYZ e_v,pcl::PointXYZ e_int, double psi)
{
  double acc_x_des, acc_y_des, acc_z_des;
  acc_x_des = -kp_hover.x*e_p.x - kd_hover.x*e_v.x - ki_hover.x*e_int.x;
  acc_y_des = -kp_hover.y*e_p.y - kd_hover.y*e_v.y - ki_hover.y*e_int.y;
  acc_z_des = -kp_hover.z*e_p.z - kd_hover.z*e_v.z - ki_hover.z*e_int.z;

  acc_des2control(acc_x_des,acc_y_des,acc_z_des,psi);

}

void dummybird::pid_path_controller(pcl::PointXYZ e_p,pcl::PointXYZ e_v,pcl::PointXYZ acc_t, double psi)
{
  double acc_x_des, acc_y_des, acc_z_des;
  acc_x_des = -kp_path.x*(e_p.x) - kd_path.x*e_v.x + acc_t.x;
  acc_y_des = -kp_path.y*(e_p.y) - kd_path.y*e_v.y + acc_t.y;
  acc_z_des = -kp_path.z*(e_p.z) - kd_path.z*e_v.z + acc_t.z;

  acc_des2control(acc_x_des,acc_y_des,acc_z_des,psi);
}

void dummybird::acc_des2control(double acc_x_des, double acc_y_des, double acc_z_des, double psi)
{
  //using linearized invert dynamics to calculate desired roll,pitch,thrust cmd
  roll_in =  -(acc_x_des/1000.0*sin(psi) - acc_y_des/1000.0*cos(psi))/g;
  pitch_in = (acc_x_des/1000.0*cos(psi) + acc_y_des/1000.0*sin(psi))/g;
  thrust_in = interpolate_thrust(acc_z_des/1000.0*heli_mass/g);
 
  roll_log.push_back(roll_in);
  pitch_log.push_back(pitch_in);
  thrust_log.push_back(thrust_in);

  msg_thrust.data = thrust_in;
  msg_pitch.data = pitch_in;
  msg_roll.data = roll_in;
    
  pub_pitch.publish(msg_pitch);
  pub_roll.publish(msg_roll);
  pub_thrust.publish(msg_thrust);
}

void dummybird::direct_drive(double acc_x,double acc_y, double acc_z,double psi)
{
  acc_des2control(acc_x,acc_y,acc_z,psi);
}

void dummybird::on()
{
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

  ROS_INFO("Motor on\n");
}

void dummybird::off()
{
  mav_srvs::SetMotorsOnOff srv_SetMotorsOnOff;
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;
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

  ROS_INFO("Motor off\n");
}
