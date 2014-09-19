#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "vicon_pos.h"
#include "math.h"

#include <sstream>

using namespace ViconDataStreamSDK::CPP;
using namespace std;

int main(int argc, char **argv)
{


  float coord[3], Eulerxyz[3],Erot[9] ; // for vicon translation and eular rotation
  vicon_pos* c = new vicon_pos("quadx","",""); //vicon initialization

  ros::init(argc, argv, "talker");

  //ros::NodeHandle thrust;
  ros::NodeHandle pitch;
  ros::NodeHandle roll;
  ros::NodeHandle yaw;


  //ros::Publisher pub_thrust = thrust.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  ros::Publisher pub_pitch = pitch.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  ros::Publisher pub_roll = roll.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  ros::Publisher pub_yaw = yaw.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);

  ros::Rate loop_rate(10);


  float acc_x, acc_y, acc_z, pitch_des, roll_des, yaw_des;
  float vel_x, vel_y, vel_z;
  float x, y, z, phi, psi, theta;
  float g = 9.81;
  float root_x, root_y, kd_x, kd_y, kp_x, kp_y;
  float pitch_roll_range = 0.14;

  int count = 0;
  float prev_x, prev_y, prev_z;
  c->update();
  c->get_subject_coord("Quad2",coord[0], coord[1], coord[2]);

  prev_x = coord[0];
  prev_y = coord[1];
  prev_z = coord[2];

  while (ros::ok())
  {

    // Vicon to grab position and rotation data..
    
    c->update();
    c->get_subject_coord("Quad2",coord[0], coord[1], coord[2]);
    c->get_subject_rotmatrix("Quad2",Erot);
    //c->get_subject_euler_xyz("Quad1",Eulerxyz[0],Eulerxyz[1],Eulerxyz[2]);
    
    cout <<"coord" << coord[0] << "\t"<<coord[1] <<"\t"<< coord[2]<< endl;
    //cout << "Eulerxyz" << Eulerxyz[0] <<"\t"<<Eulerxyz[1] <<"\t"<< Eulerxyz[2] << endl;
    cout << "EulerRotation" << Erot[0] <<"\t"<<Erot[1] <<"\t"<< Erot[2]<<"\t"<< Erot[3] <<"\t"<<Erot[4] <<"\t"<< Erot[5]<<"\t"<<  Erot[6] <<"\t"<<Erot[7] <<"\t"<< Erot[8] << endl;
    // quadrotor trial controller
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Float64 msg_thrust;
    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_roll;
    std_msgs::Float64 msg_yaw;
    
    
    //write a PD control calculation for thrust pitch roll and yaw commands and put it in a loop
    
    x = coord[0];
    y = coord[1];
    z = coord[2];
    
    vel_x = (prev_x - x)/6;
    vel_y = (prev_y - y)/6;
    vel_z = (prev_z - z)/6;
    
    prev_x = x;
    prev_y = y;
    prev_z = z;
    
    root_x = 0.2;
    root_y = 0.2;
    
    phi = asin(Erot[7]);
    psi = atan(-(Erot[1]/Erot[4]));
    theta = atan(-(Erot[6]/Erot[8]));
    cout << "phi" <<"\t"<< phi << "\t"<<"psi" <<"\t"<< psi <<"\t"<< "theta" << "\t"<<theta << endl;
 
    kd_x = 2*root_x;
    kp_x = pow(root_x,2);
    kd_y = 2*root_y;
    kp_y = pow(root_y,2);


    acc_x = kp_x*x + kd_x*vel_x;
    acc_y = kp_y*y + kd_y*vel_y;


    roll_des = (acc_x*sin(psi) - acc_y*cos(psi))/g;
    pitch_des = (acc_x*sin(psi) + acc_y*cos(psi))/g;
    yaw_des = 0;
    
    if(pitch_des > pitch_roll_range){
      pitch_des = pitch_roll_range;}
    else if (pitch_des < -pitch_roll_range){
      pitch_des = -pitch_roll_range;}
    
    if(roll_des > pitch_roll_range){
      roll_des = pitch_roll_range;}
    else if (roll_des < -pitch_roll_range){
      roll_des = -pitch_roll_range;}


    cout << "rolldes" << roll_des << "\t"<< "pitch_des"<< pitch_des<< endl;

    //float junk = .01;
    msg_pitch.data = pitch_des;
    msg_roll.data = roll_des;
    msg_yaw.data = yaw_des; 
    //    float thrust = .1;
    //    msg_thrust.data = thrust;
    
    //    ROS_INFO("%f", msg_thrust.data);
    
    //pub_thrust.publish(msg);

    pub_pitch.publish(msg_pitch);
    pub_roll.publish(msg_roll);
    pub_yaw.publish(msg_yaw);
    
    ros::spinOnce();
    
    loop_rate.sleep();
    ++count;
  }
  
  
  return 0;
}
