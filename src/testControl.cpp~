#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "math.h"

#include <sstream>

using namespace std;

int main(int argc, char **argv)
{


  ros::init(argc, argv, "talker");

  ros::NodeHandle thrust;
  ros::NodeHandle pitch;
  ros::NodeHandle roll;
  ros::NodeHandle yaw;


  ros::Publisher pub_thrust = thrust.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  ros::Publisher pub_pitch = pitch.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  ros::Publisher pub_roll = roll.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  ros::Publisher pub_yaw = yaw.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);

  ros::Rate loop_rate(20);
  
  while (ros::ok())
  {

    std_msgs::Float64 msg_thrust;
    std_msgs::Float64 msg_pitch;
    std_msgs::Float64 msg_roll;
    std_msgs::Float64 msg_yaw;

    
    float thrust_in,pitch_in,roll_in,yaw_in;
    
    cout << "please input thrust(0,1),pitch(-1,1),roll(-1,1),yaw(-4.43rad/s,4.43rad/s):\n";
    cin >> thrust_in >> pitch_in >> roll_in >> yaw_in;

    msg_thrust.data = thrust_in;
    msg_pitch.data = pitch_in;
    msg_roll.data = roll_in;
    msg_yaw.data = yaw_in; 

    int counter = 0;
    while(counter < 100)
      {
	pub_pitch.publish(msg_pitch);
	pub_roll.publish(msg_roll);
	pub_yaw.publish(msg_yaw);
	pub_thrust.publish(msg_thrust);
	//cout << "publish!\n";
	ros::spinOnce();
	
	loop_rate.sleep();
	counter ++;
      }
    char flag;
    cout << "continue?(y or n):";
    cin >> flag;
    if (flag == 'n')
      {
	msg_thrust.data = 0;
	msg_pitch.data = 0;
	msg_roll.data = 0;
	msg_yaw.data = 0; 
	pub_pitch.publish(msg_pitch);
	pub_roll.publish(msg_roll);
	pub_yaw.publish(msg_yaw);
	pub_thrust.publish(msg_thrust);
	break;
      }
    else
      continue;
  }
  
  
  return 0;
}
