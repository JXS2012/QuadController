#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void poseHummingbirdCallback(const geometry_msgs::TransformStamped& msg){
  static tf::TransformBroadcaster br;
  br.sendTransform(msg);
}

void poseTurtleCallback(const geometry_msgs::TransformStamped& msg){
  static tf::TransformBroadcaster br;
  br.sendTransform(msg);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber subHummingbird = node.subscribe("/vicon/Hummingbird/Hummingbird", 10, &poseHummingbirdCallback);
  ros::Subscriber subTurtle = node.subscribe("/vicon/turtle/turtle", 10, &poseTurtleCallback);
 
  ros::spin();
  return 0;
};
