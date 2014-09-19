#include "quadrotor.h"

quadrotor::quadrotor(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"quadrotor");

  //control cmds publisher
  pub_thrust = node.advertise<std_msgs::Float64>("mav/cmd_thrust", 1000);
  pub_pitch = node.advertise<std_msgs::Float64>("mav/cmd_pitch", 1000);
  pub_roll = node.advertise<std_msgs::Float64>("mav/cmd_roll", 1000);
  pub_yaw = node.advertise<std_msgs::Float64>("mav/cmd_yaw", 1000);
  //motor onoff services
  client_SetMotorsOnOff = node.serviceClient<mav_srvs::SetMotorsOnOff>("mav/setMotorsOnOff");
  client_GetMotorsOnOff = node.serviceClient<mav_srvs::GetMotorsOnOff>("mav/getMotorsOnOff");

  listener = new(tf::TransformListener);
  transform = new(tf::StampedTransform);
  
  //initiate variables
  initiation();

}

quadrotor::~quadrotor()
{
  ROS_INFO("Destroying quadrotor "); 

}

void quadrotor::initiation()
{
  init_position();
  init_parameters();
  init_pid();
  flightOrigin();
  init_pointcloud();
  //defining first flight target point
  targetPoint = cloud->points[0];
  //defining pre points for later velocity calculation
  prePoint = originPoint;
  printf("origin at %.3f %.3f %.3f\n",originPoint.x,originPoint.y,originPoint.z);

  //set motor on
  //if (fly)
  //  turnMotorOn(client_SetMotorsOnOff,client_GetMotorsOnOff);
}

void quadrotor::init_parameters()
{
  g = 9.81;
  freq = 30.0; //control frequency
  heli_mass = 615;
  flight_height = 300; //unit mm
  max_outter_radius = 1200;

  root_x = 3; //pid constants for hovering
  root_y = 3;
  
  shift_no = 0; //number of shifts performed during flight if in hovering mode
  shift_time = 300; //total time for each shift = shift_time/freq
  shift_distance = 300;
  current_shift = 0; //counting current shift steps,

  control_mode = 0; //0 for hover, 1 for path following
  counter = 0; //used in main loop for tracking time, current time = counter/freq
  
  fly = true; //turn motor on or not. If false, quadrotor won't fly. For testing purpose
  landing = false; //flag showing whether quadrotor is landing
  outRange = false;
  reachStartPoint = false;

}

void quadrotor::init_position()
{
  currentPoint.x = 0;
  currentPoint.y = 0;
  currentPoint.z = 0;
  prePoint = currentPoint;
  originPoint = currentPoint;
  targetPoint = currentPoint;
  currentVel = currentPoint;
  intError = currentPoint;
  shiftedOrigin = currentPoint;
  shiftedOrigin.y = 800;

  phi = 0;
  psi = 0;
  theta = 0;
}

void quadrotor::init_pid()
{  
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

void quadrotor::flightOrigin()
{
  //finding start point of flight
  while (true)
    {
      try{
	listener->lookupTransform("/world", "/vicon/bird/bird",  
				 ros::Time(0), *transform);
      }
      catch (tf::TransformException ex){
	ROS_ERROR("%s",ex.what());
	usleep(100);
	continue;
      }
      originPoint.x = transform->getOrigin().x()*1000;
      originPoint.y = transform->getOrigin().y()*1000;
      originPoint.z = transform->getOrigin().z()*1000;
      break;
    }

}

void quadrotor::init_pointcloud()
{
  double flight_radius = 500;
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_vel.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_acc.reset(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  cloud_vel->width = 1000;
  cloud_vel->height = 1;
  cloud_vel->points.resize (cloud->width * cloud->height);

  cloud_acc->width = 1000;
  cloud_acc->height = 1;
  cloud_acc->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = flight_radius*cos(i*3.14/180.0)+shiftedOrigin.x;
    cloud->points[i].y = flight_radius*sin(i*3.14/180.0)+shiftedOrigin.y;
    cloud->points[i].z = 300;
    cloud_vel->points[i].x = -flight_radius*sin(i*3.14/180.0);
    cloud_vel->points[i].y = flight_radius*cos(i*3.14/180.0);
    cloud_vel->points[i].z = 0;
    cloud_acc->points[i].x = -flight_radius*cos(i*3.14/180.0);
    cloud_acc->points[i].y = -flight_radius*sin(i*3.14/180.0);
    cloud_acc->points[i].z = 0;
  }

  kdtree.setInputCloud(cloud);
}


void quadrotor::transformZYXtoZXY(double roll, double pitch, double yaw)
{
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
}

void quadrotor::updateFlightStatus()
{
  double roll,pitch,yaw;
  try{
    listener->lookupTransform("/world", "/vicon/bird/bird",  
			     ros::Time(0), *transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  //get angles in ZYX
  btMatrix3x3(transform->getRotation()).getEulerZYX(yaw,pitch,roll);
  
  //transform from euler zyx to euler zxy
  transformZYXtoZXY(roll,pitch,yaw);
  
  //getting positions x,y,z
  currentPoint.x = transform->getOrigin().x()*1000;
  currentPoint.y = transform->getOrigin().y()*1000;
  currentPoint.z = transform->getOrigin().z()*1000;
  
  //calculate cummulated error
  if (!reachStartPoint || landing || outRange)
    intError = vectorPlus(intError, vectorMinus(currentPoint,targetPoint) );
  else
    intError = vectorMinus(currentPoint,currentPoint);
  
  //calculate velocity using one step differentiate, can be improved by moving average method.
  currentVel = scalarProduct(freq, vectorMinus(currentPoint,prePoint));
  
  prePoint = currentPoint;
}

double quadrotor::interpolate_thrust(double input)
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

void quadrotor::pid_hover_control()
{
  double acc_x_des, acc_y_des, acc_z_des;
  //  printf("hover called\n");
  acc_x_des = -kp_hover.x*(currentPoint.x-targetPoint.x) - kd_hover.x*currentVel.x - ki_hover.x*intError.x;
  acc_y_des = -kp_hover.y*(currentPoint.y-targetPoint.y) - kd_hover.y*currentVel.y - ki_hover.y*intError.y;
  acc_z_des = -kp_hover.z*(currentPoint.z-targetPoint.z) - kd_hover.z*currentVel.z - ki_hover.z*intError.z;
  //printf("acc x %.3f, y %.3f z %.3f\n",acc_x,acc_y,acc_z);

  acc_des2control(acc_x_des,acc_y_des,acc_z_des);

}

void quadrotor::pid_path_control()
{
  double acc_x_des, acc_y_des, acc_z_des;
  //printf("path called\n");
  acc_x_des = -kp_path.x*(e_p.x) - kd_path.x*e_v.x + acc_t.x;
  acc_y_des = -kp_path.y*(e_p.y) - kd_path.y*e_v.y + acc_t.y;
  acc_z_des = -kp_path.z*(e_p.z) - kd_path.z*e_v.z + acc_t.z;
  //printf("x %.3f y %.3f, z %.3f error x%.3f ,y%.3f ,z%.3f acc_t in x %.3f, y%.3f,z%.3f\n",currentPoint.x,currentPoint.y,currentPoint.z,e_p.x,e_p.y,e_p.z,acc_t.x,acc_t.y,acc_t.z);
  //printf("component kp_x %.3f kd_x %.3f acc %.3f\n", kp_path.x*(e_p.x),kd_path.x*e_v.x,acc_t.x);

  acc_des2control(acc_x_des,acc_y_des,acc_z_des);
}

void quadrotor::acc_des2control(double acc_x_des, double acc_y_des, double acc_z_des)
{
  //using linearized invert dynamics to calculate desired roll,pitch,thrust cmd
  roll_in =  -(acc_x_des/1000.0*sin(psi) - acc_y_des/1000.0*cos(psi))/g;
  pitch_in = (acc_x_des/1000.0*cos(psi) + acc_y_des/1000.0*sin(psi))/g;
  thrust_in = interpolate_thrust(acc_z_des/1000.0*heli_mass/g);
  //printf("input cmd roll %.3f pitch %.3f thrust %.3f\n",roll_in,pitch_in,thrust_in);
 
  roll_log.push_back(roll_in);
  pitch_log.push_back(pitch_in);
  thrust_log.push_back(thrust_in);

}

void quadrotor::invokeController()
{
  //in case vicon lost track of model
  if (currentVel.x == 0 && currentVel.y == 0 && currentVel.z == 0)
    {
      pitch_in = 0;
      roll_in = 0;
      thrust_in = interpolate_thrust(0);
    }
  else
    {
      /*
      if (norm(vectorMinus(currentPoint,cloud->points[0]))<100 && !reachStartPoint)
	{
	  reachStartPoint = true;
	  printf("Reached start point!\n");
	}
      if (!reachStartPoint || landing || outRange)
	pid_hover_control();
      else
	pid_path_control();
      */

      pid_hover_control();

    }
}

void quadrotor::shiftTargetPoint()
{
  //set new target point to achieve shifting, only available at hovering
  if (counter > shift_time*current_shift && current_shift<shift_no && control_mode == 0)
    {
      printf ("%.3f shift\n",current_shift);
      current_shift++;
      targetPoint.x += shift_distance;
      targetPoint.y += shift_distance;
    }
}

void quadrotor::safeOutRange()
{
  shiftedOrigin.z = currentPoint.z;
  if (norm(vectorMinus(currentPoint,shiftedOrigin))>max_outter_radius && !outRange)
    {
      printf("origin %.3f %.3f %.3f\n",shiftedOrigin.x,shiftedOrigin.y,shiftedOrigin.z);
      printf("out of range at point %.3f %.3f %.3f!\n",currentPoint.x,currentPoint.y,currentPoint.z);
      outRange = true;
      intError = vectorMinus(currentPoint,currentPoint);
      targetPoint = originPoint;
      targetPoint.z = originPoint.z+flight_height;
      printf("reset target point %.3f %.3f %.3f!\n",targetPoint.x,targetPoint.y,targetPoint.z);
      if (counter<700)
	counter = 700;
    }
}

void quadrotor::land()
{
  //if counter over 1000, start landing
  if (counter > 800 && !landing)
    {
      intError = vectorMinus(currentPoint,currentPoint);
      targetPoint = currentPoint;
      targetPoint.z = originPoint.z;
      landing = true;
      printf("start landing off\n");
    }
  //if in landing process and height less than 10cm, turn off motor and land
  if (currentPoint.z-originPoint.z < 100 && landing && fly)
    {
      turnMotorOff(client_SetMotorsOnOff,client_GetMotorsOnOff);
    }    
}

bool quadrotor::finish()
{
  mav_srvs::GetMotorsOnOff srv_GetMotorsOnOff;
  client_GetMotorsOnOff.call(srv_GetMotorsOnOff);
  
  if (currentPoint.z-originPoint.z < 100 && landing && fly && !srv_GetMotorsOnOff.response.on)
    return true;
  else 
    return false;
}

void quadrotor::flightLog()
{
  x_log.push_back(currentPoint.x);
  y_log.push_back(currentPoint.y);
  z_log.push_back(currentPoint.z);
  phi_log.push_back(phi);
  theta_log.push_back(theta);
  psi_log.push_back(psi);
}

void quadrotor::write_log()
{
  std::FILE * pFile;

  pFile = fopen("log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<x_log.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f vel_x %.3f vel_y %.3f vel_z %.3f roll %.6f pitch %.6f thrust %.6f\n", x_log[i], y_log[i], z_log[i], phi_log[i], theta_log[i], psi_log[i], roll_log[i], pitch_log[i], thrust_log[i]);
    }
  
}



void quadrotor::calculate_position_velocity_error()
{
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);

  if (kdtree.nearestKSearch(currentPoint,1,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
    {
      for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	{
	  pcl::PointXYZ pathPoint,pathPoint_vel,pathPoint_acc,pathTangent,pathNormal,pathBinormal;
	  pathPoint = cloud->points[ pointIdxNKNSearch[i] ];
	  pathPoint_vel = cloud_vel->points[ pointIdxNKNSearch[i] ];
	  pathPoint_acc = cloud_acc->points[ pointIdxNKNSearch[i] ];

	  pathTangent = normalize(pathPoint_vel);
	  //pathNormal = normalize(pathPoint_acc);
	  //pathBinormal = crossProduct(pathTangent,pathNormal);

	  e_p = vectorMinus(currentPoint,pathPoint);
	  e_p = vectorMinus(e_p,scalarProduct( dotProduct(e_p,pathTangent),pathTangent) );
	  //std::cout <<currentPoint.x<<"  "<<currentPoint.y<<"    "<<currentPoint.z<<"path"<<pathPoint.x <<"   "<<pathPoint.y<<"   "<<pathPoint.z<<std::endl;

	  e_v = vectorMinus(currentVel,pathPoint_vel);

	  acc_t = pathPoint_acc;
	  //std::cout <<"   "<<e_p.x<<"    "<<e_p.y<<"    "<<e_p.z<<std::endl;
	}
      
    }
}

void quadrotor::turnMotorOn(ros::ServiceClient client_SetMotorsOnOff, ros::ServiceClient client_GetMotorsOnOff)
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

void quadrotor::turnMotorOff(ros::ServiceClient client_SetMotorsOnOff, ros::ServiceClient client_GetMotorsOnOff)
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

void quadrotor::drive()
{
    //update current position,velocity,attitude,integrated error for hover control
    updateFlightStatus();

    //calculate current position error,velocity error and accelaration w.r.t closest point on path for path control
    calculate_position_velocity_error();

    //keep flight log of position and attitude
    flightLog();

    //if counter over 1000(condition can be modified), start landing process
    land();

    //out range check
    safeOutRange();

    //call hover controller or path controller according to flight_mode
    invokeController();

    //if needed, shift target point
    //shiftTargetPoint();

    msg_thrust.data = thrust_in;
    msg_pitch.data = pitch_in;
    msg_roll.data = roll_in;
    msg_yaw.data = yaw_in; 
    
    pub_pitch.publish(msg_pitch);
    pub_roll.publish(msg_roll);
    pub_yaw.publish(msg_yaw);
    pub_thrust.publish(msg_thrust);

    counter ++;
}
