#include "smartbird.h"

smartbird::smartbird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"smartbird");
  dummy = new dummybird(nh_,nh_private_);
  listener = new(tf::TransformListener);
  transform = new(tf::StampedTransform);
  
  //initiate variables
  initiation();

}

smartbird::~smartbird()
{
  ROS_INFO("Destroying smartbird "); 

}

void smartbird::initiation()
{
  init_position();
  init_parameters();
  flightOrigin();
  init_pointcloud();
  //defining first flight target point
  targetPoint = cloud->points[0];
  //defining pre points for later velocity calculation
  prePoint = originPoint;
  printf("origin at %.3f %.3f %.3f\n",originPoint.x,originPoint.y,originPoint.z);

  if (fly)
    dummy->on();
}

void smartbird::init_parameters()
{
  freq = 30.0; //control frequency
  flight_height = 300; //unit mm
  max_outter_radius = 1200; //unit mm

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
  smartbird_finish = false;
}

void smartbird::init_position()
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

void smartbird::flightOrigin()
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

void smartbird::init_pointcloud()
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

void smartbird::transformZYXtoZXY(double roll, double pitch, double yaw)
{
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
}

void smartbird::updateFlightStatus()
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
  
  //calculate velocity using one step differentiate, can be improved by moving average method.
  currentVel = scalarProduct(freq, vectorMinus(currentPoint,prePoint));
  
  prePoint = currentPoint;
}

void smartbird::invokeController()
{
  //in case vicon lost track of model
  if (currentVel.x == 0 && currentVel.y == 0 && currentVel.z == 0)
    {
      dummy->direct_drive(0,0,0,psi);
    }
  else
    {

      if (norm(vectorMinus(currentPoint,cloud->points[0]))<100 && !reachStartPoint)
	{
	  reachStartPoint = true;
	  printf("Reached start point!\n");
	}
      if (!reachStartPoint || landing || outRange)
	dummy->pid_hover_controller(vectorMinus(currentPoint,targetPoint),currentVel,intError,psi);
      else
	dummy->pid_path_controller(e_p,e_v,acc_t,psi);
      /*

      dummy->pid_hover_controller(vectorMinus(currentPoint,targetPoint),currentVel,intError,psi);
      */

    }
}

void smartbird::shiftTargetPoint()
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

void smartbird::safeOutRange()
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

void smartbird::land()
{
  //if counter over 800, start landing
  if (counter > 800 && !landing)
    {
      intError = vectorMinus(currentPoint,currentPoint);
      targetPoint = currentPoint;
      targetPoint.z = originPoint.z;
      landing = true;
      printf("start landing off\n");
    }
  //if in landing process and height less than 10cm, turn off motor and land
  if (currentPoint.z-originPoint.z < 100 && landing)
    {
      dummy->off();
      smartbird_finish = true;
    }    
}

bool smartbird::finish()
{
  return smartbird_finish;
}

void smartbird::flightLog()
{
  x_log.push_back(currentPoint.x);
  y_log.push_back(currentPoint.y);
  z_log.push_back(currentPoint.z);
  phi_log.push_back(phi);
  theta_log.push_back(theta);
  psi_log.push_back(psi);
}

void smartbird::write_log()
{
  std::FILE * pFile;

  pFile = fopen("log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<x_log.size();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f phi %.3f theta %.3f psi %.3f roll %.6f pitch %.6f thrust %.6f\n", x_log[i], y_log[i], z_log[i], phi_log[i], theta_log[i], psi_log[i], dummy->roll_log[i], dummy->pitch_log[i], dummy->thrust_log[i]);
    }
  
}



void smartbird::calculate_position_velocity_error()
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

void smartbird::drive()
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
  counter ++;
}
