#include "birdeye.h"

birdeye::birdeye(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"birdeye");
  listener = new(tf::TransformListener);
  transform = new(tf::StampedTransform);
  
  //initiate variables
  initiation();

}

birdeye::~birdeye()
{
  ROS_INFO("Destroying birdeye "); 
}

void birdeye::initiation()
{
  init_position();
  init_parameters();
  flightOrigin();
  init_pointcloud();

  //defining pre points for later velocity calculation
  prePoint = originPoint;
  printf("origin at %.3f %.3f %.3f\n",originPoint.x,originPoint.y,originPoint.z);

}

void birdeye::init_parameters()
{
  flight_radius = 500;
  freq = 30;
  zeroVector.x = 0;
  zeroVector.y = 0;
  zeroVector.z = 0;
}

void birdeye::init_position()
{
  currentPoint = zeroVector;
  prePoint = zeroVector;
  originPoint = zeroVector;
  currentVel = zeroVector;
  intError = zeroVector;
  shiftedOrigin = zeroVector;
  shiftedOrigin.y = 800;

  phi = 0;
  psi = 0;
  theta = 0;
}

void birdeye::flightOrigin()
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

void birdeye::init_pointcloud()
{
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

void birdeye::transformZYXtoZXY()
{
    phi = asin(sin(roll)*cos(pitch));
    double psi_1 = cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
    double psi_2 = cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    psi = atan2(-psi_1,psi_2);
    theta = atan2(tan(pitch),cos(roll));  
}

void birdeye::updateFlightStatus()
{
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
  transformZYXtoZXY();
  
  //getting positions x,y,z
  currentPoint.x = transform->getOrigin().x()*1000;
  currentPoint.y = transform->getOrigin().y()*1000;
  currentPoint.z = transform->getOrigin().z()*1000;
  
  //calculate velocity using one step differentiate, can be improved by moving average method.
  currentVel = scalarProduct(freq, vectorMinus(currentPoint,prePoint));
  
  prePoint = currentPoint;

  flightLog();
}

void birdeye::updateIntError(pcl::PointXYZ targetPoint)
{
  intError = vectorPlus(intError, vectorMinus(currentPoint,targetPoint) );
}

void birdeye::resetIntError()
{
  intError = zeroVector;
}

void birdeye::flightLog()
{
  x_log.push_back(currentPoint.x);
  y_log.push_back(currentPoint.y);
  z_log.push_back(currentPoint.z);
  phi_log.push_back(phi);
  theta_log.push_back(theta);
  psi_log.push_back(psi);
}

void birdeye::calculate_position_velocity_error()
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

pcl::PointXYZ birdeye::getShiftedOrigin()
{
  return shiftedOrigin;
}

pcl::PointXYZ birdeye::getStartPoint()
{
  return cloud->points[0];
}

pcl::PointXYZ birdeye::getCurrentPoint()
{
  return currentPoint;
}
 
pcl::PointXYZ birdeye::getOriginPoint()
{
  return originPoint;
}

pcl::PointXYZ birdeye::getE_p()
{
  return e_p;
}
 
pcl::PointXYZ birdeye::getE_v()
{
  return e_v;
}
 
pcl::PointXYZ birdeye::getAcc_t()
{
  return acc_t;
}
 
pcl::PointXYZ birdeye::getCurrentVel()
{
  return currentVel;
}

pcl::PointXYZ birdeye::getIntError()
{
  return intError;
}

double birdeye::getPhi()
{
  return phi;
}
 
double birdeye::getPsi()
{
  return psi;
}
 
double birdeye::getTheta()
{
  return theta;
}

float birdeye::getLog_x(int i)
{
  return x_log[i];
}

float birdeye::getLog_y(int i)
{
  return y_log[i];
}

float birdeye::getLog_z(int i)
{
  return z_log[i];
}

float birdeye::getLog_phi(int i)
{
  return phi_log[i];
}

float birdeye::getLog_theta(int i)
{
  return theta_log[i];
}

float birdeye::getLog_psi(int i)
{
  return psi_log[i];
}

int birdeye::getLogSize()
{
  return x_log.size();
}
