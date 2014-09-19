#include "HighBird.h"

HighBird::HighBird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"HighBird");

  dummy = new DummyBird(nh_,nh_private_);
  ROS_INFO("Create Dummy");
  eye = new BirdEye(nh_,nh_private_);
  ROS_INFO("Create Eye");
  nav = new Potential(nh_,nh_private_);
  ROS_INFO("Create Navigation Function");

  initiation();
  //initiate variables

}

HighBird::~HighBird()
{

  delete dummy;
  delete eye;
  delete nav;
  ROS_INFO("Destroying HighBird "); 

}

void HighBird::initiation()
{
  initParameters();

  //defining first flight target point
  //targetPoint = eye->getStartPoint();

  //readTargets();
  
  targetPoint = eye->getOriginPoint();
  //targetPoint.z += flight_height;
  targetPoint.z += flight_height;
  //targetPoint.x += flight_height;
  //targetPoint.y += flight_height;

  ROS_INFO("hover to %.3f %.3f %.3f",targetPoint.x,targetPoint.y,targetPoint.z);

  shiftOrigin.x = 1528.3;
  shiftOrigin.y = 1796.6;
  shiftOrigin.z = 0;
  sleep(5);
  if (fly)
    dummy->on();
}

void HighBird::readTargets()
{
  pcl::PointXYZ temp;
  std::istringstream iss(strTargets);

  while (iss)
    {
      std::string sub1,sub2,sub3;
      iss >>sub1 >>sub2 >>sub3;
      temp.x = atof(sub1.c_str());
      temp.y = atof(sub2.c_str());
      temp.z = atof(sub3.c_str());
      targets.push_back(temp);
    }

  targets.pop_back();
  for (int i = 0; (unsigned)i < targets.size(); i++)
    ROS_INFO("Target point %d is (%.3f,%.3f,%.3f)",i+1,targets[i].x,targets[i].y,targets[i].z);

  targetPoint = targets[0];
}

void HighBird::initParameters()
{
  if (!nh_private_.getParam("flight_height", flight_height))
    flight_height = 300; //unit mm

  if (!nh_private_.getParam("max_outter_radius", max_outter_radius))
    max_outter_radius = 1200; //unit mm

  if (!nh_private_.getParam("fly", fly))
    fly = false; //turn motor on or not. 

  if (!nh_private_.getParam("targets",strTargets) )
    strTargets = "0,600,300";

  if (!nh_private_.getParam("freq",freq))
    freq = 30;

  if (!nh_private_.getParam("total_time",total_time))
    total_time = 30;

  if (!nh_private_.getParam("sensor", sensor))
    sensor = "vicon"; //unit mm

  ROS_INFO("Flight height initialized %.3f",flight_height);
  ROS_INFO("Boundary radius initialized %.3f",max_outter_radius);
  ROS_INFO("Fly mode initialized %d",fly);
  ROS_INFO("Fly duration %.3f",total_time);
  ROS_DEBUG("%s",strTargets.c_str());

  targetIndex = 0;
  counter = 0; //used in main loop for tracking time
  
  landing = false; //flag showing whether quadrotor is landing
  outRange = false;
  reachStartPoint = false;
  highbird_finish = false;
  hover = false;
}

void HighBird::invokeController()
{
  //in case vicon lost track of model
  if (sensor == "vicon")
    {
      if (eye->getCurrentVel().x == 0 && eye->getCurrentVel().y == 0 && eye->getCurrentVel().z == 0)
	{
	  dummy->directDrive(0,0,0,eye->getPsi());
	}
      else
	{
	  if (hover)
	    {
	      eye->updateIntError(targetPoint);
	      dummy->pidHoverController(vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
	    }
	  else
	    {
	      pcl::PointXYZ acc_des = scalarProduct(1000,nav->getAccDes());
	      dummy->navController(acc_des,vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
	      //dummy->directDrive(acc_des.x,acc_des.y,acc_des.z,eye->getPsi());
	      //eye->calculatePositionVelocityError();
	      //dummy->pidPathController(eye->getE_p(),eye->getE_v(),eye->getAcc_t(),eye->getPsi());
	    }
	}
    }
}

void HighBird::checkStartPoint()
{
  //  if (norm(vectorMinus(eye->getCurrentPoint(),eye->getStartPoint()))<100 && !reachStartPoint)
  if (norm(vectorMinus(eye->getCurrentPoint(),targetPoint))<50 && !reachStartPoint)
    {
      reachStartPoint = true;
      hover = false;
      ROS_INFO("Reached start point!");
    }
}

void HighBird::safeOutRange()
{
  pcl::PointXYZ temp = eye->getOriginPoint();
  temp.z = eye->getCurrentPoint().z;
  if (norm(vectorMinus(eye->getCurrentPoint(),temp))>max_outter_radius && !outRange)
    {
      ROS_INFO("origin %.3f %.3f %.3f",temp.x,temp.y,temp.z);
      ROS_INFO("out of range at point %.3f %.3f %.3f!",eye->getCurrentPoint().x,eye->getCurrentPoint().y,eye->getCurrentPoint().z);
      outRange = true;
      
      switchHover();
      targetPoint = eye->getOriginPoint();
      targetPoint.z += flight_height;
      ROS_INFO("reset target point %.3f %.3f %.3f!",targetPoint.x,targetPoint.y,targetPoint.z);
      if (counter<700)
	counter = 700;
      /*
      if (fly)
	dummy->off();
      highbird_finish = true;
      */
    }
}

void HighBird::land()
{
  //if counter over 800, start landing
  if (sensor == "vicon")
    {
      if (counter > total_time*freq && !landing)
	{
	  switchHover();
	  targetPoint = eye->getCurrentPoint();
	  targetPoint.z = eye->getOriginPoint().z;
	  landing = true;
	  ROS_INFO("start landing off");
	}
      //if in landing process and height less than 10cm, turn off motor and land
      if (eye->getCurrentPoint().z-eye->getOriginPoint().z < 50 && landing)
	{
	  if (fly)
	    dummy->off();
	  highbird_finish = true;
	}    
    }
}

void HighBird::switchHover()
{
  hover = true;
  eye->resetIntError();
}

void HighBird::hoverFlight()
{
  if ( norm(vectorMinus(eye->getCurrentPoint(),targetPoint))<50 
       && (unsigned)targetIndex < targets.size()-1 )
    switchNextTarget();
}

void HighBird::switchNextTarget()
{
  targetPoint = targets[++targetIndex];
  switchHover();
}

void HighBird::drive()
{
  //update flight status
  eye->updateFlightStatus();
  
  ROS_DEBUG("%.3f %.3f %.3f", eye->getTargetVel().x, eye->getTargetVel().y, eye->getTargetVel().z);
  
  nav->updatePVA(scalarProduct(0.001,vectorMinus(eye->getCurrentPoint(),shiftOrigin)),eye->getTargetPos(),scalarProduct(0.001,eye->getCurrentVel()),eye->getTargetVel(),0);

  ROS_DEBUG("%.3f %.3f %.3f",vectorMinus(eye->getCurrentPoint(),shiftOrigin).x,vectorMinus(eye->getCurrentPoint(),shiftOrigin).y,vectorMinus(eye->getCurrentPoint(),shiftOrigin).z);
  //check whether arrived at start point
  checkStartPoint();

  //hover to points
  //hoverFlight();

  //check landing condition
  land();

  //check out range condition
  //safeOutRange();

  //call hover controller or path controller according to (bool hover)
  invokeController();
    
  counter ++;
}

bool HighBird::finish()
{
  return highbird_finish;
}

void HighBird::writeLog(int fileNo)
{
  std::FILE * pFile;
  char buff[100];
  sprintf(buff,"/home/cooplab/jianxin/%d.txt",fileNo);
  std::string filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<eye->getLogSize();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f vx %.3f vy %.3f vz %.3f ax %.3f ay %.3f az %.3f roll %.6f pitch %.6f thrust %.6f targetX %.3f targetY %.3f targetZ %.3f\n", eye->getLogPos(i).x, eye->getLogPos(i).y, eye->getLogPos(i).z, eye->getLogVel(i).x,eye->getLogVel(i).y,eye->getLogVel(i).z, eye->getLogAcc(i).x, eye->getLogAcc(i).y, eye->getLogAcc(i).z, dummy->getLogRoll(i), dummy->getLogPitch(i), dummy->getLogThrust(i), eye->getLogTarget(i).x, eye->getLogTarget(i).y, eye->getLogTarget(i).z);
    }
  fclose(pFile);

}

