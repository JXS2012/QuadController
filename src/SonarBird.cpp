#include "SonarBird.h"

SonarBird::SonarBird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"SonarBird");

  dummy = new DummyBird(nh_,nh_private_);
  ROS_INFO("Create Dummy");
  sonar = new SonarEye(nh_,nh_private_);
  ROS_INFO("Create Sonar");

  initiation();
}

SonarBird::~SonarBird()
{

  delete dummy;
  delete sonar;
  ROS_INFO("Destroying SonarBird "); 

}

void SonarBird::initiation()
{
  initParameters();

  targetPoint.z = flight_height;
  targetPoint.x = 0;
  targetPoint.y = 0;

  ROS_INFO("hover to %.3f %.3f %.3f",targetPoint.x,targetPoint.y,targetPoint.z);
  sleep(5);
  if (fly)
    dummy->on();
}

void SonarBird::readTargets()
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

void SonarBird::initParameters()
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
  hover = true;
}

void SonarBird::invokeController()
{
  if (sensor == "sonar")
    {
      if (hover)
	{
	  sonar->updateIntError(targetPoint);
	  dummy->pidHoverController(vectorMinus(sonar->getCurrentPoint(),targetPoint),sonar->getCurrentVel(),sonar->getIntError(),sonar->getPsi());
	}
    }
}

void SonarBird::land()
{
  if (sensor == "sonar")
    {
      if (counter > total_time*freq && !landing)
	{
	  switchHover();
	  targetPoint = sonar->getCurrentPoint();
	  targetPoint.z = 0;
	  landing = true;
	  ROS_INFO("start landing off");
	}
      //if in landing process and height less than 10cm, turn off motor and land
      if (sonar->getCurrentPoint().z < 50 && landing)
	{
	  if (fly)
	    dummy->off();
	  highbird_finish = true;
	}    
    }
}

void SonarBird::switchHover()
{
  hover = true;
  sonar->resetIntError();
}

void SonarBird::hoverFlight()
{
  if ( norm(vectorMinus(sonar->getCurrentPoint(),targetPoint))<50 
       && (unsigned)targetIndex < targets.size()-1 )
    switchNextTarget();
}

void SonarBird::switchNextTarget()
{
  targetPoint = targets[++targetIndex];
  switchHover();
}

void SonarBird::drive()
{
  sonar->updateFlightStatus();

  land();

  invokeController();
    
  counter ++;
}

bool SonarBird::finish()
{
  return highbird_finish;
}

void SonarBird::writeLog(int fileNo)
{
  std::FILE * pFile;
  char buff[100];
  std::string filename;

  sprintf(buff,"/home/jianxin/sonar%d.txt",fileNo);
  filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<sonar->getLogSize();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f vx %.3f vy %.3f vz %.3f ax %.3f ay %.3f az %.3f roll %.6f pitch %.6f thrust %.6f\n", sonar->getLogPos(i).x, sonar->getLogPos(i).y, sonar->getLogPos(i).z, sonar->getLogVel(i).x,sonar->getLogVel(i).y,sonar->getLogVel(i).z, sonar->getLogAcc(i).x, sonar->getLogAcc(i).y, sonar->getLogAcc(i).z, dummy->getLogRoll(i), dummy->getLogPitch(i), dummy->getLogThrust(i));
    }
  fclose(pFile);

}

bool SonarBird::sonarInitiate()
{
  return sonar->flightOrigin();
}
