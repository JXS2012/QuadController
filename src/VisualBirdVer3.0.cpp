#include "VisualBird.h"

VisualBird::VisualBird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"VisualBird");

  dummy = new DummyBird(nh_,nh_private_);
  ROS_INFO("Create Dummy");
  eye = new BirdEye(nh_,nh_private_);
  ROS_INFO("Create Eye");
  nav = new Potential(nh_,nh_private_);
  ROS_INFO("Create Navigation Function");

  visual_sub = node.subscribe("/pos_points_temp",1,&VisualBird::visualCallback,this);  
  initiation();
}

VisualBird::~VisualBird()
{

  delete dummy;
  delete eye;
  delete nav;
  ROS_INFO("Destroying VisualBird "); 

}

bool VisualBird::singleJump(pcl::PointXYZ now, pcl::PointXYZ last)
{
  if (norm(vectorMinus(now, last)) > jumpThreshold)
		return true;
	else
		return false;
}

bool VisualBird::jumpDetector(pcl::PointXYZ nowr, pcl::PointXYZ nowg, pcl::PointXYZ lastr, pcl::PointXYZ lastg)
{
  if (singleJump(nowr, lastr) && singleJump(nowg,lastg))
  	return true;
  else
    return false;
}

pcl::PointXYZ VisualBird::rescaleVisual(pcl::PointXYZ input)
{
	pcl::PointXYZ output,temp;
	//drifterror = scalarProduct(120./240*(eye->getCurrentPoint().z-eye->getOriginPoint().z)/300.,input);
	temp = scalarProduct(140./240*(eye->getCurrentPoint().z-eye->getOriginPoint().z)/300,input);

  output.x = temp.x * cos(M_PI*3/4) - temp.y * sin(M_PI*3/4);
  output.y = temp.x * sin(M_PI*3/4) + temp.y * cos(M_PI*3/4);

  temp.x = eye->getCurrentPoint().x + output.x * cos(eye->getPsi()) - output.y * sin(eye->getPsi());
  temp.y = eye->getCurrentPoint().y + output.x * sin(eye->getPsi()) + output.y * cos(eye->getPsi());
  temp.z = targetBound.z;

  return temp;
}

void VisualBird::visualCallback(const std_msgs::Float32MultiArray msg)
{
  //ROS_INFO("Callback triggered");
  
  pcl::PointXYZ red,green,mid;
  red.x = msg.data[0];
  red.y = msg.data[1]-120;
  red.z = msg.data[2];

  green.x = msg.data[3];
  green.y = msg.data[4]-120;
  green.z = msg.data[5];

  if ((abs(red.x) > 160) && (abs(red.y) > 120) && (abs(green.x) > 160) && (abs(green.y) > 120))
  {
    visual_feedback = false;
    return;
  }
  else
  {
    visual_feedback = true;
    
    if (start_visual)
    {
      if ((abs(red.x > 160)) || (abs(red.y) > 120))
      	mid = green;
    	else 
      	if ((abs(green.x) > 160) || (abs(green.y) > 120))
      	  mid = red;
      	else
      	  mid = scalarProduct(0.5,vectorPlus(green,red));
      
      pcl::PointXYZ tempTarget = rescaleVisual(mid);
      
      //Check Discontinuity
      red = rescaleVisual(red);
      green = rescaleVisual(green);
      
      if (!visualBuffer.empty() && jumpDetector(red,green,lastred,lastgreen))
      {
        ROS_INFO("Discontinuity Detected!");
        //return;
      }
      
      lastred = red;
      lastgreen= green;
      
      //Check Extra large points
      if (abs(tempTarget.x) > 10000){	
       	 ROS_INFO("Target Point %.3f %.3f %.3f",targetPoint.x,targetPoint.y,targetPoint.z);   
			   ROS_INFO("Red %.3f %.3f", red.x, red.y);
			   ROS_INFO("Green %.3f %.3f", green.x, green.y);return;}
			
			//Moving average applied
      visualBuffer.push_back(tempTarget);

      //if (visualBuffer.size()<=5)
        targetPoint = vectorPlus(scalarProduct((visualBuffer.size()-1.0)/visualBuffer.size(),targetPoint), scalarProduct(1./visualBuffer.size(),visualBuffer[visualBuffer.size()-1]));

      //if (visualBuffer.size()>5)
      //  targetPoint = vectorPlus(targetPoint, scalarProduct(0.2,vectorMinus(visualBuffer[visualBuffer.size()-1], visualBuffer[visualBuffer.size()-6])));
    }
  }
}


void VisualBird::initiation()
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

  shiftOrigin.z = targetPoint.z;
  
  targetBound = shiftOrigin;
  //sleep(5);
  if (fly)
  {
    dummy->on();
    double secs =ros::Time::now().toSec();
    timeTable.push_back(secs);
  }
}

void VisualBird::readTargets()
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

void VisualBird::initParameters()
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

  double tempx, tempy;
  if (!nh_private_.getParam("originX", tempx))
    tempx = 1169;
    
  if (!nh_private_.getParam("originY", tempy))
    tempy = 954;
    
  shiftOrigin.x = tempx;
  shiftOrigin.y = tempy;

  
  ROS_INFO("Flight height initialized %.3f",flight_height);
  ROS_INFO("Boundary radius initialized %.3f",max_outter_radius);
  ROS_INFO("Fly mode initialized %d",fly);
  ROS_INFO("Fly duration %.3f",total_time);
  ROS_DEBUG("%s",strTargets.c_str());

  targetIndex = 0;
  counter = 0; //used in main loop for tracking time
  jumpThreshold = 50;
  lastred.x = 0;
  lastred.y = 0;
  lastred.z = 0;
  lastgreen = lastred;
  landing = false; //flag showing whether quadrotor is landing
  dropdown = false;
  outRange = false;
  reachStartPoint = false;
  VisualBird_finish = false;
  hover = true;
  start_visual = false;
  visual_feedback = false;
}

void VisualBird::invokeController()
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
	    	eye->updateIntError(targetPoint);
	      pcl::PointXYZ acc_des = scalarProduct(1000,nav->getAccDes());
	      dummy->navController(acc_des,vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
	      //dummy->directDrive(acc_des.x,acc_des.y,acc_des.z,eye->getPsi());
	      //eye->calculatePositionVelocityError();
	      //dummy->pidPathController(eye->getE_p(),eye->getE_v(),eye->getAcc_t(),eye->getPsi());
	    }
	}
    }
}

void VisualBird::checkStartPoint()
{
  //  if (norm(vectorMinus(eye->getCurrentPoint(),eye->getStartPoint()))<100 && !reachStartPoint)
  if (abs(vectorMinus(eye->getCurrentPoint(),targetPoint).z)<50 && !reachStartPoint)
    {
      reachStartPoint = true;
      hover = false;
      eye->resetIntError();
      ROS_INFO("Reached start point!");
    }
}

void VisualBird::safeOutRange()
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
      VisualBird_finish = true;
      */
    }
}

void VisualBird::land()
{
  //if counter over 800, start landing
  if (sensor == "vicon")
    {
      if (counter > total_time*freq && !landing)
	    {
	      switchHover();
	      //targetPoint = shiftOrigin;
	      //targetPoint.x -= 700;
	      targetPoint = eye->getCurrentPoint();
	  		landing = true;
    		start_visual = false;
	  		ROS_INFO("start landing off");
	  		double secs =ros::Time::now().toSec();
			timeTable.push_back(secs);
			}
	   if (landing && !dropdown && norm(vectorMinus(targetPoint,eye->getCurrentPoint())) <100)
	   {
	      switchHover();
	   	targetPoint = eye->getCurrentPoint();
	  		targetPoint.z = eye->getOriginPoint().z+50;
	  		dropdown = true;
	   }
  		//if in landing process and height less than 10cm, turn off motor and land
  		if (eye->getCurrentPoint().z-eye->getOriginPoint().z < 50 && dropdown)
			{
	  		if (fly)
	  		  dummy->off();
	 		VisualBird_finish = true;
			}    
    }
}

void VisualBird::switchHover()
{
  hover = true;
  eye->resetIntError();
}

void VisualBird::hoverFlight()
{
  if ( norm(vectorMinus(eye->getCurrentPoint(),targetPoint))<50 
       && (unsigned)targetIndex < targets.size()-1 )
    switchNextTarget();
}

void VisualBird::switchNextTarget()
{
  targetPoint = targets[++targetIndex];
  switchHover();
}

void VisualBird::drive()
{

  //update flight status
  eye->updateFlightStatus();
  
  ROS_DEBUG("%.3f %.3f %.3f", eye->getTargetVel().x, eye->getTargetVel().y, eye->getTargetVel().z);

  nav->updatePVA(scalarProduct(0.001,vectorMinus(eye->getCurrentPoint(),shiftOrigin)),eye->getTargetPos(),scalarProduct(0.001,eye->getCurrentVel()),eye->getTargetVel(),0);


  //ROS_DEBUG("%.3f %.3f %.3f",vectorMinus(eye->getCurrentPoint(),shiftOrigin).x,vectorMinus(eye->getCurrentPoint(),shiftOrigin).y,vectorMinus(eye->getCurrentPoint(),shiftOrigin).z);
  //check whether arrived at start point
  //checkStartPoint();

  //hover to points
  //hoverFlight();

  //check landing condition
  land();

  //check out range condition
  //safeOutRange();

  //ROS_INFO("DISTANCE %f",norm(vectorMinus(eye->getCurrentPoint(), shiftOrigin)));
  //ROS_INFO("Target %f %f %f", targetPoint.x, targetPoint.y, targetPoint.z);
  if (norm(vectorMinus(eye->getCurrentPoint(), shiftOrigin)) < 150 && !landing && visual_feedback && !start_visual)
  {
	 ROS_INFO("VISUAL GUIDANCE Initiated");
    start_visual = true;
    double secs =ros::Time::now().toSec();
    timeTable.push_back(secs); 
  }

	if (start_visual && !visualBuffer.empty() && !landing && !hover)
	{
		ROS_INFO("VISUAL GUIDANCE In Effect");
		double secs =ros::Time::now().toSec();
		timeTable.push_back(secs);
		switchHover();	
		//Notice that here switchHover is not used because we want to have z stabiled with integral feedback
	}

  //call hover controller or path controller according to (bool hover)
  invokeController();
  visualTarget.push_back(targetPoint);   
  double secs =ros::Time::now().toSec();
  timeLog.push_back(secs); 
  
  counter ++;
}

bool VisualBird::finish()
{
  return VisualBird_finish;
}

void VisualBird::writeLog(int fileNo)
{
  std::FILE * pFile;
  char buff[100];
  sprintf(buff,"/home/cooplab/jianxin/%d.txt",fileNo);
  std::string filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; (unsigned)i<visualTarget.size();i++)
			fprintf(pFile, "x %.3f y %.3f z %.3f vx %.3f vy %.3f vz %.3f ax %.3f ay %.3f az %.3f roll %.6f pitch %.6f thrust %.6f targetX %.3f targetY %.3f targetZ %.3f time %.6f\n", eye->getLogPos(i).x, eye->getLogPos(i).y, eye->getLogPos(i).z, eye->getLogVel(i).x,eye->getLogVel(i).y,eye->getLogVel(i).z, eye->getLogAcc(i).x, eye->getLogAcc(i).y, eye->getLogAcc(i).z, dummy->getLogRoll(i), dummy->getLogPitch(i), dummy->getLogThrust(i), visualTarget[i].x, visualTarget[i].y, visualTarget[i].z, timeLog[i]);
    }
  fclose(pFile);


  sprintf(buff,"/home/cooplab/jianxin/Timer%d.txt",fileNo);
  filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
    	if (timeTable.size() == 3)
    	{
  			fprintf(pFile,"Motor On %.6f\n", timeTable[0]);
			fprintf(pFile,"Visual Tracking %.6f\n", timeTable[1]);
			fprintf(pFile,"Landing %.6f\n", timeTable[2]);
		}
		else
		{
			fprintf(pFile,"Motor On %.6f\n", timeTable[0]);
			fprintf(pFile,"Landing %.6f\n", timeTable[1]);
		}
    }
  fclose(pFile);
}

