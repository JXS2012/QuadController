#include "VisualBirdVer5.h"

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

  visual_vel_sub = node.subscribe("/pos_points_temp",1,&VisualBird::visualVelCallback,this); 
  visual_pos_sub = node.subscribe("/com_temp",1,&VisualBird::visualPosCallback,this); 
  target_pub = node.advertise<geometry_msgs::PointStamped>("targetPoint",1);
  pos_points_pub = node.advertise<std_msgs::Float32MultiArray>("set_target_states", 10);
  initiation();
}

VisualBird::~VisualBird()
{

  delete dummy;
  delete eye;
  delete nav;
  ROS_INFO("Destroying VisualBird "); 

}

pcl::PointXYZ VisualBird::rescaleVisual(pcl::PointXYZ input)
{
  pcl::PointXYZ output,temp;
  //drifterror = scalarProduct(120./240*(eye->getCurrentPoint().z-eye->getOriginPoint().z)/300.,input);
  temp = scalarProduct(140./120*(eye->getCurrentPoint().z-eye->getOriginPoint().z)/300,input);
  ROS_DEBUG("OUTPUT %f %f", temp.x, temp.y);

  output.x = temp.x * cos(M_PI*3/4) - temp.y * sin(M_PI*3/4);
  output.y = temp.x * sin(M_PI*3/4) + temp.y * cos(M_PI*3/4);


  temp.x = output.x * cos(eye->getPsi()) - output.y * sin(eye->getPsi());
  temp.y = output.x * sin(eye->getPsi()) + output.y * cos(eye->getPsi());
  temp.z = 0;

  return temp;

}

void VisualBird::visualVelCallback(const std_msgs::Float32MultiArray msg)
{

  pcl::PointXYZ displacement, targetVel;
  displacement.x = msg.data[0];
  displacement.y = msg.data[1];
  displacement.z = 0;

  if (displacement.x == 0 && displacement.y == 0)
  {
    visual_vel_feedback = false;
    return;
  }
  else
  {
    visual_vel_feedback = true;
    
    if (start_visual)
    { 
      if ( lastQuad.x == 0 && lastQuad.y == 0 && lastQuad.z == 0)
	{
	  lastQuad = eye->getCurrentPoint();
	}
      else
	{
	  //targetVel = scalarProduct(1/0.1, vectorPlus(rescaleVisual(displacement), vectorMinus(lastQuad, eye->getCurrentPoint())));
	  lastQuad = eye->getCurrentPoint();
	}
    }

  }
}

bool VisualBird::jumpDetector(pcl::PointXYZ now, pcl::PointXYZ last)
{
  if (norm(vectorMinus(now, last)) > jumpThreshold)
    return true;
  else
    return false;
}



void VisualBird::visualPosCallback(const std_msgs::Float32MultiArray msg)
{
  //ROS_INFO("Callback triggered");
  
  pcl::PointXYZ mid;
  mid.x = msg.data[0];
  mid.y = msg.data[1];
  mid.z = 0;


  if ((mid.x == 0) && (mid.y == 0))
  {
    visual_feedback = false;
    return;
  }
  else
  {
    visual_feedback = true;
    
    if (start_visual)
    {
      pcl::PointXYZ tempTarget = rescaleVisual(mid);
      ROS_DEBUG("Target drift %.3f %.3f %.3f Psi %.3f", tempTarget.x, tempTarget.y, tempTarget.z, eye->getPsi());
      tempTarget.x += eye->getCurrentPoint().x;
      tempTarget.y += eye->getCurrentPoint().y;
      tempTarget.z = targetBound.z;
      
      if (!visualBuffer.empty() && jumpDetector(mid,lastmid))
      {
        ROS_INFO("Discontinuity Detected!");
        //return;
      }
      
      lastmid = mid;
      
      //Check Extra large points
      if (abs(tempTarget.x) > 10000){	
	ROS_INFO("Target Point %.3f %.3f %.3f",targetPoint.x,targetPoint.y,targetPoint.z);   
	ROS_INFO("mid %.3f %.3f", mid.x, mid.y);
	return;}
			
      //Moving average applied

      visualBuffer.push_back(tempTarget);
      /*
      if (visualBuffer.size()<=5)
	targetPoint = vectorPlus(scalarProduct((visualBuffer.size()-1.0)/visualBuffer.size(),targetPoint), scalarProduct(1./visualBuffer.size(),visualBuffer[visualBuffer.size()-1]));

      if (visualBuffer.size()>5)
	targetPoint = vectorPlus(targetPoint, scalarProduct(0.2,vectorMinus(visualBuffer[visualBuffer.size()-1], visualBuffer[visualBuffer.size()-6])));
      */

      targetPoint = tempTarget;
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

  zeroVector.x = 0;
  zeroVector.y = 0;
  zeroVector.z = 0;
  lastmid = zeroVector;
  lastQuad = zeroVector;

  landing = false; //flag showing whether quadrotor is landing
  dropdown = false;
  climbup = false;
  outRange = false;
  reachStartPoint = false;
  VisualBird_finish = false;
  hover = true;
  start_visual = false;
  visual_feedback = false;

  //virtual target
  circleStart = false;
  targetCircleCenter = zeroVector;
  radius = 200;
  pi = 3.14159265;
  omega = 0.5;
  targetVel = zeroVector;
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
	      pcl::PointXYZ acc_des = scalarProduct(1000., nav->getAccDes());
	      vel_ref.push_back(nav->getVelDes());
	      acc_ref.push_back(nav->getAccDes());
	      //ROS_INFO("DESIRED ACC %.3f %.3f %.3f", acc_des.x/1000, acc_des.y/1000, acc_des.z/1000);
	      dummy->navController(acc_des,vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
	    }
	}
    }
}

void VisualBird::checkStartPoint()
{
  //  if (norm(vectorMinus(eye->getCurrentPoint(),eye->getStartPoint()))<100 && !reachStartPoint)
  if (abs(vectorMinus(eye->getCurrentPoint(),targetPoint).z)<100 && !reachStartPoint)
    {
      reachStartPoint = true;

      pcl::PointXYZ newTarget = shiftOrigin;
      newTarget.y = newTarget.y - radius - 50;
      switchNavTarget(shiftOrigin, zeroVector);
      //hover = false;
      //eye->resetIntError();
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

void VisualBird::switchNavTarget(pcl::PointXYZ pos, pcl::PointXYZ vel)
{
  hover = false;
  eye->resetIntError();
  
  std_msgs::Float32MultiArray pts;
  ROS_INFO("Switched to Navigation Mode %.3f %.3f %.3f", pos.x, pos.y, pos.z);
  pts.data.push_back((pos.x - shiftOrigin.x)/1000.);
  pts.data.push_back((pos.y - shiftOrigin.y)/1000.);
  pts.data.push_back(pos.z - shiftOrigin.z);
  
  pts.data.push_back(vel.x);
  pts.data.push_back(vel.y);
  pts.data.push_back(vel.z);

  pos_points_pub.publish(pts);
  targetPoint = pos;
}

void VisualBird::land()
{
  //if counter over 800, start landing
  if (sensor == "vicon")
    {
      if (counter > total_time*freq && !landing)
	{
	  //switchHover();
	  if (!hover)
	    {
	      switchHover();
	    };
	  targetPoint = eye->getCurrentPoint();
	  targetPoint.z = eye->getOriginPoint().z + flight_height + 100;

	  landing = true;
	  start_visual = false;
	  ROS_INFO("start landing off");
	  double secs =ros::Time::now().toSec();
	  timeTable.push_back(secs);
	}
      if ( landing && !climbup && abs(targetPoint.z - eye->getCurrentPoint().z) < 20)
	{
	  //switchHover();
	  //targetPoint = eye->getCurrentPoint();
	  //targetPoint = shiftOrigin;
	  //targetPoint.x -= 700;
	  
	  targetPoint = eye->getOriginPoint();
	  targetPoint.z += flight_height + 100;
	  climbup = true;
	  //switchNavTarget(eye->getOriginPoint(), zeroVector);
	  switchNavTarget(targetPoint, zeroVector);
	}
      //if (landing && !dropdown && norm(vectorMinus(targetPoint,eye->getCurrentPoint())) <100)
      if (landing && climbup && !dropdown && norm(vectorMinus(targetPoint,eye->getCurrentPoint())) < 200)
	{
	  switchHover();
	  targetPoint = eye->getCurrentPoint();
	  targetPoint.z = eye->getOriginPoint().z+30;
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

  /*
  if (counter > total_time*freq && !landing)
    {
      VisualBird_finish = true;
      landing = true;
    }
  */
}

void VisualBird::switchHover()
{
  hover = true;
  eye->resetIntError();
  ROS_INFO("Swithced to Hover Mode");
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

  //check whether arrived at start point
  checkStartPoint();

  //check landing condition
  land();

  if (norm(vectorMinus(eye->getCurrentPoint(), shiftOrigin)) < 150 && !landing && !circleStart) 
    {
      ROS_INFO("Start virtual target");
      targetCircleCenter = eye->getCurrentPoint();
      targetCircleCenter.y += radius + 50;
      currTheta = 0;
      circleStart = true;
    }

  if (circleStart && !landing)
    {
      targetPoint.x = targetCircleCenter.x + radius * sin(currTheta);
      targetPoint.y = targetCircleCenter.y - radius * cos(currTheta);
      targetPosLog.push_back(targetPoint);
      targetVel.x = radius * omega * cos(currTheta)/1000.;
      targetVel.y = radius * omega * sin(currTheta)/1000.;
      targetVel.z = 0;
      targetVelLog.push_back(targetVel);
      targetAcc.x = - radius * omega * omega * sin(currTheta)/1000.;
      targetAcc.y = radius * omega * omega * cos(currTheta)/1000.;
      targetAcc.z = 0;
      targetAccLog.push_back(targetAcc);
      currTheta += omega/freq;
      //ROS_INFO("Virtual target at %.3f %.3f %.3f", targetPoint.x, targetPoint.y, targetPoint.z);
    }

  if (!hover)
    nav->updatePVA(scalarProduct(0.001,vectorMinus(eye->getCurrentPoint(),shiftOrigin)),scalarProduct(0.001,vectorMinus(targetPoint,shiftOrigin)),scalarProduct(0.001,eye->getCurrentVel()),targetVel,targetAcc);

  invokeController();
  visualTarget.push_back(targetPoint);   
  
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = "world";
  msg.point.x = targetPoint.x/1000;
  msg.point.y = targetPoint.y/1000;
  msg.point.z = targetPoint.z/1000;
  msg.header.stamp = ros::Time::now();
  target_pub.publish(msg);

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

  /*
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
	fprintf(pFile,"shift origin %.3f %.3f\n", shiftOrigin.x, shiftOrigin.y);
    }
  fclose(pFile);
  */
  sprintf(buff,"/home/cooplab/jianxin/ref%d.txt",fileNo);
  filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; (unsigned)i<vel_ref.size();i++)
	fprintf(pFile, "vel ref %.3f %.3f %.3f acc ref %.3f %.3f %.3f\n",vel_ref[i].x,vel_ref[i].y, vel_ref[i].z, acc_ref[i].x, acc_ref[i].y, acc_ref[i].z);
    }
  fclose(pFile);

  sprintf(buff,"/home/cooplab/jianxin/target%d.txt",fileNo);
  filename = buff;

  pFile = fopen(filename.c_str(),"w");
  if (pFile!=NULL)
    {
      for (int i = 0; (unsigned)i<targetPosLog.size();i++)
	fprintf(pFile, "pos %.3f %.3f %.3f vel %.3f %.3f %.3f acc %.3f %.3f %.3f\n",targetPosLog[i].x,targetPosLog[i].y, targetPosLog[i].z, targetVelLog[i].x, targetVelLog[i].y, targetVelLog[i].z,targetAccLog[i].x, targetAccLog[i].y, targetAccLog[i].z);
    }
  fclose(pFile);


  dummy->writeLog(fileNo);
}

