#include "highbird.h"

highbird::highbird(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

  ros::NodeHandle node  (nh_,"highbird");
  dummy = new dummybird(nh_,nh_private_);
  eye = new birdeye(nh_,nh_private_);
  
  //initiate variables
  initiation();

}

highbird::~highbird()
{
  ROS_INFO("Destroying highbird "); 

}

void highbird::initiation()
{
  init_parameters();

  //defining first flight target point
  targetPoint = eye->getStartPoint();
  //defining pre points for later velocity calculation

  if (fly)
    dummy->on();
}

void highbird::init_parameters()
{
  flight_height = 300; //unit mm
  max_outter_radius = 1200; //unit mm

  shift_no = 0; //number of shifts performed during flight if in hovering mode
  shift_time = 300; //total time for each shift = shift_time/freq
  shift_distance = 300;
  current_shift = 0; //counting current shift steps,

  counter = 0; //used in main loop for tracking time
  
  fly = true; //turn motor on or not. 
  landing = false; //flag showing whether quadrotor is landing
  outRange = false;
  reachStartPoint = false;
  highbird_finish = false;
  hover = true;
}

void highbird::invokeController()
{
  //in case vicon lost track of model
  if (eye->getCurrentVel().x == 0 && eye->getCurrentVel().y == 0 && eye->getCurrentVel().z == 0)
    {
      dummy->direct_drive(0,0,0,eye->getPsi());
    }
  else
    {
      if (hover)
	{
	  eye->updateIntError(targetPoint);
	  dummy->pid_hover_controller(vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
	}
      else
	{
	  eye->calculate_position_velocity_error();
	  dummy->pid_path_controller(eye->getE_p(),eye->getE_v(),eye->getAcc_t(),eye->getPsi());
	}
      /*
      dummy->pid_hover_controller(vectorMinus(eye->getCurrentPoint(),targetPoint),eye->getCurrentVel(),eye->getIntError(),eye->getPsi());
      */

    }
}

void highbird::shiftTargetPoint()
{
  //set new target point to achieve shifting, only available at hovering
  if (counter > shift_time*current_shift && current_shift<shift_no && hover)
    {
      printf ("%.3f shift\n",current_shift);
      current_shift++;
      targetPoint.x += shift_distance;
      targetPoint.y += shift_distance;
    }
}

void highbird::checkStartPoint()
{
  if (norm(vectorMinus(eye->getCurrentPoint(),eye->getStartPoint()))<100 && !reachStartPoint)
    {
      reachStartPoint = true;
      hover = false;
      printf("Reached start point!\n");
    }
}

void highbird::safeOutRange()
{
  pcl::PointXYZ temp = eye->getShiftedOrigin();
  temp.z = eye->getCurrentPoint().z;
  if (norm(vectorMinus(eye->getCurrentPoint(),temp))>max_outter_radius && !outRange)
    {
      printf("origin %.3f %.3f %.3f\n",temp.x,temp.y,temp.z);
      printf("out of range at point %.3f %.3f %.3f!\n",eye->getCurrentPoint().x,eye->getCurrentPoint().y,eye->getCurrentPoint().z);
      outRange = true;
      switchHover();
      targetPoint = eye->getOriginPoint();
      targetPoint.z += flight_height;
      printf("reset target point %.3f %.3f %.3f!\n",targetPoint.x,targetPoint.y,targetPoint.z);
      if (counter<700)
	counter = 700;
    }
}

void highbird::land()
{
  //if counter over 800, start landing
  if (counter > 800 && !landing)
    {
      switchHover();
      targetPoint = eye->getCurrentPoint();
      targetPoint.z = eye->getOriginPoint().z;
      landing = true;
      printf("start landing off\n");
    }
  //if in landing process and height less than 10cm, turn off motor and land
  if (eye->getCurrentPoint().z-eye->getOriginPoint().z < 100 && landing)
    {
      dummy->off();
      highbird_finish = true;
    }    
}

void highbird::switchHover()
{
  hover = true;
  eye->resetIntError();
}

void highbird::drive()
{
  //update flight status
  eye->updateFlightStatus();
  
  //check whether arrived at start point
  checkStartPoint();

  //check landing condition
  land();

  //check out range condition
  safeOutRange();

  //call hover controller or path controller according to (bool hover)
  invokeController();
    
  //if needed, shift target point
  //shiftTargetPoint();
  counter ++;
}

bool highbird::finish()
{
  return highbird_finish;
}

void highbird::write_log()
{
  std::FILE * pFile;

  pFile = fopen("log.txt","w");
  if (pFile!=NULL)
    {
      for (int i = 0; i<eye->getLogSize();i++)
	fprintf(pFile, "x %.3f y %.3f z %.3f phi %.3f theta %.3f psi %.3f roll %.6f pitch %.6f thrust %.6f\n", eye->getLog_x(i), eye->getLog_y(i), eye->getLog_z(i), eye->getLog_phi(i),eye->getLog_theta(i),eye->getLog_psi(i), dummy->roll_log[i], dummy->pitch_log[i], dummy->thrust_log[i]);
    }
  
}
