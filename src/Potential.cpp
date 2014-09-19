#include "Potential.h"

Potential::Potential(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ros::NodeHandle node  (nh_);

  std::string strObstaclesX,strObstaclesY,strObstaclesR;

  if (!nh_private_.getParam("Potential_K", paramK))
    paramK = 2.5;
  if (!nh_private_.getParam("Work_Space_R", workspaceR))
    workspaceR = 1.5;
  if (!nh_private_.getParam("Closeset_Distance_To_Target", distanceMin))
    distanceMin = 0;
  if (!nh_private_.getParam("Numerical_Difference_Interval", eps))
    eps = 1e-4;
  if (!nh_private_.getParam("Scale_For_Navigation", epsilon))
    epsilon = 0.1;
  if (!nh_private_.getParam("PD", pd))
    pd = 5;

  if (!nh_private_.getParam("Obstacles_X", strObstaclesX))
    strObstaclesX = "0.6 0.6 -0.6 -0.6 0.1655";
  if (!nh_private_.getParam("Obstacles_Y", strObstaclesY))
    strObstaclesY = "0.6 -0.6 0.6 -0.6 -0.8322";
  if (!nh_private_.getParam("Obstacles_R", strObstaclesR))
    strObstaclesR = "0.15 0.15 0.15 0.15 0.225";

  std::vector<double> obstacleX = readTable(strObstaclesX);
  std::vector<double> obstacleY = readTable(strObstaclesY);
  std::vector<double> obstacleR = readTable(strObstaclesR);

  for (int i = 0; (unsigned)i<obstacleX.size(); i++)
    {
      pcl::PointXYZ temp;
      temp.x = obstacleX[i];
      temp.y = obstacleY[i];
      temp.z = obstacleR[i];
      obstacles.push_back(temp);
      ROS_INFO("obstacle %d is %.3f %.3f %.3f",i, obstacles[i].x, obstacles[i].y, obstacles[i].z);
    }
}

std::vector<double> Potential::readTable(std::string strTable)
{
  std::vector<double> result;
  std::istringstream iss(strTable);

  while (iss)
    {
      std::string sub;
      iss >>sub;
      result.push_back(atof(sub.c_str()));
    }

  result.pop_back();

  return result;
}


Potential::~Potential()
{
  ROS_INFO("Destroying Potential!");
}

//time is no longer needed since target vel and pos are fed back through targetObersverServer
void Potential::updatePVA(pcl::PointXYZ quad, pcl::PointXYZ target, pcl::PointXYZ quadVel, pcl::PointXYZ targetVel ,double time)
{
  _quad = quad;
  _target = target;
  currentPotential = computePotential(_quad, _target);

  gradXr = computeGradientSelf(_quad, _target);
  gradXt = computeGradientTarget(_quad, _target);
  velDes = computeVelDes(gradXr,gradXt,_target,targetVel,time);


  pcl::PointXYZ partialVpartialXr = velocityGradientSelf(gradXr, gradXt, _target, time, quadVel,targetVel);
  pcl::PointXYZ partialVpartialXt = velocityGradientTarget(gradXr, gradXt, _target, time,targetVel);
  pcl::PointXYZ partialVpartialt = velocityGradientTime(gradXr, gradXt, _target, time,targetVel);
  accDes = vectorPlus(vectorMinus(vectorPlus(vectorPlus(partialVpartialt, partialVpartialXr), partialVpartialXt) , gradXr) , scalarProduct(pd,vectorMinus(velDes,quadVel)));

  if (abs(accDes.x)>2 || abs(accDes.y)>2)
    {
      ROS_DEBUG("current pos: %.3f %.3f",_quad.x, _quad.y);
      ROS_DEBUG("des vel: %.3f %.3f",velDes.x, velDes.y);
      ROS_DEBUG("des acc: %.3f %.3f",accDes.x, accDes.y);
      ROS_DEBUG("target vel: %.3f %.3f %.3f", targetVel.x, targetVel.y, targetVel.z);
      ROS_DEBUG("gradxr: %.3f", norm(gradXr));
      ROS_DEBUG("acc time part: %.3f %.3f", partialVpartialt.x, partialVpartialt.y);
      ROS_DEBUG("acc xr part: %.3f %.3f", partialVpartialXr.x, partialVpartialXr.y);
      ROS_DEBUG("acc xt part: %.3f %.3f", partialVpartialXt.x, partialVpartialXt.y);
      ROS_DEBUG("acc gradxr part: %.3f %.3f", gradXr.x, gradXr.y);
      ROS_DEBUG("acc vel diff part: %.3f %.3f", scalarProduct(pd,vectorMinus(velDes,quadVel)).x,scalarProduct(pd,vectorMinus(velDes,quadVel)).y);
      pcl::PointXYZ vel_target = scalarProduct(dotProduct(gradXt,targetVel)/pow(norm(gradXr),2),gradXr);
      ROS_DEBUG("vel target part %.3f %.3f",vel_target.x,vel_target.y);
    }
}

double Potential::gamma(pcl::PointXYZ xr, pcl::PointXYZ xt)
{
  return pow( (pow((xr.x-xt.x),2) + pow((xr.y-xt.y),2) - pow(distanceMin,2)) , 2);
}

double Potential::betaFunction(pcl::PointXYZ obstacle, pcl::PointXYZ xr)
{
  return pow((xr.x-obstacle.x),2) + pow((xr.y-obstacle.y),2) - pow(obstacle.z,2);
}

double Potential::betaWorkSpace(pcl::PointXYZ xr)
{
  return pow(workspaceR,2) - pow(xr.x,2) - pow(xr.y,2);
}

double Potential::beta(pcl::PointXYZ xr)
{
  double tempbeta = 1;
  for (int i = 0; (unsigned)i < obstacles.size(); i++)
    {
      tempbeta = tempbeta*betaFunction(obstacles[i],xr);
    }
  tempbeta = tempbeta*betaWorkSpace(xr);
  return tempbeta;
}

double Potential::computePotential(pcl::PointXYZ xr, pcl::PointXYZ xt)
{
  double tGamma = gamma(xr,xt);
  double tBeta = beta(xr);

  return tGamma / pow( (pow(tGamma,paramK) + tBeta) , 1./paramK);
}

pcl::PointXYZ Potential::computeGradientSelf(pcl::PointXYZ xr, pcl::PointXYZ xt)
{
  pcl::PointXYZ Xper = xr;
  Xper.x += eps;
  double pXper = computePotential(Xper,xt);

  pcl::PointXYZ Yper = xr;
  Yper.y += eps;
  double pYper = computePotential(Yper,xt);

  ROS_DEBUG("xr %.5f %.5f %.5f xper %.5f %.5f %.5f yper %.5f %.5f %.5f",xr.x,xr.y,xr.z,Xper.x,Xper.y,Xper.z,Yper.x,Yper.y,Yper.z);
  double originPotential = computePotential(xr,xt);

  pcl::PointXYZ Tgradient;
  Tgradient.x = (pXper-originPotential)/eps;
  Tgradient.y = (pYper-originPotential)/eps;
  Tgradient.z = 0;

  return Tgradient;
}

pcl::PointXYZ Potential::computeGradientTarget(pcl::PointXYZ xr, pcl::PointXYZ xt)
{
  pcl::PointXYZ Xper = xt;
  Xper.x += eps;
  double pXper = computePotential(xr,Xper);
  pcl::PointXYZ Yper = xt;
  Yper.y += eps;
  double pYper = computePotential(xr,Yper);

  pcl::PointXYZ Tgradient;
  Tgradient.x = (pXper-currentPotential)/eps;
  Tgradient.y = (pYper-currentPotential)/eps;
  Tgradient.z = 0;

  return Tgradient;
}
 
pcl::PointXYZ Potential::computeVelDes(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, pcl::PointXYZ xtdot, double time)
{
  //  double scale = -1/(4*norm(gradXr)+epsilon)-dotProduct(gradXt,velocityTarget(xt,time))/pow(norm(gradXr),2);
  
  // normal controller
  double scale = -1/(4*norm(gradXr)+epsilon)-dotProduct(gradXt,xtdot)/pow(norm(gradXr),2);
  pcl::PointXYZ value = scalarProduct(scale, gradXr);

  /*
  //change to limit controller
  double scale = -1/(4*norm(gradXr)+epsilon);
  pcl::PointXYZ value = vectorPlus(scalarProduct(scale, gradXr),xtdot);
  if (xtdot.x != 0 && xtdot.y !=0 && (value.x > 1 || value.y >1))
    {
      ROS_INFO("DESIRED VEL %f %f Target Vel %f %f",value.x, value.y, xtdot.x, xtdot.y);
      ROS_INFO("Grad Xr %f %f %f", gradXr.x, gradXr.y, gradXr.z);
      ROS_INFO("Grad Xt %f %f %f", gradXt.x, gradXt.y, gradXt.z);
      ROS_INFO("XtDot scale %f", dotProduct(gradXt, xtdot));
      ROS_INFO("Norm grad xr %.9f", norm(gradXr));
      pause();
    }
  */
  //ROS_INFO("DESIRED VEL %f %f Target Vel %f %f",value.x, value.y, xtdot.x, xtdot.y);
  return value;
}
 
pcl::PointXYZ Potential::velocityGradientSelf(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, double time, pcl::PointXYZ quadVel, pcl::PointXYZ xtdot)
{
  pcl::PointXYZ Xper = gradXr;
  Xper.x += eps;
  pcl::PointXYZ Yper = gradXr;
  Yper.y += eps;
  
  pcl::PointXYZ pVpX = scalarProduct(1/eps,vectorMinus(computeVelDes(Xper,gradXt,xt,xtdot,time),velDes));
  pcl::PointXYZ pVpY = scalarProduct(1/eps,vectorMinus(computeVelDes(Yper,gradXt,xt,xtdot,time),velDes));

  pcl::PointXYZ Vx;
  Vx.x = pVpX.x;
  Vx.y = pVpY.x;
  Vx.z = 0;

  pcl::PointXYZ Vy;
  Vy.x = pVpX.y;
  Vy.y = pVpY.y;
  Vy.z = 0;

  pcl::PointXYZ value;
  value.x = dotProduct(Vx, quadVel);
  value.y = dotProduct(Vy, quadVel);
  value.z = 0;

  return value;
}  
 
pcl::PointXYZ Potential::velocityGradientTarget(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, double time, pcl::PointXYZ xtdot)
{
  pcl::PointXYZ Xper = gradXt;
  Xper.x += eps;
  pcl::PointXYZ Yper = gradXt;
  Yper.y += eps;
  
  pcl::PointXYZ pVpX = scalarProduct(1/eps,vectorMinus(computeVelDes(gradXr,Xper,xt,xtdot,time),velDes));
  pcl::PointXYZ pVpY = scalarProduct(1/eps,vectorMinus(computeVelDes(gradXr,Yper,xt,xtdot,time),velDes));

  pcl::PointXYZ Vx;
  Vx.x = pVpX.x;
  Vx.y = pVpY.x;
  Vx.z = 0;

  pcl::PointXYZ Vy;
  Vy.x = pVpX.y;
  Vy.y = pVpY.y;
  Vy.z = 0;

  pcl::PointXYZ value;
  //  pcl::PointXYZ targetVel = velocityTarget(xt,time);
  pcl::PointXYZ targetVel = xtdot;
  value.x = dotProduct(Vx, targetVel);
  value.y = dotProduct(Vy, targetVel);
  value.z = 0;

  return value;
}
 
pcl::PointXYZ Potential::velocityGradientTime(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, double time, pcl::PointXYZ xtdot)
{
  return scalarProduct(1/eps,vectorMinus(computeVelDes(gradXr,gradXt,xt,xtdot,time+eps),velDes));
}

//No longer used: velocityTarget, Now feedback through BirdEye 
pcl::PointXYZ Potential::velocityTarget(pcl::PointXYZ xt, double time)
{
  pcl::PointXYZ value;
  value.x = 0;
  value.y = 0;
  value.z = 0;

  return value;
}

double Potential::getPotential()
{
  return currentPotential;
}

  
pcl::PointXYZ Potential::getVelDes()
{
  return velDes;
}

pcl::PointXYZ Potential::getAccDes()
{
  return accDes;
}

pcl::PointXYZ Potential::getGradXr()
{
  return gradXr;
}

pcl::PointXYZ Potential::getGradXt()
{
  return gradXt;
}
