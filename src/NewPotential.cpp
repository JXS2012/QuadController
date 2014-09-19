#include "NewPotential.h"

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
    distanceMin = 0.05;
  if (!nh_private_.getParam("Numerical_Difference_Interval", eps))
    eps = 5e-5;
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


void Potential::updatePVA(pcl::PointXYZ quad, pcl::PointXYZ target, pcl::PointXYZ quadVel, pcl::PointXYZ targetVel , pcl::PointXYZ targetAcc)
{
  _quad = quad;
  _target = target;
  currentPotential = computePotential(_quad, _target);

  velDes = computeVelDes(_quad,_target,targetVel);
  gradXr = computeGradientSelf(_quad,_target);
  gradXt = computeGradientTarget(_quad,_target);

  pcl::PointXYZ partialVpartialXr = velocityGradientSelf(_quad, _target, targetVel, quadVel);
  pcl::PointXYZ partialVpartialXt = velocityGradientTarget(_quad, _target, targetVel);
  pcl::PointXYZ partialVpartialXtdot = velocityGradientTargetDot(_quad, _target, targetVel,targetAcc);
  accDes = vectorPlus(vectorMinus(vectorPlus(vectorPlus(partialVpartialXtdot, partialVpartialXr), partialVpartialXt) , gradXr) , scalarProduct(pd,vectorMinus(velDes,quadVel)));
  /*
  if (abs(accDes.x)>2 || abs(accDes.y)>2)
    {
      ROS_DEBUG("current pos: %.3f %.3f",_quad.x, _quad.y);
      ROS_DEBUG("current vel: %.3f %.3f",quadVel.x, quadVel.y);
      ROS_INFO("target pos: %.3f %.3f", _target.x, _target.y);
      ROS_DEBUG("target acc: %.3f %.3f", targetAcc.x, targetAcc.y);
      ROS_INFO("current gamma %f beta %f potential %f", gamma(_quad,_target), beta(_quad), currentPotential);
      ROS_INFO("des vel: %.3f %.3f",velDes.x, velDes.y);
      ROS_INFO("des acc: %.3f %.3f",accDes.x, accDes.y);
      ROS_DEBUG("gradxr: %.3f", norm(gradXr));
      ROS_INFO("acc xtdot part: %.3f %.3f", partialVpartialXtdot.x, partialVpartialXtdot.y);
      ROS_INFO("acc xr part: %.3f %.3f", partialVpartialXr.x, partialVpartialXr.y);
      ROS_INFO("acc xt part: %.3f %.3f", partialVpartialXt.x, partialVpartialXt.y);
      ROS_INFO(" ");
    }
  */
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
  double h = eps;
  /*
  if (xr.x != 0)
    h = xr.x * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Xper[4] = {xr,xr,xr,xr};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  double pXper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = computePotential(Xper[i],xt);
    }
  pcl::PointXYZ Tgradient;
  Tgradient.x = (-pXper[0] + 8*pXper[1] - 8*pXper[2] + pXper[3])/(12*h);
  /*
  if (xr.y != 0)
    h = xr.y * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Yper[4] = {xr,xr,xr,xr};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  double pYper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = computePotential(Yper[i],xt);
    }
  Tgradient.y = (-pYper[0] + 8*pYper[1] - 8*pYper[2] + pYper[3])/(12*h);
  Tgradient.z = 0;

  return Tgradient;

}

pcl::PointXYZ Potential::computeGradientTarget(pcl::PointXYZ xr, pcl::PointXYZ xt)
{
  pcl::PointXYZ Tgradient;
  
  double h = eps;
  /*
  if (xt.x != 0)
    h = xt.x * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Xper[4] = {xt,xt,xt,xt};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  double pXper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = computePotential(xr,Xper[i]);
    }
  Tgradient.x = (-pXper[0] + 8*pXper[1] - 8*pXper[2] + pXper[3])/(12*h);
  /*
  if (xt.y != 0)
    h = xt.y * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Yper[4] = {xt,xt,xt,xt};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  double pYper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = computePotential(xr,Yper[i]);
    }  
  Tgradient.y = (-pYper[0] + 8*pYper[1] - 8*pYper[2] + pYper[3])/(12*h);
  Tgradient.z = 0;

  return Tgradient;
}

pcl::PointXYZ Potential::computeJGradientSelf(pcl::PointXYZ xr,pcl::PointXYZ xt)
{
  double h = eps;
  /*
  if (xr.x != 0)
    h = xr.x * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Xper[4] = {xr,xr,xr,xr};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  double pXper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = gamma(Xper[i],xt);
    }
  pcl::PointXYZ Tgradient;
  Tgradient.x = (-pXper[0] + 8*pXper[1] - 8*pXper[2] + pXper[3])/(12*h);
  /*
  if (xr.y != 0)
    h = xr.y * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Yper[4] = {xr,xr,xr,xr};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  double pYper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = gamma(Yper[i],xt);
    }
  Tgradient.y = (-pYper[0] + 8*pYper[1] - 8*pYper[2] + pYper[3])/(12*h);
  Tgradient.z = 0;

  return Tgradient;

}

pcl::PointXYZ Potential::computeBetaGradientSelf(pcl::PointXYZ xr)
{
  double h = eps;
  /*
  if (xr.x != 0)
    h = xr.x * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Xper[4] = {xr,xr,xr,xr};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  double pXper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = beta(Xper[i]);
    }
  pcl::PointXYZ Tgradient;
  Tgradient.x = (-pXper[0] + 8*pXper[1] - 8*pXper[2] + pXper[3])/(12*h);
  /*
  if (xr.y != 0)
    h = xr.y * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Yper[4] = {xr,xr,xr,xr};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  double pYper[4] = {0,0,0,0};
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = beta(Yper[i]);
    }
  Tgradient.y = (-pYper[0] + 8*pYper[1] - 8*pYper[2] + pYper[3])/(12*h);
  Tgradient.z = 0;

  return Tgradient;

}
 
pcl::PointXYZ Potential::computeVelDes(pcl::PointXYZ quad, pcl::PointXYZ target, pcl::PointXYZ xtdot)
{
  pcl::PointXYZ gradXrLocal = computeGradientSelf(quad, target);
  pcl::PointXYZ gradXtLocal = computeGradientTarget(quad, target);
  pcl::PointXYZ value;

  double scale = -1/(4*norm(gradXrLocal)+epsilon)-dotProduct(gradXtLocal,xtdot)/pow(norm(gradXrLocal),2);
  value = scalarProduct(scale, gradXrLocal);

  /*
  if (computePotential(quad,target) > 5e-5)
   { 
      double scale = -1/(4*norm(gradXrLocal)+epsilon)-dotProduct(gradXtLocal,xtdot)/pow(norm(gradXrLocal),2);
      value = scalarProduct(scale, gradXrLocal);
    }
  else
    {
      value = vectorPlus(scalarProduct(-1/(4*norm(gradXrLocal)+epsilon),gradXrLocal), xtdot);
    }
  value.z = 0;
  */

  /*  
  double scale = -1/(4*norm(gradXrLocal)+epsilon);

  pcl::PointXYZ u_0 = scalarProduct(scale, gradXrLocal);

  pcl::PointXYZ gradJlocal = computeJGradientSelf(quad,target);
  pcl::PointXYZ gradBetalocal = computeBetaGradientSelf(quad);
  pcl::PointXYZ nominator = vectorMinus(scalarProduct(paramK*beta(quad), gradJlocal ), scalarProduct(gamma(quad,target), gradBetalocal));

  scale = gamma(quad,target) * dotProduct(gradBetalocal,xtdot)/pow(norm(nominator),2);
  
  pcl::PointXYZ u_1 = scalarProduct(scale, nominator);

  pcl::PointXYZ value = vectorPlus(vectorPlus(u_0, u_1), xtdot);
  */
  return value;
}
 
pcl::PointXYZ Potential::velocityGradientSelf(pcl::PointXYZ xr, pcl::PointXYZ xt, pcl::PointXYZ xtdot,pcl::PointXYZ quadVel)
{
  double h = eps;
  /*
  if (xr.x != 0)
    h = xr.x * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Xper[4] = {xr,xr,xr,xr};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  pcl::PointXYZ pXper[4];
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = computeVelDes(Xper[i],xt,xtdot);
    }

  pcl::PointXYZ xNominator;
  xNominator = scalarProduct(-1,pXper[0]);
  xNominator = vectorPlus(xNominator, scalarProduct(8, pXper[1]));
  xNominator = vectorPlus(xNominator, scalarProduct(-8, pXper[2]));
  xNominator = vectorPlus(xNominator, pXper[3]);

  pcl::PointXYZ pVpX = scalarProduct(1/(12*h),xNominator);
  /*
  if (xr.y != 0)
    h = xr.y * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Yper[4] = {xr,xr,xr,xr};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  pcl::PointXYZ pYper[4];
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = computeVelDes(Yper[i],xt,xtdot);
    }

  pcl::PointXYZ yNominator;
  yNominator = scalarProduct(-1,pYper[0]);
  yNominator = vectorPlus(yNominator, scalarProduct(8, pYper[1]));
  yNominator = vectorPlus(yNominator, scalarProduct(-8, pYper[2]));
  yNominator = vectorPlus(yNominator, pYper[3]);

  pcl::PointXYZ pVpY = scalarProduct(1/(12*h),yNominator);

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

  if (abs(value.x) > 2 || abs(value.y) >2)
    {
      ROS_DEBUG("Jacobian xr: %.3f %.3f %.3f %.3f", Vx.x, Vx.y, Vy.x, Vy.y);
      ROS_DEBUG("x nominator: %f", norm(xNominator));
      ROS_DEBUG("x h: %f", xr.x*sqrt(eps));
    }
  return value;

}  
 
pcl::PointXYZ Potential::velocityGradientTarget(pcl::PointXYZ xr, pcl::PointXYZ xt, pcl::PointXYZ xtdot)
{
  double h=eps;
  /*
  if (xt.x != 0)
    h = xt.x * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Xper[4] = {xt,xt,xt,xt};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  pcl::PointXYZ pXper[4];  
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = computeVelDes(xr,Xper[i],xtdot);
    }

  pcl::PointXYZ xNominator;
  xNominator = scalarProduct(-1,pXper[0]);
  xNominator = vectorPlus(xNominator, scalarProduct(8, pXper[1]));
  xNominator = vectorPlus(xNominator, scalarProduct(-8, pXper[2]));
  xNominator = vectorPlus(xNominator, pXper[3]);

  pcl::PointXYZ pVpX = scalarProduct(1/(12*h),xNominator);
  /*
  if (xt.y != 0)
    h = xt.y * sqrt(eps);
  else
    h = eps;
  */
  pcl::PointXYZ Yper[4] = {xt,xt,xt,xt};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  pcl::PointXYZ pYper[4];
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = computeVelDes(xr,Yper[i],xtdot);
    }

  pcl::PointXYZ yNominator;
  yNominator = scalarProduct(-1,pYper[0]);
  yNominator = vectorPlus(yNominator, scalarProduct(8, pYper[1]));
  yNominator = vectorPlus(yNominator, scalarProduct(-8, pYper[2]));
  yNominator = vectorPlus(yNominator, pYper[3]);

  pcl::PointXYZ pVpY = scalarProduct(1/(12*h),yNominator);

  pcl::PointXYZ Vx;
  Vx.x = pVpX.x;
  Vx.y = pVpY.x;
  Vx.z = 0;

  pcl::PointXYZ Vy;
  Vy.x = pVpX.y;
  Vy.y = pVpY.y;
  Vy.z = 0;

  pcl::PointXYZ value;
  pcl::PointXYZ targetVel = xtdot;
  value.x = dotProduct(Vx, targetVel);
  value.y = dotProduct(Vy, targetVel);
  value.z = 0;
  if (abs(value.x) > 2 || abs(value.y) >2)
    {
      ROS_INFO("target vel: %.3f %.3f %.3f", targetVel.x, targetVel.y, targetVel.z);
      ROS_INFO("Jacobian xt: %.3f %.3f %.3f %.3f", pVpX.x, pVpX.y, pVpY.x, pVpY.y);
      ROS_INFO("Jacobian xt V: %.3f %.3f %.3f %.3f", Vx.x, Vy.x, Vx.y, Vy.y);
      ROS_DEBUG("value: %.3f %.3f", value.x, value.y);
      ROS_INFO("y nominator: %f %f", yNominator.x, yNominator.y);
      ROS_INFO("y perturbed +2h: %f %f %f potential %f", pYper[0].x, pYper[0].y, pYper[0].z, computePotential(xr,Yper[0]));
      ROS_INFO("y perturbed +h: %f %f %f potential %f", pYper[1].x, pYper[1].y, pYper[1].z,computePotential(xr,Yper[1]));
      ROS_INFO("y perturbed -h: %f %f %f potential %f", pYper[2].x, pYper[2].y, pYper[2].z,computePotential(xr,Yper[2]));
      ROS_INFO("y perturbed -2h: %f %f %f potential %f", pYper[3].x, pYper[3].y, pYper[3].z,computePotential(xr,Yper[3]));
    }

  return value;

}
 
pcl::PointXYZ Potential::velocityGradientTargetDot(pcl::PointXYZ xr, pcl::PointXYZ xt, pcl::PointXYZ xtdot, pcl::PointXYZ xtddot)
{
  double h;
  if (xtdot.x != 0)
    h = xtdot.x * sqrt(eps);
  else
    h = eps;

  pcl::PointXYZ Xper[4] = {xtdot,xtdot,xtdot,xtdot};
  Xper[0].x += 2*h;
  Xper[1].x += h;
  Xper[2].x -= h;
  Xper[3].x -= 2*h;
  pcl::PointXYZ pXper[4];
  for (int i = 0; i < 4; i++)
    {
      pXper[i] = computeVelDes(xr,xt,Xper[i]);
    }

  pcl::PointXYZ xNominator;
  xNominator = scalarProduct(-1,pXper[0]);
  xNominator = vectorPlus(xNominator, scalarProduct(8, pXper[1]));
  xNominator = vectorPlus(xNominator, scalarProduct(-8, pXper[2]));
  xNominator = vectorPlus(xNominator, pXper[3]);

  pcl::PointXYZ pVpX = scalarProduct(1/(12*h),xNominator);

  if (xtdot.y != 0)
    h = xtdot.y * sqrt(eps);
  else
    h = eps;

  pcl::PointXYZ Yper[4] = {xtdot,xtdot,xtdot,xtdot};
  Yper[0].y += 2*h;
  Yper[1].y += h;
  Yper[2].y -= h;
  Yper[3].y -= 2*h;
  pcl::PointXYZ pYper[4];
  for (int i = 0; i < 4; i++)
    {
      pYper[i] = computeVelDes(xr,xt,Yper[i]);
    }

  pcl::PointXYZ yNominator;
  yNominator = scalarProduct(-1,pYper[0]);
  yNominator = vectorPlus(yNominator, scalarProduct(8, pYper[1]));
  yNominator = vectorPlus(yNominator, scalarProduct(-8, pYper[2]));
  yNominator = vectorPlus(yNominator, pYper[3]);

  pcl::PointXYZ pVpY = scalarProduct(1/(12*h),yNominator);

  pcl::PointXYZ Vx;
  Vx.x = pVpX.x;
  Vx.y = pVpY.x;
  Vx.z = 0;

  pcl::PointXYZ Vy;
  Vy.x = pVpX.y;
  Vy.y = pVpY.y;
  Vy.z = 0;

  pcl::PointXYZ value;
  pcl::PointXYZ targetAcc = xtddot;
  value.x = dotProduct(Vx, targetAcc);
  value.y = dotProduct(Vy, targetAcc);
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
