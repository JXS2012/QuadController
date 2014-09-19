#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include "VectorComputation.h"

#include "math.h"
#include <vector>

class Potential
{
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  double paramK,workspaceR,distanceMin,eps,epsilon,pd;

  pcl::PointXYZ _quad,_target,velDes,accDes,gradXr,gradXt;
  std::vector<pcl::PointXYZ> obstacles;

  double currentPotential;

  double betaWorkSpace(pcl::PointXYZ xr);
  double betaFunction(pcl::PointXYZ obstacle, pcl::PointXYZ xr);
  double beta(pcl::PointXYZ xr);
  double gamma(pcl::PointXYZ xr, pcl::PointXYZ xt);
  double computePotential(pcl::PointXYZ xr, pcl::PointXYZ xt);

  pcl::PointXYZ computeGradientSelf(pcl::PointXYZ xr, pcl::PointXYZ xt);
  pcl::PointXYZ computeGradientTarget(pcl::PointXYZ xr, pcl::PointXYZ xt);
  pcl::PointXYZ computeVelDes(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, pcl::PointXYZ xtdot, double time);

  pcl::PointXYZ velocityGradientSelf(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, double time, pcl::PointXYZ quadVel, pcl::PointXYZ xtdot);
  pcl::PointXYZ velocityGradientTarget(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, double time, pcl::PointXYZ xtdot);
  pcl::PointXYZ velocityGradientTime(pcl::PointXYZ gradXr, pcl::PointXYZ gradXt, pcl::PointXYZ xt, double time, pcl::PointXYZ xtdot);

  pcl::PointXYZ velocityTarget(pcl::PointXYZ xt, double time);

  std::vector<double> readTable(std::string strTable);

 public:
  Potential(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~Potential();
  
  void updatePVA(pcl::PointXYZ quad, pcl::PointXYZ target, pcl::PointXYZ quadVel, pcl::PointXYZ targetVel, double time);

  double getPotential();
  
  pcl::PointXYZ getVelDes();

  pcl::PointXYZ getAccDes();

  pcl::PointXYZ getGradXr();

  pcl::PointXYZ getGradXt();
};
