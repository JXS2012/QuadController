#include <ros/ros.h>
#include "Potential.h"

int main(int argc, char** argv)
{
  Potential *myPotential;

  ros::init(argc,argv,"potentialTest");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  double scale = 1.5;

  myPotential = new Potential(nh,nh_private);

  const int size(1000);
  double step = 2.*scale/size;

  double field[size][size];
  std::vector<std::vector<pcl::PointXYZ> > vel;
  std::vector<std::vector<pcl::PointXYZ> > acc;
  std::vector<std::vector<pcl::PointXYZ> > gradxr;
  vel.resize(size);
  for (int i = 0; i < size; ++i)
    vel[i].resize(size);
  acc.resize(size);
  for (int i = 0; i < size; ++i)
    acc[i].resize(size);
  gradxr.resize(size);
  for (int i = 0; i < size; ++i)
    gradxr[i].resize(size);

  ROS_INFO("start");
  
  for (int i = 0; i<size; i++)
    for (int j = 0; j<size; j++)
      {
	pcl::PointXYZ quad, target, quadVel;
	quad.x = -scale + j * step;
	quad.y = -scale + i * step;
	quad.z = 0;

	target.x = 0;
	target.y = 0;
	target.z = 0;

	quadVel.x = 0;
	quadVel.y = 0;
	quadVel.z = 0;

	myPotential->updatePVA(quad, target, quadVel, 0);
	field[i][j] = myPotential->getPotential();
	vel[i][j] = myPotential->getVelDes();
	acc[i][j] = myPotential->getAccDes();
	gradxr[i][j] = myPotential->getGradXt();
      }

  std::FILE * pFile;
  pFile = fopen("/home/jxsun/field.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", field[i][j]);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);
  
  pFile = fopen("/home/jxsun/velx.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", vel[i][j].x);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/vely.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", vel[i][j].y);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/velz.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", vel[i][j].z);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);


  pFile = fopen("/home/jxsun/accx.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", acc[i][j].x);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/accy.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", acc[i][j].y);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/accz.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", acc[i][j].z);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/gradxtx.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", gradxr[i][j].x);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/gradxty.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", gradxr[i][j].y);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  pFile = fopen("/home/jxsun/gradxrz.txt","w");

  if (pFile!=NULL)
    {
      for (int i = 0; i < size; i++)
	{
	  for (int j = 0; j < size; j++)
	    fprintf(pFile, "%.6f ", gradxr[i][j].z);
	  fprintf(pFile, "\n");
	}
    }

  fclose(pFile);

  ROS_INFO("end");
  return 0;
}
