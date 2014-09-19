#include "VectorComputation.h"

double norm(pcl::PointXYZ input)
{
  return sqrt(pow(input.x,2)+pow(input.y,2)+pow(input.z,2));
}

pcl::PointXYZ normalize(pcl::PointXYZ input)
{
  pcl::PointXYZ output;
  output.x = input.x/norm(input);
  output.y = input.y/norm(input);
  output.z = input.z/norm(input);
  return output;
}

pcl::PointXYZ crossProduct(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  pcl::PointXYZ output;
  output.x = v1.y*v2.z-v1.z*v2.y;
  output.y = v1.z*v2.x-v1.x*v2.z;
  output.z = v1.x*v2.y-v1.y*v2.x;
  return output;
}

double dotProduct(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
}

pcl::PointXYZ scalarProduct(double k, pcl::PointXYZ v)
{
  pcl::PointXYZ output;
  output.x = k*v.x;
  output.y = k*v.y;
  output.z = k*v.z;
  return output;
}

pcl::PointXYZ vectorMinus(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  pcl::PointXYZ output;
  output.x = v1.x - v2.x;
  output.y = v1.y - v2.y;
  output.z = v1.z - v2.z;
  return output;
}

pcl::PointXYZ vectorPlus(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
  pcl::PointXYZ output;
  output.x = v1.x + v2.x;
  output.y = v1.y + v2.y;
  output.z = v1.z + v2.z;
  return output;
}
