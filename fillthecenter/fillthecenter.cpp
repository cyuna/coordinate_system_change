#include <stdio.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#define DIR1 "/home/autoware/catkin_ws/build/fillthecenter/gridsample.pcd"


std::string file_name_read1 = DIR1;

pcl::PointCloud<pcl::PointXYZ> incloud;
pcl::PointCloud<pcl::PointXYZ> centercloud;


int main(int argc, char**argv)
{
  pcl::io::loadPCDFile<pcl::PointXYZ>(file_name_read1, incloud);

  int m = incloud.points.size();
  int a = 0;
  while(1)
  {
    for(int i = 0 ; i < incloud.points.size()-1;i++)
    { 
      float change = 0;
      pcl::PointXYZ point;

      point = incloud.points[i];
    
      centercloud.push_back(point);
      change += fabs(incloud.points[i].x - incloud.points[i+1].x);
      change += fabs(incloud.points[i].y - incloud.points[i+1].y);
      if (change > 0.1)
      { 
        point.x = (incloud.points[i].x + incloud.points[i+1].x) / 2;
        point.y = incloud.points[i].y ;
        point.z = (incloud.points[i].z + incloud.points[i+1].z) / 2;
        centercloud.push_back(point);
        a++;
      }
    }
    
    if (a==0)
    {
      break;
    }

    incloud = centercloud;
    centercloud.clear();
    std::cout<<a<<std::endl;
    a = 0;
    
  }

  std::stringstream ss;
  ss <<  "fill.pcd";
  const std::string save_dir = "/home/autoware/catkin_ws/build/fillthecenter/";
  pcl::io::savePCDFile(save_dir + ss.str(), centercloud, true);

  return 0;
}