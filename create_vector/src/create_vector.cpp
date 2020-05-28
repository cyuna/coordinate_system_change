#include <vector>
#include <iostream>
#include <utility>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

#include <std_msgs/Bool.h>
int ros_rotation = 0;
#define FIRING_COUNT 64
struct EIGEN_ALIGN16 PointXYZIR {
  PCL_ADD_POINT4D;                // quad-word XYZ
  PCL_ADD_INTENSITY;              ///< laser intensity reading // fixed by qjinchoi, 191001
  uint16_t    ring;               ///< laser ring number
  uint32_t    ts_sec;             // added by qjinchoi, 191004
  uint32_t    ts_usec;            // added by qjinchoi, 191004
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    };

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
				  (float, x, x)
				  (float, y, y)
				  (float, z, z)
				  (float, intensity, intensity)
				  (uint16_t, ring, ring)
          (uint32_t, ts_sec, ts_sec)
          (uint32_t, ts_usec, ts_usec)
				  )

int count = 0;
int rotation = 0;
float time_us = 0.0;
bool is_first = true;
int i =0;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr ndtCloud(new pcl::PointCloud<pcl::PointXYZINormal>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //transform raw point cloud

Eigen::MatrixXf getTransMat(pcl::PointXYZINormal ndt_pose)
{   
  // 변환행렬 반환(맵좌표계->라이다좌표계 변환행렬
  // getLIDAROneScan으로부터 얻은 lidar onescan point cloud의 모든 point들의 좌표계 변환을 위해 변환행렬 추출
  Eigen::AngleAxisf init_rotation_x(-ndt_pose.intensity, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(-ndt_pose.normal_x, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(-ndt_pose.curvature, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(-ndt_pose.x, -ndt_pose.y, -ndt_pose.z);
  Eigen::MatrixXf transform = ( init_translation.inverse() *  init_rotation_z.inverse() *  init_rotation_y.inverse() * init_rotation_x.inverse() ).matrix();
  return transform;
}

pcl::PointCloud<pcl::PointXYZINormal> transformPoint(pcl::PointCloud<pcl::PointXYZINormal> rawCld , Eigen::MatrixXf trans_mat)
{
  // 좌표변환된 point 반환 
  //getLIDAROneScan으로부터 얻은 lidar onescan point cloud의 모든 point들을 getTransMat에서 얻은 변환행렬로 transform적용
  pcl::PointCloud<pcl::PointXYZINormal> resultCld;
  for(int j = 0; j <  rawCld.size();j++)
      {   if(rawCld.points[j].curvature != 0)
            {
                pcl::PointXYZINormal b = rawCld.points[j];
                resultCld.push_back(b); 
                break;
            }
        Eigen::MatrixXf matrix_input(4,1);
        Eigen::MatrixXf matrix_output(4,1);
        matrix_input(0,0) = rawCld.points[j].x;
        matrix_input(1,0) = rawCld.points[j].y;
        matrix_input(2,0) = rawCld.points[j].z;
        matrix_input(3,0) = 1;

        Eigen::MatrixXf trans_mat_2(4,4);
              trans_mat_2 << 1, 0, 0, -1.2,
                0, 1, 0, 0,
                0, 0, 1, -1.865,
                0, 0, 0, 1;

        matrix_output = trans_mat * trans_mat_2.inverse() * matrix_input;
        
        pcl::PointXYZINormal b = rawCld.points[j];;
        b.x = matrix_output(0,0);
        b.y = matrix_output(1,0);
        b.z = matrix_output(2,0);
        resultCld.push_back(b);     
    }

  return resultCld;
}
void fromROSBag(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    char file_name_write[100];
    pcl::PointXYZINormal p;

    pcl::PointCloud<PointXYZIR> inCloud;
    pcl::fromROSMsg(*msg, inCloud);
    
    auto start = std::chrono::system_clock::now();
    for (pcl::PointCloud<PointXYZIR>::const_iterator item = inCloud.begin(); item != inCloud.end(); item++)
    {
        if (item->ts_usec != time_us)
        {
            time_us = item->ts_usec;
            count++;
            if(count == FIRING_COUNT)
            {   float a = item->ts_sec % 1000000;
                //  std::cout << a << std::endl;
                for(int j = 0; j < ndtCloud->points.size(); j++)
                {   
                    //std::cout << ndtCloud->points[j].normal_y << std::endl;
                    if(a == ndtCloud->points[j].normal_y)
                    {   outCloud->push_back(ndtCloud->points[j]);
                        
                        Eigen::MatrixXf transform = getTransMat(ndtCloud->points[j]);
                         *outCloud = transformPoint(*outCloud , transform);
                        // sprintf(file_name_write, "/home/autoware/yuna/rosbag/points_raw_%d.pcd", rotation++);
                        // pcl::io::savePCDFile(file_name_write, *outCloud);
                        // ROS_INFO("%s saved\n", file_name_write); 
                         *resultCloud += *outCloud;
                        i++;
                        break;
                    }
                }
            
            
                outCloud->clear();
                count =0;
            }
        }
        p.x = (float)item->x;
        p.y = (float)item->y;
        p.z = (float)item->z;
        p.intensity = (float)item->intensity;
        p.normal_x = item->ring;
        float b = item->ts_sec % 1000000;
        p.normal_y = b;
        p.normal_z = (item->ts_usec);

        outCloud->push_back(p);
    }
    double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-6;
    ROS_INFO("calculating time = %f [ms]\n", elapsed_ms);       
}

void callbackFlag(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data == true)
  {  std::cout <<i <<std::endl;
     pcl::io::savePCDFile("/home/autoware/yuna/rosbag_2.pcd", *resultCloud, false);
     ROS_INFO(" saved\n"); 
  }
}



int main(int argc, char**argv)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>("/home/autoware/yuna/yuna_point/cor_yuna.pcd", *ndtCloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read\n");
    }
    std::cout << "PCD LOADED!\n";

   ros::init(argc, argv, "create_vector");
   ros::NodeHandle nh;

   ros::Subscriber sub = nh.subscribe("/points_raw_extended",1,fromROSBag);
   ros::Subscriber sub2 = nh.subscribe("/flag", 1, &callbackFlag);

   ros::spin(); // Use to /points_raw_extended -> 1scan.pcd

}
