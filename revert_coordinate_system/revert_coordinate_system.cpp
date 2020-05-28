#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>
#include <Eigen/Dense>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //point map pcd
pcl::PointCloud<pcl::PointXYZINormal>::Ptr ndtCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //point map pcd
pcl::PointCloud<pcl::PointXYZINormal>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //original raw point cloud 
pcl::PointCloud<pcl::PointXYZINormal>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //transform raw point cloud

pcl::PointXYZINormal  getNDTPose(pcl::PointCloud<pcl::PointXYZINormal> rawCld)
{  
  // lidar onescan point cloud로부터 ndt_pose값 반환 (getLIDAROneScan에서 얻은 one scan에서 ndt_pose값만 추출) 
  pcl::PointXYZINormal ndt_;
  for(int i = rawCld.points.size()-1; i >= 0 ;i--){
          if (rawCld.points[i].curvature != 0 )
            {    
                ndt_ = rawCld.points[i];
                return ndt_;
            }
          }

  ndt_.curvature = 0;
  return ndt_;
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
 

int main(int argc, char** argv)
  {
    for(int i= 0; i <10; i++)
    { 
        std::stringstream ss1; 
        ss1 << "_" << i << ".pcd";    
        if(pcl::io::loadPCDFile<pcl::PointXYZINormal>("/home/autoware/yuna/rosbag/points_raw" + ss1.str(), *inCloud)==-1) 
            {
                PCL_ERROR("Couldn't read\n");
                continue;
            }
        if(pcl::io::loadPCDFile<pcl::PointXYZINormal>("/home/autoware/yuna/yuna_point/after_cor/" + ss1.str(), *ndtCloud)==-1) 
            {
                PCL_ERROR("Couldn't read\n");
                continue;
            }
        pcl::PointXYZINormal ndt = getNDTPose(*ndtCloud);
        Eigen::MatrixXf transform = getTransMat(ndt);
         *resultCloud += transformPoint(*inCloud , transform);
      //  *resultCloud += *inCloud;
         std::cout << i << std::endl;
    pcl::io::savePCDFile("/home/autoware/yuna/yuna_point/rosbag_map_cor.pcd", *resultCloud, false);
    }

    return 0;
}