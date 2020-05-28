#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>

pcl::PointCloud<pcl::PointXYZINormal>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //point map pcd
pcl::PointCloud<pcl::PointXYZINormal>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //original raw point cloud 
pcl::PointCloud<pcl::PointXYZINormal>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZINormal>); //transform raw point cloud
pcl::PointCloud<pcl::PointXYZINormal>::Ptr outCloud(new pcl::PointCloud<pcl::PointXYZINormal>); 
pcl::PointCloud<pcl::PointXYZINormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZINormal>); 
pcl::PointCloud<pcl::PointXYZINormal>::Ptr ransacCloud(new pcl::PointCloud<pcl::PointXYZINormal>);

pcl::PointCloud<pcl::PointXYZINormal> getLIDAROneScan()  
{ // ndt_pose이 포함되어있는 lidar onescan point cloud 반환 (pointmap.pcd를 for문으로 반복하여 여러개의 onescan들을 얻을예정)
  pcl::PointCloud<pcl::PointXYZINormal> midCloud;
  double time_sec = 0.0;

  pcl::PassThrough<pcl::PointXYZINormal> pass;
  pass.setInputCloud(inCloud);
  time_sec = inCloud->points[0].normal_z;
  pass.setInputCloud(inCloud);
  pass.setFilterFieldName("normal_z");
  pass.setFilterLimits(time_sec, time_sec);
  pass.setFilterLimitsNegative(false);
  pass.filter(*rawCloud); //raw point cloud      
  pass.setFilterLimitsNegative(true); 
  pass.filter(midCloud);//rest of point raw

  std::cout << "time_sec: " << time_sec << std::endl;
  return midCloud ;
}

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

Eigen::MatrixXf getTransMat(pcl::PointXYZINormal ndt_pose)
{   
  // 변환행렬 반환(맵좌표계->라이다좌표계 변환행렬
  // getLIDAROneScan으로부터 얻은 lidar onescan point cloud의 모든 point들의 좌표계 변환을 위해 변환행렬 추출
  Eigen::AngleAxisf init_rotation_x(-ndt_pose.intensity, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(-ndt_pose.normal_x, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(-ndt_pose.curvature, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(-ndt_pose.x, -ndt_pose.y, -ndt_pose.z);
  Eigen::MatrixXf transform = ( init_rotation_x  * init_rotation_y  * init_rotation_z *init_translation ).matrix();
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

        matrix_output = trans_mat * matrix_input;

        Eigen::MatrixXf trans_mat_2(4,4);
              trans_mat_2 <<  1, 0, 0, -1.2,
                              0, 1, 0, 0,
                              0, 0, 1, -1.865,
                              0, 0, 0, 1;

        matrix_output = trans_mat_2 * matrix_output; 
        
        pcl::PointXYZINormal b = rawCld.points[j];;
        b.x = matrix_output(0,0);
        b.y = matrix_output(1,0);
        b.z = matrix_output(2,0);
        resultCld.push_back(b);     
    }

  return resultCld;
}

pcl::PointCloud<pcl::PointXYZINormal> transformPoint2(pcl::PointCloud<pcl::PointXYZINormal> rawCld , Eigen::MatrixXf trans_mat) //vehicle -> map
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

Eigen::MatrixXf getTransMat2(pcl::PointXYZINormal ndt_pose) //vehicle -> map
{   

  Eigen::AngleAxisf init_rotation_x(-ndt_pose.intensity, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(-ndt_pose.normal_x, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(-ndt_pose.curvature, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(-ndt_pose.x, -ndt_pose.y, -ndt_pose.z);
  Eigen::MatrixXf transform = ( init_translation.inverse() *  init_rotation_z.inverse() *  init_rotation_y.inverse() * init_rotation_x.inverse() ).matrix();
  return transform;
}

void applyRANSAC(pcl::PointCloud<pcl::PointXYZINormal>::Ptr _inCld, pcl::PointCloud<pcl::PointXYZINormal>::Ptr _rscCld)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cld(new pcl::PointCloud<pcl::PointXYZINormal>);
    std::vector<int> inliers;

    for (size_t i = 0; i < _inCld->points.size(); ++i)
    {
        if (_inCld->points[i].normal_x > 35)
        {
            cld->push_back(_inCld->points[i]);
        }
    }

    
    pcl::SampleConsensusModelPlane<pcl::PointXYZINormal>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZINormal>(cld));

    pcl::RandomSampleConsensus<pcl::PointXYZINormal> ransac(model_p);
    ransac.setDistanceThreshold(.2);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::copyPointCloud(*cld, inliers, *_rscCld);
}
void getIntensityTh(pcl::PointCloud<pcl::PointXYZINormal>::Ptr _rscCld, pcl::PointCloud<pcl::PointXYZINormal>::Ptr _ftCld)
{
    float inten_sum = 0.0;
    float mean, sd;
    float var = 0.0;
    float _5th_per;
    int cnt = 0;
    for (size_t i = 0; i < _rscCld->points.size(); ++i)
    {
        inten_sum += _rscCld->points[i].intensity;
        cnt++;
    }
    mean = inten_sum / (float)cnt;
    for (size_t i = 0; i < _rscCld->points.size(); ++i)
    {
        var += pow(_rscCld->points[i].intensity - mean, 2);
    }
    var /= (float)cnt;
    sd = sqrt(var);

    _5th_per = (1.28 * sd) + mean;

    for (size_t i = 0; i < _rscCld->points.size(); ++i)
    {
        if (_rscCld->points[i].intensity > _5th_per)
        {
            // if (_rscCld->points[i].normal_x == 60)
            //     std::cout << "_rscCld->point : (" << _rscCld->points[i].x << " , " << _rscCld->points[i].y << " , " << _rscCld->points[i].z << ")\n";
            _ftCld->push_back(_rscCld->points[i]);
        }
    }
    std::cout << "_5th_per : " << _5th_per << std::endl;

    //return _5th_per;
}

int main(int argc, char** argv)
  {
       
    /*point cloud map load*/
    
    // if(pcl::io::loadPCDFile<pcl::PointXYZINormal>(argv[1], *inCloud)==-1)
     if(pcl::io::loadPCDFile<pcl::PointXYZINormal>("/home/autoware/yuna/yuna_point/yuna_binary.pcd", *inCloud)==-1)
    {
      PCL_ERROR("Couldn't read\n");
      return(-1);
    }

    std::cout << "Point cloud map read \n"; 
    std::cout << "first size :"<<inCloud->points.size() <<std::endl;

for (int i = 0;  inCloud->points.size() != 0; i++ )
//for (int i = 0;  i<10; i++ )
      {  
        /*point raw seperate*/
        *inCloud = getLIDAROneScan();

        pcl::PointXYZINormal ndt = getNDTPose(*rawCloud); //one scan 에서 ndt pose 추출 
        std::cout << "x : " << ndt.x << " y : " << ndt.y <<  " z : " << ndt.z  <<  " intensity: " << ndt.intensity  <<  " nx : " << ndt.normal_x <<  " curva : " << ndt.curvature << std::endl;
        if(ndt.curvature ==0) //ndt pose 없는 경우
          { std::cout << "no ndt"<<std::endl;
            continue;
          }         

        Eigen::MatrixXf transform = getTransMat(ndt); //map -> vehicle 변환 행렬 
        
        *resultCloud = transformPoint(*rawCloud , transform); //좌표 이동 
        
        applyRANSAC(resultCloud, ransacCloud);
        getIntensityTh(ransacCloud, outCloud);

        Eigen::MatrixXf transform2 = getTransMat2(ndt); //vehicle -> map 변환 행렬 
        *outputCloud += transformPoint2(*outCloud , transform2);

        /*save change point cloud*/ 
        // std::stringstream ss1; 
        // ss1 << "_" << i << ".pcd";
        // pcl::io::savePCDFile("/home/autoware/yuna/yuna_point/after_cor/" + ss1.str(), *resultCloud, false);
        //std::cout << "result size :" << resultCloud->size() << std::endl;
        std::cout << i << "th file saving now ... " << "rest of in cloud size: " << inCloud->size() << std::endl;

        
        rawCloud -> clear();
        resultCloud -> clear();
        outCloud ->clear();
        ransacCloud->clear();
        std::cout << "clear" << std::endl;
      }
      pcl::io::savePCDFile("/home/autoware/yuna/yuna_point/test.pcd" , *outputCloud, false);
      std::cout << "save" << std::endl;
  return 0;
}