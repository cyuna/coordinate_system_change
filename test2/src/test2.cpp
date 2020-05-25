#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>


#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZ> gnss_cloud_;
void callbackGnssPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  pcl::PointXYZ point;
  point.x = msg->pose.position.x;
  point.y = msg->pose.position.y;
  point.z = msg->pose.position.z;
  gnss_cloud_.push_back(point);
}
void callbackFlag(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data == true)
  {
    pcl::io::savePCDFile("gnss_pose_binary.pcd", gnss_cloud_, false);
  }
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "test2");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/gnss_pose", 1, &callbackGnssPose);
  ros::Subscriber sub2 = nh.subscribe("/flag", 1, &callbackFlag);

  ros::spin(); // Use to /points_raw_extended -> 1scan.pcd
}
