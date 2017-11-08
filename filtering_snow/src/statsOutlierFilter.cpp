#include <ros/ros.h>
#include <iostream>
#include <vector>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;
std::string inputTopic;
double meanK, stdDev;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setMeanK (meanK);
  sor.setStddevMulThresh (stdDev);
  // apply filter
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "voxelGridFilter");
  ros::NodeHandle nh;
  ROS_INFO("voxelGridFilter Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/stats/inputTopic", inputTopic);
  ros::param::get("/stats/meanK", meanK);
  ros::param::get("/stats/stdDev", stdDev);
  ROS_INFO("The input topic is %s" , inputTopic.c_str());
  ROS_INFO("The number of neighbours to analyze is set to: %.0f", meanK);
  ROS_INFO("The standard deviation multiplier is set to: %.1f", stdDev);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/stats/output", 1);

  // Spin
  ros::spin ();
}
