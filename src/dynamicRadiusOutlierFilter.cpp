#include <iostream>
#include <ros/ros.h>
#include <vector>

// PCL specific includes
#include "DROR.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

ros::Publisher pubOutputPoints, pubAvgDuration, pubAvgRate;
ros::Duration currentDuration(0), accumDuration(0);
ros::Time begin;
std::string inputTopic;
double radiusSearch, multiplier, azAngle, minSR;
std_msgs::Float64 averageDuration, averageRate;
int minNeighbours, noCloudsProcessed = 0;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // Count number of point clouds processed
  noCloudsProcessed++;

  // Container for original & filtered data
  pcl::PCLPointCloud2 input_cloud2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>());

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, input_cloud2);

  // Convert to from PointCloud2
  pcl::fromPCLPointCloud2(input_cloud2, *cloud_input);

  // Perform filtering
  DROR outrem;
  outrem.SetRadiusMultiplier(multiplier);
  outrem.SetAzimuthAngle(azAngle);
  outrem.SetMinSearchRadius(minSR);
  outrem.SetMinNeighbors(minNeighbours);
  // Get current time:
  ros::Time begin = ros::Time::now();

  // apply filter
  outrem.Filter(cloud_input, *cloud_filtered);

  // Get duration
  currentDuration = ros::Time::now() - begin;

  // Calculate average duration
  accumDuration = accumDuration + currentDuration;
  averageDuration.data = accumDuration.toSec() / noCloudsProcessed;
  averageRate.data = 1 / averageDuration.data;

  // Convert to pointcloud2 data type
  pcl::PCLPointCloud2 cloud_filtered_2;
  pcl::toPCLPointCloud2(*cloud_filtered, cloud_filtered_2);

  // Convert to ros msg
  sensor_msgs::PointCloud2 cloud_filtered_msg;
  pcl_conversions::fromPCL(cloud_filtered_2, cloud_filtered_msg);

  // copy header info
  cloud_filtered_msg.header = cloud_msg->header;
  // Publish the data
  pubOutputPoints.publish(cloud_filtered_msg);
  pubAvgDuration.publish(averageDuration);
  pubAvgRate.publish(averageRate);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "dynamicRadiusOutlierFilter");
  ros::NodeHandle nh;
  std::cout << std::endl;
  ROS_INFO("Dynamic Radius Outlier Removal Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/radiusDynamic/inputTopic", inputTopic);
  ros::param::get("/radiusDynamic/radius_multiplier", multiplier);
  ros::param::get("/radiusDynamic/azimuth_angle", azAngle);
  ros::param::get("/radiusDynamic/min_Neighbours", minNeighbours);
  ros::param::get("/radiusDynamic/min_search_radius", minSR);

  ROS_INFO("Filter Information: dynamicRadiusOutlierFilter");
  ROS_INFO("The input topic is %s", inputTopic.c_str());
  ROS_INFO("Radius search multiplier dimension is set to: %.2f", multiplier);
  ROS_INFO("Azimuth angle of the lidar is set to: %.2f degrees", azAngle);
  ROS_INFO("Minimum neighbours required in each search radius is set to: %d",
           minNeighbours);
  ROS_INFO("Minimum search radius set to: %.3f", minSR);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe(inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubOutputPoints =
      nh.advertise<sensor_msgs::PointCloud2>("/radiusDynamic/output", 1);
  pubAvgDuration =
      nh.advertise<std_msgs::Float64>("/radiusDynamic/AverageProcessTime", 1);
  pubAvgRate =
      nh.advertise<std_msgs::Float64>("/radiusDynamic/AverageProcessRate", 1);

  // Spin
  ros::spin();
  // pubAvgRate.publish (averageRate);
}
