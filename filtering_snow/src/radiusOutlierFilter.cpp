#include <ros/ros.h>
#include <iostream>
#include <vector>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub;
std::string inputTopic;
double radius;
int minNeighbours;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
  outrem.setInputCloud(cloudPtr);
  outrem.setRadiusSearch(radius);
  outrem.setMinNeighborsInRadius(minNeighbours);
  // apply filter
  outrem.filter (cloud_filtered);

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
  ros::init (argc, argv, "radiusOutlierFilter");
  ros::NodeHandle nh;
  ROS_INFO("Radius Outlier Removal Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/radius/inputTopic", inputTopic);
  ros::param::get("/radius/radius_search", radius);
  ros::param::get("/radius/minNeighbours", minNeighbours);
  ROS_INFO("The input topic is %s" , inputTopic.c_str());
  ROS_INFO("Radius search dimension is set to: %.2f", radius);
  ROS_INFO("Minimum neighbours required in each search radius is set to: %d", minNeighbours);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/radius/output", 1);

  // Spin
  ros::spin ();
}
