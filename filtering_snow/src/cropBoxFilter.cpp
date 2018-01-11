#include <ros/ros.h>
#include <iostream>
#include <vector>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

std::string inputTopic;
ros::Publisher pubOutputPoints;
Eigen::Vector4f minVec, maxVec;
std::vector<float> minVector, maxVector;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

  // set minimum and maximum vectors
    Eigen::Vector4f minVec(minVector[0], minVector[1], minVector[2], 0);
    Eigen::Vector4f maxVec(maxVector[0], maxVector[1], maxVector[2], 0);

  // Perform the actual filtering
    pcl::CropBox<pcl::PCLPointCloud2> cropper;
    cropper.setMax(maxVec);
    cropper.setMin(minVec);
    cropper.setInputCloud(cloudPtr);
    cropper.setNegative(true);
    cropper.filter (cloud_filtered);

  // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
    pubOutputPoints.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cropBoxFilter");
  ros::NodeHandle nh;
  std::cout << std::endl;
  ROS_INFO("Crop Box Filter Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/cropbox/inputTopic", inputTopic);
  ros::param::get("/cropbox/minvector", minVector);
  ros::param::get("/cropbox/maxvector", maxVector);

  ROS_INFO("The input topic is %s" , inputTopic.c_str());

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubOutputPoints = nh.advertise<sensor_msgs::PointCloud2> ("/cropbox/output", 1);

  // Spin
  ros::spin ();
}
