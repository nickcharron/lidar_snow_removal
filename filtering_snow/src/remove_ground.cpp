#include "remove_ground.hpp"
#include <wave/matching/ground_segmentation.hpp>
#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

std::string inputTopic;
ros::Publisher pubOutputPoints;
const auto package_path = ros::package::getPath("filtering_snow");
const auto config_path = package_path + "/config/";

void removeGround(const wave::GroundSegmentationParams &params,
                  const PointCloud::ConstPtr &input,
                  const PointCloud::Ptr &output)
{
    wave::GroundSegmentation<pcl::PointXYZ> gs{params};
    gs.setInputCloud(input);
    gs.filter(*output);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Remove ground
    removeGround(wave::GroundSegmentationParams{config_path + ground_segmentation_params.yaml},
                  cloudPtr, cloud_filtered);

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
  ros::init (argc, argv, "groundRemovalFilter");
  ros::NodeHandle nh;
  std::cout << std::endl;
  ROS_INFO("Ground Removal Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/removeGround/inputTopic", inputTopic);
  ROS_INFO("The input topic is %s" , inputTopic.c_str());

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubOutputPoints = nh.advertise<sensor_msgs::PointCloud2> ("/removeGround/output", 1);

  // Spin
  ros::spin ();
}
