#include <ros/ros.h>
#include <iostream>
#include <vector>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/dynamic_radius_outlier_removal.h>

ros::Publisher pubOutputPoints, pubAvgDuration, pubAvgRate;
ros::Duration currentDuration(0), accumDuration(0);
ros::Time begin;
std::string inputTopic;
double radiusSearch, multiplier, azAngle, minSR;
std_msgs::Float64 averageDuration, averageRate;
int minNeighbours, noCloudsProcessed = 0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Count number of point clouds processed
    noCloudsProcessed++;

  // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
    pcl::DynamicRadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
    outrem.setInputCloud(cloudPtr);
    outrem.setRadiusSearch(radiusSearch);
    outrem.setRadiusMultiplier(multiplier);
    outrem.setAzimuthAngle(azAngle);
    outrem.setMinSearchRadius(minSR);
    outrem.setMinNeighborsInRadius(minNeighbours);
    // Get current time:
      ros::Time begin = ros::Time::now();

    // apply filter
      outrem.filter (cloud_filtered);

   // Get duration
      currentDuration = ros::Time::now() - begin;

   // Calculate average duration
      accumDuration = accumDuration + currentDuration;
      averageDuration.data = accumDuration.toSec() / noCloudsProcessed;
      averageRate.data = 1/averageDuration.data;

  // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
    pubOutputPoints.publish (output);
    pubAvgDuration.publish (averageDuration);
    pubAvgRate.publish (averageRate);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dynamicRadiusOutlierFilter");
  ros::NodeHandle nh;
  std::cout << std::endl;
  ROS_INFO("Dynamic Radius Outlier Removal Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/radiusDynamic/inputTopic", inputTopic);
  ros::param::get("/radiusDynamic/radius_search", radiusSearch);
  ros::param::get("/radiusDynamic/radius_multiplier", multiplier);
  ros::param::get("/radiusDynamic/azimuth_angle", azAngle);
  ros::param::get("/radiusDynamic/min_Neighbours", minNeighbours);
  ros::param::get("/radiusDynamic/min_search_radius", minSR);

  ROS_INFO("Filter Information: dynamicRadiusOutlierFilter");
  ROS_INFO("The input topic is %s" , inputTopic.c_str());
  ROS_INFO("Radius search multiplier dimension is set to: %.2f", multiplier);
  ROS_INFO("Azimuth angle of the lidar is set to: %.2f degrees", azAngle);
  ROS_INFO("Minimum neighbours required in each search radius is set to: %d", minNeighbours);
  ROS_INFO("Minimum search radius set to: %.3f", minSR);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubOutputPoints = nh.advertise<sensor_msgs::PointCloud2> ("/radiusDynamic/output", 1);
  pubAvgDuration = nh.advertise<std_msgs::Float64> ("/radiusDynamic/AverageProcessTime", 1);
  pubAvgRate = nh.advertise<std_msgs::Float64> ("/radiusDynamic/AverageProcessRate", 1);

  // Spin
  ros::spin ();
  // pubAvgRate.publish (averageRate);
}
