#include <ros/ros.h>
#include <iostream>
#include <fstream>
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
bool set_negative, outputPoints;
double point_cloud_number_crop = 0;
std::ofstream resultsFile;

void exportPoints(pcl::PCLPointCloud2 input)
{
    point_cloud_number_crop++;

    // Send the input dataset to the spatial locator
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (input, *cloud);

    // iterate through all points and output resutls
    for (int cp = 0; cp < static_cast<int> (cloud->size ()); ++cp)
    {
      float x_i = cloud->points[cp].x;
      float y_i = cloud->points[cp].y;
      // output data into results file
      resultsFile << std::fixed << std::setprecision(6) << x_i << ", " << y_i << ", " << 0 << ", " << std::endl;
    }
}

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
    cropper.setNegative(set_negative);
    cropper.filter (cloud_filtered);

  // Output number of points remaining
    if (outputPoints)
    {
      //Create results file and open it
      std::stringstream sstm;
      sstm << "//home//nick//catkin_ws//src//lidar_snow_removal//lidar_snow_removal//results//CROPBOX//results" << point_cloud_number_crop << ".txt";
      std::string path_local = sstm.str();
      resultsFile.open (path_local.c_str());

      exportPoints(cloud_filtered);

      resultsFile.close();

    }

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
  ros::param::get(ros::this_node::getName() + "/inputTopic", inputTopic);
  ros::param::get(ros::this_node::getName() + "/minvector", minVector);
  ros::param::get(ros::this_node::getName() + "/maxvector", maxVector);
  ros::param::get(ros::this_node::getName() + "/setNegative", set_negative);
  ros::param::get(ros::this_node::getName() + "/outputNoPoints", outputPoints);

  ROS_INFO("The input topic is %s" , inputTopic.c_str());

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (inputTopic, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubOutputPoints = nh.advertise<sensor_msgs::PointCloud2> (ros::this_node::getName() + "/output", 1);

  // Spin
  ros::spin ();
}
