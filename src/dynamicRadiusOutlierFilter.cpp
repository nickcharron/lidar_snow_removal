#include "DROR.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <string>
#include <vector>

typedef std::chrono::system_clock::time_point TimePoint;

ros::Publisher pubOutputPoints, pubAvgDuration, pubAvgRate;
ros::Duration currentDuration(0), accumDuration(0);
ros::Time begin;
std::string inputTopic, outputDirectory, outputDirectoryClouds,
            outputDirectoryTime;
bool writeToKitty = false;
double radiusSearch, multiplier, azAngle, minSR;
std_msgs::Float64 averageDuration, averageRate;
int minNeighbours, noCloudsProcessed = 0;
std::vector<std::string> timestamps;

TimePoint rosTimeToChrono(const std_msgs::Header& hdr) {
  std::chrono::seconds secs(hdr.stamp.sec);
  std::chrono::nanoseconds nsecs(hdr.stamp.nsec);
  auto dur = secs + nsecs;
  return TimePoint(dur);
}

std::string convertTimeToDate(TimePoint time_) {
  using namespace std;
  using namespace std::chrono;
  system_clock::duration tp = time_.time_since_epoch();
  time_t tt = system_clock::to_time_t(time_);
  tm local_tm = *localtime(&tt);

  string outputTime =
      to_string(local_tm.tm_year + 1900) + "_" +
      to_string(local_tm.tm_mon + 1) + "_" + to_string(local_tm.tm_mday) + "_" +
      to_string(local_tm.tm_hour) + "_" + to_string(local_tm.tm_min) + "_" +
      to_string(local_tm.tm_sec);
  return outputTime;
}

std::string convertTimeToKitty(const std_msgs::Header &header) {
  TimePoint time_stamp = rosTimeToChrono(header);
  using namespace std;
  using namespace std::chrono;
  time_t tt = system_clock::to_time_t(time_stamp);
  tm local_tm = *localtime(&tt);

  std::string year, month, day, hour, minute, second, nanosecond;
  year = to_string(local_tm.tm_year + 1900);
  if(local_tm.tm_mon + 1 < 10) {
    month = "0" + to_string(local_tm.tm_mon + 1);
  } else {
    month = to_string(local_tm.tm_mon + 1);
  }
  if(local_tm.tm_mday < 10) {
    day = "0" + to_string(local_tm.tm_mday);
  } else {
    day = to_string(local_tm.tm_mday);
  }
  if(local_tm.tm_hour < 10) {
    hour = "0" + to_string(local_tm.tm_hour);
  } else {
    hour = to_string(local_tm.tm_hour);
  }
  if(local_tm.tm_min < 10) {
    minute = "0" + to_string(local_tm.tm_min);
  } else {
    minute = to_string(local_tm.tm_min);
  }
  if(local_tm.tm_sec < 10) {
    second = "0" + to_string(local_tm.tm_sec);
  } else {
    second = to_string(local_tm.tm_sec);
  }

  nanosecond = std::to_string(header.stamp.nsec);
  int missingZeros = 9 - nanosecond.size();
  for (int i = 0; i < missingZeros; i++){
    nanosecond = "0" + nanosecond;
  }

  string outputTime = year + "-" + month + "-" + day + " " + hour + ":" +
                      minute + ":" + second + "." + nanosecond;
  return outputTime;
}

void writeCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in) {
  std::string filename =
      std::to_string(noCloudsProcessed + 10000000000) + ".txt";
  filename.erase(0, 1);
  std::string filepath = outputDirectoryClouds + filename;
  std::ofstream file(filepath);
  if(file.is_open()){
    for (uint32_t i = 0; i < cloud_in->points.size(); i++){
      file << std::setprecision(4) << cloud_in->points[i].x << " "
                                   << cloud_in->points[i].y << " "
                                   << cloud_in->points[i].z << " "
                                   << cloud_in->points[i].intensity << "\n";
    }
  } else {
    ROS_ERROR("Cannot write point cloud to file: %s", filepath.c_str());
  }
  file.close();
}

void writeTimeStamps() {
  std::string filename = "timestamps.txt";
  std::string filepath = outputDirectoryTime + filename;
  std::ofstream file(filepath);
  if(file.is_open()){
    ROS_INFO("Writing time stamps to file: %s", filepath.c_str());
    for (uint32_t i = 0; i < timestamps.size(); i++){
      file << timestamps[i] << "\n";
    }
  } else {
    ROS_ERROR("Cannot write time stamps to file: %s", filepath.c_str());
  }
  file.close();
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // Count number of point clouds processed
  noCloudsProcessed++;

  // Container for original & filtered data
  pcl::PCLPointCloud2 input_cloud2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>());

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

  if (writeToKitty) {
    writeCloud(cloud_filtered);
    timestamps.push_back(convertTimeToKitty(cloud_msg->header));
  }
}

void mySigintHandler(int sig)
{
  writeTimeStamps();
  ros::shutdown();
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "dynamicRadiusOutlierFilter");
  ros::NodeHandle nh;
  signal(SIGINT, mySigintHandler);
  ROS_INFO("Dynamic Radius Outlier Removal Node Initialize");

  // Get parameters from ROS parameter server
  ros::param::get("/DROR/inputTopic", inputTopic);
  ros::param::get("/DROR/radius_multiplier", multiplier);
  ros::param::get("/DROR/azimuth_angle", azAngle);
  ros::param::get("/DROR/min_Neighbours", minNeighbours);
  ros::param::get("/DROR/min_search_radius", minSR);
  ros::param::get("/DROR/write_to_kitty_format", writeToKitty);
  ros::param::get("/DROR/output_directory", outputDirectory);

  ROS_INFO("Filter Information: dynamicRadiusOutlierFilter");
  ROS_INFO("The input topic is %s", inputTopic.c_str());
  ROS_INFO("Radius search multiplier dimension is set to: %.2f", multiplier);
  ROS_INFO("Azimuth angle of the lidar is set to: %.2f degrees", azAngle);
  ROS_INFO("Minimum neighbours required in each search radius is set to: %d",
           minNeighbours);
  ROS_INFO("Minimum search radius set to: %.3f", minSR);
  if (writeToKitty) {
    ROS_INFO("Saving clouds in kitty format to: %s", outputDirectory.c_str());
    outputDirectoryClouds =
        outputDirectory + "/" +
        convertTimeToDate(std::chrono::system_clock::now()) +
        "/velodyne_points/";
    boost::filesystem::create_directories(outputDirectoryClouds);
    outputDirectoryTime = outputDirectory + "/" +
                          convertTimeToDate(std::chrono::system_clock::now()) +
                          "/";
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe(inputTopic, 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pubOutputPoints = nh.advertise<sensor_msgs::PointCloud2>("/DROR/output", 1);
  pubAvgDuration =
      nh.advertise<std_msgs::Float64>("/DROR/AverageProcessTime", 1);
  pubAvgRate = nh.advertise<std_msgs::Float64>("/DROR/AverageProcessRate", 1);

  ros::spin();
  return 0;
}
