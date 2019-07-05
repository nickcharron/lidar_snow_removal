# lidar_snow_removal
This repo is a set of nodes for ROS to filter point clouds with the goal of removing snow in Lidar data.

To use this code, simply change the topics in the launch files to your scan topics, then play your bags and launch the appropriate launch files.

Please view the results videos on my youtube channel: https://www.youtube.com/channel/UC3FoqSLn12-dKOQ1Sn0xbFQ/featured?view_as=subscriber

Please refer to our published paper on this work: https://ieeexplore.ieee.org/abstract/document/8575761

***NOTE***
This package used to rely on my custom fork of pcl which had the filter implementation for PointCloud2 data type. This is now all self-contained in this repo. However, the DROR filter only works with pcl::PointCloud\<pcl::PointXYZI\> data type. The ROS node converts the scans to this format before filtering then converts back to ROS msg.
