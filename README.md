# lidar_snow_removal
This repo is a set of nodes for ROS to filter point clouds with the goal of removing snow in Lidar data.

You will also need to fork our custom pcl repo to run the custom dynamic radius outlier filter: 

  https://github.com/nickcharron/pcl

  Make sure you are not using pcl_ros from apt-get or it will install and use its own version of pcl.

To use this code, simply change the topics in the launch files to your scan topics, then play your bags and launch the appropriate launch files.

Please view the results videos on my youtube channel: https://www.youtube.com/channel/UC3FoqSLn12-dKOQ1Sn0xbFQ/featured?view_as=subscriber

***NOTE***
As of now, my custom fork of PCL is not publicly available. This will soon be pushed back to PCL so that it is available for all. Please comment out the followinig lines:
  https://github.com/nickcharron/lidar_snow_removal/blob/24c6a23063b059e98f36c365c1cf33960a093b5c/filtering_snow/CMakeLists.txt#L35
  https://github.com/nickcharron/lidar_snow_removal/blob/24c6a23063b059e98f36c365c1cf33960a093b5c/filtering_snow/CMakeLists.txt#L43
