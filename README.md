# lidar_snow_removal
This repo is a set of nodes for ROS to filter point clouds with the goal of removing snow in Lidar data.
You will also need to fork our custom pcl repo (pcl_1.7.0_custom) to run the custom filters: https://github.com/nickcharron/pcl_1.7.0_custom
Make sure you are not using pcl_ros from apt-get or it will install and use its own version of pcl.
