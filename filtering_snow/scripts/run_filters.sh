#!/bin/bash

set -e # exit on first error
# this is incomplete - does not work properly
roslaunch filtering_snow filter_radius.launch &
rosbag play -s 10 ~/bag_files/Lidar_Caribou/27-11-52-15/2017-01-27-11-52-15_short.bag &
sleep 10s
rostopic echo -p /radius/AverageProcessRate &
#rostopic echo -p /radius/AverageProcessRate > outputfile.txt &
rosnode kill /radius_outlier_filter
exit
