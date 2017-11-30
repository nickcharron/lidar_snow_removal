#!/usr/bin/env python
import rospy
#import roslaunch
#import rospkg
import os
from std_msgs.msg import Float64

# Initialize variables

new_time = 0
new_rate = 0

def setParameters(RS, MNN):
    rospy.set_param("/radius/radius_search", RS)
    rospy.set_param("/radius/minNeighbours", MNN)
    rospy.set_param("/radius/inputTopic", "/velodyne_points")

def rate_CB(rate_msg):
    global new_rate
    new_rate = rate_msg.data

def time_CB(time_msg):
    global new_time
    new_time = time_msg.data

def launch_subscribers():
    rospy.init_node('calculate_rates')
    rospy.Subscriber("/radius/AverageProcessRate", Float64, rate_CB )
    rospy.Subscriber("/radius/AverageProcessTime", Float64, time_CB )


def main():

    # start node to subscribe to process rate and time messages
    launch_subscribers()

    # Set parameters
    minSR = 0.1
    maxSR = 2.1
    incrSR = 2 #0.2
    SR_iter = minSR

    minNumNeighbours = 6

    # Iterate through parameters
    while (not rospy.is_shutdown()) and (SR_iter < (maxSR + incrSR)):
        setParameters(SR_iter, minNumNeighbours)
        os.system("rosrun filtering_snow radiusOutlierFilter &")
        os.system("rosbag play -s 10 ~/bag_files/Lidar_Caribou/27-11-52-15/2017-01-27-11-52-15_short.bag &")
        rospy.sleep(10)
        # rospy.spin()
        print(new_rate)
        os.system("rosnode kill radiusOutlierFilter")
        SR_iter = SR_iter + incrSR
    rospy.spin()

if __name__ == '__main__':

    main()
