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
    minSR = 0.03
    maxSR = 2
    incrSR = 0.05
    SR_iter = minSR
    count = 0

    minNumNeighbours = 6
    #ROR_times = open('scripts/radius_filter_process_times.txt','w')
    ROR_rates = open('scripts/radius_filter_process_rates.txt','w')

    # Iterate through parameters
    while (not rospy.is_shutdown()) and (SR_iter < (maxSR + incrSR)):
        setParameters(SR_iter, minNumNeighbours)
        count = count + 1
        rospy.logwarn("Running iteration %d, out of %d\n", count, (maxSR-minSR)/incrSR+1)
        os.system("rosrun filtering_snow radiusOutlierFilter &")
        os.system("rosrun filtering_snow dynamicRadiusOutlierFilter &")
        os.system("rosbag play -s 10 ~/bag_files/Caribou/2017-01-27-11-52-15_short.bag &")
        rospy.sleep(10)
        # rospy.spin()
        #f_times.write('%.6f \n' % new_time)
        ROR_rates.write('%.6f \n' % new_rate)
        os.system("rosnode kill radiusOutlierFilter")
        SR_iter = SR_iter + incrSR
    rospy.spin()

    #f_times.close
    ROR_rates.close

if __name__ == '__main__':

    main()
