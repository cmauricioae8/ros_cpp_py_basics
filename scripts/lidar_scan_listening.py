#!/usr/bin/env python3

#Use 'python' for ROS Kinetic and Melodic.
#Use 'python3' for ROS Noetic
"""This python script is the equivalent to the 'src/lidar_scan_listening.cpp', so it executes the same tasks.
For more information, please refer to that source file."""

__author__ = "C. Mauricio Arteaga-Escamilla from 'Robotica Posgrado' (YouTube channel)"

import rospy
from sensor_msgs.msg import LaserScan


def ScanCallback(msg): #Callback function to get lidar measurements
        global t
        t = rospy.Time.now().to_sec()-t0; print(t)
        i = 0
        range_size = len(msg.ranges)
        print("len(ranges): ", range_size)
        range_min = msg.range_min; print(range_min)
        range_max = msg.range_max; print(range_max)
        
        scan = [] #Define the list to be used
        for i in range(range_size):
        	scan.append(msg.ranges[i])
        	if(scan[i]>=range_min and scan[i]<=range_max):
        		print("i: ",i, "dist: ",scan[i])


def main_function():
	rospy.init_node("lidar_scan_listening", anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	
	rospy.Subscriber('/scan',LaserScan, ScanCallback) #To subscribe to the topic
	
	print("Press 'ctrl+C' to finish this node\n")

	global t0
	t0 = rospy.Time.now().to_sec()
	
	rospy.spin()
	print("\n Node finished\n")


if __name__ == '__main__':
    try:
    	main_function()  #Execute the function
	
    except rospy.ROSInterruptException:
        pass
