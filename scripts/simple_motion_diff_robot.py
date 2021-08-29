#!/usr/bin/env python3

"""This python script is the equivalent to the 'src/simple_motion_diff_robot.cpp', so it executes the same tasks.
For more information, please refer to that source file.

-- To finish this node, please press 'ctrl+C' keys."""

__author__ = "C. Mauricio Arteaga-Escamilla from 'Robotica Posgrado' (YouTube channel)"

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#Initialize variables (robot posture)
x = 0
y = 0
yaw = 0
vel_msg = Twist()

def odomCallback(msg): #Callback function to get the robot posture
	global x; global y; global yaw
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y

	#Operations to convert from quaternion to Euler angles (and vice-versa)
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, yaw) = euler_from_quaternion(quater_list) #Euler angles are given in radians
	#quat = quaternion_from_euler(roll, pitch,yaw); print quat
	

def main_function():
	rospy.init_node('simple_motion_diff_robot', anonymous=False) #Initialize the node
	"""anonymous=True, which means the node can run on multiple instances. In C++, nodehandle is used. In ROS, nodes are uniquely named. If two nodes with the same name are launched, the previous one is kicked off. The anonymous=True flag means that rospy will choose a unique name for our node so that multiple talkers can run simultaneously"""
	rate = rospy.Rate(20) #Node frequency (Hz)
	
	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #To publish in the topic
	rospy.Subscriber('/odom',Odometry, odomCallback) #To subscribe to the topic
	
	#Important: Due to a differential type mobile robot is used, the following fields are ignored
	vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x = vel_msg.angular.y = 0;

	#Assign constant velocities (circular movement)
	vel_msg.linear.x = 0.1
	vel_msg.angular.z = 0.2

	print("To finish this node, please press ctrl+C\n")
	rospy.logwarn("To start the movement, the simulation must be running\n\n")
	
	while not rospy.is_shutdown():
		vel_pub.publish(vel_msg) #Publish the velocities
		rospy.loginfo("x: %.3f, y: %.3f, yaw: %.2f\n", x, y, yaw) #Print the states using 'loginfo' function
		yaw_deg = "{:.3f}".format(57.29*yaw) #Convertion from rad to deg (from float to string)
		print("yaw: "+yaw_deg+" [deg]\n")
		rate.sleep() #spinOnce() function does not exist in python


if __name__ == '__main__':
    try:
        main_function() #Execute the function
    except rospy.ROSInterruptException:
        pass
