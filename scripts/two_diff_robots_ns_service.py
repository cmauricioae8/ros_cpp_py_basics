#!/usr/bin/env python3

"""This python script is the equivalent to the 'src/two_diff_robots_ns_service.cpp', so it executes the same tasks.
For more information, please refers to that source file.

-- To finish this node and stop the mobile robots, please press 'ctrl+C' or 'q' key."""

__author__ = "C. Mauricio Arteaga-Escamilla from 'Robotica Posgrado' (YouTube channel)"

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty #Required for "/gazebo/reset_simulation" service

import sys, select, os #Handling command line arguments
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

#Initialize Robots' states (x,y,yaw)
r1 = [0,0,0]
r2 = [0,0,0]

vel_msg1 = Twist() #Create Twist message instance
vel_msg2 = Twist()

def getKey(): #Function to use keyboard events on Linux
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def odomCallback1(msg): #Callback function to get the robot posture 1
	global r1
	r1[0] = msg.pose.pose.position.x
	r1[1] = msg.pose.pose.position.y

	#Operations to convert from quaternion to Euler angles (and vice-versa)
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, r1[2]) = euler_from_quaternion(quater_list) #Euler angles are given in radians
	#quat = quaternion_from_euler(roll, pitch,yaw); print quat

def odomCallback2(msg): #Callback function to get the robot posture 2
	global r2
	r2[0] = msg.pose.pose.position.x
	r2[1] = msg.pose.pose.position.y

	#Operations to convert from quaternion to Euler angles (and vice-versa)
	quater = msg.pose.pose.orientation
	quater_list = [quater.x, quater.y, quater.z, quater.w]
	(roll, pitch, r2[2]) = euler_from_quaternion(quater_list) #Euler angles are given in radians
	
def main_function(ns1, ns2):
	rospy.init_node(node_name, anonymous=True) #Initialize the node. anonymous=True for multiple nodes
	rate = rospy.Rate(50) #Node frequency (Hz)
	counter = 0
	
	#Namespaces are used to setup the topics
	vel_pub1 = rospy.Publisher(ns1+'/cmd_vel', Twist, queue_size=10) #To publish in the topic
	rospy.Subscriber(ns1+'/odom',Odometry, odomCallback1) #To subscribe to the topic
	vel_pub2 = rospy.Publisher(ns2+'/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber(ns2+'/odom',Odometry, odomCallback2)

	#In python, any service client is required
	#rospy.wait_for_service('/gazebo/reset_simulation')
	reset_sim_srv = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

	#Importnat: Due to a differential type mobile robot is used, the following fields are ignore
	vel_msg1.linear.y = vel_msg1.linear.z = vel_msg1.angular.x = vel_msg1.angular.y = 0
	vel_msg2.linear.y = vel_msg2.linear.z = vel_msg2.angular.x = vel_msg2.angular.y = 0

	
	reset_sim_srv() #Execute the service call. Note: any additional function is needed
	print("Simulation was reset. To continue, simulation must be running...\n")
	
	print("To finish this node and to stop the robots, please press 'q' key or 'ctrl+C'\n")
	rospy.logwarn("This node reset the simulation in Gazebo\n\n")
	
	t0 = rospy.Time.now().to_sec()
	
	while(1):
		t = rospy.Time.now().to_sec()-t0 #Compute the controller time
		
		vel_msg1.linear.x = 0.1; vel_msg1.angular.z = 0.5
		vel_pub1.publish(vel_msg1); #Publish the control signals
		
		vel_msg2.linear.x = 0.1; vel_msg2.angular.z = -0.5
		vel_pub2.publish(vel_msg2);
		
		
		if counter == 5: #Frequency divisor
			rospy.loginfo("t: %.2f, x1: %.2f, y1: %.2f, Y1: %.1f, x2: %.2f, y2: %.2f, Y2: %.1f\n", 
				t,r1[0],r1[1],r1[2],r2[0],r2[1],r2[2]) #Print in terminal some variables
			counter = 0 #Reset the counter
		else:
			counter += 1
		
		key = getKey()
		if(key == 'q' or key == '\x03'): #\x03 is ctrl+C
			break

		rate.sleep() #spinOnce() function does not exist in python
	
	#Stop the mobile robots
	vel_msg1.linear.x = vel_msg1.angular.z = vel_msg2.linear.x = vel_msg2.angular.z = 0.0
	vel_pub1.publish(vel_msg1); vel_pub2.publish(vel_msg2)
	
	print("\nRobot stops, but simulation keeps running\n")


if __name__ == '__main__':
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
    	node_name = "two_diff_robots_ns_service"
    	if(len(sys.argv) < 3): #Arguments given from shell, using sys module. sys.argv[0] = node_name
    		print ("\nNode: "+node_name+"\nNo robots' namespaces provided (args). Node finished\n\n")
    	else:
        	main_function(sys.argv[1], sys.argv[2])
			#Execute the function with arguments from command line. string type by default.
			#A simple cast can be used to convert from string to int/float, 
			#function(int(sys.argv[1]), float(sys.argv[2])
	
    except rospy.ROSInterruptException:
        pass

    #if os.name != 'nt':
    #   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
