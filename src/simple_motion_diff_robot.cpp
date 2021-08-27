/* 
@description:
This node publishes velocities (in open-loop) to the differential robot in Gazebo, and it subscribes to the robot odometry in order to obtain its position and orientation.
Both the traslational velocity (V) and the rotational velocity (W) are published in the "/cmd_vel" topic.
Both the position (x,y) and the orientation (yaw), from the quaternion, are received from the "/odom" topic.

@author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado" (YouTube channel)

-- To finish this node, please press 'ctrl+C'.
DIRECTIONS:
In different shells:
$ roslaunch ros_cpp_py_basics one_tb3_env.launch
	# Launch one simulated turtlebot3 in Gazebo using an empty world. Note: Simulation starts paused.

$ rosrun ros_cpp_py_basics simple_motion_diff_robot
	# Run this node
*/
//Header files. Use the commands: "rostopic info [topic_name]" and "rosmsg info [message_type_name]" to find out which header files are needed
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h" //Required for using the tf class

using namespace std;

//Global objects to publish and subscribe
ros::Publisher vel_pub;
ros::Subscriber pose_sub;

//Global message types
geometry_msgs::Twist vel_msg;

double x, y, yaw; //Global variables (robot posture)

void OdomCallback(const nav_msgs::Odometry::ConstPtr & msg){//Callback function to get the robot posture
  double roll, pitch; //Local variables
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;

  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);  //Euler angles are given in radians
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_motion_diff_robot"); //Initialize the node
  ros::NodeHandle nh; //Create the node handle
  ros::Rate loop_rate(20); //Node frequency (Hz)

  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); //To publish in the topic
  pose_sub = nh.subscribe("/odom", 10, OdomCallback); //To subscribe to the topic

  //Importnat: Due to a differential type mobile robot is used, the following fields are ignore 
  vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x =  vel_msg.angular.y = 0;

  //Define constant velocities (circular movement)
  vel_msg.linear.x = 0.1; //Translational velocity V [m/s]
  vel_msg.angular.z = 0.2; //Rotational velocity W [rad/s]
  
  cout << "To finish this node, please press ctrl+C" << endl;
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); //Warning message

  while(ros::ok()){
    vel_pub.publish(vel_msg); //Publish the velocities
    printf("x: %.3f, y: %.3f, yaw: %.3f\n", x, y, yaw); //Print the robot states using 'printf' function
    cout << "x: " << x << " y: " << y << " yaw: " << yaw << endl; //Print the robot states using 'cout' function

    ros::spinOnce();  //Required for publishing and receiving callback functions
    loop_rate.sleep(); //Command to wait the rest of the time to complete the loop rate
  }
  return 0;
}
