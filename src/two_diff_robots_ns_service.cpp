/*
@description:
This node requires the robots' namespace given from shell by the user, to publish and subscribe to the corresponding robots' topics. Any controller is implemented.
Both the traslational and rotational velocity of each mobile robot, are published in the "ns1/cmd_vel" and "ns2/cmd_vel" topics, respectively.
Also, the robots' posture are received from the "ns1/odom" and "ns2/odom" topics, respectively.
The kbhit function is implemented to use keyboard events.

Additonally, this node calls the /gazebo/reset_simulation service.
For more information about Gazebo services: http://gazebosim.org/tutorials/?tut=ros_comm

@author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado" (YouTube channel)

-- To finish this node and stop the mobile robots, please press 'ESC' or 'q' key.
DIRECTIONS:
In different shells:
$ roslaunch ros_cpp_py_basics two_tb3_env.launch
	# Launch two simulated turtlebot3 in Gazebo using an empty world

$ rosrun ros_cpp_py_basics two_diff_robots_ns_service ns1 ns2
	# Run this node using the arguments (robots' namespace)

$ roslaunch ros_cpp_py_basics two_diff_robots_ns.launch
	# run this node using a launch file (optional)
*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h" //Required for using the tf class
#include "std_srvs/Empty.h"  //Required for /gazebo/reset_simulation. ROS service type
//Includes needed by keyboard events (kbhit function implemented in Linux)
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
//Includes for convertions among types
#include <iostream>
#include <string>
#include <cstdlib>

using namespace std;

//Global objects to publish and subscribe
ros::Publisher vel_pub1, vel_pub2;
ros::Subscriber pose_sub1, pose_sub2;

//Global message types
geometry_msgs::Twist vel_msg1, vel_msg2;

//Global service client
ros::ServiceClient reset_sim_client;

double r1[3], r2[3]; //Robots' states. y1 cannot be used
int key;

/*int string2int(string chain){ //Function to convert a string to an integer number
    int number;
    std::stringstream s;
    s << chain;
    s >> number;
    return number;
}
double string2double(string chain){ //Function to convert a string to a double number
    double number = atof(chain.c_str());
    return number;
}*/


int kbhit(void){ //kbhit function of Windows implemented for Linux
  struct termios oldt, newt;
  int ch, oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

void srv_reset_sim(){//Function to call the service
  std_srvs::Empty reset_sim_srv;

  ros::service::call("/gazebo/reset_simulation", reset_sim_srv);
  cout << "Simulation was reset. To continue, simulation must be running...\n";
}

void OdomCallback1(const nav_msgs::Odometry::ConstPtr & msg){//Callback function to get the robot posture 1 
  double roll, pitch; //Local variables
  r1[0] = msg->pose.pose.position.x;
  r1[1] = msg->pose.pose.position.y;

  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, r1[2]);  //Euler angles are given in radians
}

void OdomCallback2(const nav_msgs::Odometry::ConstPtr & msg){//Callback function to get the robot posture 2 
  double roll, pitch; //Local variables
  r2[0] = msg->pose.pose.position.x;
  r2[1] = msg->pose.pose.position.y;

  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, r2[2]);  //Euler angles are given in radians
}


int main(int argc, char **argv)
{
  const string node_name = "two_diff_robots_ns_service";
  if(argc < 3) //Arguments given from shell, argv[0] = node_name
  {
    cout << "\nNode: " << node_name << "\nNo robots' namespaces provided (args). Node finished\n\n";
    return -1; //Exit
  }

  string ns1 = argv[1], ns2 = argv[2]; //Robots' namespace
  /*Important: To get an int/double variable from command line, a function to convert a string chain to an integer/double number must be used.
  int num_int = string2int(ns1); cout << num_int << endl;
  double num_dob = string2double(ns2); cout << num_dob << endl;*/

  ros::init(argc, argv, node_name); //Initialize the node
  ros::NodeHandle nh; //Create the node handle
  ros::Rate loop_rate(50); //Node frequency (Hz)
  int counter = 0;
  double t, t0;

  //Namespaces are used to setup the topics
  vel_pub1 = nh.advertise<geometry_msgs::Twist>(ns1+"/cmd_vel", 10); //To publish in the topic
  pose_sub1 = nh.subscribe(ns1+"/odom", 10, OdomCallback1); //To subscribe to the topic
  vel_pub2 = nh.advertise<geometry_msgs::Twist>(ns2+"/cmd_vel", 10);
  pose_sub2 = nh.subscribe(ns2+"/odom", 10, OdomCallback2);

  reset_sim_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation"); //To call the service

  //Important: Due to a differential type mobile robot is used, the following fields are ignored
  vel_msg1.linear.y = vel_msg1.linear.z = vel_msg1.angular.x =  vel_msg1.angular.y = 0;
  vel_msg2.linear.y = vel_msg2.linear.z = vel_msg2.angular.x =  vel_msg2.angular.y = 0;

  
  cout << "\nTo finish this node and to stop the robots, please press 'ESC' or 'q' key" << endl;
  ROS_WARN_ONCE("This node resets the simulation in Gazebo\n\n"); //Warning message

  srv_reset_sim(); //Execute the function

  t0 = ros::Time::now().toSec(); //Get the initial simulation time

  do{
    t = ros::Time::now().toSec()-t0;    
  
    vel_msg1.linear.x = 0.1; vel_msg1.angular.z = 0.5;
    vel_pub1.publish(vel_msg1); //Publish the control signals

    vel_msg2.linear.x = 0.1; vel_msg2.angular.z = -0.5;
    vel_pub2.publish(vel_msg2);

    if(counter == 5){ //Frequency divisor
      printf("t: %.2f, x1: %.2f, y1: %.2f, Y1: %.1f, x2: %.2f, y2: %.2f, Y2: %.1f\n", 
		t,r1[0],r1[1],r1[2], r2[0],r2[1],r2[2]); //Print in terminal some variables
      counter = 0; //Reset the counter
    }else counter++;

    if(kbhit())	key = getchar();
    if(key == 27 || key == 'q') break; //27 == 'ESC' key

    ros::spinOnce();    //Required for publishing and receiving callback functions
    loop_rate.sleep();  //Command to wait the rest of the time to complete the loop rate
  }while(1); //Note: ros::ok() is not used because some ROS functions must be executed yet


  //Stop the mobile robots
  vel_msg1.linear.x = 0; vel_msg1.angular.z = 0;  vel_msg2.linear.x = 0; vel_msg2.angular.z = 0;
  vel_pub1.publish(vel_msg1); vel_pub2.publish(vel_msg2);

  printf("\nRobots stop, but simulation keeps running\n");

  return 0;
}
