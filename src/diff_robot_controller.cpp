/*
@description:
This node executes a controller to solve the trajectory tracking problem using one differential mobile robot in Gazebo.
In the "velocity_controller" function both the desired trajectory and the signals control are computed.
For this example, the desired trajectory is a lemniscate.
The traslational and rotational velocities (V, W), respectively, are published in the "/cmd_vel" topic.
The position (x,y) and the orientation (yaw), from the quaternion, are received from the "/odom" topic.
The controller used to solve the problem takes into account an outside point of the rear axis wheels. Thus, the orientation angle is a robot state that is not directly controlled.
Also, the tracking errors (ex, ey) are published in the "/tracking_errors" topic to be visualized (using rqt_plot).
Besides, some variables are written in a file to be displayed in Matlab.
The kbhit function is implemented to use keyboard events.

@author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado" (YouTube channel)

-- To finish this node and stop the mobile robot, please press 'ESC' or 'q' key.
DIRECTIONS:
In different shells:
$ roslaunch ros_cpp_py_basics one_tb3_env.launch
	# Launch one simulated turtlebot3 in Gazebo using an empty world. Note: Simulation starts paused.

$ rosrun ros_cpp_py_basics diff_robot_controller
	# Run this node

$ rosrun rqt_plot rqt_plot
	# Open rqt_plot (a graphic tool) to visualize the signals (tracking errors)
*/
//Header files. Use the commands: "rostopic info [topic_name]" and "rosmsg info [message_type_name]" to find out which header files are needed
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h" //Required for using the ft class
#include "geometry_msgs/Pose2D.h" //To publish the tracking errors using the Pose2D message type
//Includes needed by keyboard events (kbhit function implemented in Linux)
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;

//Global objects to publish and subscribe
ros::Publisher vel_pub, error_pub;
ros::Subscriber pose_sub;

//Global message types
geometry_msgs::Twist vel_msg;
geometry_msgs::Pose2D error_msg;

FILE *fpdata; //Define a pointer to a file

double x, y, yaw, l = 0.1; //Global variables (robot posture) and the offset of the outside point
int key;

double t, t0, V_max = 0.22, W_max = 2.84; //timer, initial time, maximum velocities [m/s, rad/s], respectively, for a burger type

double T = 100, k = 0.15; //Trajectory period, controller gains kx = ky = k
double ex, ey, Xd, Yd, Xdp, Ydp; //Tracking errors, desired position, and time derivative, respectively
double V, W; //Control signals

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

void OdomCallback(const nav_msgs::Odometry::ConstPtr & msg){//Callback function to get the robot posture 
  double roll, pitch; //Local variables
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;

  //Operations to convert from quaternion to Euler angles
  tf::Quaternion quater;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quater);
  tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);  //Euler angles are given in radians
}

void velocity_controller(){ //Function to generate the desired trajectory and to compute the signals control
  //Desired trajectory: Lemniscate
  double a = 1, b = 0.5, X0 = 0, Y0 = 0, w = 2*M_PI/T;
  //Desired position on the plane
  Xd = X0+a*sin(w*t);
  Yd = Y0+b*sin(2*w*t);

  //Corresponding time derivatives
  Xdp = a*w*cos(w*t);
  Ydp = 2*b*w*cos(2*w*t);

  double p_x = x+l*cos(yaw), p_y = y+l*sin(yaw); //Compute the coordinates of the outside point
  ex = p_x-Xd; ey = p_y-Yd; //Compute tracking errors

  //Cinematic controller. Auxiliar controls, in global coordinates
  double Ux = Xdp-k*ex; double Uy = Ydp-k*ey;
  
  //Compute the velocities according to the cinematic model for a differential type mobile robot 
  V = Ux*cos(yaw)+Uy*sin(yaw);
  W = -Ux*sin(yaw)/l+Uy*cos(yaw)/l;

  //Velocities saturation
  if(abs(V)>V_max){V = V_max*abs(V)/V; printf("Sat V\t");}
  if(abs(W)>W_max){W = W_max*abs(W)/W; printf("Sat W\t");}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "diff_robot_controller"); //Initialize the node
  ros::NodeHandle nh; //Create the node handle
  ros::Rate loop_rate(50); //Node frequency (Hz)
  int counter = 0;

  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); //To publish in the topic
  pose_sub = nh.subscribe("/odom", 10, OdomCallback); //To subscribe to the topic

  error_pub = nh.advertise<geometry_msgs::Pose2D>("/tracking_errors", 10);

  //Open/create a txt file in writing mode "w"
  //fpdata = fopen("/home/mau/tutorials_catkin_ws/src/ros_cpp_ry_basics/matlab_script/data.txt","w");
  fpdata = fopen("data.txt","w"); //If only the filename is given, the program will create the file in "/home/user/"

  //Importnat: Due to a differential type mobile robot is used, the following fields are ignore 
  vel_msg.linear.y = vel_msg.linear.z = vel_msg.angular.x =  vel_msg.angular.y = 0;
  error_msg.theta = 0; //Since the orientation angle is a sub-actuated state, this field is assigned equal to zero

  
  cout << "To finish this node and to stop the robot, please press 'ESC' or 'q' key" << endl;
  ROS_WARN_ONCE("To start the movement, the simulation must be running\n\n"); //Warning message


  t0 = ros::Time::now().toSec(); //Get the initial simulation time

  do{
    t = ros::Time::now().toSec()-t0; //Compute the controller time    
    velocity_controller(); //Compute the control signals
  
    vel_msg.linear.x = V; vel_msg.angular.z = W;
    vel_pub.publish(vel_msg); //Publish the control signals

    error_msg.x = ex; error_msg.y = ey; //Assign the tracking errors
    error_pub.publish(error_msg); //Publish the tracking errors

    //Write in a file
    fprintf(fpdata,"%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.2f\n", t,Xd,Yd,x,y,V,W);

    if(counter == 25){ //Frequency divisor
      printf("t: %.2f\tex: %.3f\tey: %.3f\tV: %.3f\tW: %.2f\n", t,ex,ey,V,W); //Print in terminal some variables
      counter = 0; //Reset the counter
    }else counter++;

    if(kbhit())	key = getchar();
    if(key == 27 || key == 'q') break; //27 == 'ESC' key

    ros::spinOnce();    //Required for publishing and receiving callback functions
    loop_rate.sleep();  //Command to wait the rest of the time to complete the loop rate
  }while(1); //Note: ros::ok() is not used because some ROS functions must be executed yet

  fclose(fpdata); //Close the file

  //Stop the mobile robot
  vel_msg.linear.x = 0; vel_msg.angular.z = 0;
  vel_pub.publish(vel_msg);

  printf("Robot stops, but simulation keeps running\n");

  return 0;
}
