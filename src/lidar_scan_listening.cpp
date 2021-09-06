/* 
@description:
This node susbcribes to the "scan" topic to obtain all LIDAR measurements, but measures within the range are shown.

@author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado" (YouTube channel)

-- To finish this node, please press 'ctrl+C'.
DIRECTIONS:
In different shells:
$ roslaunch ros_cpp_py_basics one_tb3_env.launch
	# Launch one simulated turtlebot3 in Gazebo using an empty world. Note: Simulation starts paused.

$ rosrun ros_cpp_py_basics lidar_scan_listening
	# Run this node
*/
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" //Message type used for the scan topic

using namespace std;
ros::Subscriber scan_sub;

double scan[360]; //vector for ranges
double t, t0, range_min, range_max; //Minimum and maximum distance that can be measured
int range_size; //Number of points that compose the whole lidar measurements

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg){
  int i=0;
  t = ros::Time::now().toSec()-t0; printf("t: %.1f\n",t);

  range_size = msg->ranges.size(); //360. From 0 to 359, anti clockwise direction
  range_min = msg->range_min; //0.12 m
  range_max = msg->range_max; //3.5 m
  
  cout << "range_min: " << range_min << endl << "range_max: " << range_max << endl;

  for(i = 0; i < range_size; ++i){//Get ALL measures, but measures within the range are shown
     scan[i] = msg->ranges[i]; //i=0: front, i=90: left, i=180: back, i=270: right
     if(scan[i]>=range_min && scan[i]<=range_max) printf("i: %d, dist: %.2f\n",i,scan[i]);
  }  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_scan_listening");
  ros::NodeHandle nh;

  scan_sub = nh.subscribe("/scan", 360, ScanCallback);

  t0 = ros::Time::now().toSec();

  ros::spin();  //Callback function only

  printf("\n Node finished\n");
  return 0;
}
