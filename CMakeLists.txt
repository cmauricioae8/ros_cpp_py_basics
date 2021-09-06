cmake_minimum_required(VERSION 3.0.2)
project(ros_cpp_py_basics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs )

include_directories( ${catkin_INCLUDE_DIRS} )

#add_executable(simple_motion_diff_robot src/simple_motion_diff_robot.cpp)
#target_link_libraries(simple_motion_diff_robot ${catkin_LIBRARIES})

#add_executable(diff_robot_controller src/diff_robot_controller.cpp)
#target_link_libraries(diff_robot_controller ${catkin_LIBRARIES})

#add_executable(two_diff_robots_ns_service src/two_diff_robots_ns_service.cpp)
#target_link_libraries(two_diff_robots_ns_service ${catkin_LIBRARIES})

#add_executable(lidar_scan_listening src/lidar_scan_listening.cpp)
#target_link_libraries(lidar_scan_listening ${catkin_LIBRARIES})
