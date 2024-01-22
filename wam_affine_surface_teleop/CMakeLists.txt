cmake_minimum_required(VERSION 3.0.2)
project(wam_affine_surface_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  wam_msgs
  geometry_msgs
  wam_srvs
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
  rviz  
)


add_executable(spacemouse_teleop_rviz src/spacemouse_teleop_rviz.cpp)
target_link_libraries(spacemouse_teleop_rviz ${catkin_LIBRARIES})
  

