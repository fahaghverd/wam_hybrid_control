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

## GSL
set(GSL_INCLUDE_DIRS "/usr/include/gsl")
set(GSL_LIBRARY_DIRS "/usr/lib/x86_64-linux-gnu")
set(GSL_LIBRARY "/usr/lib/x86_64-linux-gnu/libgsl.so")

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})

## Boost
set(boost_components system thread python)
find_package(Boost REQUIRED ${boost_components})
link_directories(${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

set(exported_include_dirs ${exported_include_dirs} ${Boost_INCLUDE_DIRS})
set(exported_link_dirs ${exported_link_dirs} ${Boost_LIBRARY_DIRS})

## curses
find_package(Curses REQUIRED)

include_directories(
  ${CURSES_INCLUDE_DIR}
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
 
add_executable(spacemouse_teleop_wam src/spacemouse_teleop_wam.cpp)
target_link_libraries(spacemouse_teleop_wam ${catkin_LIBRARIES} barrett ${GSL_LIBRARY} config++)



