cmake_minimum_required(VERSION 3.0.2)
project(planar_surface_hybrid_control)

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
include_directories(${catkin_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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
  
)


add_executable(planar_surface_hybrid_control src/planar_surface_hybrid_control.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  planar_surface_hybrid_control
  barrett 
  ${catkin_LIBRARIES} 
  ${CURSES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GSL_LIBRARY}
  config++
  )
  


