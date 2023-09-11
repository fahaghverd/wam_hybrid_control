cmake_minimum_required(VERSION 2.6)
project(wam_force_estimation)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )


# By default, build programs with compiler optimizations and debugging symbols
if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## libbarrett
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})

## Boost
find_package(Boost 1.46 COMPONENTS system  REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#find_package(gsl REQUIRED)
#include_directories(${gsl_INCLUDE_DIR})

function(add_programs)
	foreach (prog ${ARGV})
		add_executable(${prog} ${prog}.cpp)
               	#add_executable(${prog} ${prog}.cpp)
		target_link_libraries(${prog} ${BARRETT_LIBRARIES})
	endforeach()
endfunction()


add_programs(
	wam_force_estimation_4dof
    )



