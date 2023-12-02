/*
 * planar_surface_hybrid_control.hpp
 *
 *  
 * 
 * Created on: Nov., 2023
 * Author: Faezeh
 */
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <string>
#include <sstream>
#include <fstream>



#include <boost/thread.hpp> // BarrettHand threading
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <eigen3/Eigen/Geometry>

#include "wam_srvs/StaticForceEstimationwithG.h"
#include "wam_msgs/RTCartForce.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "wam_msgs/RTOrtn.h"
#include "wam_msgs/RTCartVel.h"
#include "wam_msgs/RTVelocity.h"
#include "wam_msgs/MatrixMN.h"
#include "wam_msgs/RTToolInfo.h"
#include "std_srvs/Empty.h"
#include "wam_srvs/JointMoveBlock.h"
#include "wam_srvs/Teach.h"
#include "wam_srvs/Play.h"
#include "wam_srvs/CP_ImpedanceControl.h"


#include "ros/ros.h"

#include <barrett/exception.h>
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/standard_main_function.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>

#include <libconfig.h++>
#include <fcntl.h>
#include <termios.h>

#include "planar_surface_hybrid_control/impedence_controller.h"
#include "planar_surface_hybrid_control/static_force_estimator_withg.h"
#include "planar_surface_hybrid_control/get_jacobian_system.h"

static const int PUBLISH_FREQ = 250; // Default Control Loop / Publishing Frequency
static const double SPEED = 0.03; // Default Cartesian Velocity

using namespace barrett;
using barrett::detail::waitForEnter;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;