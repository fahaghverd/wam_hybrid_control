/*
 * spacemouse_teleop_wam.h
 *
 *  
 * 
 * Created on: Jan., 2024
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
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <libconfig.h++>
#include <fcntl.h>
#include <termios.h>

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
#include "sensor_msgs/Joy.h"
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
#include "wam_srvs/ContactControlTeleop.h"

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

#include "impedence_controller.h"
#include "static_force_estimator_withg.h"
#include "get_jacobian_system.h"

static const int PUBLISH_FREQ = 500; // Default Control Loop / Publishing Frequency
static const double SPEED = 0.03; // Default Cartesian Velocity

using namespace barrett;
using barrett::detail::waitForEnter;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt
	(const typename units::JointTorques<DOF>::type& x,
	const typename units::JointTorques<DOF>::type& limit)
{
	int index;
	double minRatio;

	/*We don't want to use the motors at the maximum torque levels. Instead we set a basic torque level of our choice
	(shown later), and make sure that the torque provided satisfies the following ratio.

	x = Torque provided to motors (remember this is a 7x1 matrix, given the 7 DOF for our WAM arm)
	limit = Torque limit (user defined)*/
	
	minRatio = limit.cwiseQuotient(x.cwiseAbs()).minCoeff(&index);
	//Functions from Eigen Core library - https://eigen.tuxfamily.org/dox/group__TutorialReductionsVisitorsBroadcasting.html
	if(minRatio < 1.0)
	{
		return minRatio*x;
	}
	else
	{
		return x;
	}
};

//PlanarHybridControl Class
template<size_t DOF>
class JoytoWAM
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    public:
        Eigen::Vector2d rotated_joy_axis, gamma_vec;
        Eigen::Vector3d x_axis, y_axis, x_naxis, y_naxis, joy_axis, cp_eigen;
        Eigen::Vector4d theta_vec;
        Eigen::Matrix2d R, R2, R1; //R1 for rotating x/y to -x/-y
		Eigen::Matrix3d pts;

        float a, b;
        double theta, gamma, alpha, betha; // theta: angles between motion vectors and x/nx axes. gamma: angles between motion vectors and y/ny axes
		//alpha: rotationm angle. betha: angle between next motion vectors.
        bool start_teleop, bases_changed, motion_dir; //if true v1 near x, false v1 near y

        std::vector<double>  speed_scale_, speed_multiplier_, speed_divider_;
        std::vector<int> prev_button_stats_, curr_button_stats_, pressedButtons;

        typedef boost::tuple<double, cp_type, jp_type> config_sample_type;
		typedef boost::tuple<double, cp_type> cp_sample_type;
		typedef boost::tuple<double, jp_type> jp_sample_type;

        cp_type cp_initial_point, p1, p2, p3, p4, surface_normal;
		jp_type jp_initial_point, P1, P2, P3, P4;
		jp_type jp_home, jp_cmd, jtLimits;
		cf_type force_norm;

		libconfig::Setting& setting;
		libconfig::Config config;

		bool locked_joints;
		bool systems_connected;		
		bool force_estimated;
		
        systems::Wam<DOF>& wam;
		systems::Callback<jt_type> jtSat;

        ros::Duration msg_timeout;

        // subscribed topics
        sensor_msgs::Joy space_joy_topic;

        // subscribers
        ros::Subscriber joy_sub_;

        //published topics
		sensor_msgs::JointState wam_joint_state;
		geometry_msgs::PoseStamped wam_pose;
		wam_msgs::MatrixMN wam_jacobian_mn;
		wam_msgs::RTToolInfo wam_tool_info;
		wam_msgs::RTCartForce force_msg;

		// publishers
		ros::Publisher wam_joint_state_pub;
		ros::Publisher wam_pose_pub;
		ros::Publisher wam_jacobian_mn_pub;
		ros::Publisher wam_tool_pub;
		ros::Publisher wam_estimated_contact_force_pub;

        // services
		ros::ServiceServer disconnect_systems_srv;
		ros::ServiceServer go_home_srv;
		ros::ServiceServer joint_move_block_srv;
		ros::ServiceServer calibrartion_srv;
		ros::ServiceServer cp_impedance_control_srv;
		ros::ServiceServer contact_control_teleop_srv;

		//Contace Force Estimation
		StaticForceEstimatorwithG<DOF> staticForceEstimator;
		getJacobian<DOF> getWAMJacobian;
		systems::GravityCompensator<DOF> gravityTerm;
		std::ofstream outputFile; 
		systems::PrintToStream<cf_type> print;

		//Impedance Control
		systems::ImpedanceController6DOF<DOF> ImpControl;
		systems::ExposedOutput<cp_type> KxSet;
		systems::ExposedOutput<cp_type> DxSet;
		systems::ExposedOutput<cp_type> XdSet;
		systems::ExposedOutput<cp_type> OrnKxSet;
		systems::ExposedOutput<cp_type> OrnDxSet;
		systems::ExposedOutput<Eigen::Quaterniond> OrnXdSet;
		systems::ExposedOutput<cp_type> KthSet;
		systems::ExposedOutput<cf_type> FeedFwdForce;
		systems::ToolForceToJointTorques<DOF> toolforce2jt;
		systems::ToolForceToJointTorques<DOF> toolforcefeedfwd2jt;
		systems::Summer<jt_type> torqueSum;
		systems::ToolTorqueToJointTorques<DOF> tt2jt_ortn_split;

    public:
        ros::NodeHandle n_;
        ProductManager* mypm;

        JoytoWAM(systems::Wam<DOF>& wam_, ProductManager& pm):
            n_("wam"),
			wam(wam_),
            jtLimits(20.0), 
			jtSat(boost::bind(saturateJt<DOF>, _1, jtLimits)),
			setting(pm.getConfig().lookup(pm.getWamDefaultConfigPath())),
			gravityTerm(setting["gravity_compensation"]),
			print(pm.getExecutionManager(),"Data: ", outputFile){}

        ~JoytoWAM(){}

        void init(ProductManager& pm);
        bool calibration(wam_srvs::Teach::Request &req, wam_srvs::Teach::Response &res);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        void scaleArray(std::vector<double>& array, const std::vector<double>& scale);
        double angleBetweenVectors(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);
        Eigen::Matrix2d findingRotationMatrix(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);
        int minAbsElement(const Eigen::VectorXd& angleVectors);
        bool goHomeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool jointMoveBlockCallback(wam_srvs::JointMoveBlock::Request &req, wam_srvs::JointMoveBlock::Response &res);
        void publishWam(ProductManager& pm);
        void disconnectSystems();
		bool disconnectSystems(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		void goHome();
		void CartImpController(std::vector<cp_type> &Trajectory, int step = 1, const cp_type &KpApplied = Eigen::Vector3d::Zero(), const cp_type &KdApplied = Eigen::Vector3d::Zero(),
                                                 bool orientation_control = false, const cp_type &OrnKpApplied = Eigen::Vector3d::Zero(), const cp_type &OrnKdApplied = Eigen::Vector3d::Zero(),
                                                 bool ext_force = false, const cf_type &des_force = Eigen::Vector3d::Zero(), bool null_space = false);         
		bool contactControlTeleop(wam_srvs::ContactControlTeleop::Request &req, wam_srvs::ContactControlTeleop::Response &res); 
		std::vector<units::CartesianPosition::type> generateCubicSplineWaypoints(const units::CartesianPosition::type& initialPos,
    																			const units::CartesianPosition::type& finalPos, double offset);

};
