/*
 * wam_force_estimator.cpp
 *
 * In this script, we use dynamic matrices derived for 4-DOF WAM by Aritra Mitra et al.
 * GitHub Repo: https://github.com/raj111samant/Model-based-control-of-4-DOF-Barrett-Wam/tree/master/WAM-C%2B%2B-Codes
 * 
 * Created on: August, 2023
 * Author: Faezeh
 */

#include <Dynamics.hpp>
#include <differentiator.hpp>
#include <force_estimator_4dof.hpp>
#include <wam_surface_Estimator.hpp>
#include <extended_ramp.hpp>
#include <get_tool_position_system.hpp>
#include <get_jacobian_system.hpp>
#include <robust_cartesian.h>
#include <extended_Tool_Orientation.hpp>

#include <libconfig.h++>
#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <unistd.h>
#include <fcntl.h>
#include <barrett/standard_main_function.h>
#include <barrett/math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <wam_force_estimation/RTCartForce.h>

using namespace barrett;
using namespace systems;

/*
// Function to generate waypoints along a cubic spline and move to them
template <size_t DOF>
std::vector<units::CartesianPosition::type> generateCubicSplineWaypointsAndMove(
    Wam<DOF>& wam,
    const units::CartesianPosition::type& initialPos,
    const units::CartesianPosition::type& finalPos,
    double waypointSpacing
) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    std::vector<units::CartesianPosition::type> waypoints;

    Eigen::Vector3d deltaPos = finalPos - initialPos;
    double totalDistance = deltaPos.norm();

    // Determine the number of waypoints
    int numWaypoints = static_cast<int>(totalDistance / waypointSpacing);
    if (numWaypoints < 2) {
        // Ensure at least two waypoints
        numWaypoints = 2;
        waypointSpacing = totalDistance / (numWaypoints - 1);
    }

    for (int i = 0; i < numWaypoints; ++i) {
        double t = static_cast<double>(i) / (numWaypoints - 1);
        Eigen::Vector3d waypoint = initialPos + t * deltaPos;
        waypoints.push_back(waypoint);

        // Move to the waypoint
        wam.moveTo(cp_type(waypoint), true, 0.05);
    }

    return waypoints;
}
*/
// Function to generate waypoints along a Cubic Bezier curve and move to them
template <size_t DOF>
std::vector<units::CartesianPosition::type> generateCubicSplineWaypointsAndMove(
    Wam<DOF>& wam,
    const units::CartesianPosition::type& initialPos,
    const units::CartesianPosition::type& finalPos,
    double offset
) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    std::vector<units::CartesianPosition::type> waypoints;

    Eigen::Vector3d deltaPos = finalPos - initialPos;
    double totalDistance = deltaPos.norm();

    // Calculate intermediate positions by offsetting x, y, and z
    Eigen::Vector3d yeeb = initialPos + deltaPos * offset;
    Eigen::Vector3d goalb = finalPos - deltaPos * offset;

    // Bezier interpolation
    int numPop = 1000;
    std::vector<Eigen::Vector3d> plPop;
    for (int i = 0; i < numPop; ++i) {
        double t = static_cast<double>(i) / (numPop - 1);
        Eigen::Vector3d pl = (1 - t) * (1 - t) * (1 - t) * initialPos +
                             3 * (1 - t) * (1 - t) * t * yeeb +
                             3 * (1 - t) * t * t * goalb +
                             t * t * t * finalPos;
        plPop.push_back(pl);
    }

    // Calculate the subset of points to sample
    int steps = 20;
    double squish = 2.0;
    std::vector<double> midArr;
    for (double t = -squish; t <= squish; t += squish / (steps - 1)) {
        midArr.push_back(static_cast<double>(numPop) / (1 + std::exp(-t)));
    }

    double minMidArr = *std::min_element(midArr.begin(), midArr.end());
    double maxMidArr = *std::max_element(midArr.begin(), midArr.end());
    double m = (numPop - 1) / (maxMidArr - minMidArr);

    std::vector<int> iArr;
    for (double t : midArr) {
        int index = static_cast<int>(m * (t - minMidArr));
        iArr.push_back(index);
    }

    iArr[0] = 0;
    iArr.back() = numPop - 1;

    // List of points as a sample of bezier
    std::vector<units::CartesianPosition::type> pl;
    for (int index : iArr) {
        pl.push_back(cp_type(plPop[index]));
    }

    for (const auto& waypoint : pl) {
        waypoints.push_back(waypoint);

        // Move to the waypoint
        wam.moveTo(waypoint, true, 0.05);
    }

    return waypoints;
}

template <size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
    typedef boost::tuple<double, cp_type, cp_type, cf_type> tuple_type;

    // Initialize ROS node and publishers
    ros::init(argc, argv, "force_estimator_node");
    ros::NodeHandle nh;
    ros::Publisher force_publisher = nh.advertise<wam_force_estimation::RTCartForce>("force_topic", 1);
    ros::Publisher force_avg_publisher = nh.advertise<wam_force_estimation::RTCartForce>("force_avg_topic", 1);

    // Setting up real-time command timeouts and initial values
    ros::Duration rt_msg_timeout;
    rt_msg_timeout.fromSec(0.2); // rt_status will be determined false if rt message is not received in the specified time

    // Temporary file for logger
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        printf("ERROR: Couldn't create a temporary file!\n");
        return 1;
    }

    // Moving to the start pose
    jp_type POS_READY;
    POS_READY << 0.002227924477643431, -0.1490540623980915, -0.04214558734519736, 1.6803055108189549;
    wam.moveTo(POS_READY);

    // Adding gravity term and unholding joints
    wam.gravityCompensate();
    usleep(1500);

    // Set the differentiator mode indicating how many data points it uses
    int mode = 5;
    ja_type zero_acc;

    // Load configuration settings
    v_type drive_inertias;
    drive_inertias[0] = 11631e-8;
    drive_inertias[1] = 11831e-8;
    drive_inertias[2] = 11831e-8;
    drive_inertias[3] = 10686e-8;
    libconfig::Setting& setting = pm.getConfig().lookup(pm.getWamDefaultConfigPath());

    // Instantiate systems
    GravityCompensator<DOF> gravityTerm(setting["gravity_compensation"]);
    getJacobian<DOF> getWAMJacobian;
    ForceEstimator<DOF> forceEstimator(true);
    Dynamics<DOF> wam4dofDynamics;
    differentiator<DOF, jv_type, ja_type> diff(mode);
    ExtendedRamp time(pm.getExecutionManager(), 1.0);
    Constant<ja_type> zero(zero_acc);
    const LowLevelWam<DOF>& llw = wam.getLowLevelWam();
    Gain<ja_type, sqm_type, jt_type> driveInertias(llw.getJointToMotorPositionTransform().transpose() * drive_inertias.asDiagonal() * llw.getJointToMotorPositionTransform());
    TupleGrouper<double, cp_type, cp_type, cf_type> tg;
    PrintToStream<cf_type> print(pm.getExecutionManager());

    // Add surface estimator parts
    SurfaceEstimator<DOF> surface_estimator;
    ExtendedToolOrientation<DOF> rot;
    getToolPosition<DOF> cp;

    // First-order filter instead of differentiator
    double omega_p = 180.0;
    FirstOrderFilter<jv_type> hp;
    hp.setHighPass(jv_type(omega_p), jv_type(omega_p));
    pm.getExecutionManager()->startManaging(hp);
    Gain<jv_type, double, ja_type> changeUnits(1.0);

    // Real-time data logger
    const size_t PERIOD_MULTIPLIER = 1;
    PeriodicDataLogger<tuple_type> logger(
        pm.getExecutionManager(),
        new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
        PERIOD_MULTIPLIER);

    // Connecting system ports
    connect(time.output, tg.template getInput<0>());

    // Using First-order filter instead of differentiator
    // connect(wam.jvOutput, hp.input);
    // connect(hp.output, changeUnits.input);
    // connect(changeUnits.output, forceEstimator.jaInput);
    // connect(changeUnits.output, driveInertias.input);

    // Using differentiator
    connect(time.output, diff.time);
    connect(wam.jvOutput, diff.inputSignal);
    connect(diff.outputSignal, forceEstimator.jaInput);
    connect(diff.outputSignal, driveInertias.input);

    // Zero Acc
    // connect(zero.output, forceEstimator.jaInput);
    
    connect(wam.kinematicsBase.kinOutput, getWAMJacobian.kinInput);
    connect(getWAMJacobian.output, forceEstimator.Jacobian);

    connect(wam.kinematicsBase.kinOutput, gravityTerm.kinInput);
    connect(gravityTerm.output, forceEstimator.g);

    // connect(zero.output, driveInertias.input);
    connect(driveInertias.output, forceEstimator.rotorInertiaEffect);

    connect(wam.jtSum.output, forceEstimator.jtInput);

    connect(wam.jpOutput, wam4dofDynamics.jpInputDynamics);
    connect(wam.jvOutput, wam4dofDynamics.jvInputDynamics);
    connect(wam4dofDynamics.MassMAtrixOutput, forceEstimator.M);
    connect(wam4dofDynamics.CVectorOutput, forceEstimator.C);

    // systems::connect(forceEstimator.cartesianForceOutput, tg.template getInput<1>());

    // Connecting input and outputs for surface estimator
    connect(wam.kinematicsBase.kinOutput, rot.kinInput);
    connect(rot.output, surface_estimator.rotInput);

    connect(wam.kinematicsBase.kinOutput, cp.kinInput);
    connect(cp.output, surface_estimator.cpInput);

    connect(forceEstimator.cartesianForceOutput, surface_estimator.cfInput);
    // connect(forceEstimator.cartesianForceOutput, print.input);

    connect(surface_estimator.P1, tg.template getInput<1>());
    connect(surface_estimator.P2, tg.template getInput<2>());
    connect(surface_estimator.P3, tg.template getInput<3>());
    connect(tg.output, logger.input);

    // Initialization Move when starting
    // jp_type wam_init = wam.getHomePosition();
    // wam_init[3] -= .35;
    // wam.moveTo(wam_init); // Adjust the elbow, moving the end-effector out of the haptic boundary and hold position for haptic force initialization.

    // Reset and start the time counter
    {
        BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
        time.reset();
        time.start();
    }

    // Change decrease our tool position controller gains slightly
    cp_type cp_kp, cp_kd;
    for (size_t i = 0; i < 3; i++) {
        cp_kp[i] = 1500;
        cp_kd[i] = 5.0;
    }
    wam.tpController.setKp(cp_kp);
    wam.tpController.setKd(cp_kd);

    // Making a spline from the current cp to the start cp
    cp_type start_pose;
    start_pose[0] = 0.554666;
    start_pose[1] = 0.019945;
    start_pose[2] = -0.18;
    std::vector<cp_type> waypoints = generateCubicSplineWaypointsAndMove(wam, wam.getToolPosition(), start_pose, 0.05);

    printf("Logging started.\n");

    // Set terminal input to non-blocking mode
    struct termios oldSettings, newSettings;
    tcgetattr(STDIN_FILENO, &oldSettings);
    newSettings = oldSettings;
    newSettings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    bool inputReceived = false;
    std::string lineInput2;
    std::cout << "Press [Enter] to stop." << std::endl;

    ros::Rate pub_rate(500);

    cp_type next_pose;
    next_pose << 0.6560848991017444, 0.019945, -0.18;
    std::vector<cp_type> waypoints2 = generateCubicSplineWaypointsAndMove(wam, wam.getToolPosition(), next_pose, 0.05);
    usleep(10);

	cp_type test_pose, p_test;
	Eigen::Matrix3d s;
	s << 1, 0, 0,
		0, 1, 0,
		0, 0, 0;
	test_pose << 0.7060848991017444, 0.119945, 0.4;
	p_test = wam.getToolOrientation()*surface_estimator.p.inverse()*s*surface_estimator.p*wam.getToolOrientation().inverse()*test_pose;
	//std::vector<cp_type> waypoints3 = generateCubicSplineWaypointsAndMove(wam, wam.getToolPosition(), p_test, 0.05);
    std::cout<<surface_estimator.p*wam.getToolOrientation().inverse()<<std::endl;
    
    

    //double f_d = 1.0;
    Eigen::MatrixXd I(3, 3);
    I.setIdentity();
    cf_type u_cf; 
    //u_cf = surface_estimator.p.inverse()*(I-s)*surface_estimator.p*(f_d - forceEstimator.computedF);
    std::cout << "Estimated F:" << forceEstimator.computedF << std::endl;
    std::cout << p_test << std::endl;



    int i = 0;
    cf_type cf_avg;
    wam_force_estimation::RTCartForce force_msg;
    wam_force_estimation::RTCartForce force_avg_msg;
    btsleep(1);
    while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
        // Process pending ROS events and execute callbacks
        ros::spinOnce();

        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                inputReceived = true;
                break;
            }
        }

        i++;
        cf_avg = cf_avg + forceEstimator.computedF;
        force_msg.force[0] = forceEstimator.computedF[0];
        force_msg.force[1] = forceEstimator.computedF[1];
        force_msg.force[2] = forceEstimator.computedF[2];
        force_msg.force_norm = forceEstimator.computedF.norm() / 9.81;
        // force_msg.force_dir = forceEstimator.computedF.normalize();
        force_msg.time = time.getYValue();
        force_publisher.publish(force_msg);

        if (i == 20) {
            cf_avg = cf_avg / i;
            force_avg_msg.force[0] = cf_avg[0];
            force_avg_msg.force[1] = cf_avg[1];
            force_avg_msg.force[2] = cf_avg[2];
            force_avg_msg.time = time.getYValue();
            force_avg_msg.force_norm = cf_avg.norm() / 9.81;
            // force_avg_msg.force_dir = cf_avg.normalize();
            force_avg_publisher.publish(force_avg_msg);
            i = 0;
            cf_avg << 0.0, 0.0, 0.0;
        }

        pub_rate.sleep();
    }

    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    fcntl(STDIN_FILENO, F_SETFL, 0);

    // Stop the time counter and disconnect controllers
    time.stop();
    wam.idle();
    wam.moveHome();

    logger.closeLog();
    printf("Logging stopped.\n");

    log::Reader<tuple_type> lr(tmpFile);
    lr.exportCSV(argv[1]);
    printf("Output written to %s.\n", argv[1]);
    std::remove(tmpFile);

    return 0;
}