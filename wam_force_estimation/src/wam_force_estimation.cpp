/*
 * wam_force_estimator.cpp
 *
 * 
 *  Created on: August, 2023
 *      Author: Faezeh
 */

#include <differentiator.hpp>
#include <force_estimator.hpp>
#include <extended_ramp.hpp>
#include <get_jacobian_system.hpp>
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

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <wam_force_estimation/RTCartForce.h>

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,	systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	//Initialising ROS node and publishers
	ros::init(argc, argv, "force_estimator_node");
    ros::NodeHandle nh;
	ros::Publisher force_publisher = nh.advertise<wam_force_estimation::RTCartForce>("force_topic", 1);
	ros::Publisher force_avg_publisher = nh.advertise<wam_force_estimation::RTCartForce>("force_avg_topic", 1);

	//Setting up real-time command timeouts and initial values
	ros::Duration rt_msg_timeout;
    rt_msg_timeout.fromSec(0.2); //rt_status will be determined false if rt message is not received in specified time

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	//Moving to start pose
	jp_type start_pose;
	start_pose<< 0.0,1.0,0.0, 2.0;
	wam.moveTo(start_pose);

	//Adding gravity term and unholding joints
	wam.gravityCompensate();
	usleep(1500);

	// Set the differentiator mode indicating how many data points it uses
	int mode = 5;  
	ja_type zero_acc; 

	// Load configuration settings
	//libconfig::Config config;
	//config.readFile("inverse_dynamics_test.conf");		
	v_type drive_inertias;
	drive_inertias[0] = 11631e-8;
	drive_inertias[1] = 11831e-8;
	drive_inertias[2] = 11831e-8;
	drive_inertias[3] = 10686e-8; 
	
	//Instantiating systems
	typedef boost::tuple<double, cf_type, ct_type> tuple_type;
	const LowLevelWam<DOF>& llw = wam.getLowLevelWam();
	systems::Gain<ja_type, sqm_type, jt_type> driveInertias(llw.getJointToMotorPositionTransform().transpose() * drive_inertias.asDiagonal() * llw.getJointToMotorPositionTransform());
	systems::InverseDynamics<DOF> id(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["dynamics"]);
	systems::Summer<jt_type> idSum;
	ForceEstimator<DOF> forceEstimator;
	getJacobian<DOF> getWAMJacobian;
	systems::Constant<ja_type> zero(zero_acc);
	differentiator<DOF, jv_type, ja_type> diff(mode);
	ExtendedRamp time(pm.getExecutionManager(), 1.0);
	systems::TupleGrouper<double, cf_type, ct_type> tg;
	systems::Gain<jv_type, double, ja_type> changeUnits(1.0);
	
	double omega_p = 130.0;
	systems::FirstOrderFilter<jv_type> hp;
	hp.setHighPass(jv_type(omega_p), jv_type(omega_p));
	pm.getExecutionManager()->startManaging(hp);

	//FrictionCompensator<DOF> friction_comp();

	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	//Connecting system potrs
	systems::connect(time.output, diff.time);
	systems::connect(time.output, tg.template getInput<0>());


	//systems::connect(wam.jvOutput, hp.input);
	//systems::connect(hp.output, changeUnits.input);
	//systems::connect(changeUnits.output, id.input);
	//systems::connect(changeUnits.output, driveInertias.input);

	//systems::connect(hp.output, diff.inputSignal);
	//systems::connect(diff.outputSignal, id.input);
	//systems::connect(diff.outputSignal, driveInertias.input);

	systems::connect(zero.output, id.input);
	systems::connect(zero.output, driveInertias.input);

	systems::connect(wam.jvOutput, id.jvInput);
	systems::connect(wam.kinematicsBase.kinOutput, id.kinInput);
	systems::connect(wam.kinematicsBase.kinOutput, getWAMJacobian.kinInput);
	systems::connect(id.output, idSum.getInput(0));
	systems::connect(driveInertias.output, idSum.getInput(1));
	systems::connect(idSum.output, forceEstimator.jtCompInput);
	systems::connect(getWAMJacobian.output, forceEstimator.Jacobian);

	systems::connect(wam.jtSum.output, forceEstimator.jtWAMInput);

	systems::connect(forceEstimator.cartesianForceOutput, tg.template getInput<1>());
	systems::connect(forceEstimator.cartesianTorqueOutput, tg.template getInput<2>());
	
	systems::connect(tg.output, logger.input);

	// Reset and start the time counter
	{BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
	time.reset();
	time.start();}

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

	int i = 0;
	cf_type cf_avg;	
	wam_force_estimation::RTCartForce force_msg;
	wam_force_estimation::RTCartForce force_avg_msg;
	btsleep(1);
	while (ros::ok() && pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE){
		// Process pending ROS events and execute callbacks
		ros::spinOnce();

		char c;		
		if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                inputReceived = true;
                break;
            }
        }
		
		//std::cout<< cf_avg << std::endl;

		i++;
		cf_avg = cf_avg + forceEstimator.computedF;
		//std::cout << "Estimated F:" <<forceEstimator.computedF << std::endl;
		force_msg.force[0] = forceEstimator.computedF[0];
		force_msg.force[1] = forceEstimator.computedF[1];
		force_msg.force[2] = forceEstimator.computedF[2];
		force_msg.time = time.getYValue();
		force_publisher.publish(force_msg);
		
		if(i == 20){
			cf_avg = cf_avg/i;
			force_avg_msg.force[0] = cf_avg[0];
			force_avg_msg.force[1] = cf_avg[1];
			force_avg_msg.force[2] = cf_avg[2];
			force_avg_msg.time= time.getYValue();
			force_avg_publisher.publish(force_avg_msg);
			i = 0;
			cf_avg<< 0.0, 0.0, 0.0;
			}
		
		pub_rate.sleep();
	}


	// Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    fcntl(STDIN_FILENO, F_SETFL, 0);
	
	// Stop the time counter and disconnect controllers
	time.stop();
	wam.idle();

	logger.closeLog();
	printf("Logging stopped.\n");


	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);

	// Wait for user input before moving home
	barrett::detail::waitForEnter();
	wam.moveHome();
	return 0;
}