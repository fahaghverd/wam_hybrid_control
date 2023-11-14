/*
 * wam_joint_space_ID_controller_4dof.cpp (inverse dynamic: ID)
 *
 *  Created on: June, 2023
 *      Author: Faezeh
 */


#include <Dynamics.hpp>
#include <regulation_refference_trajectory.hpp>
#include <constant_vel_refference_traj.hpp>
#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/log.h>
#include <unistd.h>
#include <fcntl.h>
#include <barrett/standard_main_function.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace barrett;
using detail::waitForEnter;

#include <ros/ros.h>

//	systems::PIDController<jp_type, ja_type> pid(config.lookup("control_joint"));
//	systems::InverseDynamics<DOF> id(pm.getConfig().lookup(pm.getWamDefaultConfigPath())["dynamics"]);
//	connect(wam.jpOutput, pid.feedbackInput);
//	connect(wam.jvOutput, id.jvInput);
//	connect(wam.kinematicsBase.kinOutput, id.kinInput);
//	connect(pid.controlOutput, id.input);
//	wam.supervisoryController.registerConversion(systems::makeIOConversion(pid.referenceInput, id.output));

//The WAM arm has a torque saturation limit.

template <size_t DOF>
typename units::JointTorques<DOF>::type SaturateJointTorque
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


template<size_t DOF>
class jsIDController :  public systems::System{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
	Input<jp_type> refJPInput;	// reference joint pos. input   
	Input<jv_type> refJVInput;	// reference joint vel. input
	Input<ja_type> refJAInput;	// reference joint acc. input

public:
	Input<jp_type> feedbackjpInput;     // joint pos. input       
	Input<jv_type> feedbackjvInput;  	// joint vel. input

public:
	Input<Eigen::Matrix4d> M; // mass matrix input
	Input<Eigen::Vector4d> C; // coriolis vector input

// IO  (outputs)
public:
	Output<jt_type> controlJtOutput;    // output joint torque

protected:
	typename Output<jt_type>::Value* controljtOutputValue;

public:
jt_type computedT;

public:
	explicit jsIDController(Eigen::Matrix4d proportionalGains, Eigen::Matrix4d dampingGains, const std::string& sysName = "jsIDController"):
		System(sysName), refJPInput(this), refJVInput(this), refJAInput(this), feedbackjpInput(this), feedbackjvInput(this), controlJtOutput(this, &controljtOutputValue),M(this), C(this), kp(proportionalGains), kd(dampingGains){}

	virtual ~jsIDController() { this->mandatoryCleanUp(); }

protected:
	Eigen::Matrix4d M_inside;
	Eigen::Vector4d C_inside;
	Eigen::Matrix4d kp, kd;

	jt_type jt_out;
	jp_type jp_sys, jp_ref;
	jv_type jv_sys, jv_ref;
	ja_type ja_ref;

	Eigen::Vector4d tmp_p, tmp_v, tmp_control, tmp_aref, tmp_pref, tmp_vref, jt_out_tmp;	

	virtual void operate() {
		/*Taking reference values from the input terminal of this system*/
		jp_ref = this->refJPInput.getValue();
		jv_ref = this->refJVInput.getValue();
		ja_ref = this->refJAInput.getValue();

		/*Taking feedback values from the input terminal of this system*/
		jp_sys = this->feedbackjpInput.getValue();
		jv_sys = this->feedbackjvInput.getValue();

		/*Taking M, C and J values from the input terminal of this system*/
		M_inside = this->M.getValue();
		C_inside = this->C.getValue();

		//std::cout << M_inside << std::endl;

		tmp_p << jp_sys[0], jp_sys[1], jp_sys[2], jp_sys[3];
		tmp_v << jv_sys[0], jv_sys[1], jv_sys[2], jv_sys[3];
		tmp_aref << ja_ref[0], ja_ref[1], ja_ref[2], ja_ref[3];
		tmp_vref << jv_ref[0], jv_ref[1], jv_ref[2], jv_ref[3];
		tmp_pref << jp_ref[0], jp_ref[1], jp_ref[2], jp_ref[3];

		jt_out_tmp = Eigen::Vector4d::Zero();

		//jt_out_tmp = C_inside + M_inside * (tmp_aref + kp * (tmp_pref) + kd * (tmp_vref));
		jt_out_tmp = C_inside + M_inside * (tmp_aref + kp * (tmp_pref - tmp_p) + kd * (tmp_vref - tmp_v));
		

		jt_out[0] = jt_out_tmp[0];
		jt_out[1] = jt_out_tmp[1];
		jt_out[2] = jt_out_tmp[2];
		jt_out[3] = jt_out_tmp[3];

		computedT = jt_out;
		
		controljtOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(jsIDController);
};


class ExtendedRamp : public systems::Ramp {
public:
    using Ramp::Ramp;  // Inherit constructors from Ramp

    double getYValue() const {
        return y;
    }
};

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,	systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	
	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}


	//Moving to start pose
	jp_type start_pose;
	start_pose<< 0.5, 0.5, 1, 1.0;
	wam.moveTo(start_pose);

	//Adding gravity term
	wam.gravityCompensate();

	//Controller gains
	Eigen::Vector4d kp,kd;
	//Reg
	kp<< 90, 120,  330,  45;
	kd<< 15,   15,   20,  6;

	/*//Const. Vel.
	kp<< 40, 	100,		24,	20;
	kd<< 2.5,    5,		2.5,	0.5;*/
	

	//Instantiating controller and dynamics
	jsIDController<DOF> compTorqueController(kp.asDiagonal(),kd.asDiagonal());
	Dynamics<DOF> wam4dofDynamics;

	//Changing velocity limits
	pm.getSafetyModule()->setVelocityLimit(1.2);

	//Set Torque Limits
	jt_type jtLimits(30.0);
	//Connect and pass the parameter to the Joint Torque Saturation system
	systems::Callback<jt_type> jtSat(boost::bind(SaturateJointTorque<DOF>,_1,jtLimits));
	//Note we have used the boost library to bind function objects to arguments.

	//time input for trajectory generation
	const double TRANSITION_DURATION = 0.5;
	ExtendedRamp time(pm.getExecutionManager(), 1.0);

	double t; //time
	jp_type theta_des; //desired position for regulation
	jv_type thetad_des; //desired vel for const. vel. trj.

	systems::System* refTraj = nullptr; // Declare raw ptr
	bool trjSet = false;
	char trj;
	std::string lineInput;

	while(!trjSet){	
		printf("Trajectories:\n");
		printf("  1  Regulation \n");
		printf("  2  Constant velocity profile\n");
		printf("  3  Trapezoidal velocity profile\n");

		std::getline(std::cin, lineInput);

		switch (lineInput[0]) {
			case('1'):{
				theta_des << 0.0, 0.0, 0.0, 0.0;
				refTraj = new regulationRefTrajectory<DOF, jp_type, jv_type, ja_type>(theta_des);

				trjSet = true;
				trj = 'r'; //regulation

			}

			break;

			case('2'):{
				thetad_des << 0.0, 0.0, 0, 0.1;
				refTraj = new constVelRefTrajectory<DOF, jp_type, jv_type, ja_type>(thetad_des, start_pose);

				trjSet = true;
				trj = 'c'; //constant vel.
			}
			break;
		}

		if (trjSet) {
        break;
    	}
	}

	systems::TupleGrouper<double, jp_type, jv_type, ja_type, jp_type, jv_type, jt_type, jt_type> tg;
	systems::connect(time.output, tg.template getInput<0>());
	
	if (auto regRefTraj = dynamic_cast<regulationRefTrajectory<DOF, jp_type, jv_type, ja_type>*>(refTraj)) {
		systems::connect(time.output, regRefTraj->timef);
		systems::connect(regRefTraj->referencePTrack, compTorqueController.refJPInput);
		systems::connect(regRefTraj->referenceVTrack, compTorqueController.refJVInput);
		systems::connect(regRefTraj->referenceATrack, compTorqueController.refJAInput);

		systems::connect(regRefTraj->referencePTrack, tg.template getInput<1>());
		systems::connect(regRefTraj->referenceVTrack, tg.template getInput<2>());
		systems::connect(regRefTraj->referenceATrack, tg.template getInput<3>());

	} else if (auto constRefTraj = dynamic_cast<constVelRefTrajectory<DOF, jp_type, jv_type, ja_type>*>(refTraj)) {
		systems::connect(time.output, constRefTraj->timef);
		systems::connect(constRefTraj->referencePTrack, compTorqueController.refJPInput);
		systems::connect(constRefTraj->referenceVTrack, compTorqueController.refJVInput);
		systems::connect(constRefTraj->referenceATrack, compTorqueController.refJAInput);

		systems::connect(constRefTraj->referencePTrack, tg.template getInput<1>());
		systems::connect(constRefTraj->referenceVTrack, tg.template getInput<2>());
		systems::connect(constRefTraj->referenceATrack, tg.template getInput<3>());
		
	} 

	systems::connect(wam.jpOutput, compTorqueController.feedbackjpInput);
	systems::connect(wam.jvOutput, compTorqueController.feedbackjvInput);

	systems::connect(wam.jpOutput, tg.template getInput<4>());
	systems::connect(wam.jvOutput, tg.template getInput<5>());

	systems::connect(wam.jpOutput, wam4dofDynamics.jpInputDynamics);
	systems::connect(wam.jvOutput, wam4dofDynamics.jvInputDynamics);

	systems::connect(wam4dofDynamics.MassMAtrixOutput, compTorqueController.M);
	systems::connect(wam4dofDynamics.CVectorOutput, compTorqueController.C);

	//systems::connect(wam4dofDynamics.MassMAtrixOutput, tg.template getInput<6>());
	//systems::connect(wam4dofDynamics.CVectorOutput, tg.template getInput<7>());

	systems::connect(compTorqueController.controlJtOutput, jtSat.input);
	
	wam.trackReferenceSignal(jtSat.output);
	//wam.trackReferenceSignal(compTorqueController.controlJtOutput);
	systems::connect(jtSat.output, tg.template getInput<6>());
	systems::connect(wam.jtSum.output, tg.template getInput<7>());

	typedef boost::tuple<double, jp_type, jv_type, ja_type, jp_type, jv_type, jt_type, jt_type> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);


	time.reset();
	time.smoothStart(TRANSITION_DURATION);
	systems::connect(tg.output, logger.input);
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

	while (!inputReceived){
		
		char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                inputReceived = true;
                break;
            }
        }

		if(trj == 'r')
			{std::cout << "position error:" << theta_des - wam.getJointPositions()<< std::endl;}

		else if(trj == 'c')
			{t = time.getYValue();
			theta_des = compTorqueController.refJPInput.getValue();
			
			std::cout << "velocity error:" << thetad_des << std::endl;
			std::cout << "position error:" <<  theta_des << std::endl;
			std::cout << "time:" << t << std::endl;}


		btsleep(0.2);

	}

	// Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    fcntl(STDIN_FILENO, F_SETFL, 0);
	
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();

	logger.closeLog();
	printf("Logging stopped.\n");

	// Open the output file in append mode to add header line and write data
	std::ofstream outputFile(argv[1], std::ios_base::app);

	// Check if the file is successfully opened
	if (!outputFile.is_open()) {
		printf("Error opening the output file: %s\n", argv[1]);
		return 1;
	}

	// Write the header line to the file
	outputFile << "Time, RefJointPosition, RefJointsVelocity, RefJointsAcceleration, RealJointPosition, RealJointsVelocity, RealJointVelocities3, JointTorquesControllerCommand, JointTorquesCommandswithGravity" << std::endl;

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(outputFile);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
	outputFile.close();

	
	waitForEnter();
	wam.moveHome();
	return 0;
}