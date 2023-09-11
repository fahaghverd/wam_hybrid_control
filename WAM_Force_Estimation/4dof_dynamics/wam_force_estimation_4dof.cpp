/*
 * wam_force_estimator.cpp
 *
 * 
 * 
 * In this script we use dynamic matrices drived for 4dof wam by Aritra Mitra et al., at this repo :
 * https://github.com/raj111samant/Model-based-control-of-4-DOF-Barrett-Wam/tree/master/WAM-C%2B%2B-Codes
 * 
 *  Created on: August, 2023
 *      Author: Faezeh
 */
#include <libconfig.h++>
#include <Dynamics.hpp>
#include <differentiator.hpp>
#include <force_estimator_4dof.hpp>
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

class ExtendedRamp : public systems::Ramp {
public:
    using Ramp::Ramp;  // Inherit constructors from Ramp
	
    double getYValue() const {
        return y;
    }
};

template<size_t DOF>
class getJacobian : public System, public systems::SingleOutput<math::Matrix<6,DOF>>,
					public systems::KinematicsInput<DOF>
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	getJacobian(const std::string& sysName = "getJacobian"): System(sysName), SingleOutput<math::Matrix<6,DOF>>(this), KinematicsInput<DOF>(this) {}
	virtual ~getJacobian() {this->mandatoryCleanUp(); }

protected:
	math::Matrix<6,DOF> Jacobian;
	gsl_matrix * j;
	virtual void operate() {
		j = this->kinInput.getValue().impl->tool_jacobian;
		// Convert GSL matrix to Eigen matrix
		for (size_t row = 0; row < 6; ++row) {
			for (size_t col = 0; col < DOF; ++col) {
				// Access the element in the GSL matrix and copy it to the math::Matrix
				double value = gsl_matrix_get(j, row, col);
				Jacobian(row, col) = value;
			}
		}
		this->outputValue->setData(&Jacobian);
	}
private:
	DISALLOW_COPY_AND_ASSIGN(getJacobian);

};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,	systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	
	// Define the start pose for the robot
	jp_type start_pose;
	start_pose<< 0.5, 0.5, 1, 2.0;

	//Moving to start pose
	wam.moveTo(start_pose);

	//Adding gravity term and unholding joints
	wam.gravityCompensate();
	usleep(1500);

	// Set the differentiator mode indicating how many data points it uses
	int mode = 5;  

	// Load configuration settings
	libconfig::Config config;
	config.readFile("inverse_dynamics_test.conf");
	const libconfig::Setting& setting = config.lookup(pm.getWamDefaultConfigPath());
	
	//Instantiating systems
	GravityCompensator<DOF> gravityTerm(setting["gravity_compensation"]);
	typedef boost::tuple<double, cf_type> tuple_type;
	//ForceEstimator<DOF> forceEstimator(false);
	getJacobian<DOF> getWAMJacobian;
	ForceEstimator<DOF> forceEstimator(true);
	Dynamics<DOF> wam4dofDynamics;
	differentiator<DOF, jv_type, ja_type> diff(mode);
	ExtendedRamp time(pm.getExecutionManager(), 1.0);
	const LowLevelWam<DOF>& llw = wam.getLowLevelWam();
	systems::Gain<ja_type, sqm_type, jt_type> driveInertias(llw.getJointToMotorPositionTransform().transpose() * v_type(config.lookup("drive_inertias")).asDiagonal() * llw.getJointToMotorPositionTransform());
	systems::TupleGrouper<double, cf_type> tg;
	//FrictionCompensator<DOF> friction_comp();

	double omega_p = 130.0;
	systems::FirstOrderFilter<jv_type> hp;
	hp.setHighPass(jv_type(omega_p), jv_type(omega_p));
	pm.getExecutionManager()->startManaging(hp);
	systems::Gain<jv_type, double, ja_type> changeUnits(1.0);

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
	systems::connect(wam.jvOutput, diff.inputSignal);
	systems::connect(diff.outputSignal, forceEstimator.jaInput);
	systems::connect(diff.outputSignal, driveInertias.input);
	//systems::connect(changeUnits.output, forceEstimator.jaInput);

	systems::connect(wam.kinematicsBase.kinOutput, getWAMJacobian.kinInput);
	systems::connect(wam.kinematicsBase.kinOutput, gravityTerm.kinInput);

	systems::connect(getWAMJacobian.output, forceEstimator.Jacobian);
	systems::connect(driveInertias.output, forceEstimator.rotorInertiaEffect);
	systems::connect(gravityTerm.output, forceEstimator.g);

	systems::connect(wam.jtSum.output, forceEstimator.jtInput);
	
	systems::connect(wam.jpOutput, wam4dofDynamics.jpInputDynamics);
	systems::connect(wam.jvOutput, wam4dofDynamics.jvInputDynamics);
	systems::connect(wam4dofDynamics.MassMAtrixOutput, forceEstimator.M);
	systems::connect(wam4dofDynamics.CVectorOutput, forceEstimator.C);

	systems::connect(forceEstimator.cartesianForceOutput, tg.template getInput<1>());
	//systems::connect(tg.output, print.input);
	systems::connect(tg.output, logger.input);

	
	// Reset and start the time counter
	//time.reset();
	time.start();
	
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

	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE && !inputReceived){

		char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == '\n') {
                inputReceived = true;
                break;
            }
        }
			btsleep(0.2);

			{//std::cout << "Timestamp:" << time.getYValue() << std::endl;
			 std::cout << "Estimated Ja:" << forceEstimator.Jacobian.getValue() << std::endl;
			 std::cout << "Estimated Ja:" << forceEstimator.jacobianPseudoInverse << std::endl;
			 //std::cout << "Estimated rotor:" << forceEstimator.rotorInertiaEffect.getValue() << std::endl;
			 std::cout << "Estimated F:" << forceEstimator.computedF << std::endl;
			 std::cout << "Joint Torques:" << wam.getJointTorques() << std::endl;
			 std::cout << "Est Torques:" << forceEstimator.jt << std::endl;
			 }


	
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