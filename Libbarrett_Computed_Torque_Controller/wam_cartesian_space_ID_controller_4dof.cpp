/*
 *  wam_cartesian_space_ID_controller_4dof.cpp (inverse dynamic: ID)
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
#include <barrett/log.h>
#include <barrett/standard_main_function.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>


template<size_t DOF>
class csIDController :  public systems::System, 
						public KinematicsInput<DOF>	
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
	Input<cp_type> refCPInput;	// reference cart pos. input   
	Input<cv_type> refCVInput;	// reference cart vel. input
	Input<ca_type> refCAInput;	// reference cart acc. input

public:
	Input<cp_type> feedbackcpInput;     // cart pos. input       
	Input<cv_type> feedbackcvInput;  	// cart vel. input

public:
	Input<Eigen::Matrix4d> M; // mass matrix input
	Input<Eigen::Vector4d> C; // coriolis vector input

//public:
	//Input<math::Matrix<3,DOF>> linearJacobian; 

// IO  (outputs)
public:
	Output<jt_type> controlJtOutput;    // output joint torque

protected:
	typename Output<jt_type>::Value* controljtOutputValue;

public:
	explicit csIDController(Eigen::Matrix3d proportionalGains, Eigen::Matrix3d dampingGains, const std::string& sysName = "csIDController"):
		System(sysName), KinematicsInput<DOF>(this), refCPInput(this), refCVInput(this),  refCAInput(this), feedbackcpInput(this), feedbackcvInput(this),
		controlJtOutput(this, &controljtOutputValue),M(this), C(this), kp(proportionalGains), kd(dampingGains){}

	virtual ~csIDController() { this->mandatoryCleanUp(); }

protected:
	Eigen::Matrix4d M_inside;
	Eigen::Vector4d C_inside;
	Eigen::Matrix3d kp, kd;

	jt_type jt_out;
	cp_type cp_sys, cp_ref;
	cv_type cv_sys, cv_ref;
	ca_type ca_ref;

	gsl_matrix* linearJacobian; 
	gsl_matrix* linearJacobianInverse;
	gsl_permutation* perm;
	int signum;

	Eigen::Vector3d tmp_p, tmp_v, tmp_pref, tmp_vref, tmp_aref;
	Eigen::Vector4d jt_out_tmp;	

	virtual void operate() {
		/*Taking reference values from the input terminal of this system*/
		cp_ref = this->refCPInput.getValue();
		cv_ref = this->refCVInput.getValue();
		ca_ref = this->refCAInput.getValue();

		/*Taking feedback values from the input terminal of this system*/
		cp_sys = this->feedbackcpInput.getValue();
		cv_sys = this->feedbackcvInput.getValue();

		/*Taking M, C and J values from the input terminal of this system*/
		M_inside = this->M.getValue();
		C_inside = this->C.getValue();

		tmp_p << cp_sys[0], cp_sys[1], cp_sys[2];
		tmp_v << cv_sys[0], cv_sys[1], cv_sys[2];
		tmp_aref << ca_ref[0], ca_ref[1], ca_ref[2];
		tmp_vref << cv_ref[0], cv_ref[1], cv_ref[2];
		tmp_pref << cp_ref[0], cp_ref[1], cp_ref[2];

		jt_out_tmp = Eigen::Vector4d::Zero();

		linearJacobian = this->kinInput.getValue().impl->tool_jacobian_linear;
		
		// Convert GSL matrix to Eigen matrix
		gsl_linalg_LU_decomp(linearJacobian, perm, &signum);
		gsl_linalg_LU_invert(linearJacobian, perm, linearJacobianInverse);
		Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> gslToEigenMatrix(
        gsl_matrix_ptr(linearJacobianInverse, 0, 0),
        linearJacobianInverse->size1,
        linearJacobianInverse->size2);

		jt_out_tmp = C_inside + M_inside * gslToEigenMatrix * (tmp_aref + kp * (tmp_pref - tmp_p) + kd * (tmp_vref - tmp_v));

		jt_out[0] = jt_out_tmp[0];
		jt_out[1] = jt_out_tmp[1];
		jt_out[2] = jt_out_tmp[2];
		jt_out[3] = jt_out_tmp[3];
		
		controljtOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(csIDController);
};


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,	systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	//Moving to start pose
	jp_type start_pose;
	start_pose<< 0.0, 0.558759518104764, 0.0, 2.32406132303686;
	wam.moveTo(start_pose);

	//Adding gravity term
	wam.gravityCompensate();

	//Controller gains
	Eigen::Matrix3d kp,kd;
	kp<<0.0, 0.0, 0.0;
		0.0, 0.0, 0.0;
		0.0, 0.0, 0.0;

	kd<<0.0, 0.0, 0.0;
		0.0, 0.0, 0.0;
		0.0, 0.0, 0.0;

	cp_type start_cpose;
	start_cpose<< 0.448377885936026, 0.0, 0.346132439275328; //make sure its the right coordination

	csIDController<DOF> compTorqueController(kp,kd);
	Dynamics<DOF> wam4dofDynamics;

	//pm.getSafetyModule()->setVelocityLimit(1.5);
	std::string lineInput;
	printf(" press '1' for configuration regulation. \n");
	printf(" Or press 2 for constant velocity profile. \n");
		std::getline(std::cin, lineInput);

	switch (lineInput[0]) {
		case('1'):{
			cp_type x_des;
			x_des << 0.0, 0.0, 0.0;
			regulationRefTrajectory<DOF, cp_type, cv_type, ca_type> refTraj(x_des);

			systems::connect(refTraj.referencePTrack, compTorqueController.refCPInput);
			systems::connect(refTraj.referenceVTrack, compTorqueController.refCVInput);
			systems::connect(refTraj.referenceATrack, compTorqueController.refCAInput);

		}
		break;

		case('2'):{
			cv_type xd_des;
			xd_des << 0.0, 0.0, 0.0;
			constVelRefTrajectory<DOF, cp_type, cv_type, ca_type> refTraj(xd_des, start_cpose);

			systems::connect(refTraj.referencePTrack, compTorqueController.refCPInput);
			systems::connect(refTraj.referenceVTrack, compTorqueController.refCVInput);
			systems::connect(refTraj.referenceATrack, compTorqueController.refCAInput);

		}
		break;
		

	}

	
	systems::connect(wam.toolPosition.output, compTorqueController.feedbackcpInput);
	systems::connect(wam.toolVelocity.output, compTorqueController.feedbackcvInput);

	systems::connect(wam.jpOutput, wam4dofDynamics.jpInputDynamics);
	systems::connect(wam.jvOutput, wam4dofDynamics.jvInputDynamics);
	systems::connect(wam4dofDynamics.MassMAtrixOutput, compTorqueController.M);
	systems::connect(wam4dofDynamics.CVectorOutput, compTorqueController.C);

	systems::forceConnect(wam.kinematicsBase.kinOutput, compTorqueController.kinInput);

	wam.trackReferenceSignal(compTorqueController.controlJtOutput);

    // Release the WAM if we're holding. This is convenient because it allows
	// users to move the WAM back to some collapsed position before exiting, if
	// they want.
	wam.idle();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}