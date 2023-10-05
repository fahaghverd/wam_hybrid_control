/*
 * wam_force_estimator.cpp
 *
 * 
 *  Created on: August, 2023
 *      Author: Faezeh
 */


#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>

using namespace barrett;

template<size_t DOF>
class ForceEstimator :  public systems::System
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:     
	Input<jt_type> jtWAMInput;		// WAM joint torque input
	Input<jt_type> jtCompInput;		// Computed joint torque input


public:
	Input<math::Matrix<6,DOF>> Jacobian;

// IO  (outputs)
public:
	Output<cf_type> cartesianForceOutput;    // output cartesian force
	Output<ct_type> cartesianTorqueOutput;    // output cartesian torque

protected:
	typename Output<cf_type>::Value* cartesianForceOutputValue;
	typename Output<ct_type>::Value* cartesianTorqueOutputValue;

public:
cf_type computedF;
ct_type computedT;
math::Matrix<6,DOF> J;


public:
	explicit ForceEstimator( const std::string& sysName = "ForceEstimator"):
		System(sysName), Jacobian(this), jtWAMInput(this), jtCompInput(this), cartesianForceOutput(this, &cartesianForceOutputValue),
		 cartesianTorqueOutput(this, &cartesianTorqueOutputValue){
		 jt = Eigen::MatrixXd(DOF, 1);
		 tmp_jaco = Eigen::MatrixXd(DOF, 6);}

	virtual ~ForceEstimator() { this->mandatoryCleanUp(); }

protected:
	jt_type jt_wam_sys, jt_comp_sys;
	Eigen::VectorXd cf_out;
	Eigen::MatrixXd tmp_jaco, jt;

	virtual void operate() {
		/*Taking feedback values from the input terminal of this system*/
		jt_wam_sys = this->jtWAMInput.getValue();
		jt_comp_sys = this->jtCompInput.getValue();

		J = this->Jacobian.getValue();
		tmp_jaco = J.transpose();

		jt = jt_wam_sys - jt_comp_sys;
		Eigen::ColPivHouseholderQR<Eigen::MatrixXd> system(tmp_jaco);
		cf_out = system.solve(jt);

		computedF << cf_out[0], cf_out[1], cf_out[2];
		computedT << cf_out[3], cf_out[4], cf_out[5];

		cartesianForceOutputValue->setData(&computedF);
 		cartesianTorqueOutputValue->setData(&computedT);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ForceEstimator);
};