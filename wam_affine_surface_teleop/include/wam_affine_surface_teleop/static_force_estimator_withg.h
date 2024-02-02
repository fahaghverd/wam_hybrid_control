/*
 * wam_force_estimator_4dof.cpp
 *
 * 
 *  Created on: August, 2023
 *      Author: Faezeh
 */


#pragma once
#include <eigen3/Eigen/Dense>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/math/kinematics.h> 

using namespace barrett;

template<size_t DOF>
class StaticForceEstimatorwithG: public systems::System
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
	Input<jt_type> jtInput;		// joint torque input

public:
	Input<math::Matrix<6,DOF>> Jacobian;
	Input<jt_type> g;

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
	explicit StaticForceEstimatorwithG(const std::string& sysName = "ForceEstimator"):
		System(sysName), jtInput(this), Jacobian(this), g(this), cartesianForceOutput(this, &cartesianForceOutputValue), cartesianTorqueOutput(this, &cartesianTorqueOutputValue){}

	virtual ~StaticForceEstimatorwithG() { this->mandatoryCleanUp(); }

protected:
	cf_type cf;
	ct_type ct;

	jt_type jt_sys, G;
	jt_type jt;
	Eigen::VectorXd estimatedF;

	virtual void operate() {
		/*Taking feedback values from the input terminal of this system*/
		jt_sys = this->jtInput.getValue();
		G = this->g.getValue();	
		J = this->Jacobian.getValue();	

		Eigen::ColPivHouseholderQR<Eigen::MatrixXd> system(J.transpose());

		jt = jt_sys - (G);
		estimatedF = system.solve(jt);

		computedF << estimatedF[0], estimatedF[1], estimatedF[2];
		computedT << estimatedF[3], estimatedF[4], estimatedF[5];
		
		cf = computedF;
		ct = computedT;

		cartesianForceOutputValue->setData(&computedF);
 		cartesianTorqueOutputValue->setData(&computedT);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(StaticForceEstimatorwithG);
};