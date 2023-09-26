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

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>

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
Eigen::MatrixXd J;


public:
	explicit ForceEstimator( const std::string& sysName = "ForceEstimator"):
		System(sysName), Jacobian(this), jtWAMInput(this), jtCompInput(this), cartesianForceOutput(this, &cartesianForceOutputValue),
		 cartesianTorqueOutput(this, &cartesianTorqueOutputValue){}

	virtual ~ForceEstimator() { this->mandatoryCleanUp(); }

protected:
	jt_type jt_wam_sys, jt_comp_sys;

	Eigen::Vector4d cf_out;
	Eigen::MatrixXd jacobianPseudoInverse;

	virtual void operate() {
		/*Taking feedback values from the input terminal of this system*/
		jt_wam_sys = this->jtWAMInput.getValue();
		jt_comp_sys = this->jtCompInput.getValue();

		J = this->Jacobian.getValue();

		jacobianPseudoInverse = J.completeOrthogonalDecomposition().pseudoInverse();

		cf_out = jacobianPseudoInverse.transpose()*(jt_wam_sys - jt_comp_sys);
		
		computedF = cf_out.segment(0,3);
		computedT = cf_out.segment(3,6);

		cartesianForceOutputValue->setData(&computedF);
 		cartesianTorqueOutputValue->setData(&computedT);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ForceEstimator);
};