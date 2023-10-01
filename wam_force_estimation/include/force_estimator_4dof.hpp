/*
 * wam_force_estimator_4dof.cpp
 *
 * 
 *  Created on: August, 2023
 *      Author: Faezeh
 */


#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/math/kinematics.h>

using namespace barrett;

template<size_t DOF>
class ForceEstimator: public systems::System
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
	Input<ja_type> jaInput;  	// joint acc. input
	Input<jt_type> jtInput;		// joint torque input

public:
	Input<Eigen::Matrix4d> M; // mass matrix input
	Input<Eigen::Vector4d> C; // coriolis vector input

public:
	Input<jt_type> rotorInertiaEffect;

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


public:
	explicit ForceEstimator(bool driveInertias = false, const std::string& sysName = "ForceEstimator"):
		System(sysName), jaInput(this), M(this), C(this), jtInput(this), Jacobian(this), rotorInertiaEffect(this), g(this), driveInertias(driveInertias),
		 cartesianForceOutput(this, &cartesianForceOutputValue), cartesianTorqueOutput(this, &cartesianTorqueOutputValue){}

	virtual ~ForceEstimator() { this->mandatoryCleanUp(); }

protected:
	bool driveInertias;

	Eigen::Vector4d C_inside;
	Eigen::Matrix4d M_inside;
	math::Matrix<6,DOF> J;

	ja_type ja_sys;
	jt_type jt_sys, jt_drive, G;

	Eigen::Matrix<double, 6, 4> jaco, jacobianPseudoInverse;

	Eigen::Vector4d tmp_a, tmp_jt, tmp_jt_inertia, tmp_g, tmp_j1, tmp_j2, tmp_j3, tmp_j4, tmp_j5, tmp_j6;
	Eigen::Matrix<double, 6, 1> cf_out;

	virtual void operate() {
		/*Taking feedback values from the input terminal of this system*/
		ja_sys = this->jaInput.getValue();
		jt_sys = this->jtInput.getValue();

		/*Taking M, C, J and rotor inertia torque values from the input terminal of this system*/
		M_inside = this->M.getValue();
		C_inside = this->C.getValue();

		jt_drive = this->rotorInertiaEffect.getValue();

		J = this->Jacobian.getValue();
		
		//jacobianPseudoInverse = jaco.completeOrthogonalDecomposition().pseudoInverse();
	
		
		G = this->g.getValue();		
		
		tmp_a << ja_sys[0], ja_sys[1], ja_sys[2], ja_sys[3];
		tmp_jt << jt_sys[0], jt_sys[1], jt_sys[2], jt_sys[3];
		tmp_jt_inertia << jt_drive[0], jt_drive[1], jt_drive[2], jt_drive[3];
		tmp_g << G[0], G[1], G[2], G[3];
		tmp_j1 << J[0,0], J[0,1], J[0,2], J[0,3];
		tmp_j2 << J[1,0], J[1,1], J[1,2], J[1,3];
		tmp_j3 << J[2,0], J[2,1], J[2,2], J[2,3];
		tmp_j4 << J[3,0], J[3,1], J[3,2], J[3,3];
		tmp_j5 << J[4,0], J[4,1], J[4,2], J[4,3];
		tmp_j6 << J[5,0], J[5,1], J[5,2], J[5,3];
		
		jaco.row(0) << tmp_j1;
		jaco.row(1) << tmp_j2;
		jaco.row(2) << tmp_j3;
		jaco.row(3) << tmp_j4;
		jaco.row(4) << tmp_j5;
		jaco.row(5) << tmp_j6;

		jacobianPseudoInverse = jaco.completeOrthogonalDecomposition().pseudoInverse();
	
	
		if (driveInertias){	
			tmp_jt_inertia << jt_drive[0], jt_drive[1], jt_drive[2], jt_drive[3];
			cf_out = jacobianPseudoInverse*(tmp_jt - (C_inside + M_inside * tmp_a + tmp_jt_inertia + tmp_g)); 
			}
		else {cf_out = jacobianPseudoInverse*(tmp_jt - (C_inside + M_inside * tmp_a + tmp_g));}
		
		computedF = cf_out.segment(0,3);
		computedT = cf_out.segment(3,6);
		
		cartesianForceOutputValue->setData(&computedF);
 		cartesianTorqueOutputValue->setData(&computedT);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(ForceEstimator);
};