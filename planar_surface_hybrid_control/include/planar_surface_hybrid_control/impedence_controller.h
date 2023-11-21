/*
 * wam_impedance_control_6dof.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: mas
 */

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <cmath>
#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <libconfig.h++>
#include <barrett/detail/ca_macro.h>
#include <barrett/cdlbt/calgrav.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>

#include <iostream>
#include <string>
#include <barrett/log.h>
#include <math.h>

#include <cstdlib>  // For mkstmp()
#include <cstdio>  // For remove()
#include <barrett/log.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
//#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/systems.h>

namespace barrett {
namespace systems {

template<size_t DOF>
class ImpedanceController6DOF : public System, public KinematicsInput<DOF>
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	// IO  (inputs)
	public:
		Input<Eigen::Quaterniond> OrnReferenceInput;
		Input<Eigen::Quaterniond> OrnFeedbackInput;
		Input<cp_type> OrnKpGains;
		Input<cp_type> OrnKdGains;

		Input<cp_type> CpInput;            // cart pos. input
		Input<cv_type> CvInput;            // cart vel. input
		// Input<Eigen::Quaterniond> OrnInput;   // orientation input (quaternion)
		Input<cp_type> KxInput; // Kx - cart. stiffness { Kx, Ky, Kz }
		Input<cp_type> DxInput; // Dx - cart. damping   { Dx, Dy, Dz }
		// Input<cp_type> KthInput; // Kth - rot.  stiffness { Ktx , Kty ,Ktz }
		Input<cp_type> XdInput;   // Xd - center of spring [xd , yd , zd ] commanded cart pos. : fixed for static spring
		// Input<Eigen::Quaterniond> ThetadInput; // Thetad - center of rotational spring [ ThetaXd , ThetaYd , ThetaZd ]
		//Input<cv_type> commandedCvInput;   // commanded cart vel. : set to zero for now

	// IO  (outputs)
	public:
		Output<cf_type> CFOutput;    // output Cartesian forces
		Output<ct_type> CTOutput;    // output Cartesian torque

	protected:
		typename Output<cf_type>::Value* cfOutputValue;
		typename Output<ct_type>::Value* ctOutputValue;

	public:
		cf_type computedF;          // can be used to display later in the program
		ct_type computedT;          // can be used to display later in the program
		cp_type cart_pos;
		cp_type orientation;

	public:
		ImpedanceController6DOF(
			const std::string& sysName = "ImpedanceController6DOF") :
				System(sysName), KinematicsInput<DOF>(this), CpInput(this), CvInput(this), 
				OrnReferenceInput(this), OrnFeedbackInput(this), OrnKpGains(this), OrnKdGains(this),
				KxInput(this), DxInput(this), XdInput(this), 
				CFOutput(this, &cfOutputValue), CTOutput(this, &ctOutputValue)
		{
		}
		virtual ~ImpedanceController6DOF()
		{
			this->mandatoryCleanUp();
		}
	protected:
		cf_type cf;
		ct_type ct;

		cp_type Xd ;
		cp_type Kx;
		cp_type Dx;

		Eigen::Quaterniond OrnXd;
		cp_type OrnKx;
		cp_type OrnDx;

		cp_type Xcurr;
		cv_type Vcurr;
		Eigen::Quaterniond OrnCurr;

		Eigen::AngleAxisd error;
		cp_type tempVect;
		cp_type tempVect2;
		cp_type errorVect;

	virtual void operate() {
		Xcurr = CpInput.getValue();       // current Cart. Pose
		Vcurr = CvInput.getValue();       // current Cart. Velocity
		// OrnCurr = OrnReferenceInput.getValue();    // current tool Orientation (quaternion)

	  	Xd = XdInput.getValue();      // Xd : cartesian positional input to the system
	  	Kx = KxInput.getValue();      // Kx : cartesian stiffness input to the system
	  	Dx = DxInput.getValue();      // Dx : cartesian damping input to the system

	  	// OrnXd = OrnFeedbackInput.getValue();      // OrnXd : orientation (quaternion) input to the system
	  	OrnKx = OrnKpGains.getValue();      // OrnKx : orientation stiffness input to the system
	  	OrnDx = OrnKdGains.getValue();      // OrnDx : orientation damping input to the system

		for (int i = 0; i<3 ; i++) {
			cf[i] = (Xd[i] -  Xcurr[i]) * Kx[i] + (0.0 - Vcurr[i]) * Dx[i];
		}
		computedF =  cf;
		computedT =  ct;
		cart_pos = Xcurr;
		
		error = this->OrnFeedbackInput.getValue() * this->OrnReferenceInput.getValue().inverse();  // but CD's math (that works, BTW) does it this way
		double angle = error.angle();
		// TODO(dc): I looked into Eigen's implementation and noticed that angle will always be between 0 and 2*pi. We should test for this so if Eigen changes, we notice.
		if (angle > M_PI) {
			angle -= 2.0*M_PI;
		}

		if (math::abs(angle) > 3.13) {	// a little dead-zone near the discontinuity at +/-180 degrees
			ct.setZero();
		} else {
		    tempVect = OrnKx ;				// copy the ProportiocalGains
		    errorVect = error.axis() * angle ;
		    gsl_vector_mul (tempVect.asGslType() , errorVect.asGslType()  ) ; 	// tempVect <- tempVect * errorVect
		    ct = this->OrnReferenceInput.getValue().inverse() * ( tempVect );

		}

		tempVect2 = OrnDx;
		gsl_vector_mul ( tempVect2.asGslType() , this->kinInput.getValue().impl->tool_velocity_angular ) ;
		ct -= tempVect2 ;
//		ct -= DerivativeGains.dot( this->kinInput.getValue().impl->tool_velocity_angular ) ;
//		gsl_blas_daxpy( -kd, this->kinInput.getValue().impl->tool_velocity_angular, ct.asGslType());

		this->ctOutputValue->setData(&ct);
		this->cfOutputValue->setData(&cf);
//		error = this->referenceInput.getValue() * this->feedbackInput.getValue().inverse();  // I think it should be this way
		// error = OrnXd * OrnCurr.inverse();  // but CD's math (that works, BTW) does it this way

		// double angle = error.angle();
		// // TODO(dc): I looked into Eigen's implementation and noticed that angle will always be between 0 and 2*pi. We should test for this so if Eigen changes, we notice.
		// if (angle > M_PI) {
		// 	angle -= 2.0*M_PI;
		// }

		// if (math::abs(angle) > 3.13) {	// a little dead-zone near the discontinuity at +/-180 degrees
		// 	ct.setZero();
		// } else {
		//     errorVect = error.axis() * angle;
		//     gsl_vector_mul(OrnKx.asGslType(), errorVect.asGslType()); 	// OrnKx <- OrnKx * errorVect
		//     ct = OrnCurr.inverse() * (OrnKx);

		// }
		// gsl_vector_mul(OrnDx.asGslType(), this->kinInput.getValue().impl->tool_velocity_angular);
		// ct -= OrnDx ;
//		ct -= DerivativeGains.dot( this->kinInput.getValue().impl->tool_velocity_angular ) ;
//		gsl_blas_daxpy( -kd, this->kinInput.getValue().impl->tool_velocity_angular, ct.asGslType());
		// cfOutputValue->setData(&cf);
		// ctOutputValue->setData(&ct);
		// //-----------------------------------------------------------------------------------------------
		// // convert Current Quaternion to Euler Angles -----------------------------------------------------------
		// pi = 3.14159265359;
		// test_singularity = OrnCurr.x()*OrnCurr.y() + OrnCurr.z()*OrnCurr.w();
		// if (test_singularity > 0.499) { // singularity at north pole
		// 	heading  = 2 * atan2(OrnCurr.x() , OrnCurr.w());
		// 	attitude = pi/2;
		// 	bank = 0.0;
		// 	return;
		// }
		// if (test_singularity < -0.499) { // singularity at south pole
		// 	heading  = -2 * atan2(OrnCurr.x() , OrnCurr.w());
		// 	attitude = -pi/2;
		// 	bank = 0.0;
		// 	return;
		// }
		// sqx = OrnCurr.x()*OrnCurr.x();
		// sqy = OrnCurr.y()*OrnCurr.y();
		// sqz = OrnCurr.z()*OrnCurr.z();
		// heading  = atan2(2*OrnCurr.y()*OrnCurr.w()-2*OrnCurr.x()*OrnCurr.z() , 1 - 2*sqy - 2*sqz);
		// attitude = asin (2*test_singularity);
		// bank = atan2(2*OrnCurr.x()*OrnCurr.w()-2*OrnCurr.y()*OrnCurr.z() , 1 - 2*sqx - 2*sqz);
		// //-----------------------------------------------------------------------------------------------

		// //-----------------------------------------------------------------------------------------------
		// // convert Quaternion Set Point to Euler Angles -----------------------------------------------------------
		// // pi = 3.14159265359;
		// test_singularity2 = Thetad.x()*Thetad.y() + Thetad.z()*Thetad.w();
		// if (test_singularity2 > 0.499) { // singularity at north pole
		// 	heading2  = 2 * atan2(Thetad.x() , Thetad.w());
		// 	attitude2 = pi/2;
		// 	bank2 = 0.0;
		// 	return;
		// }
		// if (test_singularity2 < -0.499) { // singularity at south pole
		// 	heading2  = -2 * atan2(Thetad.x() , Thetad.w());
		// 	attitude2 = -pi/2;
		// 	bank2 = 0.0;
		// 	return;
		// }
		// sqx = Thetad.x()*Thetad.x();
		// sqy = Thetad.y()*Thetad.y();
		// sqz = Thetad.z()*Thetad.z();
		// heading2  = atan2(2*Thetad.y()*Thetad.w()-2*Thetad.x()*Thetad.z() , 1 - 2*sqy - 2*sqz);
		// attitude2 = asin (2*test_singularity);
		// bank2 = atan2(2*Thetad.x()*Thetad.w()-2*Thetad.y()*Thetad.z() , 1 - 2*sqx - 2*sqz);
		// //-----------------------------------------------------------------------------------------------

		// ct[0] = Kth[0] * (heading2 - heading)  ;
		// ct[1] = Kth[1] * (attitude2 - attitude) ;
		// ct[2] = Kth[2] * (bank2 - bank) ;

		// // ct[0] = Kth[0] * (Thetad[0] - heading)  ;
		// // ct[1] = Kth[1] * (Thetad[1] - attitude) ;
		// // ct[2] = Kth[2] * (Thetad[2] - bank) ;


		// orientation[0] = heading  * 180 / 3.1416 ;
		// orientation[1] = attitude * 180 / 3.1416 ;
		// orientation[2] = bank     * 180 / 3.1416 ;
	}

	private:
		DISALLOW_COPY_AND_ASSIGN(ImpedanceController6DOF);
};
}
}
