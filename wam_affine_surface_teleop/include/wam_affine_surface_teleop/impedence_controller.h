/*
 * wam_impedance_control_6dof.cpp
 *
 *  Created on: Aug 28, 2015
 *      Author: mas
 * 
 *  Modified on: Dec, 2023
 *  Author: Faezeh
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

#include <cstdlib> // For mkstmp()
#include <cstdio>  // For remove()
#include <barrett/log.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
// #define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/systems.h>

namespace barrett
{
	namespace systems
	{

		template <size_t DOF>
		class ImpedanceController6DOF : public System, public KinematicsInput<DOF>
		{
			BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

			// IO  (inputs)
		public:
			Input<cp_type> CpInput; // cart pos. input
			Input<cv_type> CvInput; // cart vel. input
			Input<Eigen::Quaterniond> OrnInput;   // orientation input (quaternion)

			Input<cp_type> KxInput; // Kx - cart. stiffness { Kx, Ky, Kz }
			Input<cp_type> DxInput; // Dx - cart. damping   { Dx, Dy, Dz }
			Input<cp_type> OrnKpGains;
			Input<cp_type> OrnKdGains;

			Input<cp_type> XdInput; // Xd - center of spring [xd , yd , zd ] commanded cart pos. : fixed for static spring
			// Input<cv_type> commandedCvInput;   // commanded cart vel. : set to zero for now
			Input<Eigen::Quaterniond> OrnReferenceInput;

			// IO  (outputs)
		public:
			Output<cf_type> CFOutput; // output Cartesian forces
			Output<ct_type> CTOutput; // output Cartesian torque

		protected:
			typename Output<cf_type>::Value *cfOutputValue;
			typename Output<ct_type>::Value *ctOutputValue;

		public:
			cf_type computedF; // can be used to display later in the program
			ct_type computedT; // can be used to display later in the program
			cp_type cart_pos;
			cp_type orientation;

		public:
			ImpedanceController6DOF(
				const std::string &sysName = "ImpedanceController6DOF") : System(sysName), KinematicsInput<DOF>(this), CpInput(this), CvInput(this),
																		  OrnReferenceInput(this), OrnInput(this), OrnKpGains(this), OrnKdGains(this),
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

			cp_type Xd;
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

			virtual void operate()
			{
				Xcurr = CpInput.getValue(); // current Cart. Pose
				Vcurr = CvInput.getValue(); // current Cart. Velocity
				OrnCurr = OrnInput.getValue();    // current tool Orientation (quaternion)

				Xd = XdInput.getValue(); // Xd : cartesian positional input to the system
				Kx = KxInput.getValue(); // Kx : cartesian stiffness input to the system
				Dx = DxInput.getValue(); // Dx : cartesian damping input to the system

				OrnXd = OrnReferenceInput.getValue();      // OrnXd : orientation (qct = this->OrnReferenceInput.getValue().inverse() * (tempVect);uaternion) input to the system
				OrnKx = OrnKpGains.getValue(); // OrnKx : orientation stiffness input to the system
				OrnDx = OrnKdGains.getValue(); // OrnDx : orientation damping input to the system

				for (int i = 0; i < 3; i++)
				{
					cf[i] = (Xd[i] - Xcurr[i]) * Kx[i] + (0.0 - Vcurr[i]) * Dx[i];
				}
				computedF = cf;
				computedT = ct;
				cart_pos = Xcurr;

				// Orientation error
				if (OrnXd.coeffs().dot(OrnCurr.coeffs()) < 0.0)	{
					OrnCurr.coeffs() << -OrnCurr.coeffs();
				}
				//error = OrnCurr * OrnXd.inverse(); // but CD's math (that works, BTW) does it this way
				error = OrnXd * OrnCurr.inverse(); 

				if (math::abs(error.angle()) > 3.13)
				{ // a little dead-zone near the discontinuity at +/-180 degrees
					ct.setZero();
				}
				else
				{
					tempVect = OrnKx; // copy the ProportiocalGains
					errorVect = error.axis() * error.angle();
					gsl_vector_mul(tempVect.asGslType(), errorVect.asGslType()); // tempVect <- tempVect * errorVect
					ct = this->OrnReferenceInput.getValue().inverse() * (tempVect);
					//ct = tempVect;
				}

				tempVect2 = OrnDx;
				gsl_vector_mul(tempVect2.asGslType(), this->kinInput.getValue().impl->tool_velocity_angular);
				ct -= tempVect2;

				this->ctOutputValue->setData(&ct);
				this->cfOutputValue->setData(&cf);
			}

		private:
			DISALLOW_COPY_AND_ASSIGN(ImpedanceController6DOF);
		};
	}
}
