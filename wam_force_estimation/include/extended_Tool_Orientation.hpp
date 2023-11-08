#pragma once

#include <iostream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <gsl/gsl_matrix.h>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>
#include <barrett/systems/kinematics_base.h>


namespace barrett {
namespace systems {


// yields a quaternion representing the rotation from the world frame to the tool frame.
template<size_t DOF>
class ExtendedToolOrientation : public System, public KinematicsInput<DOF>,
						        public SingleOutput<math::Matrix<3,3>> {
public:
	ExtendedToolOrientation(const std::string& sysName = "ExtendedToolOrientation") :
		System(sysName), KinematicsInput<DOF>(this),
		SingleOutput<math::Matrix<3,3>>(this){}
	virtual ~ExtendedToolOrientation() { mandatoryCleanUp(); }

protected:
	gsl_matrix * r;
	math::Matrix<3,3> rot, R;

	virtual void operate() {
		r = this->kinInput.getValue().impl->tool->rot_to_world;
		// Convert GSL matrix to Eigen matrix
		for (size_t row = 0; row < 3; ++row) {
			for (size_t col = 0; col < 3; ++col) {
				// Access the element in the GSL matrix and copy it to the math::Matrix
				double value = gsl_matrix_get(r, row, col);
				rot(row, col) = value;
			}
		}
		
		R = rot.transpose(); // Transpose to get world-to-tool rotation
		//std::cout<<R<<std::endl;
		this->outputValue->setData(&R);
	}

	

private:
	DISALLOW_COPY_AND_ASSIGN(ExtendedToolOrientation);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}
