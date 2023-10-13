/*
 *  Created on: Oct, 2023
 *      Author: Faezeh
 */
 
#pragma once


#include <barrett/units.h>
#include <barrett/systems.h>

using namespace barrett;

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
