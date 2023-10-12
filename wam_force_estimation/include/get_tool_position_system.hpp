/*
 *  Created on: August, 2023
 *      Author: Faezeh
 */
 
#pragma once


#include <barrett/units.h>
#include <barrett/systems.h>

using namespace barrett;

template<size_t DOF>
class getToolPosition : public System, public systems::SingleOutput<units::CartesianPosition::type>,
						public systems::KinematicsInput<DOF>
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	getToolPosition(const std::string& sysName = "getToolPosition"): System(sysName), SingleOutput<cp_type>(this), KinematicsInput<DOF>(this) {}
	virtual ~getToolPosition() {this->mandatoryCleanUp(); }

protected:
	cp_type cp;
	virtual void operate() {
		cp = cp_type(this->kinInput.getValue().impl->tool->origin_pos);
		this->outputValue->setData(&cp);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(getToolPosition);

};
