/*
 * wam_surface_estimator.cpp
 *
 * 
 *  Created on: Oct., 2023
 *      Author: Faezeh
 */


#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>

using namespace barrett;

template<size_t DOF>
class SurfaceEstimator :  public systems::System
{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:     
    Input<cp_type> cpInput; //WAM cartesian position input
    Input<cf_type> cfInput; //wam cartesian force normal input
    Input<math::Matrix<3,3>> rotInput; //WAM world-to-tool rotation matrix 

// IO  (outputs)
public:
	Output<math::Matrix<3,3>> P;    // P matrix including position dir, force dir, and surface normal

protected:
	typename Output<math::Matrix<3,3>>::Value* PValue;

public:
    Eigen::Vector3d P3;

public:
	explicit SurfaceEstimator(const std::string& sysName = "SurfaceEstimator"):
		System(sysName), cpInput(this), cfInput(this), rotInput(this), P(this, &PValue){}

	virtual ~SurfaceEstimator() { this->mandatoryCleanUp(); }

protected:
    cp_type pre_cp, P1;
    cf_type P2;
    
    math::Matrix<3,3> R, p;
    Eigen::Matrix<double, 3, 2> cps; //cartesian position, current and previous

    virtual void operate() {
        /*Taking force and position values from the input*/
        cps.col(0) = cps.col(1);
        cps.col(1) = this->cpInput.getValue(); 
        P1 =  cps.col(1) - cps.col(0);
        P2 = this->cfInput.getValue();
        P2 = R * P2; //transforming from world to tool
        P3 = P1.cross(P2);

        p.col(0) = P1;
        p.col(1) = P2;
        p.col(2) = P3;

        PValue->setData(&p);
    }

private:
	DISALLOW_COPY_AND_ASSIGN(SurfaceEstimator);
};