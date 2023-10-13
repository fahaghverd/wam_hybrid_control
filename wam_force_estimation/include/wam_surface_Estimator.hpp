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
	Output<cp_type> P1;    //  position dir 
    Output<cf_type> P2;    // force dir
    Output<cp_type> P3; 

protected:
	typename Output<cp_type>::Value* P1Value;
    typename Output<cf_type>::Value* P2Value;
    typename Output<cp_type>::Value* P3Value;
public:
    math::Matrix<3,3> p;

public:
	explicit SurfaceEstimator(const std::string& sysName = "SurfaceEstimator"):
		System(sysName), cpInput(this), cfInput(this), rotInput(this), P1(this, &P1Value), P3(this, &P3Value), P2(this, &P2Value){}

	virtual ~SurfaceEstimator() { this->mandatoryCleanUp(); }

protected:
    cp_type p3;
    cp_type pre_cp, p1,p1n;
    cf_type p2, p2n;
    math::Matrix<3,3> R;
    Eigen::Matrix<double, 3, 2> cps; //cartesian position, current and previous

    virtual void operate() {
        /*Taking force and position values from the input*/
        cps.col(0) = cps.col(1);
        cps.col(1) = this->cpInput.getValue(); 
        p1 =  cps.col(1) - cps.col(0);
        p1n = p1.normalized();
        p2 = this->cfInput.getValue();
        R = this->rotInput.getValue();
        p2n = p2.normalized();
        p2 = R * p2n; //transforming from world to tool
        
        p3 = p1n.cross(p2);

        p.col(0) = p1n;
        p.col(1) = p2;
        p.col(2) = p3;

        P1Value->setData(&p1n);
        P2Value->setData(&p2);
        P3Value->setData(&p3);
    }

private:
	DISALLOW_COPY_AND_ASSIGN(SurfaceEstimator);
};