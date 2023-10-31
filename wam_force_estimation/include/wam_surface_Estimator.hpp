/*
 * surface_estimator.h
 *
 * Created on: October 2023
 * Author: Faezeh
 */

#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>
#include <ros/ros.h>

using namespace barrett;

template <size_t DOF>
class SurfaceEstimator : public systems::System {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
    // Inputs
    Input<cp_type> cpInput;               // WAM cartesian position input
    Input<cf_type> cfInput;               // WAM cartesian force normal input
    Input<math::Matrix<3, 3>> rotInput;  // WAM world-to-tool rotation matrix

    // Outputs
    Output<cp_type> P1;  // Position direction
    Output<cp_type> P2;  // Force direction
    Output<cf_type> P3;  // Position vector

protected:
    typename Output<cp_type>::Value* P1Value;
    typename Output<cp_type>::Value* P2Value;
    typename Output<cf_type>::Value* P3Value;

public:
    math::Matrix<3, 3> p;

    explicit SurfaceEstimator(const std::string& sysName = "SurfaceEstimator") :
        System(sysName), cpInput(this), cfInput(this), rotInput(this), P1(this, &P1Value), P3(this, &P3Value), P2(this, &P2Value) {}

    virtual ~SurfaceEstimator() { this->mandatoryCleanUp(); }

protected:
    double t;
    cp_type p3;
    cp_type p1, p1n;
    cf_type p2, p2n;
    math::Matrix<3, 3> R;
    Eigen::Matrix<double, 3, 2> cps;  // Cartesian position, current and previous
    bool contacted_first_time = true;
    double reach;

    virtual void operate() {
        // Taking force and position values from the input
        p2 = this->cfInput.getValue();
        p2n = p2.normalized();

        if (p2.norm() > 15) {
            if (contacted_first_time) {
                ROS_INFO_STREAM("Contact detected");
                cps.col(0) = this->cpInput.getValue();
                contacted_first_time = false;
            }

            cps.col(1) = this->cpInput.getValue();
            p1 = cps.col(1) - cps.col(0);
            p1n = p1.normalized();

            if (p1.norm() > 0.05) {
                cps.col(0) = this->cpInput.getValue();
                R = this->rotInput.getValue();
            
                p2 = R * p2n;  // Transforming from world to tool

                p3 = p1n.cross(p2);

                p.col(0) = p1n;
                p.col(1) = p3;
                p.col(2) = p2;

                P1Value->setData(&p1n);
                P2Value->setData(&p3);
                P3Value->setData(&p2);
            }
        }
    }

private:
    DISALLOW_COPY_AND_ASSIGN(SurfaceEstimator);
};