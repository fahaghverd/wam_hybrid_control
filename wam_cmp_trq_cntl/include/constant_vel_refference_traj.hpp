#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>

template<size_t DOF, typename Tp, typename Tv, typename Ta>
class constVelRefTrajectory:  public systems::System
 {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	// IO
	Input<double> timef;
	Output<Tp> referencePTrack;
	Output<Tv> referenceVTrack;
	Output<Ta> referenceATrack;

protected:
	typename System::Output<Tp>::Value* referencePOpValue;
	typename System::Output<Tv>::Value* referenceVOpValue;
	typename System::Output<Ta>::Value* referenceAOpValue;

public:
	explicit constVelRefTrajectory(Tp start_pose, Tv v_desired, const std::string& sysName = "constVelRefTrajectory"):
		System(sysName), timef(this), referencePTrack(this, &referencePOpValue), referenceVTrack(this, &referenceVOpValue),
		referenceATrack(this, &referenceAOpValue), start_pose(start_pose), v_des(v_desired){}

	virtual ~constVelRefTrajectory() {this->mandatoryCleanUp();}

	Tp referenceP;
protected:
	Tp start_pose, refPTrack;
	Tv v_des, refVTrack;
	Ta refATrack;

	virtual void operate() {
		refPTrack = v_des * this->timef.getValue() + start_pose;
		refVTrack = v_des;

		referenceP = refPTrack;

		this->referencePOpValue->setData(&refPTrack);
		this->referenceVOpValue->setData(&refVTrack);
		this->referenceAOpValue->setData(&refATrack);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(constVelRefTrajectory);
};
