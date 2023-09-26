#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>

template<size_t DOF, typename Tp, typename Tv, typename Ta>
class regulationRefTrajectory:  public systems::System
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
	explicit regulationRefTrajectory(Tp p_desired, const std::string& sysName = "regulationRefTrajectory"):
		System(sysName), timef(this), referencePTrack(this, &referencePOpValue), referenceVTrack(this, &referenceVOpValue),
		referenceATrack(this, &referenceAOpValue), refPTrack(p_desired){}

	virtual ~regulationRefTrajectory() {this->mandatoryCleanUp();}

protected:
	Tp refPTrack;
	Tv refVTrack;
	Ta refATrack;

	virtual void operate() {
		refVTrack.Zero();
		refATrack.Zero();

		this->referencePOpValue->setData(&refPTrack);
		this->referenceVOpValue->setData(&refVTrack);
		this->referenceAOpValue->setData(&refATrack);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(regulationRefTrajectory);
};
