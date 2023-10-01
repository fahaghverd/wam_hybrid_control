class ExtendedRamp : public systems::Ramp {
public:
    using Ramp::Ramp;  // Inherit constructors from Ramp
	
    double getYValue() const {
        return y;
    }
};
