#ifndef INTERVAL_H_
#define INTERVAL_H_

class Interval {
public:
	double maxOmega;
	double minOmega;

	Interval(double _maxOmega, double _minOmega) {
		maxOmega = _maxOmega;
		minOmega = _minOmega;
	}

	bool included(double omega) {
		if (omega <= maxOmega && omega > minOmega)
			return true;
		return false;
	}
};

#endif