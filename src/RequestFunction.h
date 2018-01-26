#ifndef REQUESTFUNCTION_H_
#define REQUESTFUNCTION_H_

#include "Utility.h"

class OmegaPoint {
public:
	double releaseTime;
	double omega; // in rev/msec

	OmegaPoint(double _releaseTime, double _omega) {
		releaseTime = _releaseTime;
		omega = _omega;
	}
};

class RequestFunction {
public:
	double initialOmega; // in rev/msec
	vector<OmegaPoint*> points;

	RequestFunction(double _initialOmega) {
		initialOmega = _initialOmega;
	}

	RequestFunction(RequestFunction* rf, OmegaPoint* point) {
		initialOmega = rf->initialOmega;
		for (vector<OmegaPoint*>::iterator iter = rf->points.begin(); iter != rf->points.end(); iter ++) {
			points.push_back(new OmegaPoint((*iter)->releaseTime,(*iter)->omega));
		}
		points.push_back(point);
	}

	~RequestFunction() {
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++)
			delete *iter;
	}

	void addPoint(OmegaPoint* point) {
		points.push_back(point);
	}

	int getValue(int t, double omegas[], int wcets[], int numMode) {
		if ( t <=0 ) return 0;

		int rf = 0;
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			if (point->releaseTime < t) 
				rf += Utility::getWCET(point->omega, omegas,wcets,numMode);
		}

		return rf;
	}

	map<int,double> collectInfo(double omegas[], int wcets[], int numMode) {
		map<int,double> ret;

		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			int wcet = Utility::getWCET(point->omega, omegas,wcets,numMode);
			if (ret.find(wcet) != ret.end()) {
				ret[wcet] = max(ret[wcet],point->omega);
			}
			else {
				ret[wcet] = point->omega;
			}
		}

		return ret;
	}

	void output(ostream& out,double omegas[], int wcets[], int numMode) {
		out << "Times=[";
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			out << point->releaseTime;
			if (iter != --points.end()) out << ",";
		}
		out << "]" << endl;

		out << "Omegas=[";
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			out << point->omega;
			if (iter != --points.end()) out << ",";
		}
		out << "]" << endl;

		out << "Speeds=[";
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			out << 60000*point->omega;
			if (iter != --points.end()) out << ",";
		}
		out << "]" << endl;

		out << "WCETs=[";
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			int wcet = Utility::getWCET(point->omega, omegas,wcets,numMode);
			out << wcet;
			if (iter != --points.end()) out << ",";
		}
		out << "]" << endl;

		out << "RF=[";
		int rf = 0;
		for (vector<OmegaPoint*>::iterator iter = points.begin(); iter != points.end(); iter++) {
			OmegaPoint* point = *iter;
			int wcet = Utility::getWCET(point->omega, omegas,wcets,numMode);
			rf += wcet;
			out << rf;
			if (iter != --points.end()) out << ",";
		}
		out << "]" << endl;
	}
};

#endif