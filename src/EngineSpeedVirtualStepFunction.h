#ifndef ENGINESPEEDVIRTUALSTEPFUNCTION_H_
#define ENGINESPEEDVIRTUALSTEPFUNCTION_H_

#include "Engine.h"

class EngineSpeedVirtualStepFunction {
public:
	int minEngineSpeed;
	int maxEngineSpeed;
	map<int,int> dc;
	Engine engine;

	map<int,double> leftVirtualStep_map; // engine speed - left virtual step map
	map<int,double> rightVirtualStep_map; // engine speed - right virtual step map
	map<double, int> engineSpeed_map; // virtual step - engine speed map

	EngineSpeedVirtualStepFunction(map<int,int> _dc, Engine _engine);

	double getLeftVirtualStep(int speed);
	double getRightVirtualStep(int speed);
	double getIntervalVirtualStep(int speed);
	double getPreviousVirtualStep(double virtualStep);

	int getEngineSpeed(double virtualStep);
	int getModeNumber(vector<double> virtualSteps);
	vector<double> getInitialVirtualSteps(vector<double> ubSpeeds);
	vector<vector<double>> getVirtualStepsVector(vector<double> ubSpeeds);
	vector<double> getNextVirtualSteps(vector<double> currentVirtualSteps, vector<double> reducedVirtualSteps);
	vector<double> getEngineSpeeds(vector<double> virtualSteps);
	bool checkOnlyOneTwoModes(vector<int> engineSpeeds);
	bool checkReasonable(vector<double> ubVirtualSteps, vector<double> currVirtualSteps);
};

#endif // !ENGINESPEEDVIRTUALSTEPFUNCTION_H_


