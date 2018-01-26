#include "EngineSpeedVirtualStepFunction.h"

EngineSpeedVirtualStepFunction::EngineSpeedVirtualStepFunction(map<int,int> _dc, Engine _engine)
{
	dc = _dc;
	engine = _engine;

	minEngineSpeed = (*dc.begin()).first;
	maxEngineSpeed = (*dc.rbegin()).first;

	int num = 0;
	for (auto e:dc) {
		num += e.second;
	}

	double currentVirtualStep  = 0.0;
	for (auto e:dc) {
		leftVirtualStep_map[e.first] = currentVirtualStep;

		if (e.first == maxEngineSpeed) {
			currentVirtualStep = 1.0;
		}
		else 
			currentVirtualStep += 1.0*e.second/num;
		
		rightVirtualStep_map[e.first] = currentVirtualStep;

		engineSpeed_map[currentVirtualStep] = e.first;
	}
}

double EngineSpeedVirtualStepFunction::getLeftVirtualStep(int speed)
{
	if (leftVirtualStep_map.find(speed)==leftVirtualStep_map.end()) {
		cerr << "No speed = " << speed << " rpm in the driving cycle!" << endl;
		Utility::output_one_map(cerr,"DC",dc);
		exit(EXIT_FAILURE);
	}
	return leftVirtualStep_map[speed];
}

double EngineSpeedVirtualStepFunction::getRightVirtualStep(int speed)
{
	if (rightVirtualStep_map.find(speed)==rightVirtualStep_map.end()) {
		cerr << "No speed = " << speed << " rpm in the driving cycle!" << endl;
		Utility::output_one_map(cerr,"DC",dc);
		exit(EXIT_FAILURE);
	}
	return rightVirtualStep_map[speed];
}

double EngineSpeedVirtualStepFunction::getIntervalVirtualStep(int speed)
{
	return getRightVirtualStep(speed) - getLeftVirtualStep(speed);
}

double EngineSpeedVirtualStepFunction::getPreviousVirtualStep(double virtualStep)
{
	int speed = getEngineSpeed(virtualStep);
	if (speed < 0) return -1.0;

	map<int,double>::const_iterator iter = rightVirtualStep_map.lower_bound(speed);
	if (iter != rightVirtualStep_map.end()) {
		if (fabs(iter->second-rightVirtualStep_map.begin()->second)<=Utility::EPSILON)
			return -1.0;
		else
			iter--;
		return iter->second;
	}
	else
		return -1.0;
}

int EngineSpeedVirtualStepFunction::getEngineSpeed(double virtualStep)
{
	if (virtualStep < 0.0 || virtualStep > 1.0)
		return -1;

	map<double,int>::const_iterator iter = engineSpeed_map.lower_bound(virtualStep);
	if (iter == engineSpeed_map.end()) {
		cerr << "Cannot find the lower bound of " << virtualStep << " in engine_speed map!" << endl;
		Utility::output_one_map(cerr,"engineSpeed_map",engineSpeed_map);
		exit(EXIT_FAILURE);
	}

	return iter->second;
}

int EngineSpeedVirtualStepFunction::getModeNumber(vector<double> virtualSteps)
{
	int num = 0;
	for (auto e : virtualSteps) {
		if (e > 0) num ++;
	}
	return num;
}

std::vector<double> EngineSpeedVirtualStepFunction::getInitialVirtualSteps(vector<double> ubSpeeds)
{
	vector<double> initialVirtualSteps;
	for(int i=0; i<ubSpeeds.size()-1; i++) {
		double ubSpeed = ubSpeeds[i];

		if (ubSpeed < minEngineSpeed) {
			initialVirtualSteps.push_back(-1.0);
			continue;
		}

		if (ubSpeed >= maxEngineSpeed) {
			if (i==0) initialVirtualSteps.push_back(1.0);
			else {
				double prevSpeed = ubSpeeds[i-1];
				if (prevSpeed > maxEngineSpeed) initialVirtualSteps.push_back(-1.0);
				else initialVirtualSteps.push_back(1.0);
			}
			continue;
		}

		map<int,double>::const_iterator iter = rightVirtualStep_map.upper_bound(ubSpeed);
		
		iter--;
		if (i==0) initialVirtualSteps.push_back(iter->second);
		else {
			double prevSpeed = ubSpeeds[i-1];
			if (iter->first > prevSpeed) initialVirtualSteps.push_back((iter->second));
			else initialVirtualSteps.push_back(-1.0);
		}
	}

	// the simplest control strategy should be added for guarantee the schedulability
	initialVirtualSteps.push_back(2.0);

	return initialVirtualSteps;
}

std::vector<std::vector<double>> EngineSpeedVirtualStepFunction::getVirtualStepsVector(vector<double> ubSpeeds)
{
	vector<vector<double>> ret;

	vector<double> initialVirtualSteps = getInitialVirtualSteps(ubSpeeds);
	ret.push_back(initialVirtualSteps);

	int size = ubSpeeds.size();
	int currModeNum = getModeNumber(initialVirtualSteps);

	// First: left remove the control settings with the less mode number than current mode number
	for (int reducedNum = 1; reducedNum < currModeNum; reducedNum ++) {
		vector<double> currentVirtualSteps = initialVirtualSteps;

		int tempNum = 0;
		for (auto &e : currentVirtualSteps) {
			if (e > 0) {
				e = -1.0;
				tempNum ++;
			}
			if (tempNum == reducedNum) break;
		}

		if (checkReasonable(initialVirtualSteps,currentVirtualSteps))
			ret.push_back(currentVirtualSteps);
	}

	// Second, move to the less advanced control implementation
	for (int i=0; i<initialVirtualSteps.size()-1; i++) {
		if (initialVirtualSteps[i] < 0) continue;
		if (initialVirtualSteps[i+1] > 0) continue;

		double prev = getPreviousVirtualStep(initialVirtualSteps[i]);
		if (prev > 0 && i > 0)
			if (fabs(initialVirtualSteps[i-1]-prev) <= Utility::EPSILON) 
				prev = -1.0;

		//exchange the two elements, i and i+1
		vector<double> currentVirtualSteps = initialVirtualSteps;
		currentVirtualSteps[i] = prev;
		currentVirtualSteps[i+1] = initialVirtualSteps[i];

		if (checkReasonable(initialVirtualSteps,currentVirtualSteps))
			ret.push_back(currentVirtualSteps);
	}

	// Third, add the middle control implementations
	vector<double> addedVirtualSteps = initialVirtualSteps;
	bool found = false;
	for (int i=addedVirtualSteps.size()-2; i >= 0; i--) {
		if (!found && fabs(addedVirtualSteps[i]-1.0) <= Utility::EPSILON) {
			found = true;
			continue;
		}

		if (found) {
			addedVirtualSteps[i] = getPreviousVirtualStep(addedVirtualSteps[i+1]);
		}
	}
	if (checkReasonable(initialVirtualSteps,addedVirtualSteps))
		ret.push_back(addedVirtualSteps);

	return ret;
}

std::vector<double> EngineSpeedVirtualStepFunction::getNextVirtualSteps(vector<double> currentVirtualSteps, vector<double> reducedVirtualSteps)
{
	vector<double> nextVirtualSteps;

	vector<double> currentEngineSpeeds = getEngineSpeeds(currentVirtualSteps);

	while(true) {
		for (int i=0; i<currentVirtualSteps.size()-1; i++) {
			double nextVirtualStep = currentVirtualSteps[i]-reducedVirtualSteps[i];
			if (nextVirtualStep < 0) nextVirtualStep = -1.0;
			nextVirtualSteps.push_back(nextVirtualStep);
		}
		nextVirtualSteps.push_back(2.0);
		
		vector<double> nextEngineSpeeds = getEngineSpeeds(nextVirtualSteps);

		bool hasChanged = false;
		for (int i=0; i<currentEngineSpeeds.size(); i++) {
			if (fabs(currentEngineSpeeds[i] - nextEngineSpeeds[i]) > Utility::EPSILON) {
				hasChanged = true;
				break;
			}
		}

		if (hasChanged) break;

		/*
		cout << "1" << endl;
		Utility::output_one_vector(cout,"current",currentEngineSpeeds);
		Utility::output_one_vector(cout,"next",nextEngineSpeeds);
		Utility::output_one_vector(cout,"current",currentVirtualSteps);
		Utility::output_one_vector(cout,"next",reducedVirtualSteps);
		*/
		

		// Iteratively reduce the virtual steps
		currentVirtualSteps = nextVirtualSteps;
		nextVirtualSteps.clear();
	}

	// enforce constraint
	while (1) {
		bool found = false;
		for (int i =  nextVirtualSteps.size()-2; i > 0; i--) {
			if (nextVirtualSteps[i] < 0) continue;
			if(nextVirtualSteps[i] < nextVirtualSteps[i-1]) {
				nextVirtualSteps[i] = nextVirtualSteps[i-1] + 0.01;
				found = true;
			}
		}

		if (!found) break;
		//cout << "2" << endl;
	}

	return nextVirtualSteps;
}

std::vector<double> EngineSpeedVirtualStepFunction::getEngineSpeeds(vector<double> virtualSteps)
{
	vector<double> speeds;

	for (int i=0; i<virtualSteps.size(); i++) {
		if (i==virtualSteps.size()-1) {
			speeds.push_back(engine.RPM_MAX);
			break;
		}

		double virtualStep = virtualSteps[i];
		if (virtualStep < 0) speeds.push_back(0); // Do not use this mode
		else speeds.push_back(getEngineSpeed(virtualStep));
	}

	// Enforce constraint
	for (int i =  speeds.size()-2; i > 0; i--) {
		if (fabs(speeds[i]) <= Utility::EPSILON)
			continue;

		if(speeds[i] - 1.0 < speeds[i-1]) {
			speeds[i] = speeds[i-1] + 1.0;
		}
	}

	return speeds;
}

bool EngineSpeedVirtualStepFunction::checkOnlyOneTwoModes(vector<int> engineSpeeds)
{
	int modeNum = 0;
	for (auto e: engineSpeeds) {
		if (e > 0) modeNum ++;
	}
	if (modeNum <= 2) return true;
	else false;
}

bool EngineSpeedVirtualStepFunction::checkReasonable(vector<double> ubVirtualSteps, vector<double> currVirtualSteps)
{
	for (int i=0; i<ubVirtualSteps.size()-1; i++) {
		if (ubVirtualSteps[i] > 0) {
			if (ubVirtualSteps[i] < currVirtualSteps[i])
				return false;
		}
	}
	return true;
}
