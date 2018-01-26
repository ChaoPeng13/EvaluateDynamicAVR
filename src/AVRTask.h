#ifndef AVRTASK_H_
#define AVRTASK_H_

#include "Config.h"

#include <map>
#include <vector>
#include <set>

#ifdef LINUX_PLATFORM
#include <cmath>
#include <climits>
#endif
//#include <windows.h>

#include "Engine.h"
#include "Utility.h"
//#include "CombinedSpeedNodeAndEdge.h"
#include "RequestFunction.h"
#include "Definitions.h"
#include "Task.h"
#include "EngineSpeedVirtualStepFunction.h"
#include "Timer.h"
#include "Digraph.h"

#ifdef __USING_ILPCPLEX__
#include <ilcplex/ilocplex.h>
#endif

#ifdef __DEBUG__
#include <assert.h>
#endif

using namespace std;

class AVRTask;
typedef AVRTask* AVRTaskPointer;

class AVRTask : public Task {
public:
	Engine engine;
	double period;
	int numMode;
	map<int,double> speeds; // in rpm
	map<int,double> omegas; // in rev/msec
	map<int,int> wcets; // in musec
	map<int,int> modePeriods; // in musec
	map<int,double> modeUtils; 

	Digraph* digraph;

	double _maxU;
	double _maxC;

	map<int,map<int,map<int,int>>> rbfRecord;  // 6e-7 * rev/msec, 10 * musec, 10 * musec 
	map<int,map<int,map<int,int>>> ibfRecord;  // 6e-7 * rev/msec, 10 * musec, 10 * musec
	map<int,map<int,map<int,int>>> ibfs;       // 6e-7 * rev/msec, 10 * musec, 10 * musec
	map<int,vector<double>> dominantsRecord;      // 6e-7 * rev/msec

	map<int,map<int,vector<map<int,int>>>> rfRecord; // 6e-7 * rev/msec, 10 * musec, 10 * musec

	vector<double> initialDominants;
	map<double, map<int,int>> digraph_rbf;
	map<int,int> digraph_rbf_envelope;

#ifdef __USING_ILPCPLEX__
	map<int,int> ILPs; // (interval time (us), ilp (us)): record the ILP value for the one AVR task
	map<int,map<int,int>> ILPCONs; // (initial speed (rpm), interval time (us), ilpcon (us)): record the ILPCON value for the one AVR task
#endif
	
	AVRTask(Engine _engine, double _period, vector<int> _wcets) {
		engine = _engine;
		period = _period;
		numMode = _wcets.size();
		
		for (int i=0; i<numMode; i++) {
			wcets[i] = _wcets[i];
		}

		_maxU = -1;
		_maxC = -1;

		digraph = NULL;
	}

	AVRTask(Engine _engine, double _period, vector<double> _speeds) {
		engine = _engine;
		period = _period;

		vector<double> _realSpeeds;
		for (int i=0; i<_speeds.size(); i++) {
			double _speed = _speeds[i];
			if (_speed>0) {
				_realSpeeds.push_back(_speed);
			}
		}

		numMode = _realSpeeds.size();

		for (int i=0; i<numMode; i++) {
			speeds[i] = _realSpeeds[i];
			omegas[i] =  RPM_to_RPmSEC(_realSpeeds[i]);
		}

		_maxU = -1;
		_maxC = -1;

		digraph = NULL;
	}

	AVRTask(Engine _engine, double _period, vector<double> _speeds, vector<int> _wcets) {
		engine = _engine;
		period = _period;

		#ifdef __DEBUG__
		assert(_speeds.size() == _wcets.size());
		#endif

		vector<double> _realSpeeds;
		vector<int> _realWCETs;

		for (int i=0; i<_speeds.size(); i++) {
			double _speed = _speeds[i];
			if (_speed>0) {
				_realSpeeds.push_back(_speed);
				_realWCETs.push_back(_wcets[i]);
			}
		}

		numMode = _realWCETs.size();

		for (int i=0; i<numMode; i++) {
			speeds[i] = _realSpeeds[i];
			omegas[i] =  RPM_to_RPmSEC(_realSpeeds[i]);
			wcets[i] = _realWCETs[i];
		}

		modePeriods = getModePeriods();
		modeUtils = getModeUtilizations();

		_maxU = -1;
		_maxC = -1;

		digraph = NULL;
	}

	AVRTask(Engine _engine, double _period, vector<AVRTask> _avrTasks) {
		engine = _engine;
		period = _period;

		vector<int> _wcets;
		vector<double> _speeds;

		for (auto _avrTask:_avrTasks) {
			for (auto e:_avrTask.speeds)
				_speeds.push_back(e.second);
		}

		sort(_speeds.begin(),_speeds.end());
		_speeds.erase(unique(_speeds.begin(),_speeds.end()),_speeds.end());

#if 1
		int prevWCET = -1;
		vector<double> temp;

		for (auto _speed : _speeds) {
			int _wcet = INT_MIN;
			for (auto _avrTask : _avrTasks) 
				_wcet = max(_wcet,_avrTask.getWCET_RPM(_speed));

			if (prevWCET == _wcet) {
				temp[temp.size()-1] = _speed;
			}
			else {
				temp.push_back(_speed);
				_wcets.push_back(_wcet);
			}
			prevWCET = _wcet;
		}

		_speeds = temp;
#else
		for (auto _speed : _speeds) {
			int _wcet = INT_MIN;
			for (auto _avrTask : _avrTasks) 
				_wcet = max(_wcet,_avrTask.getWCET_RPM(_speed));
			
			_wcets.push_back(_wcet);
		}
#endif

		


#ifdef __DEBUG__
		assert(_speeds.size() == _wcets.size());
		Utility::output_one_vector(cout,"speeds",_speeds);
		Utility::output_one_vector(cout,"WCETs",_wcets);
#endif

		vector<double> _realSpeeds;
		vector<int> _realWCETs;

		for (int i=0; i<_speeds.size(); i++) {
			double _speed = _speeds[i];
			if (_speed>0) {
				_realSpeeds.push_back(_speed);
				_realWCETs.push_back(_wcets[i]);
			}
		}

		numMode = _realWCETs.size();

		for (int i=0; i<numMode; i++) {
			speeds[i] = _realSpeeds[i];
			omegas[i] =  RPM_to_RPmSEC(_realSpeeds[i]);
			wcets[i] = _realWCETs[i];
		}

		modePeriods = getModePeriods();
		modeUtils = getModeUtilizations();

		_maxU = -1;
		_maxC = -1;

		digraph = NULL;
	}

	AVRTask(EngineSpeedVirtualStepFunction esvsf, Engine _engine, double _period, vector<double> _virtualSteps, vector<int> _wcets) {
		engine = _engine;
		period = _period;
		
		vector<double> _speeds = esvsf.getEngineSpeeds(_virtualSteps);

		vector<double> _realSpeeds;
		vector<int> _realWCETs;

		for (int i=0; i<_speeds.size(); i++) {
			double _speed = _speeds[i];
			if (_speed>0) {
				_realSpeeds.push_back(_speed);
				_realWCETs.push_back(_wcets[i]);
			}
		}

		numMode = _realWCETs.size();

		for (int i=0; i<numMode; i++) {
			speeds[i] = _realSpeeds[i];
			omegas[i] =  RPM_to_RPmSEC(_realSpeeds[i]);
			wcets[i] = _realWCETs[i];
		}

		modePeriods = getModePeriods();
		modeUtils = getModeUtilizations();

		_maxU = -1;
		_maxC = -1;

		digraph = NULL;
	}
	/*
	~AVRTask(void) {
	}
	*/
	void updateSpeeds(vector<double> _speeds);
	void updateReducedSpeeds(vector<double> reducedSpeeds);
	void updateReducedSpeeds(map<int,int> dc, vector<double> reducedSpeeds);

	void prepareLinearUppperBound();

	void scaleWCETs(int factor);

	int maxN(double omega); 
	double maxM(double omega);

	int getModeNumberWithNonZero();

	vector<double> getDominants(double omega_b, double omega_a, double timeWindowLength=-1);
	vector<double> getAllDominants();
	void collectSubDominants(double omega, vector<double>& allDominants);
	bool alreadyStored(double omega, vector<double> dominants);
	bool descendingSorting(const double&a, const double& b) {return a>b; }

	/// ECRTS2017, Refinement of Workload Models for Engine Controllers by State Space Partitioning
	vector<double> getAllSpeedRanges();
	vector<double> getSpeedPartitionWithGranularity(int partitionGranularity);
	
	void buildDigraph();
	void buildSimpleODRT();
	void buildComplexODRT();

	map<int,double> collectInfo(double omega, double t);
	map<int,double> collectInfo(vector<double> dominants, double t);

	bool checkMode(double omega);

	int getWCET(double omega);
	int getWCET_RPM(double rpm);
	double getOmega(double omega);
	int getOmega_RPM(double rpm);
	int getModeNum(double omega);

	double calDeadline(double omega);

	map<int,int> getRecord(map<int,map<int,map<int,int>>> &record, int rpm, int t);
	void storeRBFRecord(map<int,map<int,map<int,int>>> &record,int rpm, int t, map<int,int> points);
	map<int,int> combineRBFPoints(map<int,int> m1, map<int,int> m2);
	map<int,int> createRBFPoints(map<int,int> m1, int tNext, int wcet);

	void storeIBFRecord(map<int,map<int,map<int,int>>> &record,int rpm, int t, map<int,int> points);
	map<int,int> combineIBFPoints(map<int,int> m1, map<int,int> m2);
	map<int,int> createIBFPoints(map<int,int> m1, int tNext, int wcet);

	vector<map<int,int>> getRecord(map<int,map<int,vector<map<int,int>>>> &record, int rpm, int t);
	void storeRecord(map<int,map<int,vector<map<int,int>>>> &record,int rpm, int t, vector<vector<map<int,int>>> points);
	vector<map<int,int>> createPoints(vector<map<int,int>> m1, int tNext, int wcet);
	
	map<int,int> getLastRecord(map<int,map<int,map<int,int>>> record,int rpm);

	map<int,int> calRequestBoundFunction(double omega, int t);
	map<int,int> calRequestBoundFunction(double omega, int t, vector<OmegaPoint*>& points);
	static int getRequestBoundFunction(map<int,int> record, int t);

	map<int,int> calInterferenceBoundFunction(double omega, int t);
	map<int,int> getInterferenceBoundFunction(double omega, map<int,int> points, int t);
	static int getInterferenceBoundFunction(map<int,int> record, int t);

	double getLinearUpperBound(double t);
	double getResponseTimeWithWCET(double wcet);

	map<int,int> getModePeriods();
	map<int,double> getModeUtilizations();

#ifdef __USING_ILPCPLEX__
	int calILP(int t);
#endif

	int calAJLessThan(double omega, int objMode, int t);
	int calAJEqualTo(double omega, int objMode, int t);
	int calAJGreaterThan(double omega, int objMode, int t);
	int calAMax(double omega, int t);
	int calRhoMinus(int middleMode);
	int calRhoAdd(int middleMode);

#ifdef __USING_ILPCPLEX__
	int calILPCON(double omega, int t);
#endif

	double calHighestSpeed(double omega, int t);
	double calLowestSpeed(double omega, int t);

	vector<map<int,int>> generateRequestFunctions(double omega, int t);

	int getJobSequencesNumber(int maxTime);
	int getJobSequencesNumberNO1(int maxTime);
	int getJobSequencesNumberNO2(int maxTime);
	int getJobSequencesNumberNO3(int maxTime);

	int getJobSequencesNumber(int maxTime, double omega, int sumPeriod);
	int getJobSequencesNumberBinary(int maxTime, double omega, int sumPeriod);

	bool checkExist(vector<vector<map<int,int>>> record, vector<map<int,int>> points);

	void output(ostream &out);
	void outputWCETs(ostream &out);
	void outputSpeeds(ostream &out);
	void outputDomiants(ostream &out, vector<double> dominants);
	void outputRequestFunctions(ostream &out, double omega, int t);
	
	void clear(map<int,map<int,map<int,int>>> &m);
	void clearAll();
};

#endif // AVRTask.h