#ifndef OPTIMIZATIONALGORITHM_H_
#define OPTIMIZATIONALGORITHM_H_

#include "AVRTask.h"
#include "PeriodicTask.h"
#include "AsynchronousPeriodicTask.h"
#include "RequestFunction.h"
#include "Interval.h"
#include "Utility.h"
#include "Global.h"
#include "CollectionResult.h"
#include "EngineSpeedVirtualStepFunction.h"
#include "StateGraph.h"

#include <set>

struct my_two_params {double k1; double k2;};
struct my_three_params {double k1; double k2; double k3;};
struct my_four_params {double k1; double k2; double k3; double k4;};

enum MethodChoice {
	EXACT = 0, // exact schedulablity analysis
	RBF,
	RBF_ENVELOPE,
	DIGRAPH_RBF,
	DIGRAPH_RBF_ENVELOPE,
	IBF,
	NECESSARY_ONLY1,
	SEG_NECESSARY_ONLY1,
	SEG_LUB_NECESSARY_ONLY1,
	NECESSARY_ONLY2,
	NECESSARY_ONLY3,
	NECESSARY_ONLY1A,
	SEG_NECESSARY_ONLY1A,
	NECESSARY_ONLY2A,
	LUB,
	ILPCON,
	ILP,
	SEG_ILPCON_EXACT,
	SEG_ILPCON_DIGRAPH_RBF
};

struct RecordPair {
	double prevSpeed;
	int prevIndex;
	double speed;
	int index;
};

class OptimizationAlgorithm {
public:
	static string funcNames[19];

	static int nonSchedNum;
	static int schedNum;
	
	//=====================================================================================
	// Collect Info Start
	//=====================================================================================
	static double level0_LowerSpeeds_VerificationTimes;
	static double level0_LocalSearch_VerificationTimes;
	static double level1_LowerSpeeds_VerificationTimes;
	static double level1_LocalSearch_VerificationTimes;
	static double level2_LowerSpeeds_VerificationTimes;
	static double level2_LocalSearch_VerificationTimes;
	static double level3_LowerSpeeds_VerificationTimes;
	static double level3_LocalSearch_VerificationTimes;
	static double level4_LowerSpeeds_VerificationTimes;
	static double level4_LocalSearch_VerificationTimes;

	static double level0_LowerSpeeds_FinalCheckTimes;
	static double level1_LowerSpeeds_FinalCheckTimes;
	static double level2_LowerSpeeds_FinalCheckTimes;
	static double level3_LowerSpeeds_FinalCheckTimes;
	static double level4_LowerSpeeds_FinalCheckTimes;

	static vector<double> level0_LowerSpeeds_VerificationTimes_vec;
	static vector<double> level0_LocalSearch_VerificationTimes_vec;
	static vector<double> level1_LowerSpeeds_VerificationTimes_vec;
	static vector<double> level1_LocalSearch_VerificationTimes_vec;
	static vector<double> level2_LowerSpeeds_VerificationTimes_vec;
	static vector<double> level2_LocalSearch_VerificationTimes_vec;
	static vector<double> level3_LowerSpeeds_VerificationTimes_vec;
	static vector<double> level3_LocalSearch_VerificationTimes_vec;
	static vector<double> level4_LowerSpeeds_VerificationTimes_vec;
	static vector<double> level4_LocalSearch_VerificationTimes_vec;

	static vector<double> level0_LowerSpeeds_FinalCheckTimes_vec;
	static vector<double> level1_LowerSpeeds_FinalCheckTimes_vec;
	static vector<double> level2_LowerSpeeds_FinalCheckTimes_vec;
	static vector<double> level3_LowerSpeeds_FinalCheckTimes_vec;
	static vector<double> level4_LowerSpeeds_FinalCheckTimes_vec;

	static double level0_LowerSpeeds_Performance;
	static double level0_LocalSearch_Performance;
	static double level1_LowerSpeeds_Performance;
	static double level1_LocalSearch_Performance;
	static double level2_LowerSpeeds_Performance;
	static double level2_LocalSearch_Performance;
	static double level3_LowerSpeeds_Performance;
	static double level3_LocalSearch_Performance;
	static double level4_LowerSpeeds_Performance;
	static double level4_LocalSearch_Performance;

	static double level0_LowerSpeeds_FinalCheckPerf;
	static double level1_LowerSpeeds_FinalCheckPerf;
	static double level2_LowerSpeeds_FinalCheckPerf;
	static double level3_LowerSpeeds_FinalCheckPerf;
	static double level4_LowerSpeeds_FinalCheckPerf;

	static vector<double> level0_LowerSpeeds_Performance_vec;
	static vector<double> level0_LocalSearch_Performance_vec;
	static vector<double> level1_LowerSpeeds_Performance_vec;
	static vector<double> level1_LocalSearch_Performance_vec;
	static vector<double> level2_LowerSpeeds_Performance_vec;
	static vector<double> level2_LocalSearch_Performance_vec;
	static vector<double> level3_LowerSpeeds_Performance_vec;
	static vector<double> level3_LocalSearch_Performance_vec;
	static vector<double> level4_LowerSpeeds_Performance_vec;
	static vector<double> level4_LocalSearch_Performance_vec;

	static vector<double> level0_LowerSpeeds_FinalCheckPerf_vec;
	static vector<double> level1_LowerSpeeds_FinalCheckPerf_vec;
	static vector<double> level2_LowerSpeeds_FinalCheckPerf_vec;
	static vector<double> level3_LowerSpeeds_FinalCheckPerf_vec;
	static vector<double> level4_LowerSpeeds_FinalCheckPerf_vec;

	static double level0_LowerSpeeds_Runtime;
	static double level0_LocalSearch_Runtime;
	static double level1_LowerSpeeds_Runtime;
	static double level1_LocalSearch_Runtime;
	static double level2_LowerSpeeds_Runtime;
	static double level2_LocalSearch_Runtime;
	static double level3_LowerSpeeds_Runtime;
	static double level3_LocalSearch_Runtime;
	static double level4_LowerSpeeds_Runtime;
	static double level4_LocalSearch_Runtime;

	static vector<double> level0_LowerSpeeds_Runtime_vec;
	static vector<double> level0_LocalSearch_Runtime_vec;
	static vector<double> level1_LowerSpeeds_Runtime_vec;
	static vector<double> level1_LocalSearch_Runtime_vec;
	static vector<double> level2_LowerSpeeds_Runtime_vec;
	static vector<double> level2_LocalSearch_Runtime_vec;
	static vector<double> level3_LowerSpeeds_Runtime_vec;
	static vector<double> level3_LocalSearch_Runtime_vec;
	static vector<double> level4_LowerSpeeds_Runtime_vec;
	static vector<double> level4_LocalSearch_Runtime_vec;

	static void save_results();
	static void set_zero();
	static void set_scale(int scale);
	static void reset();
	static void outputCollectInfo(ostream& out);

	//=====================================================================================
	// Collect Info End
	//=====================================================================================

	static bool performanceCoeffsComparator(const pair<int,double>& p1, const pair<int,double>& p2);
	static bool performanceCoeffsComparatorMin(const pair<int,double>& p1, const pair<int,double>& p2);
	static double getPerformance(map<int,double> speeds_map, vector<double> k, Engine engine);
	static double getPerformance(vector<double> speeds, vector<double> k, Engine engine);
	static double getPerformanceIndex(vector<double> speeds, vector<double> k, Engine engine, int index);
	static double getPerformanceMax(vector<double> speeds, vector<double> k, Engine engine);
	static double getPerformanceMin(vector<double> speeds, vector<double> k, Engine engine);

	static double integral(double k1, double k2, double speed);
	static double getPerformanceExp(vector<double> speeds, vector<double> k1, vector<double> k2, Engine engine);
	static double getPerformanceExp(map<int, double> speeds_map, vector<double> k1, vector<double> k2, Engine engine);

	static double integralExpGsl(double x, void * params);
	static double getPerformanceExpGsl(double k1, double k2, double a, double b);
	static double getPerformanceExpGsl(vector<double> speeds, vector<double> k1, vector<double> k2, Engine engine);
	static double getPerformanceExpGsl(map<int, double> speeds_map, vector<double> k1, vector<double> k2, Engine engine);

	static double integralPoly3(double x, void * params);
	static double getPerformancePoly(double k1, double k2, double k3, double a, double b);
	static double getPerformancePoly(vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3, Engine engine);
	static double getPerformancePoly(map<int,double> speeds_map, vector<double> k1, vector<double> k2, vector<double> k3, Engine engine);

	static double integralPoly4(double x, void * params);
	static double getPerformancePoly(double k1, double k2, double k3, double k4, double a, double b);
	static double getPerformancePoly(vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, Engine engine);
	static double getPerformancePoly(map<int,double> speeds_map, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, Engine engine);

	static vector<double> getPerformanceDerivatives(vector<double> k);
	static double getPerformanceDerivativeIndex(vector<double> k, int index);
	static double getPerformanceDerivativeMax(vector<double> k);
	static double getPerformanceDerivativeMin(vector<double> k);

	static vector<double> getPerformanceDerivativesExp(vector<double> k1, vector<double> k2, vector<double> speeds);
	static vector<double> getPerformanceDerivativesDCExp(map<int,int> dc, vector<double> k1, vector<double> k2, vector<double> speeds);
	static vector<double> getPerformanceDerivativesPoly(vector<double> k1, vector<double> k2, vector<double> k3, vector<double> speeds);

	static int getSpeedIndex(vector<double> speeds, double speed);
	static double getPerformanceDC(map<int,int> dc, vector<double> speeds, vector<double> k);
	static double getPerformanceDC(list<double> dc, vector<double> speeds, vector<double> k);
	static double getPerformanceDC(map<int,int> dc, map<int,double> speeds, vector<double> k);
	static double getPerformanceDC(list<int> indexs, vector<double> k);
	static double getPerformanceDC(list<double> dc, list<int> indexs, vector<double> k);

	static double getPerformanceDCExp(map<int,int> dc, vector<double> speeds, vector<double> k1, vector<double> k2);
	static double getPerformanceDCExp(map<int,int> dc, map<int,double> speeds, vector<double> k1, vector<double> k2);
	static double getPerformanceDCExp(list<double> dc, vector<double> speeds, vector<double> k1, vector<double> k2);
	static double getPerformanceDCExp(list<double> dc, map<int,double> speeds, vector<double> k1, vector<double> k2);
	static double getPerformanceDCExp(list<double> dc, list<int> indexs, vector<double> k1, vector<double> k2);

	static double getPerformanceDCPoly3(map<int,int> dc, vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3);
	static double getPerformanceDCPoly3(map<int,int> dc, map<int,double> speeds, vector<double> k1, vector<double> k2, vector<double> k3);

	static double getPerformanceDCPoly(double speed, double torque, double k1, double k2, double k3, double k4);
	static double getPerformanceDCPoly(vector<double> dc_speeds, vector<double> dc_torques, vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4);
	static double getPerformanceDCPoly(vector<double> dc_speeds, vector<double> dc_torques, map<int,double> speeds_map, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4);

	static vector<double> getMaxUtilizations(vector<double> speeds, vector<int> wcets, double angular_period, Engine engine);
	static vector<double> getUtilizations(vector<double> speeds, vector<int> wcets, double angular_period);
	static double getUtilizationIndex(vector<double> speeds, vector<int> wcets, int index,double angular_period);
	static double getUtilizationMax(vector<double> speeds, vector<int> wcets, double angular_period);
	static double getUtilizationMin(vector<double> speeds, vector<int> wcets, double angular_period);

	static double calStepIndex(vector<double> speeds, vector<int> wcets, vector<double> k, int index, double angular_period);
	static vector<double> calSteps(vector<double> speeds, vector<int> wcets, vector<double> k, double angular_period);
	static vector<double> calSteps(vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period);
	static vector<double> calSteps(vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, vector<double> k3, double angular_period);

	static vector<double> calStepsMin(vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period);

	// compute ubs: fig. 5 in Biondi's journal paper
	static vector<double> computeUBSpeeds(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask = NULL);
	static vector<double> computeUBSpeedsFast(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask = NULL);

	static vector<double> computeUBFPSpeeds(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex);
	static vector<double> computeUBFPSpeeds(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, list<double> dcList, vector<double> k1, vector<double> k2, int& avrTaskIndex);
	
	static double computeUBPerfDC(ostream& out, list<double> dc, MethodChoice mc, vector<int> wcets, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, Engine engine, double period);
	static list<int> computeUBPerfDC(list<double> dc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int& optNum, double& optTime, double& maxOptTime, int granularity = -1);
	static list<int> computeUBFPPerfDC(list<double> dc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int avrTaskIndex, int& optNum, double& optTime, double& maxOptTime, int granularity = -1);
	static list<int> computeUBPerfDC(list<double> dc, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds);
	/// Analyze on the three-mode AVR task, which reduces the complexity
	static list<int> computeUBPerfDCFast(list<double> dc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds);
	static list<int> computeUBFPPerfDCFast(list<double> dc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int avrTaskIndex);
	static list<int> computeUBPerfDCFast(list<double> dc, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds);

	static list<int> computeUBFPPerfDCFastCoarse(list<double> dc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int avrTaskIndex);

	static vector<double> computeUBSpeeds(MethodChoice mc, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeUBSpeeds(MethodChoice mc, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period, AVRTask prevAvrTask, int switchTime);

	static StateGraph* generateStateGraph(MethodChoice mc, PerformanceType pt, vector<vector<double>> ks, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static StateGraph* generateStateGraphFast(MethodChoice mc, PerformanceType pt, vector<vector<double>> ks, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static StateGraph* generateStateGraph(MethodChoice mc, PerformanceType pt, vector<vector<double>> ks, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);

	static void enforceModeGuards(vector<double> &maxSpeeds);
	static void enforceModeReduced(vector<double> &speeds, double cStep);
	static void doLocalSearch(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchFP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine, int avrTaskIndex);
	static void doLocalSearchExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchExp2(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchExp2FP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine, int avrTaskIndex);
	static void doLocalSearchPoly(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	
	static void doLocalSearch(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<AsynchronousPeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchExp2(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<AsynchronousPeriodicTask> tasks, AVRTask* avrTask, Engine engine);

	static void doLocalSearchExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchExpMin2(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);
	static void doLocalSearchDCExpMin2(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine);

	static vector<double> doLowerSpeeds(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearch(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> doLowerSpeeds_CollectInfo(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, double &times);
	static vector<double> doLocalSearch_CollectInfo(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, double &times);

	static vector<double> computeBiondiBS(MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBS(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBiondiBSFP(MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex);
	static vector<double> computeBiondiBSFP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskindex);

	static vector<double> computeBiondiBS(MethodChoice mc, vector<double> k, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBS(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);

	static vector<double> doLowerSpeedsExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearchExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> doLowerSpeedsExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearchExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearchExpMin2(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> doLowerSpeedsDCExp(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearchDCExp(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> doLowerSpeedsDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearchDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBiondiBSExp(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBSExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBiondiBSExpFP(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex);
	static vector<double> computeBiondiBSExpFP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex);

	static vector<double> computeBiondiBSExp(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBSExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBiondiBSExpMin(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBSExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	// a*x^2+b*x+c
	static vector<double> computeBiondiBSPoly3(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBSPoly3(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	// a*x^2+b*x+c*x*T+d
	static vector<double> computeBiondiBSPoly4(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBSPoly4(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static double findMaxSpeed(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, double lb, double ub, unsigned int mode);
	static double findMaxSpeed(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, double lb, double ub, unsigned int mode);
	
	static double getUBPerformance(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k);
	static void doBiondiBBSearch(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k, vector<double>& result);
	static vector<double> computeBiondiBB(MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static void doBiondiBBSearch(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<AsynchronousPeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k, vector<double>& result);
	static vector<double> computeBiondiBB(MethodChoice mc, vector<double> k, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period);

	static double getUBPerformanceExp(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2);
	static void doBiondiBBSearchExp(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result);
	static vector<double> computeBiondiBBExp(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	
	static double getUBPerformanceExpMin(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2);
	static void doBiondiBBSearchExpMin(MethodChoice mc, double& minPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result);
	static vector<double> computeBiondiBBExpMin(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	// a*x^2+b*x+c
	static double getUBPerformancePoly3(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3);
	static void doBiondiBBSearchPoly3(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double>& result);
	static vector<double> computeBiondiBBPoly3(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	// a*x^2+b*x+c*x*T+d
	static double getUBPerformancePoly4(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4);
	static void doBiondiBBSearchPoly4(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<double>& result);
	static vector<double> computeBiondiBBPoly4(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	
	static double getDrivingCycleUBPerformance(map<int,int> dc, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k);
	static void doDrivingCycleBBSearch(map<int,int> dc, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k, vector<double>& result);
	static vector<double> computeDrivingCycleBB(map<int,int> dc, MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	
	static double getUBPerformanceDCExp(map<int,int> dc, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2);
	static void doBiondiBBSearchDCExp(map<int,int> dc, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result);
	static vector<double> computeBiondiBBDCExp(map<int, int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static void doBiondiBBSearchDCExpMin(map<int,int> dc, MethodChoice mc, double& minPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result);
	static vector<double> computeBiondiBBDCExpMin(map<int, int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static double getUBPerformanceDCPoly3(map<int,int> dc, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3);
	static void doBiondiBBSearchDCPoly3(map<int,int> dc, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double>& result);
	static vector<double> computeBiondiBBDCPoly3(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static double getUBPerformanceDCPoly4(vector<double> dc_speeds, vector<double> dc_torques, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4);
	static void doBiondiBBSearchDCPoly4(vector<double> dc_speeds, vector<double> dc_torques, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<double>& result);
	static vector<double> computeBiondiBBDCPoly4(vector<double> dc_speeds, vector<double> dc_torques, MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeChaoBS(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeChaoBS(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	
	static void enforceDCGuards(map<int,int> dc, vector<double>& speeds);
	static vector<double> calStepsDC(map<int,int> dc, vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period);
	static vector<double> calStepsDCMin(map<int,int> dc, vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period);

	static vector<double> computeBiondiBSDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBiondiBSDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	// using engine speed - virtual step function
	static double getPerformanceDCExpWithESVSF(map<int,int>dc, EngineSpeedVirtualStepFunction esvsf, vector<double> virtualSteps, vector<double> k1, vector<double> k2);

	static vector<double> calVirtualStepsMax(EngineSpeedVirtualStepFunction esvsf, vector<double> virtualSteps, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period);
	
	static vector<double> doLowerSpeedsDCExpMaxWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask = NULL);
	static vector<double> doLocalSearchDCExpMaxWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> LBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask = NULL);
	
	static vector<double> computeBSDCExpMaxWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSDCExpMaxWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBSDCExpMaxWithESVSF_NECESSARYONLY(map<int,int> dc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask = NULL);
	static vector<double> computeBSDCExpMaxWithESVSF_NECESSARYONLY(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask = NULL);

	static vector<double> computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(map<int,int> dc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	
	static vector<double> calVirtualStepsMin(EngineSpeedVirtualStepFunction esvsf, vector<double> virtualSteps, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period);
	static vector<double> doLowerSpeedsDCExpMinWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> doLocalSearchDCExpMinWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> LBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSDCExpMinWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSDCExpMinWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBSDCExpMinWithESVSF_NECESSARYONLY(map<int,int> dc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSDCExpMinWithESVSF_NECESSARYONLY(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	
	//----------------------------------------------------------------------------------------------------
	//----------------------------------------------------------------------------------------------------
	//----------------------------------------------------------------------------------------------------
	//--------------------[[[ HEURISTIC WITH NECESSARY ONLY CONDITIONS ]]]--------------------------------
	//----------------------------------------------------------------------------------------------------
	//----------------------------------------------------------------------------------------------------
	//----------------------------------------------------------------------------------------------------
	static vector<double> computeBS_NECESSARYONLY(vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBS_NECESSARYONLY_CollectInfo(vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBS_NECESSARYONLY_WithoutDebug(vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeBSExp_NECESSARYONLY(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSExp_NECESSARYONLY_WithoutDebug(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeBSExp_NECESSARYONLY_WithoutDebug_Min(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeDCBSExp_NECESSARYONLY_WithoutDebug(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> computeDCBSExp_NECESSARYONLY_Min(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);
	static vector<double> computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	// Integrate All
	static bool doAllOptimizationsExpMin(vector<double>& UBSpeeds, double &pUB, double &tUB,
										bool bOPT, vector<double>& OPTSpeeds, double &pOPT, double &tOPT,
										bool bBS, vector<double>& BSSpeeds, double &pBS, double &tBS,
										bool bOPTDC, vector<double>& OPTDCSpeeds, double &pOPTDC, double &tOPTDC,
										bool bBSDC, vector<double>& BSDCSpeeds, double &pBSDC, double &tBSDC,
										bool bBSDCSec, vector<vector<double>>& BSDCSecSpeedsVector, double &pBSDCSec, double &tBSDCSec,
										map<int,int> dc, vector<map<int,int>> dcVector, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static bool doAllOptimizationsExpMax(vector<double>& UBSpeeds, double &pUB, double &tUB,
										bool bOPT, vector<double>& OPTSpeeds, double &pOPT, double &tOPT,
										bool bBS, vector<double>& BSSpeeds, double &pBS, double &tBS,
										bool bOPTDC, vector<double>& OPTDCSpeeds, double &pOPTDC, double &tOPTDC,
										bool bBSDC, vector<double>& BSDCSpeeds, double &pBSDC, double &tBSDC,
										bool bBSDCSec, vector<vector<double>>& BSDCSecSpeedsVector, double &pBSDCSec, double &tBSDCSec,
										map<int,int> dc, vector<map<int,int>> dcVector, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static bool doAllOptimizationsExpMax(vector<double>& UBSpeeds, double &pUB, double &tUB, double &pUBd, double &tUBd,
		bool bOPT, vector<double>& OPTSpeeds, double &pOPT, double &tOPT,
		bool bBS, vector<double>& BSSpeeds, double &pBS, double &tBS,
		bool bOPTDC, vector<double>& OPTDCSpeeds, double &pOPTDC, double &tOPTDC,
		bool bBSDC, vector<double>& BSDCSpeeds, double &pBSDC, double &tBSDC,
		bool bBSDCSec, vector<vector<double>>& BSDCSecSpeedsVector, double &pBSDCSec, double &tBSDCSec,
		list<double> dc, vector<map<int,int>> dcVector, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period);

	static vector<double> generatePerformanceFunctionsByWCET(vector<int> WCETs);
	static vector<double> generatePerformanceFunctionsByWCETExponential(vector<int> WCETs);

	// Without driving cycles
	static CollectionResult doOptimalAlgorithmDFSMappingAndBnBMin(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	static void doDepthFirstSearchMin(double &pOpt, vector<PeriodicTask> &sOpt, vector<double> &wOpt, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	// With driving cycles
	static CollectionResult doOptimalAlgorithmDFSMappingAndBnBMin(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	static void doDepthFirstSearchMin(double &pOpt, vector<PeriodicTask> &sOpt, vector<double> &wOpt,map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	

	static bool SortByDeadline(const PeriodicTask task0, const PeriodicTask task1);
	//************************************
	// Method:    calClusteringHeuristicFactor
	// FullName:  OptimizationAlgorithm::calClusteringHeuristicFactor
	// Access:    public static 
	// Returns:   double
	// Qualifier: the paper "Minimizing a real-time task set through task clustering"
	// Parameter: vector<PeriodicTask> tasks
	//************************************
	static double calClusteringHeuristicFactor(vector<PeriodicTask> tasks);

	// Without driving cycles
	static CollectionResult doHeuristicAlgorithmBFSMappingAndBSMin(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	static vector<PeriodicTask> doBestFirstSearchMin(double pUBMax, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	// With driving cycles
	static CollectionResult doHeuristicAlgorithmBFSMappingAndBSMin(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
	static vector<PeriodicTask> doBestFirstSearchMin(double pUBMax, map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint);
};

#endif