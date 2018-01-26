#ifndef RUNTEST_H_
#define RUNTEST_H_

#include <iostream>
//#include <windows.h>
#include <future>
#include <chrono>
#include <stdlib.h>
//#include <direct.h>

#include "AVRTask.h"
#include "Timer.h"
#include "Utility.h"
#include "TaskSystemGenerator.h"
#include "SchedAnalysis.h"
#include "OptimizationAlgorithm.h"
#include "FileWriter.h"
#include "FileReader.h"

#include "AVLTree.h"

#ifdef __USING_ILPCPLEX__
#include <ilcplex/ilocplex.h>
#endif

class RunTest {
public:
	static void test(int max, int scale);
	static void testRBFPeriodicity(int max, int scale);
	static void testLUB(int max, int scale);
	static void test(int rpm, int max, int step, int scale);
	static void test(int rpm, int tMin, int tMax, int tStep, int scale);
	static void test(string inputDir, string dcName);

#ifdef __USING_ILPCPLEX__
	static void testCplexILP();
	static int testCplexILP(int wcet0, int period0, int wcet1, int period1, int con0, int con1);
	static void testVRBILPExample();
#endif

	/// the examples used in our paper
	static void testInitialDominantSpeeds();
	static void testMinMaxTime();
	static void testJobSequences();
	static void seekMotivatedExample(string dcFile);
	static void testSpeedIntervals();

	/// tests on the asynchronous periodic tasks
	/// the example in this paper
	static void testAsynchronousPeriodicTasks();
	static void testAsynchronousPeriodicTasks2();
	static void testAsynchronousPeriodicTasks3();
	static void testAsynchronousBiondiExample();
	static void testAsynchronousBiondiExample2();
	static void testAsynchronousBiondiExample3();

	static void AVLTest();

	/// A running example in Biondi's ICCPS16 paper
	static void testBiondiExample();
	static void testBiondiExampleSchedAnalysis();
	static void testBiondiExampleRelationTMinTMax();
	static void testBiondiExampleStateGraph();
	static void testBiondiExampleStateGraph2();
	static void testBiondiExampleJobSequence();
	static void testBiondiExampleWithoutBB();
	static void testBiondiExampleWithoutBBMapping();
	static void testBiondiExampleWithoutBBColllectInfo();
	static void testBiondiExampleRBFBased();
	static void testBiondiExampleExp();
	static void testBiondiExampleExpWithoutBB();
	static void testBiondiExamplePoly();
	static void testMultiInjectionsExp();
	static void testMultiInjectionsExpWithoutBB();

	static void testBiondiExampleDC(string inputDir, string dcName, vector<double>& perf_results, vector<double>& time_results);

	static void generateBiondiTestedSystems(string directory, int numSystem, int numPeriodicTask, int tUtil, int numMode);
	
	static void doBBForBiondiTestedSystems(string resultDir, string inputDir, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK);
	
	static void doBSForBiondiTestedSystems(string resultDir, string inputDir, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK);
	static void doBSForBiondiTestedSystemsCollectInfo(string resultDir, string inputDir, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK);
	static void doBSForSingleBiondiTestedSystem(string inputDir, int tUtil, int noSystem, int noFactor, vector<double> k);

	static void doOptimizationOverBiondiTestedSystemsExp(bool isMax, vector<bool> vecTest, string resultDir, string inputDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, int numK, int sizeK, double k2Min, double k2Max, double k2Step);
	static void doOptimizationOverBiondiTestedSystemsExpConstantK(bool isMax, vector<bool> vecTest, string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, vector<double> k1, vector<double> k2);

	static void doOptimizationZeroOffsetExpConstantK(string resultDir, string resultDAVRDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, vector<double> k1, vector<double> k2);

	static void doDCBBForBiondiTestedSystems(string resultDir, string inputDir, string dcFile, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK);
	
	// Statistic Results
	static void computePerformanceDC(string inputDir, string dcName, string BiondiResultDir, string DCResultDir, int tUtil);
	static void computePerformanceDCExp(string inputDir, string dcName, string BiondiResultDir, string DCResultDir, int tUtil);
	static void computePerformanceTwoDCsExp(string inputDir, string dcAFileName, string dcBFileName, string BiondiResultDir, string DCResultDir, int tUtil);
	static void computePerformanceTwoDCsExpBS(string inputDir, string dcAFileName, string dcBFileName, string speedsResultDir, string dataOutputDir, int tUtil);

	// Test Exponential Function
	static void testBiondiExampleExp(string inputDir, string dcName, vector<double>& results);
	static void testBiondiExampleDCExp(string inputDir, string dcName, vector<double>& results);

	static void testBiondiExampleEmissionExp(string inputDir, string dcName, vector<double>& results);
	static void testBiondiExampleEmissionDCExp(string inputDir, string dcName, vector<double>& results);

	static void testBiondiExampleEmissionExp(string inputDir, string dcName, string systemDir, int tUtil, int factor, int run, vector<double>& results);
	static void testBiondiExampleEmissionDCExp(string inputDir, string dcName, string systemDir, int tUtil, int factor, int run, vector<double>& results);

	static void testBiondiExampleEmissionExpMapping(string inputDir, string dcName, vector<double>& results);
	static void testBiondiExampleEmissionDCExpMapping(string inputDir, string dcName, vector<double>& results);

	// Test Polynomial Function without the torques
	static void testBiondiExamplePoly(string inputDir, string dcName, vector<double>& results);
	static void testBiondiExampleDCPoly(string inputDir, string dcName, vector<double>& results);

	// Test Fuel Consumptions
	// the real data for the single, double and triple injections
	static void testFuelConsumptionExample(vector<double>& speeds_results, vector<double>& dc_perfs, string inputDir, string dcName);
	static void testFuelConsumptionExampleDC(vector<vector<double>>& speeds_results, vector<double>& dc_perfs, vector<double>& dc_times, string inputDir, string dcName); 

	//************************************
	// Method:    generateBiondiRTATestedSystems, o generate the random systems similar to the ICCPS15 paper
	//			  "Response-Time Analysis for Real-Time Tasks in Engine Control Applications"
	// FullName:  RunTest::generateBiondiRTATestedSystems
	// Access:    public static 
	// Returns:   void
	// Qualifier:
	// Parameter: string directory
	// Parameter: int numSystem
	// Parameter: int numPeriodicTask
	// Parameter: int U_p
	// Parameter: int U_a
	//************************************
	static void generateBiondiRTATestedSystems(string directory, int numSystem, int numPeriodicTask, int tUtil, double rho, int minModeNum, int maxModeNum);
	static void generateBiondiRTATestedSystems2(string directory, int numSystem, int numPeriodicTask, int U_P, int U_A, int minModeNum, int maxModeNum);
	static void generateBiondiRTATestedSystems3(string directory, int numSystem, int numPeriodicTask, double tUtil, int rho, int minModeNum, int maxModeNum);
	static void generateBiondiRTATestedSystems4(string directory, int numSystem, int numPeriodicTask, double tUtil, double rho, int modeNum);
	
	static void testBiondiRTATestedSystems(vector<bool> vecTest, ofstream& fout, string directory, int minSystem, int maxSystem, int tUtil, vector<int> & tUtils, vector<vector<double>> & pResults, vector<vector<double>> & tResults);
	static void testBiondiRTATestedSystems2(vector<bool> vecTest, ofstream& fout, string directory, int minSystem, int maxSystem, int rho, vector<int> & rhos, vector<vector<double>> & pResults, vector<vector<double>> & tResults);
	static void testBiondiRTATestedSystems3(vector<bool> vecTest, ofstream& fout, string directory, int minSystem, int maxSystem, int modeNum, vector<int> & modeNums, vector<vector<double>> & pResults, vector<vector<double>> & tResults);

	static void generateECRTS18RTATestedSystems(string directory, int numSystem, int numAvrTask, int numPeriodicTask, int tUtil, double rho, int minModeNum, int maxModeNum);
	static void generateECRTS18RTATestedSystems2(string directory, int numSystem, int numAvrTask, int numPeriodicTask, double tUtil, int rho, int minModeNum, int maxModeNum);
	static void generateECRTS18RTATestedSystems3(string directory, int numSystem, int numAvrTask, int numPeriodicTask, double tUtil, double rho, int modeNum);
	static void generateECRTS18RTATestedSystems4(string directory, int numSystem, int numAvrTask, int numPeriodicTask, double tUtil, double rho, int minModeNum, int maxModeNum);

	static void testECRTS18RTATestedSystems(ofstream& fout, string directory, int minSystem, int maxSystem, int tUtil, vector<int> & tUtils, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double rho);
	static void testECRTS18RTATestedSystems2(ofstream& fout, string directory, int minSystem, int maxSystem, int rho, vector<int> & rhos, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double tUtil);
	static void testECRTS18RTATestedSystems3(ofstream& fout, string directory, int minSystem, int maxSystem, int modeNum, vector<int> & modeNums, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double tUtil, double rho);
	static void testECRTS18RTATestedSystems4(ofstream& fout, string directory, int minSystem, int maxSystem, int avrTaskNum, vector<int> & avrTaskNums, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double tUtil, double rho);
	static void testECRTS18RTATestedSystems5(ofstream& fout, string directory, int minSystem, int maxSystem, int tUtil, vector<int> & tUtils, vector<vector<double>> & pResults, vector<vector<double>> & tResults, vector<vector<double>> & dResults, double rho);

	// Evaluation on Response Time Analysis
	// Workload generation same to ICCPS15
	static void ICCPS15_Experiment1(vector<bool> vecTest, string resultDir, string resultFile, int numPeriodicTasks, double rho, int numSystem);
	static void ICCPS15_Experiment2(vector<bool> vecTest, string resultDir, string resultFile, int numPeriodicTasks, double tUtil, int numSystem);
	static void ICCPS15_Experiment3(vector<bool> vecTest, string resultDir, string resultFile, int numPeriodicTasks, double tUtil, double rho, int numSystem);
	static void mytest_Experiment1(vector<bool> vecTest,string resultDir, string resultFile, int numSystem);

	// Workload generation same to ICCPS15
	static void ECRTS18_Experiment1(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double rho, int numSystem);
	static void ECRTS18_Experiment2(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double tUtil, int numSystem);
	static void ECRTS18_Experiment3(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double tUtil, double rho, int numSystem);
	static void ECRTS18_Experiment4(string resultDir, string resultFile, int numPeriodicTasks, double tUtil, double rho, int numSystem);
	static void ECRTS18_Experiment5(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double rho, int tUtil, int numSystem);

	// Runnable-to-task mapping
	static void generateMappingRunnables(string directory, int numSystem, int numPeriodicTask, int tUtil, double d1, double d2, int numMode);
	static void doDCBSForMappingRunnablesExpMinConstantKNecessaryOnly(string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, int numConstraint, vector<double> k1, vector<double> k2);
	static void doDCBSForMappingRunnablesExpMinConstantKNecessaryOnlyBaseline(string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, int numConstraint, vector<double> k1, vector<double> k2);

	//////////////////////////////////////////////////////////////////////////
	////////////The first set of experiments for RTA_DAVR/////////////////////
	//////////////////////////////////////////////////////////////////////////

	static void generateFuelInjectionApplications(string directory, double tUtil, int numSystem, bool fixedWCET, int numMode);
	static void generateFuelInjectionApplications2(string directory, double tUtil, int numSystem, bool fixedWCET, int numMode);

	static void generateExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK, double k2Min, double k2Max);
	static void generateExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK, double k2Min, double k2Max, double k2Step);
	static void generateExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK, double k1Min, double k1Max, double k1Step, double k2Min, double k2Max);
	static void generateConstantPerformanceFunctions(string director, string filename, int numK, int sizeK, double minK, double maxK, double stepK);
	static void generateEmissionExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK);

	static void RTADAVRExp(bool checkCaseStudy, string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, string kRatio, int tUtil, int numSystem, int minFactor, int maxFactor, int granularity = -1);
	static void RTADAVRExp2(bool checkCaseStudy, string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, string kRatio, int tUtil, int numSystem, int minFactor, int maxFactor, int granularity = -1);
	
	static void RTADAVRConst(string director, string systemDir, string constFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor);
	static void RTADAVRConst2(bool checkCaseStudy, string director, string systemDir, string constFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor, int granularity = -1);

	static void RTADAVRExpMin(string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor);

	static void generateRandomSystemsWithOffset(string directory, int numSystem, int numPeriodicTask, int tUtil, double d1, double d2,  int numMode);
	static void generateRandomSystemsWithOffset(string directory, int numSystem, int numPeriodicTask, int tUtil, double o1, double o2, double d1, double d2,  int numMode);
	
	static void RTADAVRRandomOffsetExp(string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor);
	static void RTADAVRRandomOffsetConst(string director, string systemDir, string constFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor);
};

#endif