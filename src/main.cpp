#include <iostream>
//#include <windows.h>
#include <future>
#include <chrono>
#include <stdlib.h>
//#include <direct.h>
#include <numeric>

#include "AVRTask.h"
#include "Timer.h"
#include "Utility.h"
#include "TaskSystemGenerator.h"
#include "SchedAnalysis.h"
#include "OptimizationAlgorithm.h"
#include "FileWriter.h"
#include "FileReader.h"

#include "RunTest.h"
#include "Definitions.h"
#include "RunTest.h"

#ifdef VLDTEST
#include <vld.h>
#if _WIN64 || _WIN32 
#if _WIN64
#pragma comment(lib,"C:\\Program Files (x86)\\Visual Leak Detector\\lib\\Win64\\vld.lib")
#else
#pragma comment(lib,"C:\\Program Files (x86)\\Visual Leak Detector\\lib\\Win32\\vld.lib")
#endif
#endif
#endif

// using GSL library
#if 1
#ifdef _DEBUG
#if _WIN64 || _WIN32 
#if _WIN64
#pragma comment(lib,"E:\\博士课题研究\\IDEAS\\AVRTask\\gsl-1.16\\gsl-1.16\\lib\\x64\\Debug\\gsl.lib")
#else
#pragma comment(lib,"E:\\博士课题研究\\IDEAS\\AVRTask\\gsl-1.16\\gsl-1.16\\lib\\Win32\\Debug\\gsl.lib")
#endif
#endif
#else
#if _WIN64 || _WIN32 
#if _WIN64
//#pragma comment(lib,"E:\\博士课题研究\\IDEAS\\AVRTask\\gsl-1.16\\gsl-1.16\\lib\\x64\\Release\\gsl.lib")
#pragma comment(lib,"G:\\chenkai\\cp\\gsl-1.16\\gsl-1.16\\lib\\x64\\Release\\gsl.lib")
//#pragma comment(lib,"G:\\gsl-1.16\\lib\\x64\\Release\\gsl.lib")
//#pragma comment(lib,"D:\\gsl-1.16\\gsl-1.16\\lib\\x64\\Release\\gsl.lib") // for my own server
#else
#pragma comment(lib,"E:\\博士课题研究\\IDEAS\\AVRTask\\gsl-1.16\\gsl-1.16\\lib\\Win32\\Release\\gsl.lib")
#endif
#endif
#endif
#endif

//#pragma comment(lib,"E:\\博士课题研究\\IDEAS\\AVRTask\\gsl-1.16\\gsl-1.16\\lib\\x64\\Debug\\gsl.lib")

using namespace std;

string INPUT_PREFIX = "Inputs5"+Utility::linkNotation();
string INPUT_SUFFIX = ".info";
string RESULT_PREFIX = "Results"+Utility::linkNotation();
string RESULT_SUFFIX = ".result";

void SchedAnalysisEvaluation(int argc, char* argv[]);
void OptimizationEvaluationConst(int argc, char* argv[]);
void OptimizationEvaluationExp(int argc, char* argv[]);

int main(int argc, char* argv[])
{

	SchedAnalysisEvaluation(argc,argv);
	//OptimizationEvaluationConst(argc,argv);
	//OptimizationEvaluationExp(argc,argv);

	return 0;
}

void SchedAnalysisEvaluation(int argc, char* argv[]) {
#if 0

	if (argc != 10) {
		cerr << "Error parameter number => " << argc << endl;
		cerr << "Info: *.exe 1 0 0 0 0 0 0 0 util=300" << endl;
		exit(EXIT_FAILURE);
	}

	int testExp1A = atoi(argv[1]);
	int testExp1B = atoi(argv[2]);
	int testExp2 = atoi(argv[3]);
	int testExp3A = atoi(argv[4]);
	int testExp3B = atoi(argv[5]);
	int testExp4A = atoi(argv[6]);
	int testExp4B = atoi(argv[7]);
	int testExp5 = atoi(argv[8]);
	int testExp5Util = atoi(argv[9]);

#else

	bool testExp1A = true;
	bool testExp1B = false;
	bool testExp2 = true;
	bool testExp3A = false;
	bool testExp3B = false;
	bool testExp4A = false;
	bool testExp4B = false;
	bool testExp5 = false;
	int testExp5Util = 750;

#endif

	int testNum = 1000;

	if (testExp1A) RunTest::ECRTS18_Experiment1("RTAResults", "Exp1Rho40.txt", 10, 20, 0.4,testNum);
	if (testExp1B) RunTest::ECRTS18_Experiment1("RTAResults", "Exp1Rho60.txt", 10, 20, 0.6,testNum);
	if (testExp2)  RunTest::ECRTS18_Experiment2("RTAResults", "Exp2Util85.txt",10, 20, 0.85,testNum);
	if (testExp3A) RunTest::ECRTS18_Experiment3("RTAResults", "Exp3Util85Rho40.txt",10,20,0.85,0.4,testNum);
	if (testExp3B) RunTest::ECRTS18_Experiment3("RTAResults", "Exp3Util85Rho60.txt",10,20,0.85,0.6,testNum);
	if (testExp4A) RunTest::ECRTS18_Experiment4("RTAResults", "Exp4Util85Rho40.txt",20,0.85,0.4,testNum);
	if (testExp4B) RunTest::ECRTS18_Experiment4("RTAResults", "Exp4Util85Rho60.txt",20,0.85,0.6,testNum);
	if (testExp5) RunTest::ECRTS18_Experiment5("RTAResults", "Exp5Rho40Util"+Utility::int_to_string(testExp5Util)+".txt", 10, 20, 0.4,testExp5Util,testNum);
}

void OptimizationEvaluationConst(int argc, char* argv[]) {
#if 0
	if (argc != 12) {
		cerr << "Error parameter number => " << argc << endl;
		cerr << "Info: *.exe tUtil=75 numMode=6 numRun=500  "
			<< "            minFactor=1 maxFactor=10 numTask=5 "
			<< "            d1 = 1.0 d2 = 1.0  numK=30" << endl;
		exit(EXIT_FAILURE);
	}

	int tUtil = atoi(argv[1]);
	int numMode = atoi(argv[2]);
	int numRun = atoi(argv[3]);
	int minFactor = atoi(argv[4]);
	int maxFactor = atoi(argv[5]);
	int numTask = atoi(argv[6]);
	double d1 = atof(argv[7]);
	double d2 = atof(argv[8]);
	int numK = atoi(argv[9]);
	double k2Min = atof(argv[10]);
	double k2Max =  atof(argv[11]);
#else
	int tUtil = 75;
	int numMode = 6;
	int numRun = 10;
	int minFactor = 8;
	int maxFactor = 10;
	int numTask = 5;
	double d1 = 1.0;
	double d2 = 1.0;
	double numK = 5;
#endif

	double k1Min = 1;
	double k1Max = 50;
	double k1Step = 1;

	int granularity = -1;

	static const string _dcName[] = {"UDC","NEDC", "FTP75", "JA1015", "HIGHWAY", "IDC", "EUDC", "US_SC03", "Artemis_Road", "Artemis_Urban"};
	//static const string _dcName[] = {"UDC","NEDC"};
	vector<string> dcFiles(_dcName,_dcName+sizeof(_dcName)/sizeof(_dcName[0]));
	RunTest::generateConstantPerformanceFunctions("RTADAVRRandomSystemConst","ConstantFunctions",numK,numMode,k1Min,k1Max,k1Step);
	RunTest::generateRandomSystemsWithOffset("RTADAVRRandomSystemConst"+Utility::linkNotation()+"TestedSystems",numRun,numTask,tUtil,d1,d2,numMode);
	RunTest::RTADAVRConst2(false,"RTADAVRRandomSystemConst", "TestedSystems", "ConstantFunctions", INPUT_PREFIX, dcFiles, tUtil, numRun,minFactor,maxFactor,granularity);
}

void OptimizationEvaluationExp(int argc, char* argv[]) {
#if 1
	if (argc != 11) {
		cerr << "Error parameter number => " << argc << endl;
		cerr << "Info: *.exe tUtil=75 numMode=6 numRun=500  "
			<< "            minFactor=1 maxFactor=10 numTask=5 "
			<< "            d1 = 1.0 d2 = 1.0  "
			<< "            numK=30 k2Ratio=100  " << endl;
		exit(EXIT_FAILURE);
	}

	int tUtil = atoi(argv[1]);
	int numMode = atoi(argv[2]);
	int numRun = atoi(argv[3]);
	int minFactor = atoi(argv[4]);
	int maxFactor = atoi(argv[5]);
	int numTask = atoi(argv[6]);
	double d1 = atof(argv[7]);
	double d2 = atof(argv[8]);
	int numK = atoi(argv[9]);
	double k2Ratio = atof(argv[10]);
	//double k2Max =  atof(argv[11]);
#else
	int tUtil = 75;
	int numMode = 6;
	int numRun = 100;
	int minFactor = 8;
	int maxFactor = 10;
	int numTask = 5;
	double d1 = 1.0;
	double d2 = 1.0;
	double numK = 30;
	//double k2Min = 100.0;
	//double k2Min = 1.0;
	double k2Ratio = 10.0;
	//double k2Max = 10000.0;
	//double k2Step = 100;
#endif

	double k1Min = 1;
	double k1Max = 50;
	double k1Step = 1;

	int granularity = -1;
	
	//int k2Ratio = 100;
	double k2Min = RPM_to_W(50);
	double k2Max = k2Ratio*k2Min;
	/*
	for (int rpm = 500; rpm <=6500; rpm+=500) 
		cout << rpm << " => " << exp(-k2Min/RPM_to_W(rpm)) << endl;
		*/

	string kRatio = Utility::int_to_string(k2Ratio);
	//static const string _dcName[] = {"UDC","NEDC", "FTP75", "JA1015", "HIGHWAY", "IDC", "EUDC", "US_SC03", "Artemis_Road", "Artemis_Urban"};
	static const string _dcName[] = {"UDC","NEDC"};
	vector<string> dcFiles(_dcName,_dcName+sizeof(_dcName)/sizeof(_dcName[0]));
	RunTest::generateExponentialPerformanceFunctions("RTADAVRRandomSystemExp","ExponentialFunctionsKRatio"+kRatio,numK,numMode,k2Min,k2Max);
	//RunTest::generateExponentialPerformanceFunctions("RTADAVRRandomSystemExp","ExponentialFunctionsKRatio"+kRatio,numK,numMode,k1Min, k1Max, k1Step, k2Min,k2Max);
	RunTest::generateRandomSystemsWithOffset("RTADAVRRandomSystemExp"+Utility::linkNotation()+"TestedSystems",numRun,numTask,tUtil,d1,d2,numMode);
	RunTest::RTADAVRExp2(false,"RTADAVRRandomSystemExp", "TestedSystems", "ExponentialFunctionsKRatio"+kRatio, INPUT_PREFIX, dcFiles,kRatio, tUtil, numRun,minFactor,maxFactor,granularity);
}