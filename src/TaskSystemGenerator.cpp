#include "TaskSystemGenerator.h"
#include "OptimizationAlgorithm.h"
#include "Utility.h"
#include "SchedAnalysis.h"

#include <numeric>

#ifdef __USING_BIONDI_PERIOD_SETTING__
int TaskSystemGenerator::PERIODS[6] = {5000,10000,20000,50000,80000,100000};
//int TaskSystemGenerator::PERIODS[5] = {5000,10000,20000,50000,100000};
//int TaskSystemGenerator::PERIODS[6] = {5000,6000,7000,8000,9000,10000};
//int TaskSystemGenerator::PERIODS[6] = {8000,9000,10000,50000,80000,100000};
#endif

#ifdef __USING_FUEL_INJECTION_PERIOD_SETTING__
int TaskSystemGenerator::PERIODS[5] = {4000,12000,50000,100000,1000000};
#endif

#ifdef __USING_REAL_WORLD_PERIOD_SETTTING__
//int TaskSystemGenerator::PERIODS[7] = {1000,2000,5000,10000,20000,50000,100000};
int TaskSystemGenerator::PERIODS[8] = {1000,2000,5000,10000,20000,50000,100000,200000};
//int TaskSystemGenerator::PERIODS[9] = {1000,2000,5000,10000,20000,50000,100000,200000,300000};
//int TaskSystemGenerator::PERIODS[9] = {1000,2000,5000,10000,20000,50000,100000,200000,500000};
//int TaskSystemGenerator::PERIODS[10] = {1000,2000,5000,10000,20000,50000,100000,200000,500000,1000000};
#endif

int TaskSystemGenerator::RPM_MIN = 500;
int TaskSystemGenerator::RPM_MAX = 6500;
//int TaskSystemGenerator::RPM_MAX = 6000;
//int TaskSystemGenerator::RPM_MAX = 3000;
//double TaskSystemGenerator::ALPHA = 1.62037 * 0.0001;
double TaskSystemGenerator::ALPHA = 1.62 * 0.0001;
double TaskSystemGenerator::ANGULAR_PERIOD = 1.0;

int TaskSystemGenerator::CMIN = 100;
int TaskSystemGenerator::CMAX = 1000;
int TaskSystemGenerator::CSEP = 100;

/*
int TaskSystemGenerator::CMIN = 100;
int TaskSystemGenerator::CMAX = 1300;
int TaskSystemGenerator::CSEP = 100;
*/

int TaskSystemGenerator::KMIN = 1;
int TaskSystemGenerator::KMAX = 50;
int TaskSystemGenerator::KSEP = 1;

double TaskSystemGenerator::UTILMIN = 0.005;
int TaskSystemGenerator::PERIODMIN = 3;
//int TaskSystemGenerator::PERIODMAX = 20;
int TaskSystemGenerator::PERIODMAX = 100;
//int TaskSystemGenerator::PERIODMAX = 200;
//int TaskSystemGenerator::PERIODMAX = 200;
int TaskSystemGenerator::SCALE_FACTOR = 1000;
double TaskSystemGenerator::UTIL_FACTOR = 0.85;
//double TaskSystemGenerator::UTIL_FACTOR = 1.0;

double TaskSystemGenerator::MODE_SPEED_MIN = 1000;
double TaskSystemGenerator::MODE_SPEED_MAX = 6000;
double TaskSystemGenerator::MODE_SPEED_STEP = 3000;

double TaskSystemGenerator::MIN_SEPERATION_FACTOR = 3000;

Engine TaskSystemGenerator::generate_engine() {
	return Engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);
}

void TaskSystemGenerator::generate_random_system_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, AVRTask* &avrTask, int modeNum) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,tUtil);

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;

		PeriodicTask task(wcet,period);
		tasks.push_back(task);

		#ifdef __DEBUG_DESIGN__
		cout << "Task " << i << ": wcet = " << wcet << "\tperiod=" <<  period << endl;
		#endif
	}

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

	#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
	#endif

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}

void TaskSystemGenerator::generate_random_system_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, vector<int> &wcets, int modeNum) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,tUtil);

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;

		PeriodicTask task(wcet,period);
		tasks.push_back(task);

#ifdef __DEBUG_DESIGN__
		cout << "Task " << i << ": wcet = " << wcet << "\tperiod=" <<  period << endl;
#endif
	}

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
#endif
}

vector<double> TaskSystemGenerator::generate_coefficients(int n) {
	return Utility::uniforms(false,KMIN,KMAX,KSEP,n);
}

vector<double> TaskSystemGenerator::generate_coefficients_k2(int n, double a, double b) {
	vector<double> ret;

	double logA = log(a);
	double logB = log(b);

#if 0
	ret.push_back(0.0);
	vector<double> uniforms = Utility::uniforms(true,logA, logB, n-1);
#else
	vector<double> uniforms = Utility::uniforms(true,logA, logB, n);
#endif

	ret.insert(ret.end(),uniforms.begin(),uniforms.end());

	return ret;
}

vector<double> TaskSystemGenerator::generate_coefficients_k2(int n, double kMin, double kMax, double kStep) {
	vector<double> ret;

	vector<double> uniforms = Utility::uniforms(true,kMin,kMax,kStep,n);

#if 0
	double vMin = uniforms.front();
	for (int i=0; i<n; i++) {
		if (i==0) ret.push_back(0.0);
		else ret.push_back(uniforms[i]-vMin);
	}
#else
	ret = uniforms;
#endif

	return ret;
}

vector<double> TaskSystemGenerator::generate_Biondi_coefficients_k2(int n, double kMin, double kMax) {
	vector<double> coeffs;
	double kStep = 0.4;

	if ((log(kMax)-log(kMin))<(n-1)*kStep)
		return coeffs;

REGENERATION:
	int run = 0;

	coeffs.clear();
	coeffs.push_back(0);
	coeffs.push_back(kMax);

	for (unsigned int i=0; i<n-2; i++) {
		double newLogK;
		while(true) {
			if (run >100) goto REGENERATION;
			newLogK = Utility::random(log(kMin),log(kMax));
			//cout << newLogK << endl;
			run++;
			bool ok = true;
			for (auto k2:coeffs) {
				if(fabs(log(k2)-newLogK) < kStep) {
					ok = false;
					break;
				}
			}
			if (ok) break;
		}
		coeffs.push_back(exp(newLogK));
	}

	sort(coeffs.begin(), coeffs.end(), less<double>());
	return coeffs;
}

void TaskSystemGenerator::generate_random_system_for_response_time_analysis(vector<PeriodicTask> &tasks, int n, double U_p, AVRTask* &avrTask, double U_a, int minModeNum, int maxModeNum, int &avrTaskIndex) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,U_p,UTILMIN);

#ifdef __DEBUG_DESIGN__
	Utility::output_one_vector(cout,"utils",utils);
	double sum = accumulate(utils.begin(),utils.end(),0.0);
	cout << "total utilizations = " << sum << endl;
#endif

	double avrTask_Period = ANGULAR_PERIOD/RPM_to_RPmuSEC(RPM_MAX);

	avrTaskIndex = 0;

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;

		PeriodicTask task(wcet,period);
		tasks.push_back(task);

		if (avrTask_Period > period) avrTaskIndex = i+1;

#ifdef __DEBUG_DESIGN__
		cout << "Task " << i << ": wcet = " << wcet << "\tperiod=" <<  period << endl;
#endif
	}

#ifdef __DEBUG_DESIGN__
	cout << "avrTask period = " << avrTask_Period << endl;
	cout << "avrTask index = " << avrTaskIndex << endl;
#endif

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	int modeNum;
	if (minModeNum == maxModeNum) modeNum = minModeNum;
	else {
		modeNum = minModeNum + rand()%(maxModeNum-minModeNum+1);
	}

	int maxModeIndex = rand()%modeNum;

#ifdef __DEBUG_DESIGN__
	cout << "modeNum = " << modeNum << endl;
	cout << "maxModeIndex = " << maxModeIndex << endl;
#endif

	vector<double> avrTaskUtils;
	for (int i=0; i<modeNum; i++) {
		if (i==maxModeIndex)
			avrTaskUtils.push_back(U_a);
		else {
			avrTaskUtils.push_back(Utility::random(UTIL_FACTOR*U_a,U_a));
		}
	}

#ifdef __DEBUG_DESIGN__
	Utility::output_one_vector(cout,"avrtask Utils", avrTaskUtils);
#endif

	vector<double> speeds;
	vector<int> wcets;

	while (true) {
		speeds.clear();
		wcets.clear();
		speeds= Utility::uniforms(true,MODE_SPEED_MIN,MODE_SPEED_MAX,MODE_SPEED_STEP/modeNum,modeNum-1);
		speeds.push_back(RPM_MAX);

		wcets = generate_wcets(avrTaskUtils,speeds);
		if (checkMonotonicallyIncrease(wcets)) break;
	}

#ifdef __DEBUG_DESIGN__
	Utility::output_one_vector(cout,"Speeds",speeds);
	Utility::output_one_vector(cout,"wcets",wcets);
#endif

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,speeds,wcets);
}

void TaskSystemGenerator::generate_dynamic_random_system_for_response_time_analysis(vector<PeriodicTask> &tasks, int n, double U_p, vector<AVRTask*> &avrTasks, int numAVRTask, double U_a, int minModeNum, int maxModeNum, int &avrTaskIndex) {
GENERATION:
	
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,U_p,UTILMIN);

#ifdef __DEBUG_DESIGN__
	Utility::output_one_vector(cout,"utils",utils);
	double sum = accumulate(utils.begin(),utils.end(),0.0);
	cout << "total utilizations = " << sum << endl;
#endif

	double avrTask_Period = ANGULAR_PERIOD/RPM_to_RPmuSEC(RPM_MAX);

	avrTaskIndex = 0;

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;

		PeriodicTask task(wcet,period);
		tasks.push_back(task);

		if (avrTask_Period > period) avrTaskIndex = i+1;

#ifdef __DEBUG_DESIGN__
		cout << "Task " << i << ": wcet = " << wcet << "\tperiod=" <<  period << endl;
#endif
	}

	/*
	bool pSchedulable = false;
	for (int i=0; i<tasks.size(); i++) {
		pSchedulable = SchedAnalysis::exact_analysis_index(tasks,i);
		if (!pSchedulable) {
			break;
		}
	}
	if (!pSchedulable) {
		cout << "Again!" << endl;
		//Utility::output_one_vector(cout,"periods",periods);
		//Utility::output_one_vector(cout,"utils",utils);
		goto GENERATION;
	}
	*/

#ifdef __DEBUG_DESIGN__
	cout << "avrTask period = " << avrTask_Period << endl;
	cout << "avrTask index = " << avrTaskIndex << endl;
#endif

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

#if 0 // Generate all the execution modes, and randomly choose a few into one static AVR task
	int totalModeNum = 8;
	int maxModeIndex = rand()%totalModeNum;

	vector<double> avrTaskUtils;
	for (int i=0; i<totalModeNum; i++) {
		if (i==maxModeIndex)
			avrTaskUtils.push_back(U_a);
		else {
			avrTaskUtils.push_back(Utility::random(UTIL_FACTOR*U_a,U_a));
		}
	}

	vector<double> speeds;
	vector<int> wcets;

	while (true) {
		speeds.clear();
		wcets.clear();
		speeds= Utility::uniforms(true,MODE_SPEED_MIN,MODE_SPEED_MAX,MODE_SPEED_STEP/totalModeNum,totalModeNum-1);
		speeds.push_back(RPM_MAX);

		wcets = generate_wcets(avrTaskUtils,speeds);
		if (checkMonotonicallyIncrease(wcets)) break;
	}

	// Each AVR task has four execution modes
	int eachModeNum = 1;

	for (int i=0; i<numAVRTask; i++) {
		set<int> modeIndexSet;
		while (modeIndexSet.size() != eachModeNum) {
			int modeIndex = rand()%(totalModeNum-1);
			modeIndexSet.insert(modeIndex);
		}

		vector<double> localSpeeds;
		vector<int> localWCETs;
		for (auto modeIndex : modeIndexSet) {
			localSpeeds.push_back(speeds[modeIndex]);
			localWCETs.push_back(wcets[modeIndex]);
		}
		localSpeeds.push_back(speeds.back());
		localWCETs.push_back(wcets.back());

		avrTasks.push_back(new AVRTask(engine,ANGULAR_PERIOD,localSpeeds,localWCETs));
	}

#else // generate the execution modes for each one sAVR

	for (int i=0; i<numAVRTask; i++) {

		int modeNum;
		if (minModeNum == maxModeNum) modeNum = minModeNum;
		else {
			modeNum = minModeNum + rand()%(maxModeNum-minModeNum+1);
		}

		int maxModeIndex = rand()%modeNum;

#ifdef __DEBUG_DESIGN__
		cout << "modeNum = " << modeNum << endl;
		cout << "maxModeIndex = " << maxModeIndex << endl;
#endif

		vector<double> avrTaskUtils;
		for (int j=0; j<modeNum; j++) {
			if (j==maxModeIndex)
				avrTaskUtils.push_back(U_a);
			else {
				avrTaskUtils.push_back(Utility::random(UTIL_FACTOR*U_a,U_a));
			}
		}

		vector<double> speeds;
		vector<int> wcets;

		AVRTask *pAvrTask = NULL;

		int count = 10;

		while (true) {
			speeds.clear();
			wcets.clear();
			speeds= Utility::uniforms(true,MODE_SPEED_MIN,MODE_SPEED_MAX,MODE_SPEED_STEP/modeNum,modeNum-1);
			speeds.push_back(RPM_MAX);

			//if (rand()%4==0) speeds[0] = rand()%150 + 950;

			wcets = generate_wcets(avrTaskUtils,speeds);
			if (!checkMonotonicallyIncrease(wcets)) continue;

			pAvrTask = new AVRTask(engine,ANGULAR_PERIOD,speeds,wcets);

#if 0
	
#if 0 // All in One
			bool schedulable = SchedAnalysis::necessary_only_analysis(tasks,*pAvrTask,avrTaskIndex);
#else
			// Separation: similar to UB
			bool schedulable = false;
			for (int j=0; j< modeNum-1; j++) {
				vector<int> localWCETs;
				vector<double> localSpeeds; 

				localWCETs.push_back(wcets[j]);
				localWCETs.push_back(wcets.back());

				localSpeeds.push_back(speeds[j]);
				localSpeeds.push_back(speeds.back());

				AVRTask localAvrTask(engine,ANGULAR_PERIOD,localSpeeds,localWCETs);
				schedulable = SchedAnalysis::necessary_only_analysis(tasks,localAvrTask,avrTaskIndex);
				if (!schedulable) break;
			}
#endif


			if (schedulable) break;
			else {
				delete pAvrTask;
			}

			count --;
			if (count == 0) { // regenerate the task system
				// Clear all
				for (auto avrTask : avrTasks) 
					delete avrTask;
				avrTasks.clear();
				tasks.clear();
				//cout << "Again!" << endl;
				goto GENERATION;
			}
#else
			break;
#endif

			/*
			cout << "Count!" << endl;
			Utility::output_one_vector(cout,"speeds",speeds);
			Utility::output_one_vector(cout,"wcets",wcets);
			*/
		}


		avrTasks.push_back(pAvrTask);
	}

#endif
}

vector<int> TaskSystemGenerator::generate_wcets(vector<double> utils, vector<double> speeds) {
	vector<int> ret;
	for(int i=0; i<utils.size(); i++) {
		double util = utils[i];
		double speed = speeds[i];
#ifdef __USING_CONSTANT_ACCELERATION__
		int wcet = Utility::round(util*ANGULAR_PERIOD/RPM_to_RPmuSEC(speed));
#else
		Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);
		double omega = RPM_to_RPmSEC(speed);
		int wcet = Utility::round(1000.0*util*engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega,ANGULAR_PERIOD));
#endif
		ret.push_back(wcet);
	}
	return ret;
}

std::vector<int> TaskSystemGenerator::generate_periods(int& n)
{

	vector<int> periods;

#ifdef  __USING_FIXED_PERIODS__
	int nPeriod = sizeof(PERIODS)/sizeof(PERIODS[0]);

#ifdef __IMPLICIT_DEADLINE__
#if 0
	n = min(n,nPeriod);
	set<int> tempPeriods;
	while(tempPeriods.size()!=n) {
		tempPeriods.insert(PERIODS[rand()%nPeriod]);
	}

	for (auto e:tempPeriods) 
		periods.push_back(e);
#else
	while(periods.size()!=n) {
		periods.push_back(PERIODS[rand()%nPeriod]);
	}
#endif

#else
	for (int i=0; i<n; i++) {
		periods.push_back(PERIODS[rand()%nPeriod]);
	}
#endif

	sort(periods.begin(), periods.end());
	
#else
	periods = Utility::integerUniforms(true,PERIODMIN,PERIODMAX,n);
	for (auto &e: periods)
		e *= SCALE_FACTOR;
#endif

	return periods;
}

bool TaskSystemGenerator::checkMonotonicallyIncrease(vector<int> wcets) {
	for(int i=0; i<wcets.size()-1; i++) 
		if (wcets[i] < wcets[i+1]) return false;

	return true;
}

void TaskSystemGenerator::generate_random_runnables_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, double d1, double d2, AVRTask* &avrTask, int modeNum) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,tUtil);

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;
		int deadline = Utility::round(1.0*(period-wcet)*Utility::random(d1,d2))+wcet;

		PeriodicTask task(wcet,deadline,period);
		tasks.push_back(task);

		#ifdef __DEBUG_DESIGN__
		cout << "Task " << i << ": wcet = " << wcet << "\tperiod=" <<  period << endl;
		#endif
	}

	// Sort the periodic tasks based on the deadline, using the deadline monotonic priority
	sort(tasks.begin(),tasks.end(),OptimizationAlgorithm::SortByDeadline);

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);


	#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
	#endif

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}

void TaskSystemGenerator::generate_fuel_injection_applications_for_engine_optimization(vector<PeriodicTask> &tasks, AVRTask* &avrTask, bool fixedWCET, int modeNum) {
#if 1
	PeriodicTask task0(208,4000,4000);
	PeriodicTask task1(340,8000,8000);
	PeriodicTask task2(39,4000,4000);
	PeriodicTask task3(148,8000,8000);
	PeriodicTask task4(100,8000,8000);
	PeriodicTask task5(5,5000,5000);
	PeriodicTask task6(300,200000,200000);
	PeriodicTask task7(1000,200000,200000);
	PeriodicTask task8(1000,50000,50000);
	PeriodicTask task9(820,12000,12000);
	PeriodicTask task10(9846,100000,100000);
	PeriodicTask task11(30000,200000,200000);
	PeriodicTask task12(22000,200000,200000);
	PeriodicTask task13(22000,200000,200000);
	PeriodicTask task14(22000,200000,200000);
	PeriodicTask task15(26220,200000,200000);

	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);
	tasks.push_back(task5);
	tasks.push_back(task6);
	tasks.push_back(task7);
	tasks.push_back(task8);
	tasks.push_back(task9);
	tasks.push_back(task10);
	tasks.push_back(task11);
	tasks.push_back(task12);
	tasks.push_back(task13);
	tasks.push_back(task14);
	tasks.push_back(task15);

#else

	PeriodicTask task0(208,4000,4000);
	PeriodicTask task1(340,8000,8000);
	PeriodicTask task2(39,4000,4000);
	PeriodicTask task3(148,8000,8000);
	PeriodicTask task4(100,8000,8000);
	PeriodicTask task5(5,5000,5000);
	PeriodicTask task6(1000,50000,50000);
	PeriodicTask task7(820,12000,12000);
	PeriodicTask task8(9846,100000,100000);
	PeriodicTask task9(123520,200000,200000);

	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);
	tasks.push_back(task5);
	tasks.push_back(task6);
	tasks.push_back(task7);
	tasks.push_back(task8);
	tasks.push_back(task9);

#endif

	// test the total utilization
	double tUtil = 0;
	for (auto task : tasks) {
		double util = (1.0*task.wcet)/task.period;
		cout << task.wcet << "," << task.period << "=>" << util << "," << tUtil <<  endl;
		tUtil += util;

	}
	cout << "total utilization = " << tUtil << endl;

	// test the schedulability
	bool schedulable = false;
	for (int i=0; i<tasks.size(); i++) {
		schedulable = SchedAnalysis::exact_analysis_index(tasks,i);
		if (!schedulable) break;
	}
	cout << "schedulability of the periodic tasks is " << schedulable << endl;


	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets;
	
	if (fixedWCET) {
		int s =1;
		wcets.push_back(s*30);
		wcets.push_back(s*278);
		wcets.push_back(s*344);
		wcets.push_back(s*425);
		wcets.push_back(s*576);
		wcets.push_back(s*966);
		sort(wcets.begin(),wcets.end(),greater<int>());
	}
	else wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
#endif

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}

void TaskSystemGenerator::generate_fuel_injection_applications_for_engine_optimization2(vector<PeriodicTask> &tasks, AVRTask* &avrTask, bool fixedWCET, int modeNum) {
	
#if 1
	PeriodicTask task0(39,4000,4000);
	PeriodicTask task1(300,200000,200000);
	PeriodicTask task2(1000,200000,200000);
	PeriodicTask task3(1000,50000,50000);
	PeriodicTask task4(820,12000,12000);
	PeriodicTask task5(9846,100000,100000);
	PeriodicTask task6(30000,200000,200000);
	PeriodicTask task7(20000,200000,200000);
	PeriodicTask task8(22000,200000,200000);
	PeriodicTask task9(24000,200000,200000);
	PeriodicTask task10(26220,200000,200000);

	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);
	tasks.push_back(task5);
	tasks.push_back(task6);
	tasks.push_back(task7);
	tasks.push_back(task8);
	tasks.push_back(task9);
	tasks.push_back(task10);
#else
	PeriodicTask task0(39,4000,4000);
	PeriodicTask task1(1000,50000,50000);
	PeriodicTask task2(820,12000,12000);
	//PeriodicTask task3(9846,100000,100000);
	//PeriodicTask task4(123520,200000,200000);

	PeriodicTask task3(71606,100000,100000);

	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	//tasks.push_back(task4);
#endif

	// test the total utilization
	double tUtil = 0;
	for (auto task : tasks) {
		double util = (1.0*task.wcet)/task.period;
		cout << task.wcet << "," << task.period << "=>" << util << "," << tUtil <<  endl;
		tUtil += util;

	}
	cout << "total utilization = " << tUtil << endl;

	// test the schedulability
	bool schedulable = false;
	for (int i=0; i<tasks.size(); i++) {
		schedulable = SchedAnalysis::exact_analysis_index(tasks,i);
		if (!schedulable) break;
	}
	cout << "schedulability of the periodic tasks is " << schedulable << endl;


	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets;

	if (fixedWCET) {
		double s = 1.0; // 1010.0/150; // (796.0/150.0);
		wcets.push_back(s*150);
		wcets.push_back(s*278);
		wcets.push_back(s*344);
		wcets.push_back(s*425);
		wcets.push_back(s*576);
		wcets.push_back(s*966);
		sort(wcets.begin(),wcets.end(),greater<int>());
	}
	else wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
#endif

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}

void TaskSystemGenerator::generate_fuel_injection_applications_for_engine_optimization2(vector<AsynchronousPeriodicTask> &tasks, AVRTask* &avrTask, bool fixedWCET, int modeNum) {

	PeriodicTask task0(39,4000,4000);
	PeriodicTask task1(300,200000,200000);
	PeriodicTask task2(1000,200000,200000);
	PeriodicTask task3(1000,50000,50000);
	PeriodicTask task4(820,12000,12000);
	PeriodicTask task5(9850,100000,100000);
	PeriodicTask task6(30000,200000,200000);
	PeriodicTask task7(20000,200000,200000);
	PeriodicTask task8(22000,200000,200000);
	PeriodicTask task9(24000,200000,200000);
	PeriodicTask task10(26220,200000,200000);

	vector<PeriodicTask> pTasks;
	pTasks.push_back(task0);
	pTasks.push_back(task1);
	pTasks.push_back(task2);
	pTasks.push_back(task3);
	pTasks.push_back(task4);
	pTasks.push_back(task5);
	pTasks.push_back(task6);
	pTasks.push_back(task7);
	pTasks.push_back(task8);
	pTasks.push_back(task9);
	pTasks.push_back(task10);

	/// Add the offset for the periodic tasks
	int gcd = 0;
	for (auto task : pTasks) {
		gcd = Utility::math_gcd(gcd,task.period);
	}

	for (auto task : pTasks) {
		int offset = (rand() % (task.period/gcd)) * gcd;
		tasks.push_back(AsynchronousPeriodicTask(task,offset));
	}

	// test the total utilization
	double tUtil = 0;
	for (auto task : tasks) {
		double util = (1.0*task.wcet)/task.period;
#ifndef __DISABLE_MOST_OUTPUT__
		cout << task.wcet << "," << task.period << "=>" << util << "," << tUtil <<  endl;
#endif
		tUtil += util;

	}
#ifndef __DISABLE_MOST_OUTPUT__
	cout << "total utilization = " << tUtil << endl;
#endif

	// test the schedulability
	bool schedulable = false;
	for (int i=0; i<tasks.size(); i++) {
		schedulable = SchedAnalysis::asynchronous_exact_analysis_index(tasks,i);
		if (!schedulable) break;
	}
#ifndef __DISABLE_MOST_OUTPUT__
	cout << "schedulability of the periodic tasks is " << schedulable << endl;
#endif


	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets;

	if (fixedWCET) {
		double s = 1.0; // 1010.0/150; // (796.0/150.0);
		wcets.push_back(s*150);
		wcets.push_back(s*278);
		wcets.push_back(s*344);
		wcets.push_back(s*425);
		wcets.push_back(s*576);
		wcets.push_back(s*966);
		sort(wcets.begin(),wcets.end(),greater<int>());
	}
	else {
		if (modeNum == 3) {
			int tempSize = 6;
			vector<int> temp = Utility::integerUniforms(false,CMIN,CMAX,CSEP,tempSize);
			for (int i=0;i<modeNum; i++) wcets.push_back(temp[i]);
		}
		else 
			wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);
	}

#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
#endif

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}

void TaskSystemGenerator::generate_random_runnables_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, double d1, double d2, vector<int> &wcets, int modeNum) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,tUtil);

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;
		int deadline = Utility::round(1.0*(period-wcet)*Utility::random(d1,d2))+wcet;

		PeriodicTask task(wcet,deadline,period);
		tasks.push_back(task);

#ifdef __DEBUG_DESIGN__
		cout << "Task " << i << ": wcet = " << wcet << "\tperiod=" <<  period << endl;
#endif
	}

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

#ifdef __DEBUG_DESIGN__
	cout << "wcets = ";
	for (int i=0; i<modeNum; i++) cout << wcets[i] << "\t";
	cout << endl;
#endif
}

void TaskSystemGenerator::generate_random_runnables_for_engine_optimization(vector<AsynchronousPeriodicTask> &tasks, int n, double tUtil, double d1, double d2, AVRTask* &avrTask, int modeNum) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,tUtil);

	/// Add the offset for the periodic tasks
	int gcd = 0;
	for (auto period : periods) {
		gcd = Utility::math_gcd(gcd,period);
	}

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;
		int deadline = Utility::round(1.0*(period-wcet)*Utility::random(d1,d2))+wcet;
		int offset = (rand() % (period/gcd)) * gcd;

		AsynchronousPeriodicTask task(wcet,deadline,period,offset);
		tasks.push_back(task);
	}

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}

void TaskSystemGenerator::generate_random_runnables_for_engine_optimization(vector<AsynchronousPeriodicTask> &tasks, int n, double tUtil,double o1, double o2, double d1, double d2, AVRTask* &avrTask, int modeNum) {
	vector<int> periods = generate_periods(n);
	vector<double> utils = Utility::uniformly_distributed(n,tUtil);

	for (int i=0; i<periods.size(); i++) {
		int period = periods[i];
		int wcet = utils[i] * period;
		int deadline = Utility::round(1.0*(period-wcet)*Utility::random(d1,d2))+wcet;
		int offset = Utility::round(1.0*period*Utility::random(o1,o2));

		AsynchronousPeriodicTask task(wcet,deadline,period,offset);
		tasks.push_back(task);
	}

	Engine engine(RPM_MIN,RPM_MAX,ALPHA,ALPHA);

	vector<int> wcets = Utility::integerUniforms(false,CMIN,CMAX,CSEP,modeNum);

	avrTask = new AVRTask(engine,ANGULAR_PERIOD,wcets);
}