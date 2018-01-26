#ifndef TASKSYSTEMGENERATOR_H_
#define TASKSYSTEMGENERATOR_H_

#include "AVRTask.h"
#include "PeriodicTask.h"
#include "AsynchronousPeriodicTask.h"
#include "Global.h"

#include <algorithm>

class TaskSystemGenerator {
public:
#ifdef __USING_BIONDI_PERIOD_SETTING__
	static int PERIODS[6];
	//static int PERIODS[5];
#endif

#ifdef __USING_FUEL_INJECTION_PERIOD_SETTING__
	static int PERIODS[5];
#endif

#ifdef __USING_REAL_WORLD_PERIOD_SETTTING__
	//static int PERIODS[10];
	static int PERIODS[8];
#endif

	static int RPM_MIN;
	static int RPM_MAX;
	static double ALPHA;
	static double ANGULAR_PERIOD;

	static int CMIN;
	static int CMAX;
	static int CSEP;

	static int KMIN;
	static int KMAX;
	static int KSEP;

	static double UTILMIN;
	static int PERIODMIN;
	static int PERIODMAX;
	static int SCALE_FACTOR; // msec to musec
	static double UTIL_FACTOR;

	static double MODE_SPEED_MIN;
	static double MODE_SPEED_MAX;
	static double MODE_SPEED_STEP;

	static double MIN_SEPERATION_FACTOR;

	static Engine generate_engine();
	//************************************
	// Method:    generate_random_system_for_engine_optimization, to generate the random system similar to the ICCPS16 paper 
	//            "Performance-driven Design of Engine Control Tasks".
	// FullName:  TaskSystemGenerator::generate_random_system_for_engine_optimization
	// Access:    public static 
	// Returns:   void
	// Qualifier:
	// Parameter: vector<PeriodicTask> & tasks
	// Parameter: int n
	// Parameter: double tUtil
	// Parameter: AVRTask *  & avrTask
	// Parameter: int modeNum
	//************************************
	static void generate_random_system_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, AVRTask* &avrTask, int modeNum);
	static void generate_random_system_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, vector<int> &wcets, int modeNum);
	static vector<double> generate_coefficients(int n);
	static vector<double> generate_coefficients_k2(int n, double a, double b);
	static vector<double> generate_coefficients_k2(int n, double kMin, double kMax, double kStep);
	static vector<double> generate_Biondi_coefficients_k2(int n, double kMin, double kMax);
	//************************************
	// Method:    generate_random_system_for_response_time_analysis, to generate the random system similar to the ICCPS15 paper
	//			  "Response-Time Analysis for Real-Time Tasks in Engine Control Applications"
	// FullName:  TaskSystemGenerator::generate_random_system_for_response_time_analysis
	// Access:    public static 
	// Returns:   void
	// Qualifier:
	// Parameter: vector<PeriodicTask> & tasks
	// Parameter: int n
	// Parameter: double U_p
	// Parameter: AVRTask *  & avrTask
	// Parameter: double U_a
	// Parameter: int minModeNum
	// Parameter: int maxModeNum
	// Parameter: int & avrTaskIndex
	//************************************
	static void generate_random_system_for_response_time_analysis(vector<PeriodicTask> &tasks, int n, double U_p, AVRTask* &avrTask, double U_a, int minModeNum, int maxModeNum, int &avrTaskIndex);
	static void generate_dynamic_random_system_for_response_time_analysis(vector<PeriodicTask> &tasks, int n, double U_p, vector<AVRTask*> &avrTasks, int numAvrTask, double U_a, int minModeNum, int maxModeNum, int &avrTaskIndex);
	static vector<int> generate_wcets(vector<double> utils, vector<double> speeds);
	static vector<int> generate_periods(int& n);
	static bool checkMonotonicallyIncrease(vector<int> wcets);

	static void generate_random_runnables_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, double d1, double d2, AVRTask* &avrTask, int modeNum);
	static void generate_random_runnables_for_engine_optimization(vector<PeriodicTask> &tasks, int n, double tUtil, double d1, double d2, vector<int> &wcets, int modeNum);

	/// fuel injection applications in the paper "Efficient implementation of AUTOSAR components with minimal memory usage"
	static void generate_fuel_injection_applications_for_engine_optimization(vector<PeriodicTask> &tasks, AVRTask* &avrTask, bool fixedWCET, int modeNum);
	/// fuel injection applications in the paper "Improving the Size of Communication Buffers in Synchronous Models With Time Constraints"
	static void generate_fuel_injection_applications_for_engine_optimization2(vector<PeriodicTask> &tasks, AVRTask* &avrTask, bool fixedWCET, int modeNum);
	static void generate_fuel_injection_applications_for_engine_optimization2(vector<AsynchronousPeriodicTask> &tasks, AVRTask* &avrTask, bool fixedWCET, int modeNum);

	static void generate_random_runnables_for_engine_optimization(vector<AsynchronousPeriodicTask> &tasks, int n, double tUtil, double d1, double d2, AVRTask* &avrTask, int modeNum);
	static void generate_random_runnables_for_engine_optimization(vector<AsynchronousPeriodicTask> &tasks, int n, double tUtil, double o1, double o2, double d1, double d2, AVRTask* &avrTask, int modeNum);
};

#endif