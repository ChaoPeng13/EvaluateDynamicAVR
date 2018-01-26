#ifndef SCHEDANALYSIS_H_
#define SCHEDANALYSIS_H_

#include "AVRTask.h"
#include "Digraph.h"
#include "PeriodicTask.h"
#include "AsynchronousPeriodicTask.h"
#include "RequestFunction.h"
#include "OptimizationAlgorithm.h"
#include "Global.h"
#include "ODRTSearchPath.h"
#include "AVLTree.h"

class SchedAnalysis {
public:
	static int allTestNumber;
	static int nonPairNumber;
	static int falseNumber;
	static double savedTime;
	static map<int,int> rts;

#ifdef __USING_ILPCPLEX__
	static map<int,int> ILPs; // (interval time (us), ilp (us)): record the ILP value for the one AVR task
	static map<int,map<int,int>> ILPCONs; // (initial speed (rpm), interval time (us), ilpcon (us)): record the ILPCON value for the one AVR task
#endif

	static vector<double> getSpeedPartition(AVRTask avrTask, int partitionGranularity = -1);
	static vector<double> getSpeedPartition(AVRTask avrTask0, AVRTask avrTask1, int switchTime, int partitionGranularity = -1);
	static vector<double> getSpeedPartitionWithConstLength(AVRTask avrTask, int rpmRange = -1);
	static vector<double> getSpeedPartitionWithConstLength(AVRTask avrTask0, AVRTask avrTask1, int switchTime, int rpmRange = -1);
	static Digraph* generateOffsetBasedDRT(AVRTask avrTask, vector<double> speedPartition);
	static Digraph* generateOffsetBasedDRT(AVRTask avrTask0, AVRTask avrTask1, int switchTime, vector<double> speedPartition);

	static bool checkSchedulabilityAllPriorities(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer nextAvrTask = NULL);
	static bool checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer nextAvrTask = NULL);
	static bool checkSchedulabilityBiondiFP(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask);
	static bool checkSchedulabilityBiondiFP(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);

	static bool checkSchedulabilityAllPriorities(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int switchTime, int granularity = -1);
	static bool checkSchedulabilityAllPrioritiesFP(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int avrTaskIndex, int switchTime, int granularity = -1);
	static bool checkSchedulabilityAllPrioritiesDebug(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int switchTime, int& partitionNum, int granularity = -1);

	static bool checkSchedulabilityAllPriorities(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);
	static bool checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static bool checkSchedulabilityAllPriorities(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int switchTime);
	static bool checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask prevAvrTask, AVRTask avrTask, int switchTime);

	static bool checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<Task*> tasks, Task* task, bool synchronous, AVRTaskPointer nextAvrTask = NULL);
	static bool checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<Task*> tasks, Task* task, AVRTask prevAvrTask, AVRTask avrTask, int switchTime);

	static bool checkSchedulabilityAudsleyOPA(vector<PeriodicTask> tasks);

	static bool exact_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer nextAvrTask = NULL);
	static bool exact_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, int& uIndex, AVRTaskPointer nextAvrTask = NULL);
	static bool exact_analysis_index(vector<PeriodicTask> tasks, int i);
	static bool exact_analysis_index(vector<PeriodicTask> tasks, int i, int wcet, int deadline);
	static bool exact_analysis_index_for_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);
	static bool exact_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer nextAvrTask = NULL);
	static void exact_response_time_analysis(vector<PeriodicTask> tasks, int i, AVRTask avrtask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludedList, vector<double>& returnExcludeList, AVRTaskPointer nextAvrTask = NULL);
	
	static bool exact_analysis_avrtask_to_digraph(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, int granularity = -1);
	static bool exact_analysis_index_with_digraph(vector<PeriodicTask> tasks, int i, Digraph* digraph);
	static void exact_response_time_analysis_digraph(vector<PeriodicTask> tasks, int i, Digraph* digraph, int &maxRT, Node* node, int sum_wcet, int sum_period, int maxTime, vector<Node*> excludedList, vector<Node*>& returnExcludeList);

	static bool necessary_only_analysis(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool necessary_only_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);
	static void necessary_only_response_time_analysis(bool updown, vector<PeriodicTask> tasks, int i, AVRTask avrtask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludedList, vector<double>& returnExcludeList);

	static bool segmental_necessary_only_analysis(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool segmental_necessary_only_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static bool segmental_lub_necessary_only_analysis(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool segmental_lub_necessary_only_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static bool necessary_only_analysis2(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool necessary_only_analysis_index_with_avrtask2(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);
	static void necessary_only_response_time_analysis2(bool updown, vector<PeriodicTask> tasks, int i, AVRTask avrtask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludedList, vector<double>& returnExcludeList);

	static bool necessary_only_analysis3(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool necessary_only_analysis_index_with_avrtask3(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);
	static void necessary_only_response_time_analysis3(bool updown, vector<PeriodicTask> tasks, int i, AVRTask avrtask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludedList, vector<double>& returnExcludeList);

	static bool necessary_only_analysis1A(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool necessary_only_analysis_index_with_avrtask1A(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static bool segmental_necessary_only_analysis1A(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool segmental_necessary_only_analysis_index_with_avrtask1A(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static bool necessary_only_analysis2A(vector<PeriodicTask> tasks, AVRTask AVRTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool necessary_only_analysis_index_with_avrtask2A(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static bool rbf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);
	static bool rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega);
	
	static bool rbf_envelope_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool rbf_envelope_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);

	static bool digraph_rbf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool digraph_rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);
	static bool digraph_rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega);

	static bool digraph_rbf_envelope_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool digraph_rbf_envelope_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);

#ifdef __USING_ILPCPLEX__
	static bool ilp_con_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool ilp_con_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);
	static bool ilp_con_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega);

	static bool ilp_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool ilp_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);

	static bool segmental_ilp_con_exact_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool segmental_ilp_con_exact_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);

	static bool segmental_ilp_con_digraph_rbf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool segmental_ilp_con_digraph_rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);
#endif

	static bool ibf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool ibf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask);
	static bool ibf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega);

	static bool lub_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask = NULL);
	static bool lub_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask = NULL);

	static double calculate_response_time(vector<PeriodicTask> tasks, int n, int wcet);
	static double calculate_response_time(PeriodicTask** tasks, int n, RequestFunction* rf, double omegas[], int wcets[], int numMode, int wcet);

	static double calculate_rbf_response_time(vector<PeriodicTask> tasks, int n, map<int,int> interference, int sum_wcet, int sum_period);
	static double calculate_ibf_response_time(vector<PeriodicTask> tasks, int n, map<int,int> interference, int sum_wcet, int sum_period);
	static double calculate_lub_response_time(vector<PeriodicTask> tasks, int n, AVRTask avrTask, int wcet);
	static double calculate_lub_response_time(vector<PeriodicTask> tasks, int n, AVRTask avrTask, int wcet, int deadline, AVRTaskPointer prevAvrTask = NULL);

	static void draw_response_time(PeriodicTask** tasks, int n, int wcet, int deadline);

	//////////////////////////////////////////////////////////////////////////
	/// using the analysis method proposed in the reference paper 
	/// "Response Time Analysis of Asynchronous Periodic 
	/// and Sporadic Tasks Scheduled by a Fixed-Priority Preemptive Algorithm" 
	//////////////////////////////////////////////////////////////////////////

	static bool asynchronous_exact_analysis(vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static bool asynchronous_exact_analysis_index(vector<AsynchronousPeriodicTask> tasks, int i);
	static bool asynchronous_exact_analysis_index(vector<AsynchronousPeriodicTask> tasks, int i, int wcet, int deadline, int start);
	static bool asynchronous_exact_analysis_index_for_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask);
	static bool asynchronous_exact_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask);
	static void asynchronous_exact_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrtask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, int start, int release, vector<double> excludedList, vector<double>& returnExcludeList);
	static void asynchronous_exact_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrtask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, int start, int pWCET, int release, vector<double> excludedList, vector<double>& returnExcludeList);

	/// Note: this function should be called when we have determined avrTask0 and avrTask1 are schedulable respectively.
	static bool asynchronous_exact_analysis(vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask0, AVRTask avrTask1, int avrTaskIndex, int switchTime);
	static bool asynchronous_exact_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask0, AVRTask avrTask1, int switchTime);
	static void asynchronous_exact_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask0, AVRTask avrTask1, int switchTime, AVRTask mixedAvrTask, bool hasSwitched, double initialOmega, int initialWCET, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, int start, int release, vector<double> excludedList, vector<double>& returnExcludeList);

	static bool asynchronous_sched_analysis(vector<AsynchronousPeriodicTask> tasks, Digraph* digraph, AVRTask mixedAvrTask, int avrTaskIndex, int switchTime);
	static bool asynchronous_sched_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime);
	static void asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int release, vector<Node*> excludeList, vector<Node*>& returnedExcudeList);
	static void asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int pWCET, int release, vector<Node*> excludeList, vector<Node*>& returnedExcudeList);
	static void asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, list<Node*> nodeList, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int release, vector<Node*> excludeList, vector<Node*>& returnedExcudeList);
	static void asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, list<Node*> nodeList, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int pWCET, int release, vector<Node*> excludeList, vector<Node*>& returnedExcudeList);
	
	// RTA_dAVR/v0.3/2017-10-28
	static bool asynchronous_sched_analysis_index_with_avrtask_refinement(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, AVRTask mixedAvrTask, int switchTime);
	static void asynchronous_sched_response_time_analysis_refinement(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime, int start, int pWCET, int release);
	static void asynchronous_sched_response_time_analysis_refinement(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime, int start, int pWCET, int release,vector<NodeItem> excludedList, vector<NodeItem>& returnExcludedList);
	static int minimum_binary_search(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, int start, int pWCET, int release, Edge* edge, int tLB, int tUB);
	static int maximum_binary_search(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, int start, int pWCET, int release, Edge* edge, int tLB, int tUB);
	static ODRTSearchPath generate_ODRT_search_path(ODRTSearchPath path, int tLB, int tUB, int tMinus, int tPlus, Node* node, Edge* edge);
	static bool check_next_vertex(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath& path, int CSum, int start, int pWCET, int release, int fMinus, int fPlus, Node* node, int& tMinus, int& tPlus);

	// RTA_dAVR/v0.3/2017-10-28
	static bool sched_analysis_with_avrtask_refinement(vector<PeriodicTask> tasks, Digraph* digraph, AVRTask mixedAvrTask, AVRTask avrTask, int avrTaskIndex, int switchTime);
	static bool sched_analysis_index_with_avrtask_refinement(vector<PeriodicTask> tasks, int i, Digraph* digraph, AVRTask mixedAvrTask, AVRTask avrTask, int switchTime);
	static void sched_response_time_analysis_refinement(vector<PeriodicTask> tasks, int i, Digraph* digraph, AVRTask avrTask, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime, map<Node*,AVLTree*>& record);
	static void sched_response_time_analysis_refinement(vector<PeriodicTask> tasks, int i, Digraph* digraph, AVRTask avrTask, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime, map<Node *, AVLTree *>& record,vector<NodeItem> excludedList, vector<NodeItem>& returnExcludedList);
	static int minimum_binary_search(vector<PeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, Edge* edge, int tLB, int tUB);
	//static int maximum_binary_search(vector<PeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, Edge* edge, int tLB, int tUB);
	static void exact_response_time_analysis_digraph_max(vector<PeriodicTask> tasks, int i, Digraph* digraph, int &maxRT, Node* node, int sum_wcet, int sum_period, int maxTime, vector<Node*> excludedList, vector<Node*>& returnExcludeList);
	
	static bool asynchronous_sufficient_sched_analysis(vector<AsynchronousPeriodicTask> tasks, Digraph* digraph, int avrTaskIndex, int switchTime);
	static bool asynchronous_sufficient_sched_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime);
	static void asynchronous_sufficient_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, bool afterSwitch, int &maxRT, Node* node, int sum_wcet, int sum_period, int maxTime, int start, int release, vector<Node*> excludeList, vector<Node*>& returnedExcudeList);

	static int computingMaxReleaseTime(vector<AsynchronousPeriodicTask> tasks, int i);
	static int computingPeriodLCM(vector<AsynchronousPeriodicTask> tasks, int i);
	/// Computing L_i(t) shown in Fig. 3 in this paper and the improved one shown in Fig. 11
	static int computingLit(vector<AsynchronousPeriodicTask> tasks, int i, int t, int last_known_idle = 0);
	static vector<int> computing_asynchronous_critial_instants(vector<AsynchronousPeriodicTask> tasks, int i, int t_min, int t_max, int last_known_idle = 0);
	static int computing_next_idle_instants(vector<AsynchronousPeriodicTask> tasks, int i, int work_instant);
	static double calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, int wcet, int start, int release = NULL);
	static double calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, int wcet, int start, int pWCET, int release);
	static double calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, PeriodicTask pTask, int wcet, int start, int release = NULL);
	static double calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, PeriodicTask pTask, int wcet, int start, int pWCET, int release);
};

#endif