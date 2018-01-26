#include "SchedAnalysis.h"
#include "Timer.h"
#include "Digraph.h"
#include "RunTest.h"
#include "ODRTSearchPath.h"

#include <algorithm>
#include <functional>
#include <math.h>
#include <set>

int SchedAnalysis::allTestNumber = 0;
int SchedAnalysis::nonPairNumber = 0;
int SchedAnalysis::falseNumber = 0;
double SchedAnalysis::savedTime = 0;

map<int,int> SchedAnalysis::rts;

#ifdef __USING_ILPCPLEX__
map<int,int> SchedAnalysis::ILPs; // (interval time (us), ilp (us)): record the ILP value for the one AVR task
map<int,map<int,int>> SchedAnalysis::ILPCONs; // (initial speed (rpm), interval time (us), ilpcon (us)): record the ILPCON value for the one AVR task
#endif

std::vector<double> SchedAnalysis::getSpeedPartition(AVRTask avrTask, int partitionGranularity /*= -1*/)
{
	vector<double> speedIntervals;
	if (partitionGranularity <= 0)
		speedIntervals = avrTask.getAllSpeedRanges();
	else
		speedIntervals = avrTask.getSpeedPartitionWithGranularity(partitionGranularity);

	return speedIntervals;
}

std::vector<double> SchedAnalysis::getSpeedPartition(AVRTask avrTask0, AVRTask avrTask1, int switchTime, int partitionGranularity /*= -1*/)
{
	/// computing the dominants of the mixed AVR task 
	Engine engine = avrTask0.engine;
	vector<double> tranSpeeds0;
	for(auto e : avrTask0.speeds)
		tranSpeeds0.push_back(e.second);

	vector<double> tranSpeeds1;
	for (auto e : avrTask1.speeds)
		tranSpeeds1.push_back(e.second);

	vector<double> mixedTranSpeeds = Utility::merge(tranSpeeds0,tranSpeeds1);
	//cout << mixedTranSpeeds.size() << endl;
	//Utility::output_one_vector(cout,"trans",mixedTranSpeeds);
	//exit(EXIT_FAILURE);
	AVRTask mixedAvrTask(engine,avrTask0.period,mixedTranSpeeds);
	vector<double> speedIntervals;
	if (partitionGranularity <= 0)
		speedIntervals = mixedAvrTask.getAllSpeedRanges();
	else
		speedIntervals = mixedAvrTask.getSpeedPartitionWithGranularity(partitionGranularity);

	return speedIntervals;
}

std::vector<double> SchedAnalysis::getSpeedPartitionWithConstLength(AVRTask avrTask, int rpmRange /*= -1*/)
{
	/// computing the dominants of the AVR task 
	Engine engine = avrTask.engine;
	vector<double> tranSpeeds;
	for(auto e : avrTask.speeds)
		tranSpeeds.push_back(e.second);

	vector<double> speedIntervals;
	if (rpmRange <= 0)
		speedIntervals = avrTask.getAllSpeedRanges();
	else {
		double speed = engine.RPM_MAX;
		while (speed > engine.RPM_MIN) {
			speedIntervals.push_back(speed);
			speed -= rpmRange;
		}

		speedIntervals = Utility::merge(speedIntervals,tranSpeeds);
		speedIntervals.push_back(engine.RPM_MIN);

		// Transform RPM to RPmSEC
		for (auto& e : speedIntervals)
			e = RPM_to_RPmSEC(e);
	}

	return speedIntervals;
}

std::vector<double> SchedAnalysis::getSpeedPartitionWithConstLength(AVRTask avrTask0, AVRTask avrTask1, int switchTime, int rpmRange /*= -1*/)
{
	/// computing the dominants of the mixed AVR task 
	Engine engine = avrTask0.engine;
	vector<double> tranSpeeds0;
	for(auto e : avrTask0.speeds)
		tranSpeeds0.push_back(e.second);

	vector<double> tranSpeeds1;
	for (auto e : avrTask1.speeds)
		tranSpeeds1.push_back(e.second);

	vector<double> mixedTranSpeeds = Utility::merge(tranSpeeds0,tranSpeeds1);

#if 0
	cout << mixedTranSpeeds.size() << endl;
	Utility::output_one_vector(cout,"trans",mixedTranSpeeds);
	exit(EXIT_FAILURE);
#endif

	AVRTask mixedAvrTask(engine,avrTask0.period,mixedTranSpeeds);
	vector<double> speedIntervals;
	if (rpmRange <= 0)
		speedIntervals = mixedAvrTask.getAllSpeedRanges();
	else {
		double speed = engine.RPM_MAX;
		while (speed > engine.RPM_MIN) {
			speedIntervals.push_back(speed);
			speed -= rpmRange;
		}

		speedIntervals = Utility::merge(speedIntervals,mixedTranSpeeds);
		speedIntervals.push_back(engine.RPM_MIN);

		// Transform RPM to RPmSEC
		for (auto& e : speedIntervals)
			e = RPM_to_RPmSEC(e);
	}

	return speedIntervals;
}

Digraph* SchedAnalysis::generateOffsetBasedDRT(AVRTask avrTask, vector<double> speedPartition)
{
	Digraph* digraph = new Digraph(0);
	//digraph->switchTime = switchTime;
	digraph->minVertexC = (*avrTask.wcets.rbegin()).second;
	map<int,int> modePeriods = avrTask.getModePeriods();
	digraph->minVertexT = (*modePeriods.rbegin()).second;

	/// computing the dominants of the mixed AVR task 
	Engine engine = avrTask.engine;

	vector<double> omegaLs;
	vector<double> omegaHs;

	for(int i=0; i<speedPartition.size()-1; i++) {
		omegaHs.push_back(speedPartition[i]);
		omegaLs.push_back(speedPartition[i+1]);
	}

	// Build a node for each speed interval
	// Force vector nodes to be sorted in the descending order of engine speed
	for (int i=0; i<omegaLs.size(); i++) {
		double omegaL = omegaLs[i];
		double omegaH = omegaHs[i];

		string name = "v"+Utility::int_to_string(i) + "_L" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaL)) + "RPM"
			+ "_H" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaH)) + "RPM";

		Node* node = new Node(name);
		node->omegaL = omegaL;
		node->omegaH = omegaH;

		node->wcet = avrTask.getWCET(omegaH);

		digraph->add_node(node);
	}

	// Build all the edges
	// Force list node->out->snk_node to be sorted in the descending order of engine speed
	for (int i=0; i<digraph->node_vec.size(); i++) {
		Node* src = digraph->node_vec[i];

		for (int j=0; j<digraph->node_vec.size(); j++) {
			Node* snk = digraph->node_vec[j];

			double pmin = engine.getMinTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,avrTask.period);
			double pmax = engine.getMaxTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,avrTask.period);

			if (pmin > pmax)
				cout << "here" << endl;

			if (pmin > 0 && pmax > 0) {
				Edge* edge = new Edge(src,snk);
				edge->separationTime = mSEC_to_muSEC(pmin);
				edge->maxSeparationTime = mSEC_to_muSEC(pmax);
				digraph->add_edge(edge);
			}
		}
	}

	return digraph;
}

Digraph* SchedAnalysis::generateOffsetBasedDRT(AVRTask avrTask0, AVRTask avrTask1, int switchTime, vector<double> speedPartition)
{
	Digraph* digraph = new Digraph(0);
	digraph->switchTime = switchTime;
	digraph->minVertexC = (*avrTask0.wcets.rbegin()).second;
	map<int,int> modePeriods = avrTask0.getModePeriods();
	digraph->minVertexT = (*modePeriods.rbegin()).second;

	/// computing the dominants of the mixed AVR task 
	Engine engine = avrTask0.engine;

	vector<double> omegaLs;
	vector<double> omegaHs;

	for(int i=0; i<speedPartition.size()-1; i++) {
		omegaHs.push_back(speedPartition[i]);
		omegaLs.push_back(speedPartition[i+1]);
	}

	// Build a node for each speed interval
	// Force vector nodes to be sorted in the descending order of engine speed
	for (int i=0; i<omegaLs.size(); i++) {
		double omegaL = omegaLs[i];
		double omegaH = omegaHs[i];

		string name = "v"+Utility::int_to_string(i) + "_L" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaL)) + "RPM"
													+ "_H" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaH)) + "RPM";

		Node* node = new Node(name);
		node->omegaL = omegaL;
		node->omegaH = omegaH;

		node->wcet = avrTask0.getWCET(omegaH);
		node->nextWCET = avrTask1.getWCET(omegaH);
		node->swichTime = switchTime;

		digraph->add_node(node);
	}

	// Build all the edges
	// Force list node->out->snk_node to be sorted in the descending order of engine speed
	for (int i=0; i<digraph->node_vec.size(); i++) {
		Node* src = digraph->node_vec[i];

		for (int j=0; j<digraph->node_vec.size(); j++) {
			Node* snk = digraph->node_vec[j];

			double pmin = engine.getMinTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,avrTask0.period);
			double pmax = engine.getMaxTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,avrTask0.period);

			if (pmin > pmax)
				cout << "here" << endl;

			if (pmin > 0 && pmax > 0) {
				Edge* edge = new Edge(src,snk);
				edge->separationTime = mSEC_to_muSEC(pmin);
				edge->maxSeparationTime = mSEC_to_muSEC(pmax);
				digraph->add_edge(edge);
			}
		}
	}

	return digraph;
}

bool SchedAnalysis::checkSchedulabilityAllPriorities(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer nextAvrTask) {
#ifdef __USING_ILPCPLEX__
	ILPs.clear();
	ILPCONs.clear();
#endif

#ifdef __USING_AUDSLEY_OPA__
	//Timer timer;
	// Build the digraph for the avr task
	if (mc == DIGRAPH_RBF || mc == DIGRAPH_RBF_ENVELOPE) {
		avrTask.buildDigraph();

		/// Prepare the rbfs
		int maxPeriod = 0;
		for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);

		if (mc == DIGRAPH_RBF)
			if (avrTask.digraph_rbf.size()==0) {
				//timer.start();
				avrTask.digraph->calculate_rbf_without_periodicity_DP(maxPeriod,avrTask.digraph_rbf);
				//timer.end();
				//cout << maxPeriod << ">====>" << timer.getTime() << endl;
				//cout << avrTask.digraph->node_vec.size() << "\t" << avrTask.digraph->edge_vec.size() << endl;

				// using periodicity
				/*
				Digraph* digraph = avrTask.digraph;
				digraph->check_strongly_connected();

				if (!digraph->strongly_connected) {
					cerr << "Not strongly connected!" << endl;
					exit(EXIT_FAILURE);
				}

				digraph->calculate_linear_factor();

				digraph->unit_digraph = new UnitDigraph(digraph);
				digraph->unit_digraph->prepare3(false);

				avrTask.digraph->calculate_rbf_with_periodicity_DP(maxPeriod,digraph->linear_factor, digraph->unit_digraph->lper, avrTask.digraph_rbf);
				*/
			}

		if (mc == DIGRAPH_RBF_ENVELOPE)
			if (avrTask.digraph_rbf_envelope.size()==0)
				avrTask.digraph->calculate_rbf_without_periodicity_DP(maxPeriod,avrTask.digraph_rbf_envelope);
	}

	if (mc == SEG_ILPCON_DIGRAPH_RBF) {
		avrTask.buildDigraph();
		if (avrTask.digraph_rbf.size()==0) {
			//timer.start();
			avrTask.digraph->calculate_rbf_without_periodicity_DP(ANALYSIS_SEGMENT,avrTask.digraph_rbf);
			//timer.end();
			//cout << timer.getTime() << endl;
		}
	}

	if (mc == LUB || mc == SEG_LUB_NECESSARY_ONLY1) { // prepare the linear upper bounds
		//avrTask.buildDigraph();
		avrTask.prepareLinearUppperBound();
		if (nextAvrTask != NULL)
			nextAvrTask->prepareLinearUppperBound();

		/*
		Digraph* digraph = avrTask.digraph;
		digraph->linear_factor = avrTask._maxU;
		digraph->calculate_linear_upper_bounds();
		*/
	}

	/*
	// Prepare the seg-ilp-con
	if (mc == SEG_ILPCON) {
		timer.start();
		int maxPeriod = 0;
		for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);

		int nDeadline = floor(1.0*maxPeriod/ANALYSIS_SEGMENT);

		set<int> sDeadlines; // segmental deadlines
		for (int j=1; j<=nDeadline; j++)
			sDeadlines.insert(j*ANALYSIS_SEGMENT);
		sDeadlines.insert(maxPeriod);

		vector<double> dominants = avrTask.getAllDominants();

		for (auto sD : sDeadlines) {
			if (sD == maxPeriod) continue;
			for (auto omega : dominants)
				avrTask.calILPCON(omega,sD);
		}
		timer.end();
		cout << timer.getTime() << endl;
	}
	*/

	//timer.start();
	bool finalSchedulable = checkSchedulabilityAudsleyOPA(mc,tasks,avrTask,nextAvrTask);
	//timer.end();
	//cout << "tDoSched = " << timer.getTime() << endl;

	// delete digraph
	if (mc == DIGRAPH_RBF || mc == DIGRAPH_RBF_ENVELOPE || mc == SEG_ILPCON_DIGRAPH_RBF /*|| mc == SEG_LUB_NECESSARY_ONLY1*/) {
		delete avrTask.digraph;
		avrTask.digraph = NULL;
		avrTask.digraph_rbf.clear();
		avrTask.digraph_rbf_envelope.clear();
	}

	return finalSchedulable;
#endif


#ifdef __USING_PERIODIC_TASKS_DMPO__
	for (unsigned int i = 0; i<tasks.size()+1; i++) {
		bool schedulable;
		switch (mc)
		{
		case EXACT:
			schedulable = exact_analysis(tasks,avrTask,i);
			break;
		case RBF:
			schedulable = rbf_analysis(tasks,avrTask,i);
			break;
		case RBF_ENVELOPE:
			schedulable = rbf_envelope_analysis(tasks,avrTask,i);
			break;
		case DIGRAPH_RBF:
			schedulable = digraph_rbf_analysis(tasks,avrTask,i);
			break;
		case DIGRAPH_RBF_ENVELOPE:
			schedulable = digraph_rbf_envelope_analysis(tasks,avrTask,i);
			break;
		case IBF:
			schedulable = ibf_analysis(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY1:
			schedulable = necessary_only_analysis(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY2:
			schedulable = necessary_only_analysis2(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY3:
			schedulable = necessary_only_analysis3(tasks,avrTask,i);
			break;
		case LUB:
			schedulable = lub_analysis(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY1A:
			schedulable = necessary_only_analysis1A(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY2A:
			schedulable = necessary_only_analysis2A(tasks,avrTask,i);
			break;
		case ILPCON:
			#ifdef __USING_ILPCPLEX__
				schedulable = ilp_con_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		case ILP:
			#ifdef __USING_ILPCPLEX__
				schedulable = ilp_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		case SEG_ILPCON_EXACT:
			#ifdef __USING_ILPCPLEX__
				schedulable = segmental_ilp_con_exact_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		case SEG_ILPCON_DIGRAPH_RBF:
			#ifdef __USING_ILPCPLEX__
				schedulable = segmental_ilp_con_digraph_rbf_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		default:
			cerr << "Error the method choice: " << mc << endl;
			exit(EXIT_FAILURE);
			break;
		}
		if (schedulable) return true;
	}
	return false;
#endif

#ifdef __USING_ALL_TASKS_DMPO__
	/// determine the index of the AVR task
	int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
	int AVRIndex = 0;
	for (int i=0; i<tasks.size(); i++) {
		if (tasks[i].deadline < AVRDeadline)
			AVRIndex = i+1;
	}
	//AVRIndex = 1;
#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif
	//bool schedulable = exact_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif

#ifdef __USING_CONSTANT_PRIORITIES__
	int AVRIndex = 1;
#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif
	//bool schedulable = exact_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif
}

bool SchedAnalysis::checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer nextAvrTask) {
	vector<Task*> vecTask;

	//////////////////////////////////////////////////////////////////////////
	/////////// Collect the tasks in order of the periods        /////////////
	//////////////////////////////////////////////////////////////////////////

	double avrTask_Deadline = avrTask.period/RPM_to_RPmuSEC(avrTask.engine.RPM_MAX);

	bool hasContained = false;

	for (int i=0; i<tasks.size(); i++) {
		int deadline = tasks[i].deadline;

		if (!hasContained && avrTask_Deadline <= deadline) {
			vecTask.push_back(&avrTask);
			hasContained = true;
		}
		vecTask.push_back(&tasks[i]);
	}

	if (!hasContained)
		vecTask.push_back(&avrTask);

	//////////////////////////////////////////////////////////////////////////
	///////////               Audsley's OPA Test                 /////////////
	//////////////////////////////////////////////////////////////////////////

	while (!vecTask.empty()) {
		bool schedulable = false;

		for (int i = vecTask.size()-1; i >= 0; i--) {
			Task* pTask = vecTask[i];

			vector<Task*> hpTasks;
			for (int j=0; j<vecTask.size(); j++) {
				if (j==i) continue;
				hpTasks.push_back(vecTask[j]);
			}

			schedulable = checkSchedulabilityAudsleyOPA(mc,hpTasks,pTask,true,nextAvrTask);

			if (schedulable) {
				vecTask.erase(vecTask.begin()+i);
				break;
			}
		}

		if (!schedulable) return false;
	}

	return true;
}

bool SchedAnalysis::checkSchedulabilityBiondiFP(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask) {
	for (unsigned int i = 0; i<tasks.size()+1; i++) {
		bool schedulable = checkSchedulabilityBiondiFP(mc,tasks,avrTask,i);
		if (schedulable) return true;
	}
	return false;
}

bool SchedAnalysis::checkSchedulabilityBiondiFP(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	bool schedulable;
	switch (mc)
	{
	case EXACT:
		schedulable = exact_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case RBF:
		schedulable = rbf_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case RBF_ENVELOPE:
		schedulable = rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case DIGRAPH_RBF:
		schedulable = digraph_rbf_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case DIGRAPH_RBF_ENVELOPE:
		schedulable = digraph_rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case IBF:
		schedulable = ibf_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case NECESSARY_ONLY1:
		schedulable = necessary_only_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case NECESSARY_ONLY2:
		schedulable = necessary_only_analysis2(tasks,avrTask,avrTaskIndex);
		break;
	case NECESSARY_ONLY3:
		schedulable = necessary_only_analysis3(tasks,avrTask,avrTaskIndex);
		break;
	case LUB:
		schedulable = lub_analysis(tasks,avrTask,avrTaskIndex);
		break;
	case NECESSARY_ONLY1A:
		schedulable = necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
		break;
	case NECESSARY_ONLY2A:
		schedulable = necessary_only_analysis2A(tasks,avrTask,avrTaskIndex);
		break;
	case ILPCON:
#ifdef __USING_ILPCPLEX__
		schedulable = ilp_con_analysis(tasks,avrTask,avrTaskIndex);
#else
		schedulable = true;
#endif
		break;
	case ILP:
#ifdef __USING_ILPCPLEX__
		schedulable = ilp_analysis(tasks,avrTask,avrTaskIndex);
#else
		schedulable = true;
#endif
		break;
	case SEG_ILPCON_EXACT:
#ifdef __USING_ILPCPLEX__
		schedulable = segmental_ilp_con_exact_analysis(tasks,avrTask,avrTaskIndex);
#else
		schedulable = true;
#endif
		break;
	case SEG_ILPCON_DIGRAPH_RBF:
#ifdef __USING_ILPCPLEX__
		schedulable = segmental_ilp_con_digraph_rbf_analysis(tasks,avrTask,avrTaskIndex);
#else
		schedulable = true;
#endif
		break;
	default:
		cerr << "Error the method choice: " << mc << endl;
		exit(EXIT_FAILURE);
		break;
	}

	return schedulable;
}

bool SchedAnalysis::checkSchedulabilityAllPriorities(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int switchTime, int granularity)
{
#ifdef __USING_ILPCPLEX__
	ILPs.clear();
	ILPCONs.clear();
#endif

#ifdef __USING_AUDSLEY_OPA__
	return true;
#endif

#ifdef __USING_ALL_TASKS_DMPO__
	/// determine the index of the AVR task
	int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
	int AVRIndex = 0;
	for (int i=0; i<tasks.size(); i++) {
		if (tasks[i].deadline < AVRDeadline)
			AVRIndex = i+1;
	}
	//AVRIndex = 1;

#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif

#ifdef __PARTITIONING_WITH_CONSTANT_SCALE__
	vector<double> speedPartition = getSpeedPartition(prevAvrTask,avrTask,switchTime,granularity);
#endif

#ifdef __PARTITIONING_WITH_CONSTANT_LENGTH__
	vector<double> speedPartition = getSpeedPartitionWithConstLength(prevAvrTask,avrTask,switchTime,granularity);
#endif

#if 0
	Utility::output_one_vector(cout,"SpeedPartiton",speedPartition);
	cout << speedPartition.size() << endl;
	exit(EXIT_FAILURE);
#endif 

	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime,speedPartition);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;

	bool schedulable = sched_analysis_with_avrtask_refinement(tasks,digraph,mixedAvrTask,avrTask,AVRIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif

#ifdef __USING_CONSTANT_PRIORITIES__
	int AVRIndex = 1;

#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif

	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;

	bool schedulable = sched_analysis_under_arbitrary_offsets(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif
}

bool SchedAnalysis::checkSchedulabilityAllPrioritiesFP(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int avrTaskIndex, int switchTime, int granularity)
{

#ifdef __PARTITIONING_WITH_CONSTANT_SCALE__
	vector<double> speedPartition = getSpeedPartition(prevAvrTask,avrTask,switchTime,granularity);
#endif

#ifdef __PARTITIONING_WITH_CONSTANT_LENGTH__
	vector<double> speedPartition = getSpeedPartitionWithConstLength(prevAvrTask,avrTask,switchTime,granularity);
#endif

#if 0
	Utility::output_one_vector(cout,"SpeedPartiton",speedPartition);
	cout << speedPartition.size() << endl;
	exit(EXIT_FAILURE);
#endif 

	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime,speedPartition);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;

	bool schedulable = sched_analysis_with_avrtask_refinement(tasks,digraph,mixedAvrTask,avrTask,avrTaskIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
}

bool SchedAnalysis::checkSchedulabilityAllPrioritiesDebug(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int switchTime, int& partitionNum, int granularity /* =1 */)
{
#ifdef __USING_ILPCPLEX__
	ILPs.clear();
	ILPCONs.clear();
#endif

#ifdef __USING_AUDSLEY_OPA__
	return true;
#endif

#ifdef __USING_ALL_TASKS_DMPO__
	/// determine the index of the AVR task
	int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
	int AVRIndex = 0;
	for (int i=0; i<tasks.size(); i++) {
		if (tasks[i].deadline < AVRDeadline)
			AVRIndex = i+1;
	}
	//AVRIndex = 1;

#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif

#ifdef __PARTITIONING_WITH_CONSTANT_SCALE__
	vector<double> speedPartition = getSpeedPartition(prevAvrTask,avrTask,switchTime,granularity);
#endif

#ifdef __PARTITIONING_WITH_CONSTANT_LENGTH__
	vector<double> speedPartition = getSpeedPartitionWithConstLength(prevAvrTask,avrTask,switchTime,granularity);
#endif

	partitionNum = speedPartition.size()-1;

#if 0
	Utility::output_one_vector(cout,"SpeedPartiton",speedPartition);
	cout << speedPartition.size() << endl;
	exit(EXIT_FAILURE);
#endif 

	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime,speedPartition);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;

	bool schedulable = sched_analysis_with_avrtask_refinement(tasks,digraph,mixedAvrTask,avrTask,AVRIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif

#ifdef __USING_CONSTANT_PRIORITIES__
	int AVRIndex = 1;

#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif

	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;

	bool schedulable = sched_analysis_under_arbitrary_offsets(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif
}

bool SchedAnalysis::checkSchedulabilityAllPriorities(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer nextAvrTask)
{
	#ifdef __USING_ILPCPLEX__
	ILPs.clear();
	ILPCONs.clear();
#endif

#ifdef __USING_AUDSLEY_OPA__
	//Timer timer;
	// Build the digraph for the avr task
	if (mc == DIGRAPH_RBF || mc == DIGRAPH_RBF_ENVELOPE) {
		avrTask.buildDigraph();

		/// Prepare the rbfs
		int maxPeriod = 0;
		for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);

		if (mc == DIGRAPH_RBF)
			if (avrTask.digraph_rbf.size()==0) {
				//timer.start();
				avrTask.digraph->calculate_rbf_without_periodicity_DP(maxPeriod,avrTask.digraph_rbf);
				//timer.end();
				//cout << maxPeriod << ">====>" << timer.getTime() << endl;
				//cout << avrTask.digraph->node_vec.size() << "\t" << avrTask.digraph->edge_vec.size() << endl;

				// using periodicity
				/*
				Digraph* digraph = avrTask.digraph;
				digraph->check_strongly_connected();

				if (!digraph->strongly_connected) {
					cerr << "Not strongly connected!" << endl;
					exit(EXIT_FAILURE);
				}

				digraph->calculate_linear_factor();

				digraph->unit_digraph = new UnitDigraph(digraph);
				digraph->unit_digraph->prepare3(false);

				avrTask.digraph->calculate_rbf_with_periodicity_DP(maxPeriod,digraph->linear_factor, digraph->unit_digraph->lper, avrTask.digraph_rbf);
				*/
			}

		if (mc == DIGRAPH_RBF_ENVELOPE)
			if (avrTask.digraph_rbf_envelope.size()==0)
				avrTask.digraph->calculate_rbf_without_periodicity_DP(maxPeriod,avrTask.digraph_rbf_envelope);
	}

	if (mc == SEG_ILPCON_DIGRAPH_RBF) {
		avrTask.buildDigraph();
		if (avrTask.digraph_rbf.size()==0) {
			//timer.start();
			avrTask.digraph->calculate_rbf_without_periodicity_DP(ANALYSIS_SEGMENT,avrTask.digraph_rbf);
			//timer.end();
			//cout << timer.getTime() << endl;
		}
	}

	if (mc == LUB || mc == SEG_LUB_NECESSARY_ONLY1) { // prepare the linear upper bounds
		//avrTask.buildDigraph();
		avrTask.prepareLinearUppperBound();
		/*
		Digraph* digraph = avrTask.digraph;
		digraph->linear_factor = avrTask._maxU;
		digraph->calculate_linear_upper_bounds();
		*/
	}

	/*
	// Prepare the seg-ilp-con
	if (mc == SEG_ILPCON) {
		timer.start();
		int maxPeriod = 0;
		for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);

		int nDeadline = floor(1.0*maxPeriod/ANALYSIS_SEGMENT);

		set<int> sDeadlines; // segmental deadlines
		for (int j=1; j<=nDeadline; j++)
			sDeadlines.insert(j*ANALYSIS_SEGMENT);
		sDeadlines.insert(maxPeriod);

		vector<double> dominants = avrTask.getAllDominants();

		for (auto sD : sDeadlines) {
			if (sD == maxPeriod) continue;
			for (auto omega : dominants)
				avrTask.calILPCON(omega,sD);
		}
		timer.end();
		cout << timer.getTime() << endl;
	}
	*/

	//timer.start();
	bool finalSchedulable = checkSchedulabilityAudsleyOPA(mc,tasks,avrTask);
	//timer.end();
	//cout << "tDoSched = " << timer.getTime() << endl;

	// delete digraph
	if (mc == DIGRAPH_RBF || mc == DIGRAPH_RBF_ENVELOPE || mc == SEG_ILPCON_DIGRAPH_RBF /*|| mc == SEG_LUB_NECESSARY_ONLY1*/) {
		delete avrTask.digraph;
		avrTask.digraph = NULL;
		avrTask.digraph_rbf.clear();
		avrTask.digraph_rbf_envelope.clear();
	}

	return finalSchedulable;

#endif

// #ifdef __USING_PERIODIC_TASKS_DMPO__
#if 0
	for (unsigned int i = 0; i<tasks.size()+1; i++) {
		bool schedulable;
		switch (mc)
		{
		case EXACT:
			schedulable = exact_analysis(tasks,avrTask,i);
			break;
		case RBF:
			schedulable = rbf_analysis(tasks,avrTask,i);
			break;
		case RBF_ENVELOPE:
			schedulable = rbf_envelope_analysis(tasks,avrTask,i);
			break;
		case DIGRAPH_RBF:
			schedulable = digraph_rbf_analysis(tasks,avrTask,i);
			break;
		case DIGRAPH_RBF_ENVELOPE:
			schedulable = digraph_rbf_envelope_analysis(tasks,avrTask,i);
			break;
		case IBF:
			schedulable = ibf_analysis(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY1:
			schedulable = necessary_only_analysis(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY2:
			schedulable = necessary_only_analysis2(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY3:
			schedulable = necessary_only_analysis3(tasks,avrTask,i);
			break;
		case LUB:
			schedulable = lub_analysis(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY1A:
			schedulable = necessary_only_analysis1A(tasks,avrTask,i);
			break;
		case NECESSARY_ONLY2A:
			schedulable = necessary_only_analysis2A(tasks,avrTask,i);
			break;
		case ILPCON:
			#ifdef __USING_ILPCPLEX__
				schedulable = ilp_con_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		case ILP:
			#ifdef __USING_ILPCPLEX__
				schedulable = ilp_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		case SEG_ILPCON_EXACT:
			#ifdef __USING_ILPCPLEX__
				schedulable = segmental_ilp_con_exact_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		case SEG_ILPCON_DIGRAPH_RBF:
			#ifdef __USING_ILPCPLEX__
				schedulable = segmental_ilp_con_digraph_rbf_analysis(tasks,avrTask,i);
			#else
				schedulable = true;
			#endif
			break;
		default:
			cerr << "Error the method choice: " << mc << endl;
			exit(EXIT_FAILURE);
			break;
		}
		if (schedulable) return true;
	}
	return false;

#endif

//#ifdef __USING_ALL_TASKS_DMPO__
#if 1
	/// determine the index of the AVR task
	int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
	int AVRIndex = 0;
	for (int i=0; i<tasks.size(); i++) {
		if (tasks[i].deadline < AVRDeadline)
			AVRIndex = i+1;
	}
	//AVRIndex = 1;
#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif
	bool schedulable = asynchronous_exact_analysis(tasks,avrTask,AVRIndex);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif

#ifdef __USING_CONSTANT_PRIORITIES__
	int AVRIndex = 1;
#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif
	bool schedulable = asynchronous_exact_analysis(tasks,avrTask,AVRIndex);

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif
}

bool SchedAnalysis::checkSchedulabilityAllPriorities(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask mixedAvrTask, AVRTask prevAvrTask, AVRTask avrTask, int switchTime)
{
	#ifdef __USING_ILPCPLEX__
	ILPs.clear();
	ILPCONs.clear();
	#endif

#ifdef __USING_AUDSLEY_OPA__
	return true;
#endif 

#ifdef __USING_ALL_TASKS_DMPO__
	/// determine the index of the AVR task
	int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
	int AVRIndex = 0;
	for (int i=0; i<tasks.size(); i++) {
		if (tasks[i].deadline < AVRDeadline)
			AVRIndex = i+1;
	}
	//AVRIndex = 1;
	
#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif

	vector<double> speedPartition = getSpeedPartition(prevAvrTask,avrTask,switchTime);
	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime,speedPartition);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;
	
	bool schedulable = asynchronous_sched_analysis(tasks,digraph,mixedAvrTask, AVRIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif

#ifdef __USING_CONSTANT_PRIORITIES__
	int AVRIndex = 1;

#ifdef __DEBUG_VERIFICATION_TIMES__
	Timer timer;
	timer.start();
#endif

	Digraph* digraph = generateOffsetBasedDRT(prevAvrTask,avrTask,switchTime);
	//digraph->write_graphviz_ODRT(cout);
	//cout << digraph->minVertexC << " " << digraph->minVertexT << endl;

	bool schedulable = asynchronous_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = asynchronous_sufficient_sched_analysis(tasks,digraph,AVRIndex,switchTime);
	//bool schedulable = necessary_only_analysis(tasks,avrTask,AVRIndex,nextAvrTask);
	delete digraph;

#ifdef __DEBUG_VERIFICATION_TIMES__
	timer.end();
	cout << "Time = " << timer.getTime() << endl;
#endif

	return schedulable;
#endif
}

bool SchedAnalysis::checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask prevAvrTask, AVRTask avrTask, int switchTime)
{
	vector<Task*> vecTask;

	//////////////////////////////////////////////////////////////////////////
	/////////// Collect the tasks in order of the periods        /////////////
	//////////////////////////////////////////////////////////////////////////

	double avrTask_Deadline = avrTask.period/RPM_to_RPmuSEC(avrTask.engine.RPM_MAX);

	bool hasContained = false;

	for (int i=0; i<tasks.size(); i++) {
		int deadline = tasks[i].deadline;

		if (!hasContained && avrTask_Deadline <= deadline) {
			vecTask.push_back(&avrTask);
			hasContained = true;
		}
		vecTask.push_back(&tasks[i]);
	}

	if (!hasContained)
		vecTask.push_back(&avrTask);

	//////////////////////////////////////////////////////////////////////////
	///////////               Audsley's OPA Test                 /////////////
	//////////////////////////////////////////////////////////////////////////

	while (!vecTask.empty()) {
		bool schedulable = false;

		for (int i = vecTask.size()-1; i >= 0; i--) {
			Task* pTask = vecTask[i];

			vector<Task*> hpTasks;
			for (int j=0; j<vecTask.size(); j++) {
				if (j==i) continue;
				hpTasks.push_back(vecTask[j]);
			}

			schedulable = checkSchedulabilityAudsleyOPA(mc,hpTasks,pTask,prevAvrTask,avrTask,switchTime);

			if (schedulable) {
				vecTask.erase(vecTask.begin()+i);
				break;
			}
		}

		if (!schedulable) return false;
	}

	return true;
}

bool SchedAnalysis::checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<Task*> tasks, Task* task, bool synchronous, AVRTaskPointer nextAvrTask) {
	AVRTask* pTask = NULL;
	if (synchronous) { // synchronous periodic task
		vector<PeriodicTask> periodicTasks;

		bool hasContainedAVR = false;
		for (auto e : tasks) {
			if (dynamic_cast<AVRTask*> (e) != NULL) {
				hasContainedAVR = true;
				pTask = dynamic_cast<AVRTask*> (e);
			}
			else {
				PeriodicTask periodicTask = *dynamic_cast<PeriodicTask*> (e);
				periodicTasks.push_back(periodicTask);
			}
		}

		bool schedulable = false;

		if (!hasContainedAVR) {
			// if the task with lowest priority is periodic ...
			if (dynamic_cast<AVRTask*> (task) != NULL) {
				AVRTask avrTask = *dynamic_cast<AVRTask*> (task);
				schedulable = exact_analysis_index_for_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
			}
			else {
				PeriodicTask periodicTask = *dynamic_cast<PeriodicTask*> (task);
				periodicTasks.push_back(periodicTask);
				schedulable = exact_analysis_index(periodicTasks,periodicTasks.size()-1);
			}
		} 
		else {
			AVRTask avrTask = *pTask;
			//avrTask.prepareLinearUppperBound();

			PeriodicTask periodicTask = *dynamic_cast<PeriodicTask*> (task);
			periodicTasks.push_back(periodicTask);

			switch (mc)
			{
			case EXACT:
				schedulable = exact_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case RBF:
				schedulable = rbf_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
				break;
			case RBF_ENVELOPE:
				schedulable = rbf_envelope_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
				break;
			case DIGRAPH_RBF:
				schedulable = digraph_rbf_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
				break;
			case DIGRAPH_RBF_ENVELOPE:
				schedulable = digraph_rbf_envelope_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
				break;
			case IBF:
				schedulable = ibf_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
				break;
			case NECESSARY_ONLY1:
				schedulable = necessary_only_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case SEG_NECESSARY_ONLY1:
				schedulable = segmental_necessary_only_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case SEG_LUB_NECESSARY_ONLY1:
				schedulable = segmental_lub_necessary_only_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case NECESSARY_ONLY2:
				schedulable = necessary_only_analysis_index_with_avrtask2(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case NECESSARY_ONLY3:
				schedulable = necessary_only_analysis_index_with_avrtask3(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case NECESSARY_ONLY1A:
				schedulable = necessary_only_analysis_index_with_avrtask1A(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case SEG_NECESSARY_ONLY1A:
				schedulable = segmental_necessary_only_analysis_index_with_avrtask1A(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case NECESSARY_ONLY2A:
				schedulable = necessary_only_analysis_index_with_avrtask2A(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case LUB:
				schedulable = lub_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask,nextAvrTask);
				break;
			case ILPCON:
#ifdef __USING_ILPCPLEX__
				schedulable = ilp_con_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
#else
				schedulable = true;
#endif
				break;
			case ILP:
#ifdef __USING_ILPCPLEX__
				schedulable = ilp_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
#else
				schedulable = true;
#endif
				break;
			case SEG_ILPCON_EXACT:
#ifdef __USING_ILPCPLEX__
				schedulable = segmental_ilp_con_exact_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
#else
				schedulable = true;
#endif
				break;
			case SEG_ILPCON_DIGRAPH_RBF:
#ifdef __USING_ILPCPLEX__
				schedulable = segmental_ilp_con_digraph_rbf_analysis_index_with_avrtask(periodicTasks,periodicTasks.size()-1,avrTask);
#else
				schedulable = true;
#endif
				break;
			default:
				cerr << "Error the method choice: " << mc << endl;
				exit(EXIT_FAILURE);
				break;
			}
		}
		return schedulable;
	}
	else { // asynchronous periodic task
		vector<AsynchronousPeriodicTask> asynchronousPeriodicTasks;

		bool hasContainedAVR = false;
		for (auto e : tasks) {
			if (dynamic_cast<AVRTask*> (e) != NULL) {
				hasContainedAVR = true;
				pTask = dynamic_cast<AVRTask*> (e);
			}
			else {
				AsynchronousPeriodicTask asynchronousPeriodicTask = *dynamic_cast<AsynchronousPeriodicTask*> (e);
				asynchronousPeriodicTasks.push_back(asynchronousPeriodicTask);
			}
		}

		bool schedulable = false;

		if (!hasContainedAVR) {
			// if the task with lowest priority is periodic ...
			if (dynamic_cast<AVRTask*> (task) != NULL) {
				AVRTask avrTask = *dynamic_cast<AVRTask*> (task);
				schedulable = asynchronous_exact_analysis_index_for_avrtask(asynchronousPeriodicTasks,asynchronousPeriodicTasks.size()-1,avrTask);
			}
			else {
				AsynchronousPeriodicTask asynchronousPeriodicTask = *dynamic_cast<AsynchronousPeriodicTask*> (task);
				asynchronousPeriodicTasks.push_back(asynchronousPeriodicTask);
				schedulable = asynchronous_exact_analysis_index(asynchronousPeriodicTasks,asynchronousPeriodicTasks.size()-1);
			}
		} 
		else {
			AVRTask avrTask = *pTask;
			//avrTask.prepareLinearUppperBound();

			AsynchronousPeriodicTask asynchronousPeriodicTask = *dynamic_cast<AsynchronousPeriodicTask*> (task);
			asynchronousPeriodicTasks.push_back(asynchronousPeriodicTask);

			switch (mc)
			{
			case EXACT:
				schedulable = asynchronous_exact_analysis_index_with_avrtask(asynchronousPeriodicTasks,asynchronousPeriodicTasks.size()-1,avrTask);
				break;
			default:
				cerr << "Error the method choice: " << mc << endl;
				exit(EXIT_FAILURE);
				break;
			}
		}

		return schedulable;
	}
}

bool SchedAnalysis::checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<Task*> tasks, Task* task, AVRTask prevAvrTask, AVRTask avrTask, int switchTime) {
	AVRTask* pTask = NULL;
	vector<AsynchronousPeriodicTask> asynchronousPeriodicTasks;

	bool hasContainedAVR = false;
	for (auto e : tasks) {
		if (dynamic_cast<AVRTask*> (e) != NULL) {
			hasContainedAVR = true;
			pTask = dynamic_cast<AVRTask*> (e);
		}
		else {
			AsynchronousPeriodicTask asynchronousPeriodicTask = *dynamic_cast<AsynchronousPeriodicTask*> (e);
			asynchronousPeriodicTasks.push_back(asynchronousPeriodicTask);
		}
	}

	bool schedulable = false;

	if (!hasContainedAVR) {
		// if the task with lowest priority is periodic ...
		if (dynamic_cast<AVRTask*> (task) != NULL) {
			AVRTask avrTask = *dynamic_cast<AVRTask*> (task);
			schedulable = asynchronous_exact_analysis_index_for_avrtask(asynchronousPeriodicTasks,asynchronousPeriodicTasks.size()-1,avrTask);
		}
		else {
			AsynchronousPeriodicTask asynchronousPeriodicTask = *dynamic_cast<AsynchronousPeriodicTask*> (task);
			asynchronousPeriodicTasks.push_back(asynchronousPeriodicTask);
			schedulable = asynchronous_exact_analysis_index(asynchronousPeriodicTasks,asynchronousPeriodicTasks.size()-1);
		}
	} 
	else {
		AVRTask avrTask = *pTask;
		//avrTask.prepareLinearUppperBound();

		AsynchronousPeriodicTask asynchronousPeriodicTask = *dynamic_cast<AsynchronousPeriodicTask*> (task);
		asynchronousPeriodicTasks.push_back(asynchronousPeriodicTask);

		switch (mc)
		{
		case EXACT:
			schedulable = asynchronous_exact_analysis_index_with_avrtask(asynchronousPeriodicTasks,asynchronousPeriodicTasks.size()-1,prevAvrTask,avrTask,switchTime);
			break;
		default:
			cerr << "Error the method choice: " << mc << endl;
			exit(EXIT_FAILURE);
			break;
		}
	}

	return schedulable;
}

bool SchedAnalysis::checkSchedulabilityAudsleyOPA(vector<PeriodicTask> tasks) {
	vector<PeriodicTask> vecTask = tasks;

	//////////////////////////////////////////////////////////////////////////
	///////////               Audsley's OPA Test                 /////////////
	//////////////////////////////////////////////////////////////////////////

	while (!vecTask.empty()) {
		bool schedulable = false;

		for (int i = vecTask.size()-1; i >= 0; i--) {
			PeriodicTask pTask = vecTask[i];

			vector<PeriodicTask> hpTasks;
			for (int j=0; j<vecTask.size(); j++) {
				if (j==i) continue;
				hpTasks.push_back(vecTask[j]);
			}
			hpTasks.push_back(pTask);

			schedulable =  exact_analysis_index(hpTasks,hpTasks.size()-1);

			if (schedulable) {
				vecTask.erase(vecTask.begin()+i);
				break;
			}
		}

		if (!schedulable) return false;
	}

	return true;
}

bool SchedAnalysis::checkSchedulabilityAudsleyOPA(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, AVRTaskPointer nextAvrTask)
{
	vector<Task*> vecTask;

	//////////////////////////////////////////////////////////////////////////
	/////////// Collect the tasks in order of the periods        /////////////
	//////////////////////////////////////////////////////////////////////////

	double avrTask_Deadline = avrTask.period/RPM_to_RPmuSEC(avrTask.engine.RPM_MAX);

	bool hasContained = false;

	for (int i=0; i<tasks.size(); i++) {
		int deadline = tasks[i].deadline;

		if (!hasContained && avrTask_Deadline <= deadline) {
			vecTask.push_back(&avrTask);
			hasContained = true;
		}
		vecTask.push_back(&tasks[i]);
	}

	if (!hasContained)
		vecTask.push_back(&avrTask);

	//////////////////////////////////////////////////////////////////////////
	///////////               Audsley's OPA Test                 /////////////
	//////////////////////////////////////////////////////////////////////////

	while (!vecTask.empty()) {
		bool schedulable = false;

		for (int i = vecTask.size()-1; i >= 0; i--) {
			Task* pTask = vecTask[i];

			vector<Task*> hpTasks;
			for (int j=0; j<vecTask.size(); j++) {
				if (j==i) continue;
				hpTasks.push_back(vecTask[j]);
			}

			schedulable = checkSchedulabilityAudsleyOPA(mc,hpTasks,pTask,false);

			if (schedulable) {
				vecTask.erase(vecTask.begin()+i);
				break;
			}
		}

		if (!schedulable) return false;
	}

	return true;
}

bool SchedAnalysis::exact_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer nextAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = exact_analysis_index_with_avrtask(tasks,i-1,avrTask,nextAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::exact_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, int& uIndex, AVRTaskPointer nextAvrTask) {
	for (int i=uIndex; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = exact_analysis_index_with_avrtask(tasks,i-1,avrTask,nextAvrTask);

		if (!schedulable) {
			uIndex = i;
			return false;
		}
	}
	return true;
}

bool SchedAnalysis::exact_analysis_index(vector<PeriodicTask> tasks, int i) {
	PeriodicTask task = tasks[i];
	if (i==0) {
		if (task.wcet > task.deadline) return false;
		else return true;
	}

	return exact_analysis_index(tasks,i-1,task.wcet,task.deadline);	
}

bool SchedAnalysis::exact_analysis_index(vector<PeriodicTask> tasks, int i, int wcet, int deadline) {
	int rt = calculate_response_time(tasks,i,wcet);
	if (rt <= deadline) return true;
	else return false;
}

bool SchedAnalysis::exact_analysis_index_for_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	if (i==-1) {
		for (int j=0; j<avrTask.numMode; j++) {
			int wcet = avrTask.wcets[j];
			int deadline = Utility::round(mSEC_to_muSEC(avrTask.calDeadline(avrTask.omegas[j])));
			if (wcet > deadline) return false;
		}
		return true;
	}

	for (int j=0; j<avrTask.numMode; j++) {
		int wcet = avrTask.wcets[j];
		int deadline = Utility::round(mSEC_to_muSEC(avrTask.calDeadline(avrTask.omegas[j])));
		int rt = calculate_response_time(tasks,i,wcet);
		if (rt > deadline) return false;
		
	}
	return true;
}

bool SchedAnalysis::exact_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer nextAvrTask) {
#if 0 // correct
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

	#ifdef __DEBUG_SCHED_ANALYSIS__
	cout << "Dominant Speeds = ";
	for(auto w0 : dominants) {
		cout << RPmSEC_to_RPM(w0) << " ";
	}
	cout << endl;
	#endif

	/// TSC has not been changed
	vector<double> toBeExcluded;
	int lastC = -1;
#if 1
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		vector<int> rts;
		int currWCET = avrTask.getWCET(*iter);

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();
		vector<double> tmpExclude;
		exact_response_time_analysis(tasks,i-1,avrTask,rts,*iter,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude,nextAvrTask);
		//exact_response_time_analysis(tasks,i-1,avrTask,rts,*iter,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
		if (*rt_iter > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#endif

	/// TSC has been changed
	if (nextAvrTask != NULL) {
		vector<double> prevTranSpeeds;
		for(auto e : nextAvrTask->speeds)
			prevTranSpeeds.push_back(e.second);

		vector<double> currTranSpeeds;
		for (auto e : avrTask.speeds)
			currTranSpeeds.push_back(e.second);

		vector<double> mixedTranSpeeds = Utility::merge(prevTranSpeeds,currTranSpeeds);
		AVRTask mixedAvrTask(engine,avrTask.period,mixedTranSpeeds);
		dominants = mixedAvrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

		toBeExcluded.clear();
		lastC = -1;

		for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
			vector<int> rts;
			//int currWCET = max(avrTask.getWCET(*iter), nextAvrTask->getWCET(*iter));
			int currWCET = nextAvrTask->getWCET(*iter);

			// If the mode is changed we clean the toBeExcluded list
			if (currWCET != lastC) toBeExcluded.clear();
			vector<double> tmpExclude;
			exact_response_time_analysis(tasks,i-1,*nextAvrTask,rts,*iter,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

			vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
			if (*rt_iter > deadline) {
				return false;
			}

			lastC = currWCET;
			toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
		}
	}
	
	return true;
#else // only considering the first instance
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);
	if (nextAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevDominants;
		for(auto e : nextAvrTask->omegas)
			prevDominants.push_back(e.second);
		dominants = Utility::merge(dominants,prevDominants);
	}

#ifdef __DEBUG_SCHED_ANALYSIS__
	cout << "Dominant Speeds = ";
	for(auto w0 : dominants) {
		cout << RPmSEC_to_RPM(w0) << " ";
	}
	cout << endl;
#endif

	vector<double> toBeExcluded;
	int lastC = -1;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		int maxRT = INT_MIN;

		int currWCET = avrTask.getWCET(*iter);
		if (nextAvrTask != NULL)
			currWCET = max(currWCET,nextAvrTask->getWCET(*iter));

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<double> tmpExclude;
		exact_response_time_analysis(tasks,i-1,avrTask,maxRT,*iter,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
	return true;
#endif
}

void SchedAnalysis::exact_response_time_analysis(vector<PeriodicTask> tasks,int i, AVRTask avrTask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludeList, vector<double>& returnedExcludeList, AVRTaskPointer nextAvrTask) {	
	int rt = calculate_response_time(tasks,i,sum_wcet);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask.engine;
	double period = avrTask.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	/// TSC has not been changed
	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime-sum_period);
	
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		// If the mode is changed we clean the toBeExcluded list
		if (lastC != avrTask.getWCET(*iter)) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;

		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext >= rt) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		exact_response_time_analysis(tasks,i,avrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude,nextAvrTask);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(*iter);
	}

	/// TSC has been changed
	if (nextAvrTask != NULL) {
		vector<double> nextTranSpeeds;
		for(auto e : nextAvrTask->speeds)
			nextTranSpeeds.push_back(e.second);

		vector<double> currTranSpeeds;
		for (auto e : avrTask.speeds)
			currTranSpeeds.push_back(e.second);

		vector<double> mixedTranSpeeds = Utility::merge(nextTranSpeeds,currTranSpeeds);
		AVRTask mixedAvrTask(engine,avrTask.period,mixedTranSpeeds);
		dominants = mixedAvrTask.getDominants(omega_b,omega_a,maxTime-sum_period);

		toBeExcluded.clear();
		lastC = -1;

		for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
			// If the mode is changed we clean the toBeExcluded list
			if (lastC != nextAvrTask->getWCET(*iter)) toBeExcluded.clear();

			// Additional pruning: if the dominant is in exclude list then SKIP IT!
			// I'm thinking why Biondi can do this
			if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;

			double omega_next = *iter;
	#ifdef __USING_CONSTANT_ACCELERATION__
			double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
	#else
			double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
	#endif
			int tNext = Utility::round(mSEC_to_muSEC(time));

			if (sum_period+tNext >= rt) continue;

			vector<double> tmpExclude;
			int sum_period2 = sum_period+ tNext;
			int sum_wcet2 = sum_wcet + nextAvrTask->getWCET(omega_next);
			exact_response_time_analysis(tasks,i,*nextAvrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

			if (!tmpExclude.empty())
				toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
			lastC = nextAvrTask->getWCET(*iter);
		}
	}

	returnedExcludeList = dominants;
}

bool SchedAnalysis::exact_analysis_avrtask_to_digraph(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, int granularity /*= -1*/)
{
	Digraph* digraph = NULL;

	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else {
			// Transform avrtask to digraph
			if (digraph == NULL) {
#ifdef __PARTITIONING_WITH_CONSTANT_SCALE__
				vector<double> speedPartition = getSpeedPartition(avrTask,granularity);
#endif

#ifdef __PARTITIONING_WITH_CONSTANT_LENGTH__
				vector<double> speedPartition = getSpeedPartitionWithConstLength(avrTask,granularity);
#endif
				digraph = generateOffsetBasedDRT(avrTask,speedPartition);
				//digraph->write_graphviz(cout);
				//exit(EXIT_FAILURE);
			}
			schedulable = exact_analysis_index_with_digraph(tasks,i-1,digraph);
		}

		if (!schedulable) {
			if (digraph!=NULL) delete digraph;
			return false;
		}
	}
	if (digraph!=NULL) delete digraph;
	return true;
}

bool SchedAnalysis::exact_analysis_index_with_digraph(vector<PeriodicTask> tasks, int i, Digraph* digraph)
{
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	vector<Node*> toBeExcluded;
	int lastC = -1;

	for (vector<Node*>::iterator iter = digraph->node_vec.begin(); iter != digraph->node_vec.end(); iter++) {
		int maxRT = INT_MIN;

		int currWCET = (*iter)->wcet;

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<Node*> tmpExclude;
		exact_response_time_analysis_digraph(tasks,i-1,digraph,maxRT,*iter,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}

	return true;
}

void SchedAnalysis::exact_response_time_analysis_digraph(vector<PeriodicTask> tasks,int i, Digraph* digraph, int &maxRT, Node* node, int sum_wcet, int sum_period, int maxTime, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {	
	int rt = calculate_response_time(tasks,i,sum_wcet);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<Node*> toBeExcluded;
	int lastC = -1;

	// Require: list node->out->snk_node sorted in the descending order of engine speed
	for (auto edge : node->out) {
		Node* nextNode = edge->snk_node;
		// If the mode is changed we clean the toBeExcluded list
		int nodeC = nextNode->wcet;
		if (lastC != nodeC) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		int tNext = edge->separationTime;

		if (sum_period+tNext >= rt) continue;

		vector<Node*> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + nodeC;
		exact_response_time_analysis_digraph(tasks,i,digraph,maxRT,nextNode,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = nodeC;
	}

	returnedExcludeList.clear();
	for (auto edge : node->out) {
		returnedExcludeList.push_back(edge->snk_node);
	}
}

bool SchedAnalysis::necessary_only_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	allTestNumber = 0;
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = necessary_only_analysis_index_with_avrtask(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) {
			//cout << i << " && " << avrTaskIndex << endl;
			return false;
		}
	}
	return true;
}

bool SchedAnalysis::necessary_only_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	//Engine engine = avrTask.engine;
	//vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

	vector<double> toBeExcluded;
	int lastC = -1;

#if 1
	vector<double> omegas;
	//vector<int> wcets;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);
	//for (auto e: avrTask.wcets) wcets.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}
	/*
	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int j = 0; j < wcets.size(); j++)
	{
		pair<int,double> newPair(j,utils[j]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);
	
	for (int index=0; index<pair_utils.size(); index++) {
		vector<int> rts;
		int mode = pair_utils[index].first;
		double omega = avrTask.omegas[mode];
		*/
	for (auto omega : omegas) {
		int maxRT = INT_MIN;

		int currWCET = avrTask.getWCET(omega);
		if (prevAvrTask != NULL)
			currWCET = max(currWCET,prevAvrTask->getWCET(omega));

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<double> tmpExclude;
		allTestNumber = 0;
		necessary_only_response_time_analysis(true,tasks,i-1,avrTask,maxRT,omega,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#else
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		vector<int> rts;
		double omega = *iter;
		if (!avrTask.checkMode(omega)) continue;

		// If the mode is changed we clean the toBeExcluded list
		if (avrTask.getWCET(omega) != lastC) toBeExcluded.clear();
		vector<double> tmpExclude;
		necessary_only_response_time_analysis(true,tasks,i-1,avrTask,rts,omega,wcet+avrTask.getWCET(omega),0,deadline,toBeExcluded,tmpExclude);

		vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
		if (*rt_iter > deadline) {
			return false;
		}

		lastC = avrTask.getWCET(omega);
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#endif
	return true;
}

void SchedAnalysis::necessary_only_response_time_analysis(bool updown, vector<PeriodicTask> tasks,int i, AVRTask avrTask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludeList, vector<double>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask.engine;
	double period = avrTask.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime-sum_period);
	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	/*
	/*
	if (!avrTask.checkMode(omega)) {
		if (lastC != avrTask.getWCET(omega)) toBeExcluded.clear();
		
		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		// I'm thinking why Biondi can do this
		if(find(excludeList.begin(),excludeList.end(),omega) != excludeList.end()) return;


		double omega_next;
		if (updown) { // up direction
			omega_next = dominants.front();
		} else { // down direction
			omega_next = dominants.back();
		}
		double time = engine.getInterArrivalTime(omega,omega_next,period);
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext >= rt) return;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		necessary_only_response_time_analysis(updown,tasks,i,avrTask,rts,omega_next,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(omega);
	} else {

	}
	*/
	vector<double> reduced_dominants;
#if 1
	reduced_dominants = dominants;
#elif 0
	int size = dominants.size();
	if (size < 2) reduced_dominants = dominants;
	else {
		set<int> record;
		while (record.size() < size/2)
			record.insert(rand()%size);
		
		for (auto e: record)
			reduced_dominants.push_back(dominants[e]);
		
	}
#else
	reduced_dominants.push_back(dominants.front());
	reduced_dominants.push_back(dominants.back());
#endif
	
	//for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
	for (vector<double>::iterator iter = reduced_dominants.begin(); iter != reduced_dominants.end(); iter++) {
		// If the mode is changed we clean the toBeExcluded list
		if (lastC != avrTask.getWCET(*iter)) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		// I'm thinking why Biondi can do this
		if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;
		
		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext >= rt) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		necessary_only_response_time_analysis(true,tasks,i,avrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(*iter);
	}

	returnedExcludeList = dominants;
}

bool SchedAnalysis::segmental_necessary_only_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = segmental_necessary_only_analysis_index_with_avrtask(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::segmental_necessary_only_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;

	// Pre-prepare
	int rt = calculate_response_time(tasks,i-1,wcet);

	int nDeadline = floor(1.0*deadline/ANALYSIS_SEGMENT);

	set<int> sDeadlines; // segmental deadlines
	for (int j=1; j<=nDeadline; j++) {
		if (rt >= j*ANALYSIS_SEGMENT) continue;
		sDeadlines.insert(j*ANALYSIS_SEGMENT);
	}
	sDeadlines.insert(deadline);

	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int k = 0; k < wcets.size(); k++)
	{
		pair<int,double> newPair(k,utils[k]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	int maxFirst = pair_utils[0].first;
	double wMax = avrTask.omegas[maxFirst];
	int wcetMax = avrTask.wcets[maxFirst];
	double tMax = mSEC_to_muSEC(avrTask.period/wMax);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	int maxRT = 0;

	for (auto sD : sDeadlines) {
		if (sD < maxRT) continue;
		if (maxRT > deadline) return false;

		bool schedulable = true;

		if (sD-ANALYSIS_SEGMENT <= 0) {
			vector<double> toBeExcluded;
			int lastC = -1;
			/*
			for (int index=0; index<pair_utils.size(); index++) {
				vector<int> rts;
				int mode = pair_utils[index].first;
				double omega = avrTask.omegas[mode];
				*/
			for (auto omega : omegas) {
				int localMaxRT = INT_MIN;

				int currWCET = avrTask.getWCET(omega);
				if (prevAvrTask != NULL)
					currWCET = max(currWCET,prevAvrTask->getWCET(omega));

				// If the mode is changed we clean the toBeExcluded list
				if (currWCET != lastC) toBeExcluded.clear();

				vector<double> tmpExclude;
				necessary_only_response_time_analysis(true,tasks,i-1,avrTask,localMaxRT,omega,wcet+currWCET,0,sD,toBeExcluded,tmpExclude);

				if (localMaxRT > sD) {
					schedulable = false;
					maxRT = max(maxRT,localMaxRT);
					break;
				}

				lastC = currWCET;
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}
		else {
			int start = 0;
			if (sD < deadline) start = sD - ANALYSIS_SEGMENT;
			else {
				if (deadline == nDeadline * ANALYSIS_SEGMENT)
					start = sD - ANALYSIS_SEGMENT;
				else
					start = nDeadline * ANALYSIS_SEGMENT;
			}

			vector<double> toBeExcluded;
			int lastC = -1;

			/*
			for (int index=0; index<pair_utils.size(); index++) {
				vector<int> rts;
				int mode = pair_utils[index].first;
				double omega = avrTask.omegas[mode];
				*/
			for (auto omega : omegas) {
				int localMaxRT = INT_MIN;

				int currWCET = avrTask.getWCET(omega);
				if (prevAvrTask != NULL)
					currWCET = max(currWCET,prevAvrTask->getWCET(omega));

				// If the mode is changed we clean the toBeExcluded list
				if (currWCET != lastC) toBeExcluded.clear();

				double tStart = 0;
				double sumWCET = currWCET;

				// Align the start time
				if (W_EQ_TOL(omega,wMax)) {
					int n = floor(1.0*start/tMax);
					tStart = n*tMax;
					sumWCET += n*wcetMax;
				}
				else {
					double currOmega = omega;
					bool isConstant = false;

					if (W_GEQ_TOL(omega,wMax)) { // deceleration
						while (tStart <= start) {
							if (isConstant) { // constant at the mode with the max utilization
								double nextStart = tStart + tMax;
								if (nextStart > start) break;
								tStart = nextStart;
								sumWCET += wcetMax;
							}
							else { // need to decelerate
								double nextOmega = engine.getLowerSpeed(currOmega,avrTask.period,1);
								if (W_LEQ_TOL(nextOmega,wMax)) { // arrive at the mode with the max utilization
									isConstant = true;
									#ifdef __USING_CONSTANT_ACCELERATION__
										double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,wMax,avrTask.period));
									#else
										double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,wMax,avrTask.period));
									#endif
									if (nextStart > start) break;
									tStart = nextStart;
									sumWCET += wcetMax;
								}
								else {
									#ifdef __USING_CONSTANT_ACCELERATION__
										double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,nextOmega,avrTask.period));
									#else
										double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,nextOmega,avrTask.period));
									#endif
									if (nextStart > start) break;
									tStart = nextStart;
									sumWCET += avrTask.getWCET(nextOmega);
									currOmega = nextOmega;
								}
							}
						}
					}
					else { // acceleration
						while (tStart <= start) {
							if (isConstant) { // constant at the mode with the max utilization
								double nextStart = tStart + tMax;
								if (nextStart > start) break;
								tStart = nextStart;
								sumWCET += wcetMax;
							}
							else { // need to accelerate
								double nextOmega = engine.getHigherSpeed(currOmega,avrTask.period,1);
								if (W_GEQ_TOL(nextOmega,wMax)) { // arrive at the mode with the max utilization
									isConstant = true;
									#ifdef __USING_CONSTANT_ACCELERATION__
										double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,wMax,avrTask.period));
									#else
										double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,wMax,avrTask.period));
									#endif
									if (nextStart > start) break;
									tStart = nextStart;
									sumWCET += wcetMax;
								}
								else {
									#ifdef __USING_CONSTANT_ACCELERATION__
										double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,nextOmega,avrTask.period));
									#else
										double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,nextOmega,avrTask.period));
									#endif
									if (nextStart > start) break;
									tStart = nextStart;
									sumWCET += avrTask.getWCET(nextOmega);
									currOmega = nextOmega;
								}
							}
						}
					}
				}

				vector<double> tmpExclude;
				necessary_only_response_time_analysis(true,tasks,i-1,avrTask,localMaxRT,omega,wcet+sumWCET,tStart,sD,toBeExcluded,tmpExclude);

				if (localMaxRT > sD) {
					schedulable = false;
					maxRT = max(maxRT,localMaxRT);
					break;
				}

				lastC = currWCET;
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}

		if (schedulable) return true;
	}

	return false;
}

bool SchedAnalysis::segmental_lub_necessary_only_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = segmental_lub_necessary_only_analysis_index_with_avrtask(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::segmental_lub_necessary_only_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;

	// Pre-prepare
	int rt = calculate_response_time(tasks,i-1,wcet);

	int nDeadline = floor(1.0*deadline/ANALYSIS_SEGMENT);

	set<int> sDeadlines; // segmental deadlines
	for (int j=1; j<=nDeadline; j++) {
		if (rt >= j*ANALYSIS_SEGMENT) continue;
		sDeadlines.insert(j*ANALYSIS_SEGMENT);
	}
	sDeadlines.insert(deadline);

	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int k = 0; k < wcets.size(); k++)
	{
		pair<int,double> newPair(k,utils[k]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	int maxFirst = pair_utils[0].first;
	double wMax = avrTask.omegas[maxFirst];
	int wcetMax = avrTask.wcets[maxFirst];
	double tMax = mSEC_to_muSEC(avrTask.period/wMax);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	int maxRT = 0;

	for (auto sD : sDeadlines) {
		if (sD < maxRT) continue;
		if (maxRT > deadline) return false;

		bool schedulable = true;

		if (sD-ANALYSIS_SEGMENT <= 0) {
			vector<double> toBeExcluded;
			int lastC = -1;
			/*
			for (int index=0; index<pair_utils.size(); index++) {
				vector<int> rts;
				int mode = pair_utils[index].first;
				double omega = avrTask.omegas[mode];
				*/
			for (auto omega : omegas) {
				int localMaxRT = INT_MIN;

				int currWCET = avrTask.getWCET(omega);
				if (prevAvrTask != NULL)
					currWCET = max(currWCET,prevAvrTask->getWCET(omega));

				// If the mode is changed we clean the toBeExcluded list
				if (currWCET != lastC) toBeExcluded.clear();

				vector<double> tmpExclude;
				necessary_only_response_time_analysis(true,tasks,i-1,avrTask,localMaxRT,omega,wcet+currWCET,0,sD,toBeExcluded,tmpExclude);

				if (localMaxRT > sD) {
					schedulable = false;
					maxRT = max(maxRT,localMaxRT);
					break;
				}

				lastC = currWCET;
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}
		else {
			int start = 0;
			if (sD < deadline) start = sD - ANALYSIS_SEGMENT;
			else {
				if (deadline == nDeadline * ANALYSIS_SEGMENT)
					start = sD - ANALYSIS_SEGMENT;
				else
					start = nDeadline * ANALYSIS_SEGMENT;
			}

			vector<double> toBeExcluded;
			int lastC = -1;
			//start -= 0.5 * ANALYSIS_SEGMENT;
			//double sumWCET = avrTask.digraph->linear_factor * start + avrTask.digraph->c_ibf;
			double sumWCET = avrTask._maxU * start + (1.0 - avrTask._maxU)  * avrTask._maxC;
			if (prevAvrTask != NULL)
				sumWCET += max(prevAvrTask->_maxC-avrTask._maxC,0.0);

#ifdef __USING_ILPCPLEX__
			if (avrTask.numMode == 2) {
				//Timer timer;
				//timer.start();
				//sumWCET = RunTest::testCplexILP(avrTask.wcets[0], avrTask.modePeriods[0], avrTask.wcets[1], avrTask.modePeriods[1],sumWCET,start);
				//timer.end();
				//cout << timer.getTime() << endl;
			}
#endif
			/*
			for (int index=0; index<pair_utils.size(); index++) {
				vector<int> rts;
				int mode = pair_utils[index].first;
				double omega = avrTask.omegas[mode];
				*/
			for (auto omega : omegas) {
				int localMaxRT = INT_MIN;

				int currWCET = avrTask.getWCET(omega);
				if (prevAvrTask != NULL)
					currWCET = max(currWCET,prevAvrTask->getWCET(omega));

				// If the mode is changed we clean the toBeExcluded list
				if (currWCET != lastC) toBeExcluded.clear();

				vector<double> tmpExclude;
				necessary_only_response_time_analysis(true,tasks,i-1,avrTask,localMaxRT,omega,wcet+sumWCET,start,sD,toBeExcluded,tmpExclude);

				if (localMaxRT > sD) {
					schedulable = false;
					maxRT = max(maxRT,localMaxRT);
					break;
				}

				lastC = currWCET;
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}

		if (schedulable) return true;
	}

	return false;
}

bool SchedAnalysis::necessary_only_analysis2(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = necessary_only_analysis_index_with_avrtask2(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::necessary_only_analysis_index_with_avrtask2(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

#ifdef __DEBUG_SCHED_ANALYSIS__
	cout << "Dominant Speeds = ";
	for(auto w0 : dominants) {
		cout << RPmSEC_to_RPM(w0) << " ";
	}
	cout << endl;
#endif

	vector<double> toBeExcluded;
	int lastC = -1;

#if 1
	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int j = 0; j < wcets.size(); j++)
	{
		pair<int,double> newPair(j,utils[j]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	/*
	for (int index=0; index<pair_utils.size(); index++) {
		vector<int> rts;
		int mode = pair_utils[index].first;
		double omega = avrTask.omegas[mode];

		// If the mode is changed we clean the toBeExcluded list
		if (avrTask.getWCET(omega) != lastC) toBeExcluded.clear();
		*/
	for (auto omega : omegas) {
		int maxRT = INT_MIN;

		int currWCET = avrTask.getWCET(omega);
		if (prevAvrTask != NULL)
			currWCET = max(currWCET,prevAvrTask->getWCET(omega));

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<double> tmpExclude;
		necessary_only_response_time_analysis2(true,tasks,i-1,avrTask,maxRT,omega,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#else
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		vector<int> rts;
		double omega = *iter;
		if (!avrTask.checkMode(omega)) continue;

		// If the mode is changed we clean the toBeExcluded list
		if (avrTask.getWCET(omega) != lastC) toBeExcluded.clear();
		vector<double> tmpExclude;
		necessary_only_response_time_analysis(true,tasks,i-1,avrTask,rts,omega,wcet+avrTask.getWCET(omega),0,deadline,toBeExcluded,tmpExclude);

		vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
		if (*rt_iter > deadline) {
			return false;
		}

		lastC = avrTask.getWCET(omega);
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#endif
	return true;
}

void SchedAnalysis::necessary_only_response_time_analysis2(bool updown, vector<PeriodicTask> tasks,int i, AVRTask avrTask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludeList, vector<double>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask.engine;
	double period = avrTask.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime-sum_period);
	
	vector<double> reduced_dominants;
#if 0
	reduced_dominants = dominants;
#elif 0
	int size = dominants.size();
	if (size < 2) reduced_dominants = dominants;
	else {
		set<int> record;
		while (record.size() < size/2)
			record.insert(rand()%size);
		
		for (auto e: record)
			reduced_dominants.push_back(dominants[e]);
		
	}
#else
	if (dominants.size() >= 2) {
		reduced_dominants.push_back(dominants.front());
		reduced_dominants.push_back(dominants.back());
	}
	else reduced_dominants = dominants;
	/*
	int r = rand()%dominants.size();
	reduced_dominants.push_back(dominants[r]);
	*/
#endif
	
	//for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
	for (vector<double>::iterator iter = reduced_dominants.begin(); iter != reduced_dominants.end(); iter++) {
		// If the mode is changed we clean the toBeExcluded list
		if (lastC != avrTask.getWCET(*iter)) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;

		
		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext >= rt) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		necessary_only_response_time_analysis2(true,tasks,i,avrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(*iter);
	}

	returnedExcludeList = dominants;
}

bool SchedAnalysis::necessary_only_analysis3(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = necessary_only_analysis_index_with_avrtask3(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::necessary_only_analysis_index_with_avrtask3(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

#ifdef __DEBUG_SCHED_ANALYSIS__
	cout << "Dominant Speeds = ";
	for(auto w0 : dominants) {
		cout << RPmSEC_to_RPM(w0) << " ";
	}
	cout << endl;
#endif

	vector<double> toBeExcluded;
	int lastC = -1;

#if 1
	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int j = 0; j < wcets.size(); j++)
	{
		pair<int,double> newPair(j,utils[j]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	/*
	for (int index=0; index<pair_utils.size(); index++) {
		vector<int> rts;
		int mode = pair_utils[index].first;
		double omega = avrTask.omegas[mode];

		// If the mode is changed we clean the toBeExcluded list
		if (avrTask.getWCET(omega) != lastC) toBeExcluded.clear();
		*/
	for (auto omega : omegas) {
		int maxRT = INT_MIN;

		int currWCET = avrTask.getWCET(omega);
		if (prevAvrTask != NULL)
			currWCET = max(currWCET,prevAvrTask->getWCET(omega));

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<double> tmpExclude;
		necessary_only_response_time_analysis3(true,tasks,i-1,avrTask,maxRT,omega,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#else
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		vector<int> rts;
		double omega = *iter;
		if (!avrTask.checkMode(omega)) continue;

		// If the mode is changed we clean the toBeExcluded list
		if (avrTask.getWCET(omega) != lastC) toBeExcluded.clear();
		vector<double> tmpExclude;
		necessary_only_response_time_analysis3(true,tasks,i-1,avrTask,rts,omega,wcet+avrTask.getWCET(omega),0,deadline,toBeExcluded,tmpExclude);

		vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
		if (*rt_iter > deadline) {
			return false;
		}

		lastC = avrTask.getWCET(omega);
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}
#endif
	return true;
}

void SchedAnalysis::necessary_only_response_time_analysis3(bool updown, vector<PeriodicTask> tasks,int i, AVRTask avrTask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, vector<double> excludeList, vector<double>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask.engine;
	double period = avrTask.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime-sum_period);
	
	vector<double> reduced_dominants;
#if 0
	reduced_dominants = dominants;
#elif 0
	int size = dominants.size();
	if (size < 2) reduced_dominants = dominants;
	else {
		set<int> record;
		while (record.size() < size/2)
			record.insert(rand()%size);
		
		for (auto e: record)
			reduced_dominants.push_back(dominants[e]);
		
	}
#else
	if (dominants.size() >= 2) {
		reduced_dominants.push_back(dominants.back());
	}
	else reduced_dominants = dominants;
	/*
	int r = rand()%dominants.size();
	reduced_dominants.push_back(dominants[r]);
	*/
#endif
	
	//for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
	for (vector<double>::iterator iter = reduced_dominants.begin(); iter != reduced_dominants.end(); iter++) {
		// If the mode is changed we clean the toBeExcluded list
		if (lastC != avrTask.getWCET(*iter)) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;

		
		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext >= rt) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		necessary_only_response_time_analysis2(true,tasks,i,avrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(*iter);
	}

	returnedExcludeList = dominants;
}

bool SchedAnalysis::necessary_only_analysis1A(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = necessary_only_analysis_index_with_avrtask1A(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::necessary_only_analysis_index_with_avrtask1A(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;

	vector<double> toBeExcluded;
	int lastC = -1;

	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int k = 0; k < wcets.size(); k++)
	{
		pair<int,double> newPair(k,utils[k]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	int rt = calculate_response_time(tasks,i-1,wcet);

	/*
	for (int index=0; index<pair_utils.size(); index++) {
		vector<int> rts;
		int mode = pair_utils[index].first;
		double omega = avrTask.omegas[mode];
		*/
	for (auto omega : omegas) {
		int maxRT = INT_MIN;

		double T = mSEC_to_muSEC(avrTask.period/omega);
#if 0
		PeriodicTask periodicTask(avrTask.getWCET(omega),T);

		vector<PeriodicTask> temp;
		for (int k=0; k<i; k++) temp.push_back(tasks[k]);
		temp.push_back(periodicTask);

		int rt = calculate_response_time(temp,temp.size(),wcet);
#endif
		
		int n = floor(rt/T);

		int currWCET = avrTask.getWCET(omega);
		if (prevAvrTask != NULL)
			currWCET = max(currWCET,prevAvrTask->getWCET(omega));

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<double> tmpExclude;
		necessary_only_response_time_analysis(true,tasks,i-1,avrTask,maxRT,omega,wcet+currWCET + avrTask.getWCET(omega)*(n),T*n,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}

	return true;
}

bool SchedAnalysis::segmental_necessary_only_analysis1A(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = segmental_necessary_only_analysis_index_with_avrtask1A(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::segmental_necessary_only_analysis_index_with_avrtask1A(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;

	// Pre-prepare
	int rt = calculate_response_time(tasks,i-1,wcet);

	int nDeadline = floor(1.0*deadline/ANALYSIS_SEGMENT);

	set<int> sDeadlines; // segmental deadlines
	for (int j=1; j<=nDeadline; j++) {
		if (rt >= j*ANALYSIS_SEGMENT) continue;
		sDeadlines.insert(j*ANALYSIS_SEGMENT);
	}
	sDeadlines.insert(deadline);

	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int k = 0; k < wcets.size(); k++)
	{
		pair<int,double> newPair(k,utils[k]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	int maxFirst = pair_utils[0].first;
	double wMax = avrTask.omegas[maxFirst];
	int wcetMax = avrTask.wcets[maxFirst];
	double tMax = mSEC_to_muSEC(avrTask.period/wMax);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	int maxRT = rt;

	for (auto sD : sDeadlines) {
		if (sD < maxRT) continue;
		if (maxRT > deadline) return false;

		bool schedulable = true;

		int start = 0;
		if (sD < deadline) start = sD - ANALYSIS_SEGMENT;
		else {
			if (deadline == nDeadline * ANALYSIS_SEGMENT)
				start = sD - ANALYSIS_SEGMENT;
			else
				start = nDeadline * ANALYSIS_SEGMENT;
		}
		start = max(start,maxRT);
		//start = (start+sD)/2;

		vector<double> toBeExcluded;
		int lastC = -1;

		/*
		for (int index=0; index<pair_utils.size(); index++) {
			vector<int> rts;
			int mode = pair_utils[index].first;
			double omega = avrTask.omegas[mode];

			// If the mode is changed we clean the toBeExcluded list
			if (avrTask.getWCET(omega) != lastC) toBeExcluded.clear();
			*/
		for (auto omega : omegas) {
			int localMaxRT = INT_MIN;

			int currWCET = avrTask.getWCET(omega);
			if (prevAvrTask != NULL)
				currWCET = max(currWCET,prevAvrTask->getWCET(omega));

			// If the mode is changed we clean the toBeExcluded list
			if (currWCET != lastC) toBeExcluded.clear();

			double tStart = 0;
			double sumWCET = currWCET;

			// Align the start time
			if (W_EQ_TOL(omega,wMax)) {
				int n = floor(1.0*start/tMax);
				tStart = n*tMax;
				sumWCET += n*wcetMax;
			}
			else {
				double currOmega = omega;
				bool isConstant = false;

				if (W_GEQ_TOL(omega,wMax)) { // deceleration
					while (tStart <= start) {
						if (isConstant) { // constant at the mode with the max utilization
							double nextStart = tStart + tMax;
							if (nextStart > start) break;
							tStart = nextStart;
							sumWCET += wcetMax;
						}
						else { // need to decelerate
							double nextOmega = engine.getLowerSpeed(currOmega,avrTask.period,1);
							if (W_LEQ_TOL(nextOmega,wMax)) { // arrive at the mode with the max utilization
								isConstant = true;
								#ifdef __USING_CONSTANT_ACCELERATION__
									double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,wMax,avrTask.period));
								#else
									double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,wMax,avrTask.period));
								#endif
								if (nextStart > start) break;
								tStart = nextStart;
								sumWCET += wcetMax;
							}
							else {
								#ifdef __USING_CONSTANT_ACCELERATION__
								double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,nextOmega,avrTask.period));
								#else
								double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,nextOmega,avrTask.period));
								#endif
								if (nextStart > start) break;
								tStart = nextStart;
								sumWCET += avrTask.getWCET(nextOmega);
								currOmega = nextOmega;
							}
						}
					}
				}
				else { // acceleration
					while (tStart <= start) {
						if (isConstant) { // constant at the mode with the max utilization
							double nextStart = tStart + tMax;
							if (nextStart > start) break;
							tStart = nextStart;
							sumWCET += wcetMax;
						}
						else { // need to accelerate
							double nextOmega = engine.getHigherSpeed(currOmega,avrTask.period,1);
							if (W_GEQ_TOL(nextOmega,wMax)) { // arrive at the mode with the max utilization
								isConstant = true;
								#ifdef __USING_CONSTANT_ACCELERATION__
								double nextStart = tStart +  mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,wMax,avrTask.period));
								#else
								double nextStart = tStart +  mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,wMax,avrTask.period));
								#endif
								if (nextStart > start) break;
								tStart = nextStart;
								sumWCET += wcetMax;
							}
							else {
								#ifdef __USING_CONSTANT_ACCELERATION__
								double nextStart = tStart + mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(currOmega,nextOmega,avrTask.period));
								#else
								double nextStart = tStart + mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(currOmega,nextOmega,avrTask.period));
								#endif
								if (nextStart > start) break;
								tStart = nextStart;
								sumWCET += avrTask.getWCET(nextOmega);
								currOmega = nextOmega;
							}
						}
					}
				}
			}

			vector<double> tmpExclude;
			necessary_only_response_time_analysis(true,tasks,i-1,avrTask,localMaxRT,omega,wcet+sumWCET,tStart,sD,toBeExcluded,tmpExclude);

			if (localMaxRT > sD) {
				schedulable = false;
				maxRT = max(maxRT,localMaxRT);
				break;
			}

			lastC = currWCET;
			toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
		}

		if (schedulable) return true;
	}

	return false;
}

bool SchedAnalysis::necessary_only_analysis2A(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = necessary_only_analysis_index_with_avrtask2A(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::necessary_only_analysis_index_with_avrtask2A(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;

	int rt = calculate_response_time(tasks,i-1,wcet);

	vector<double> toBeExcluded;
	int lastC = -1;

	vector<double> speeds;
	vector<int> wcets;
	for (auto e: avrTask.speeds) speeds.push_back(e.second);
	for (auto e: avrTask.wcets) wcets.push_back(e.second);

	vector<double> utils = OptimizationAlgorithm::getUtilizations(speeds,wcets,avrTask.period);
	vector<pair<int,double>> pair_utils;

	for(unsigned int j = 0; j < wcets.size(); j++)
	{
		pair<int,double> newPair(j,utils[j]);
		pair_utils.push_back(newPair);
	}

	sort(pair_utils.begin(),pair_utils.end(),OptimizationAlgorithm::performanceCoeffsComparator);

	vector<double> omegas;
	for (auto e: avrTask.omegas) omegas.push_back(e.second);

	if (prevAvrTask != NULL) { // Add the transition speeds of the previous avr task
		vector<double> prevTransitionOmegas;
		for(auto e : prevAvrTask->omegas)
			prevTransitionOmegas.push_back(e.second);
		omegas = Utility::merge(omegas,prevTransitionOmegas);
	}

	/*
	for (int index=0; index<pair_utils.size(); index++) {
		vector<int> rts;
		int mode = pair_utils[index].first;
		double omega = avrTask.omegas[mode];
		*/
	for (auto omega : omegas) {
		int maxRT = INT_MIN;

		int currWCET = avrTask.getWCET(omega);
		if (prevAvrTask != NULL)
			currWCET = max(currWCET,prevAvrTask->getWCET(omega));

		double T = mSEC_to_muSEC(avrTask.period/omega);
		int n = floor(rt/T);

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();

		vector<double> tmpExclude;
		necessary_only_response_time_analysis2(true,tasks,i-1,avrTask,maxRT,omega,wcet+currWCET+avrTask.getWCET(omega)*(n),T*n,deadline,toBeExcluded,tmpExclude);

		if (maxRT > deadline) {
			return false;
		}

		lastC = currWCET;
		toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
	}

	return true;
}

bool SchedAnalysis::rbf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
#ifdef __USING_PRE_PROCESSING_RBF__
	/// Prepare the rbfs
	int maxPeriod = 0;
	for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);
	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,maxPeriod);

	for (auto omega:dominants)
		map<int,int> ret = avrTask.calRequestBoundFunction(omega,maxPeriod);
#endif

	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = rbf_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,tasks[i].period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		bool schedulable = rbf_analysis_index_with_avrtask(tasks,i,avrTask,*iter);
		if (!schedulable) {
			return false;
		}
	}
	return true;
}

bool SchedAnalysis::rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	map<int,int> ret = avrTask.calRequestBoundFunction(omega,deadline);
	int rt = calculate_rbf_response_time(tasks,i-1,ret,wcet,0);
	if (rt <= deadline) return true;
	else {
#if 0
		cout << "UnSchedulable: omega=" << omega << ", i=" << i << endl;
		cout << "rt = " << rt << endl;
		Utility::output_one_map(cout,"rbf",ret);

		cout << "ILPCON-" << wcet << ", " << avrTask.calILPCON(omega,wcet);
#endif	
		return false;
	}
}


bool SchedAnalysis::rbf_envelope_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex)
{
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = rbf_envelope_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}


bool SchedAnalysis::rbf_envelope_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask)
{
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,tasks[i].period);

	map<int,int> rbf_envelope;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		map<int,int> ret = avrTask.calRequestBoundFunction(*iter,deadline);
		rbf_envelope = Utility::merge(rbf_envelope,ret);
	}

	int rt = calculate_rbf_response_time(tasks,i-1,rbf_envelope,wcet,0);
	if (rt <= deadline) return true;
	else {
#if 0
		cout << "UnSchedulable: " << "i=" << i << endl;
		cout << "rt = " << rt << endl;
		Utility::output_one_map(cout,"rbf_envelope",rbf_envelope);
#endif	
		return false;
	}
	return true;
}

bool SchedAnalysis::digraph_rbf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
#if 1
	/// Prepare the rbfs
	int maxPeriod = 0;
	for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);

	if (avrTask.digraph_rbf.size()==0)
		avrTask.digraph->calculate_rbf_without_periodicity_DP(maxPeriod,avrTask.digraph_rbf);

	/*
	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,maxPeriod);

	for (auto omega:dominants) {
		cout << "calculating digraph_rbf with " << omega << endl;
		map<int,int> ret;
		avrTask.digraph->calculate_rbf_without_periodicity_DP(omega,maxPeriod,ret);
		avrTask.digraph_rbf[omega] = ret;
	}
	*/
#endif

		for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = digraph_rbf_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::digraph_rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,tasks[i].period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		bool schedulable = digraph_rbf_analysis_index_with_avrtask(tasks,i,avrTask,*iter);
		if (!schedulable) {
			return false;
		}
	}
	return true;
}

bool SchedAnalysis::digraph_rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	map<int,int> ret;
	for (auto e:avrTask.digraph_rbf) {
		if (W_EQ_TOL(e.first,omega)) {
			ret = e.second;
			break;
		}
	}

	if (ret.size() == 0) {
		cerr << "Haven't prepared rbf for the initial speed " << omega << endl;
		//avrTask.digraph->calculate_rbf_without_periodicity_DP(deadline,avrTask.digraph_rbf);
		exit(EXIT_FAILURE);
	}
	
	int rt = calculate_rbf_response_time(tasks,i-1,ret,wcet,0);
	if (rt <= deadline) return true;
	else {
#if 0
		cout << "UnSchedulable: omega=" << omega << ", i=" << i << endl;
		cout << "rt = " << rt << endl;
		Utility::output_one_map(cout,"rbf",ret);

		cout << "ILPCON-" << wcet << ", " << avrTask.calILPCON(omega,wcet);
#endif	
		return false;
	}
}


bool SchedAnalysis::digraph_rbf_envelope_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex)
{
#if 1
	/// Prepare the rbfs
	int maxPeriod = 0;
	for (auto task : tasks) maxPeriod = max(maxPeriod,task.period);

	if (avrTask.digraph_rbf_envelope.size() == 0)
		avrTask.digraph->calculate_rbf_without_periodicity_DP(maxPeriod,avrTask.digraph_rbf_envelope);
#endif

	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = digraph_rbf_envelope_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}


bool SchedAnalysis::digraph_rbf_envelope_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask)
{
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,tasks[i].period);

	if (avrTask.digraph_rbf_envelope.size() == 0) { 
		//avrTask.digraph->calculate_rbf_without_periodicity_DP(deadline,rbf_envelope);
		cerr << "Need to pre-calculate digraph rbf envelope!" << endl;
		exit(EXIT_FAILURE);
	}

	int rt = calculate_rbf_response_time(tasks,i-1,avrTask.digraph_rbf_envelope,wcet,0);
	if (rt <= deadline) return true;
	else return false;
	return true;
}

#ifdef __USING_ILPCPLEX__
bool SchedAnalysis::ilp_con_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = ilp_con_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::ilp_con_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,tasks[i].period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		bool schedulable = ilp_con_analysis_index_with_avrtask(tasks,i,avrTask,*iter);
		if (!schedulable) {
			return false;
		}
	}
	return true;
}

bool SchedAnalysis::ilp_con_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	int currRT = wcet;

	while (true) {
		int nextRT = wcet;
		for (int j=0; j<=i-1; j++) {
			PeriodicTask task = tasks[j];
			nextRT += ceil(1.0*currRT/task.period)*task.wcet;
		}

		nextRT += avrTask.calILPCON(omega,currRT);

#ifdef __DEBUG_DESIGN__
		//cout << omega << ", " << currRT << " => " << recordY << endl;
#endif

		if (nextRT > deadline) return false;

		//////////////////////////////////////////////////////////////////////////
		/// In fact, it should be to check currRT == nextRT. 
		/// But due to the wrong results from the ILP solver, 
		/// we have to check currRT >= nextRT 
		//////////////////////////////////////////////////////////////////////////
		if (currRT >= nextRT) break;

		//if (abs(currRT - nextRT) <= 10) break;
		//cout << "ilpcon=>" << currRT << "\t" << nextRT << endl;
		/*
		cout << "ilpcon=>" << currRT << "\t" << nextRT << endl;
		cout << currRT << "=>" <<  avrTask.calILPCON(omega,currRT) << endl;
		cout << nextRT << "=>" <<  avrTask.calILPCON(omega,nextRT) << endl;
		*/

		currRT = nextRT;
	}

	return true;
}

bool SchedAnalysis::ilp_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex)
{
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = ilp_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::ilp_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask)
{
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	int currRT = wcet;

	while (true) {
		int nextRT = wcet;
		for (int j=0; j<=i-1; j++) {
			PeriodicTask task = tasks[j];
			nextRT += ceil(1.0*currRT/task.period)*task.wcet;
		}

		nextRT += avrTask.calILP(currRT);

		if (nextRT > deadline) return false;

		//////////////////////////////////////////////////////////////////////////
		/// In fact, it should be to check currRT == nextRT. 
		/// But due to the wrong results from the ILP solver, 
		/// we have to check currRT >= nextRT 
		//////////////////////////////////////////////////////////////////////////
		if (currRT >= nextRT) break;

		//if (fabs(currRT - nextRT) <= 10) break;

		/*
		cout << "ilp=>" << currRT << "\t" << nextRT << endl;
		cout << currRT << "=>" <<  avrTask.calILP(currRT) << endl;
		cout << nextRT << "=>" <<  avrTask.calILP(nextRT) << endl;
		*/
		/*
		if (currRT == 7219 && nextRT == 7220) {
			cout << "ilp=>" << currRT << "\t" << nextRT << endl;
			cout << currRT << "=>" <<  avrTask.calILP(currRT) << endl;
			cout << nextRT << "=>" <<  avrTask.calILP(nextRT) << endl;
			exit(EXIT_FAILURE);
		}
		*/
		

		currRT = nextRT;
	}

	return true;
}

bool SchedAnalysis::segmental_ilp_con_exact_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = segmental_ilp_con_exact_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::segmental_ilp_con_exact_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	int nDeadline = floor(1.0*deadline/ANALYSIS_SEGMENT);

	set<int> sDeadlines; // segmental deadlines
	for (int j=1; j<=nDeadline; j++)
		sDeadlines.insert(j*ANALYSIS_SEGMENT);
	sDeadlines.insert(deadline);

	//Timer timer;
	int maxRT = 0;

	for (auto sD : sDeadlines) {
		Engine engine = avrTask.engine;
		vector<double> dominants; // = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,ANALYSIS_SEGMENT);

		if (sD < maxRT) continue;
		if (maxRT > deadline) return false;

		bool schedulable = true;

		if (sD-ANALYSIS_SEGMENT <= 0) {
			dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,sD);

			vector<double> toBeExcluded;
			int lastC = -1;

			for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
				vector<int> rts;

				// If the mode is changed we clean the toBeExcluded list
				if (avrTask.getWCET(*iter) != lastC) toBeExcluded.clear();
				vector<double> tmpExclude;
				exact_response_time_analysis(tasks,i-1,avrTask,rts,*iter,wcet+avrTask.getWCET(*iter),0,sD,toBeExcluded,tmpExclude);

				vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
				if (*rt_iter > sD) {
					schedulable = false;
					maxRT = max(maxRT,*rt_iter);
					break;
				}

				lastC = avrTask.getWCET(*iter);
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}
		else {
			dominants = avrTask.getAllDominants();

			int start = 0;
			if (sD < deadline) start = sD - ANALYSIS_SEGMENT;
			else {
				if (deadline == nDeadline * ANALYSIS_SEGMENT)
					start = sD - ANALYSIS_SEGMENT;
				else
					start = nDeadline * ANALYSIS_SEGMENT;
			}

			vector<double> toBeExcluded;
			int lastC = -1;

			for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
				vector<int> rts;

				//cout << "here!" << endl;
				int ilpcon = NULL;

				int recordOmega = RPmSEC_to_ThousandRPM(*iter);

				if (ILPCONs.find(recordOmega) != ILPCONs.end()) {
					map<int,int> temp = ILPCONs[recordOmega];

					if (temp.find(start) != temp.end()) {
						//cout << "found!" << endl;
						ilpcon = temp[start];
					}
				}

				if (ilpcon == NULL) {
					//timer.start();
					ilpcon = avrTask.calILPCON(*iter,start);
					//timer.end();
					//cout << "tCalILPCON = " << timer.getTime() << endl;
					ILPCONs[recordOmega][start] = ilpcon;
					//cout << recordOmega << "\t" << start << endl;
				}

				// If the mode is changed we clean the toBeExcluded list
				if (avrTask.getWCET(*iter) != lastC) toBeExcluded.clear();
				vector<double> tmpExclude;

				//timer.start();
				exact_response_time_analysis(tasks,i-1,avrTask,rts,*iter,wcet+ilpcon,start,sD,toBeExcluded,tmpExclude);
				//timer.end();
				//cout << "tDoExact = " << timer.getTime() << endl;

				vector<int>::iterator rt_iter = max_element(rts.begin(), rts.end());
				if (*rt_iter > sD) {
					schedulable = false;
					maxRT = max(maxRT,*rt_iter);
					break;
				}

				lastC = avrTask.getWCET(*iter);
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}
		if (schedulable) return true;
	}

	return false;
}

bool SchedAnalysis::segmental_ilp_con_digraph_rbf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = segmental_ilp_con_digraph_rbf_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::segmental_ilp_con_digraph_rbf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	int nDeadline = floor(1.0*deadline/ANALYSIS_SEGMENT);

	set<int> sDeadlines; // segmental deadlines
	for (int j=1; j<=nDeadline; j++)
		sDeadlines.insert(j*ANALYSIS_SEGMENT);
	sDeadlines.insert(deadline);

	vector<double> dominants = avrTask.getAllDominants();

	//Timer timer;
	int maxRT = 0;

	for (auto sD : sDeadlines) {
		if (sD < maxRT) continue;
		if (maxRT > deadline) return false;

		int start = 0;
		if (sD < deadline) start = sD - ANALYSIS_SEGMENT;
		else {
			if (deadline == nDeadline * ANALYSIS_SEGMENT)
				start = sD - ANALYSIS_SEGMENT;
			else
				start = nDeadline * ANALYSIS_SEGMENT;
		}

		bool schedulable = true;

		for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
			map<int,int> ret;
			for (auto e:avrTask.digraph_rbf) {
				if (W_EQ_TOL(e.first,*iter)) {
					ret = e.second;
					break;
				}
			}

			if (ret.size() == 0) {
				cerr << "Haven't prepared rbf for the initial speed " << *iter << endl;
				//avrTask.digraph->calculate_rbf_without_periodicity_DP(deadline,avrTask.digraph_rbf);
				exit(EXIT_FAILURE);
			}

			//cout << "here!" << endl;
			int ilpcon = NULL;

			if (start == 0) ilpcon = avrTask.getWCET(*iter);
			else {
				int recordOmega = RPmSEC_to_ThousandRPM(*iter);

				if (ILPCONs.find(recordOmega) != ILPCONs.end()) {
					map<int,int> temp = ILPCONs[recordOmega];

					if (temp.find(start) != temp.end()) {
						//cout << "found!" << endl;
						ilpcon = temp[start];
					}
				}

				if (ilpcon == NULL) {
					//timer.start();
					ilpcon = avrTask.calILPCON(*iter,start);
					//timer.end();
					//cout << "tCalILPCON = " << timer.getTime() << endl;
					ILPCONs[recordOmega][start] = ilpcon;
					//cout << recordOmega << "\t" << start << endl;
				}
			}

			int rt = calculate_rbf_response_time(tasks,i-1,ret,wcet+ilpcon-avrTask.getWCET(*iter),start);
			if (rt > sD) {
				schedulable = false;
				maxRT = max(maxRT,rt);
				break;
			}
		}

		if (schedulable) return true;
	}

	return false;
}
#endif

bool SchedAnalysis::ibf_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = ibf_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::ibf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask) {
	Engine engine = avrTask.engine;
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,tasks[i].period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		bool schedulable = ibf_analysis_index_with_avrtask(tasks,i,avrTask,*iter);
		if (!schedulable) {
			return false;
		}
	}
	return true;
}

bool SchedAnalysis::ibf_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, double omega) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	map<int,int> ret = avrTask.calInterferenceBoundFunction(omega,DEADLINE_FACTOR*deadline);
	map<int,int> ibfs = avrTask.getInterferenceBoundFunction(omega,ret,DEADLINE_FACTOR*deadline);

	
	int rt = calculate_ibf_response_time(tasks,i-1,ibfs,wcet,0);
	if (rt <= deadline) return true;
	else return false;
}

bool SchedAnalysis::lub_analysis(vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex, AVRTaskPointer prevAvrTask) {
	//avrTask.prepareLinearUppperBound();

	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = lub_analysis_index_with_avrtask(tasks,i-1,avrTask,prevAvrTask);

		if (!schedulable) return false;
	}
	return true;
}

bool SchedAnalysis::lub_analysis_index_with_avrtask(vector<PeriodicTask> tasks, int i, AVRTask avrTask, AVRTaskPointer prevAvrTask) {
	int wcet = tasks[i].wcet;
	int deadline = tasks[i].period;
	
	int rt = calculate_lub_response_time(tasks,i-1,avrTask,wcet,deadline,prevAvrTask);
	//int rt = calculate_lub_response_time(tasks,i-1,avrTask,wcet);
	if (rt > deadline) return false;
	return true;
}

double SchedAnalysis::calculate_response_time(vector<PeriodicTask> tasks, int n, int wcet) {
	double curRT = wcet;
	//int times = 0;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			PeriodicTask task = tasks[i];
			nextRT += ceil(curRT/task.period)*task.wcet;
		}

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
		/*
		if (times > 100)
			cout << times++ << endl;
			*/
	}
	return curRT;
}

double SchedAnalysis::calculate_response_time(PeriodicTask** tasks, int n, RequestFunction* rf, double omegas[], int wcets[], int numMode, int wcet) {
	double curRT = wcet;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			PeriodicTask* task = tasks[i];
			nextRT += ceil(curRT/task->period)*task->wcet;
		}
		nextRT += rf->getValue(curRT,omegas,wcets,numMode);

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}
	return curRT;
}

double SchedAnalysis::calculate_rbf_response_time(vector<PeriodicTask> tasks, int n, map<int,int> interference, int sum_wcet, int sum_period) {
	double curRT = sum_wcet;
	while (true) {
		double nextRT = sum_wcet;
		for (int i=0; i<=n; i++) {
			PeriodicTask task = tasks[i];
			nextRT += ceil(curRT/task.period)*task.wcet;
		}
		nextRT += AVRTask::getRequestBoundFunction(interference,curRT-sum_period);

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}
	return curRT;
}

double SchedAnalysis::calculate_ibf_response_time(vector<PeriodicTask> tasks, int n, map<int,int> interference, int sum_wcet, int sum_period) {
	double curRT = sum_wcet;
	while (true) {
		double nextRT = sum_wcet;
		for (int i=0; i<=n; i++) {
			PeriodicTask task = tasks[i];
			nextRT += ceil(curRT/task.period)*task.wcet;
		}
		nextRT += AVRTask::getInterferenceBoundFunction(interference,curRT-sum_period);

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}
	return curRT;
}

double SchedAnalysis::calculate_lub_response_time(vector<PeriodicTask> tasks, int n, AVRTask avrTask, int wcet) {
#ifdef __DEBUG_DESIGN__
	for (auto task : tasks) 
		task.output(cout);
	
	cout << "AVR task U_{max} = " << avrTask._maxU << "\tC_{max} = " << avrTask._maxC << endl;
	cout << "WCET = " << wcet << endl;
	cout << "tase number = " << tasks.size() << endl;
	cout << " n = " << n << endl;
#endif

	double curRT = wcet;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			PeriodicTask task = tasks[i];
			nextRT += ceil(curRT/task.period)*task.wcet;
		}

		//nextRT += avrTask.getLinearUpperBound(nextRT);
		nextRT = avrTask.getResponseTimeWithWCET(nextRT);

		//cout << "Calculating lub repsonet time: " << curRT << "\t" << nextRT << endl;
		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}
	return curRT;
}

double SchedAnalysis::calculate_lub_response_time(vector<PeriodicTask> tasks, int n, AVRTask avrTask, int wcet, int deadline, AVRTaskPointer prevAvrTask) {
	double curRT = wcet;
	while (true) {
		double nextRT = wcet;
		if (prevAvrTask != NULL) nextRT += max(prevAvrTask->_maxC-avrTask._maxC,0.0);

		for (int i=0; i<=n; i++) {
			PeriodicTask task = tasks[i];
			nextRT += ceil(curRT/task.period)*task.wcet;
		}

		//nextRT += avrTask.getLinearUpperBound(nextRT);
		nextRT = avrTask.getResponseTimeWithWCET(nextRT);
		
		if (nextRT > deadline) return INT_MAX;

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}
	return curRT;
}

void SchedAnalysis::draw_response_time(PeriodicTask** tasks, int n, int wcet, int deadline) {
	set<int> points;
	points.insert(deadline);
	points.insert(0);

	for (int i=0;i<n; i++) {
		PeriodicTask* task = tasks[i];
		int period = task->period;
		int nPeriod = deadline/period;
		for (int j=1; j<=nPeriod; j++) points.insert(j*period);
	}


	map<int,int> rbfs;
	for (set<int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int period = *iter;
		int sum = wcet;
		for (int i=0;i<n; i++) {
			PeriodicTask* task = tasks[i];
			sum += task->getValue(period);
		}
		rbfs[period] = sum;
	}

	Utility::output_one_map(cout,"rbf",rbfs);
}

bool SchedAnalysis::asynchronous_exact_analysis(vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	for (int i=0; i<tasks.size()+1; i++) {
		bool schedulable = false;
		if (i < avrTaskIndex)
			schedulable = asynchronous_exact_analysis_index(tasks,i);
		else if (i == avrTaskIndex)
			schedulable = asynchronous_exact_analysis_index_for_avrtask(tasks,i-1,avrTask);
		else
			schedulable = asynchronous_exact_analysis_index_with_avrtask(tasks,i-1,avrTask);

		if (!schedulable) 
			return false;
	}
	return true;
}

bool SchedAnalysis::asynchronous_exact_analysis_index(vector<AsynchronousPeriodicTask> tasks, int i) {
	AsynchronousPeriodicTask task = tasks[i];
	if (i==0) {
		if (task.wcet > task.deadline) return false;
		else return true;
	}

	int left = 0, right = 1;
	for (int j=0; j<=i; j++) {
		left = max(left,tasks[j].offset);
		right = Utility::math_lcm(right,tasks[j].period);
	}
	right += left;

	int lmin = ceil((1.0*left-task.offset)/task.period);
	int lmax = ceil((1.0*right-task.offset)/task.period)-1;

	for (int j=lmin; j<=lmax; j++) {
		bool schedulable = asynchronous_exact_analysis_index(tasks,i-1,task.wcet,task.deadline, j*task.period + task.offset);
		if (!schedulable) 
			return false;
	}
	return true;
}

bool SchedAnalysis::asynchronous_exact_analysis_index(vector<AsynchronousPeriodicTask> tasks, int i, int wcet, int deadline, int start) {
	int rt = calculate_response_time(tasks,i,wcet,start);

// record the worst-case response time
	if (rts.find(i+1)==rts.end())
		rts[i+1] = rt;
	else
		rts[i+1] = max(rts[i+1],rt);

	//cout << i << "=>" << rt << endl;
	if (rt <= deadline) return true;
	else {
		//cout << "UnSched: " << i << "=>" << rt << endl;
		return false;
	}
}

bool SchedAnalysis::asynchronous_exact_analysis_index_for_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask) {
	if (i==-1) {
		for (int j=0; j<avrTask.numMode; j++) {
			int wcet = avrTask.wcets[j];
			int deadline = Utility::round(mSEC_to_muSEC(avrTask.calDeadline(avrTask.omegas[j])));
			if (wcet > deadline) return false;
		}
		return true;
	}

	int tmin = computingMaxReleaseTime(tasks,i) + tasks[i].period;
	int tmax = tmin + computingPeriodLCM(tasks,i);
	vector<int> asynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,tmin,tmax);

	for (int j=0; j<avrTask.numMode; j++) {
		int wcet = avrTask.wcets[j];
		int deadline = Utility::round(mSEC_to_muSEC(avrTask.calDeadline(avrTask.omegas[j])));
		for (auto start : asynchronousCriticalInstants) {
			int rt = calculate_response_time(tasks,i,wcet,start);
			if (rt > deadline) return false;
		}
	}
	return true;
}

bool SchedAnalysis::asynchronous_exact_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask) {
	AsynchronousPeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int deadline = task.deadline;

	Engine engine = avrTask.engine;
	//vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);
	vector<double> dominants;
	for (auto e: avrTask.omegas) dominants.push_back(e.second);

#ifdef __DEBUG_SCHED_ANALYSIS__
	cout << "Dominant Speeds = ";
	for(auto w0 : dominants) {
		cout << RPmSEC_to_RPM(w0) << " ";
	}
	cout << endl;
#endif

	int tmin = computingMaxReleaseTime(tasks,i) + tasks[i].period;
	int tmax = tmin + computingPeriodLCM(tasks,i);

	int lmin = ceil((1.0*tmin-task.offset)/task.period);
	int lmax = ceil((1.0*tmax-task.offset)/task.period)-1;

	for (int l=lmin; l <= lmax; l++) {
		int release = l*task.period + task.offset;
		int prevRelease = (l-1)*task.period + task.offset;

		vector<int> asynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevRelease,release);

		for (auto start : asynchronousCriticalInstants) {
			vector<double> toBeExcluded;
			int lastC = -1;

			for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
				int maxRT = INT_MIN;
				int currWCET = avrTask.getWCET(*iter);

				// If the mode is changed we clean the toBeExcluded list
				if (currWCET != lastC) toBeExcluded.clear();

				vector<double> tmpExclude;
				//asynchronous_exact_response_time_analysis(tasks,i-1,avrTask,maxRT,*iter,wcet+currWCET,0,deadline,start,release,toBeExcluded,tmpExclude);
				asynchronous_exact_response_time_analysis(tasks,i-1,avrTask,maxRT,*iter,currWCET,0,deadline,start, wcet,release,toBeExcluded,tmpExclude);

				if (maxRT > deadline) {
					return false;
				}

				lastC = currWCET;
				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}
	}

	return true;
}

void SchedAnalysis::asynchronous_exact_response_time_analysis(vector<AsynchronousPeriodicTask> tasks,int i, AVRTask avrTask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, int start, int release, vector<double> excludeList, vector<double>& returnedExcludeList) {	
	int rt = calculate_response_time(tasks,i,sum_wcet,start,release);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask.engine;
	double period = avrTask.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,release-start+maxTime-sum_period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		// If the mode is changed we clean the toBeExcluded list
		if (lastC != avrTask.getWCET(*iter)) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;

		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext - (release-start) >= rt) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		asynchronous_exact_response_time_analysis(tasks,i,avrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,start,release,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(*iter);
	}

	returnedExcludeList = dominants;
}

void SchedAnalysis::asynchronous_exact_response_time_analysis(vector<AsynchronousPeriodicTask> tasks,int i, AVRTask avrTask, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, int start, int pWCET, int release, vector<double> excludeList, vector<double>& returnedExcludeList) {	
	int rt = calculate_response_time(tasks,i,sum_wcet,start,pWCET,release);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask.engine;
	double period = avrTask.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,release-start+maxTime-sum_period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		// If the mode is changed we clean the toBeExcluded list
		if (lastC != avrTask.getWCET(*iter)) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),*iter) != excludeList.end()) continue;

		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext - (release-start) >= rt) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + avrTask.getWCET(omega_next);
		asynchronous_exact_response_time_analysis(tasks,i,avrTask,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,start,pWCET,release,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = avrTask.getWCET(*iter);
	}

	returnedExcludeList = dominants;
}

bool SchedAnalysis::asynchronous_exact_analysis(vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask0, AVRTask avrTask1, int avrTaskIndex, int switchTime)
{
	for (int i=0; i<tasks.size()+1; i++) {
		if (i<= avrTaskIndex) continue;
		bool schedulable = asynchronous_exact_analysis_index_with_avrtask(tasks,i-1,avrTask0, avrTask1, switchTime);
		if (!schedulable) 
			return false;
	}
	return true;
}

bool SchedAnalysis::asynchronous_exact_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask0, AVRTask avrTask1, int switchTime) {
	AsynchronousPeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int period = task.period;
	int deadline = task.deadline;
	int offset = task.offset;

	/// computing the dominants of the mixed AVR task 
	Engine engine = avrTask0.engine;
	vector<double> tranSpeeds0;
	for(auto e : avrTask0.speeds)
		tranSpeeds0.push_back(e.second);

	vector<double> tranSpeeds1;
	for (auto e : avrTask1.speeds)
		tranSpeeds1.push_back(e.second);

	vector<double> mixedTranSpeeds = Utility::merge(tranSpeeds0,tranSpeeds1);
	AVRTask mixedAvrTask(engine,avrTask0.period,mixedTranSpeeds);
	vector<double> dominants = mixedAvrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

	int right = ceil((1.0*switchTime-offset)/period);
	int left = right -1;

	for (int l=left; l <= right; l++) {
		int release = l*task.period + task.offset;
		int prevRelease = (l-1)*task.period + task.offset;

		vector<int> asynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevRelease,release);

		for (auto start : asynchronousCriticalInstants) {
			vector<double> toBeExcluded;
			int lastC = -1;

			for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
				int wcet0 = avrTask0.getWCET(*iter);
				int wcet1 = avrTask1.getWCET(*iter);
				int currWCET = 0;
				int maxRT = INT_MIN;
				if (start >= switchTime) {
					if (wcet0 <= wcet1 || start-switchTime <= wcet0)
						continue;
					else
						currWCET = max(wcet0,wcet1);

					// If the mode is changed we clean the toBeExcluded list
					if (currWCET != lastC) toBeExcluded.clear();

					vector<double> tmpExclude;
					asynchronous_exact_response_time_analysis(tasks,i-1,avrTask0,avrTask1,switchTime,mixedAvrTask,true,*iter,currWCET,maxRT,*iter,wcet+currWCET,0,deadline,start,release,toBeExcluded,tmpExclude);

					if (maxRT > deadline) {
						return false;
					}

					lastC = currWCET;
					toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
				}
				else {
					currWCET = wcet0;

					// If the mode is changed we clean the toBeExcluded list
					if (currWCET != lastC) toBeExcluded.clear();

					vector<double> tmpExclude;
					asynchronous_exact_response_time_analysis(tasks,i-1,avrTask0,avrTask1,switchTime,mixedAvrTask,false,*iter,currWCET,maxRT,*iter,wcet+currWCET,0,deadline,start,release,toBeExcluded,tmpExclude);

					if (maxRT > deadline) {
						return false;
					}

					lastC = currWCET;
					toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
				}
			}
		}
	}

	return true;
}

void SchedAnalysis::asynchronous_exact_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, AVRTask avrTask0, AVRTask avrTask1, int switchTime, AVRTask mixedAvrTask, bool hasSwitched, double initialOmega, int initialWCET, int &maxRT, double omega, int sum_wcet, int sum_period, int maxTime, int start, int release, vector<double> excludedList, vector<double>& returnExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet,start,release);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<double> toBeExcluded;
	int lastC = -1;

	Engine engine = avrTask0.engine;
	double period = avrTask0.period;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants; 
	if (hasSwitched)
		dominants = avrTask1.getDominants(omega_b,omega_a,release-start+maxTime-sum_period);
	else
		dominants = mixedAvrTask.getDominants(omega_b,omega_a,release-start+maxTime-sum_period);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		double omega_next = *iter;
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));

		if (sum_period+tNext - (release-start) >= rt) continue;

		int currWCET = 0;
		if (hasSwitched) 
			currWCET = avrTask1.getWCET(omega_next);
		else {
			currWCET = avrTask0.getWCET(omega_next);
			if (sum_period + tNext >= switchTime-start-engine.getMinDeadline(period,initialOmega) && sum_period + tNext <= switchTime-start+initialWCET)
				currWCET = max(currWCET,avrTask1.getWCET(omega_next));
		}

		// If the mode is changed we clean the toBeExcluded list
		if (lastC != currWCET) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludedList.begin(),excludedList.end(),*iter) != excludedList.end()) continue;

		vector<double> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + currWCET;

		if (!hasSwitched && sum_period2 > switchTime-start + initialWCET) hasSwitched = true;

		asynchronous_exact_response_time_analysis(tasks,i,avrTask0,avrTask1,switchTime,mixedAvrTask,hasSwitched,initialOmega,initialWCET,maxRT,omega_next,sum_wcet2,sum_period2,maxTime,start,release,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = currWCET;
	}

	returnExcludeList = dominants;
}

bool SchedAnalysis::asynchronous_sched_analysis(vector<AsynchronousPeriodicTask> tasks, Digraph* digraph , AVRTask mixedAvrTask, int avrTaskIndex, int switchTime)
{
	for (int i=0; i<tasks.size()+1; i++) {
		if (i<= avrTaskIndex) continue;
#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
		if (i != tasks.size()) continue;
#endif
		//bool schedulable = asynchronous_sched_analysis_index_with_avrtask(tasks,i-1,digraph, switchTime);
		bool schedulable = asynchronous_sched_analysis_index_with_avrtask_refinement(tasks,i-1,digraph, mixedAvrTask, switchTime);
		if (!schedulable) 
			return false;
	}
	return true;
}

bool SchedAnalysis::asynchronous_sched_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime) {
	AsynchronousPeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int period = task.period;
	int deadline = task.deadline;
	int offset = task.offset;

	PeriodicTask pAVR(digraph->minVertexC,digraph->minVertexT);

	int right = ceil((1.0*switchTime-offset)/period);
	int left = right -1;

	Timer timer;

	for (int l=left; l <= right; l++) {
		int release = l*task.period + task.offset;

		if (release+deadline <= switchTime) continue;

		int prevRelease = (l-1)*task.period + task.offset;
		//cout << "period = " << period << ", offset=" << offset << ", release = " << release << endl;

		/// Calculate the response time of job instance tau_{i,l-1} without the AVR task
		int prevPrevRelease = max(0,(l-2)*task.period + task.offset);
		vector<int> prevAsynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevPrevRelease,prevRelease);
		int prevRT = 0;
		for (auto start: prevAsynchronousCriticalInstants) {
			int rt = calculate_response_time(tasks,i-1,pAVR,0,start,wcet,prevRelease);
			prevRT = max(prevRT,rt);
		}
		//cout << "i = " << i << ", prevRelease = " << prevRelease << ", wcet = " << wcet << ", prevRT = " << prevRT << endl;

		vector<int> asynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevRelease,release);

		/*
		Utility::output_one_vector(cout,"CriticalInstants",asynchronousCriticalInstants);
		cerr << "exit" << endl;
		exit(EXIT_FAILURE);
		*/

		for (auto start : asynchronousCriticalInstants) {
			if (start <= prevRT+prevRelease) {
				//cout << "Prunning: prevRT + prevRelease = " << prevRT + prevRelease << ", start = " << start << endl;
				continue;
			}

			//int start = release;
			vector<Node*> toBeExcluded;
			int lastC0 = -1;
			int lastC1 = -1;

#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__
 			for (auto node : digraph->node_vec) {
#else
			auto node = digraph->node_vec.back();
#endif
				int maxWCET = max(node->wcet,node->nextWCET);
				int nodeWCET = 0;
				if (start < switchTime) 
					nodeWCET = node->wcet;
				else if (start-switchTime < maxWCET) 
					nodeWCET = maxWCET;
				else 
					nodeWCET = node->nextWCET;

				//cout << "i = " << i << ", release = " << release << ", start = " << start << ", node is " << node->toString() << endl;

				int minT = INT_MAX;
				for (auto inEdge : node->in) {
					minT = min(minT,inEdge->separationTime);
				}

				// If the mode is changed we clean the toBeExcluded list
				if (lastC0 != node->wcet || lastC1 != node->nextWCET) toBeExcluded.clear();

				vector<Node*> tmpExclude;

				//int rangeL = start-nodeWCET;
				int rangeL = start;
				int rangeH = start+minT;

				//cout << "No Switch! start = " << start << ", switchTime = " << switchTime << ", minT+start = " << minT + start << endl;
				int maxRT = INT_MIN;
				timer.start();
#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__
				//asynchronous_sched_response_time_analysis(tasks, i-1, digraph, switchTime, maxRT, node, wcet + nodeWCET, rangeL, rangeH,deadline,start,release,toBeExcluded, tmpExclude);
				asynchronous_sched_response_time_analysis(tasks, i-1, digraph, switchTime, maxRT, node, nodeWCET, rangeL, rangeH,deadline,start,wcet, release,toBeExcluded, tmpExclude);
#else
				list<Node*> nodeList;
				nodeList.push_back(node);
				//asynchronous_sched_response_time_analysis(tasks, i-1, digraph, switchTime, maxRT, node, nodeList, wcet + nodeWCET, rangeL, rangeH,deadline,start,release,toBeExcluded, tmpExclude);
				asynchronous_sched_response_time_analysis(tasks, i-1, digraph, switchTime, maxRT, node, nodeList, nodeWCET, rangeL, rangeH,deadline,start,wcet, release,toBeExcluded, tmpExclude);
#endif
				timer.end();
				//cout << "P0: " << timer.getTime() << endl;
				if (maxRT > deadline) {
#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
					cout << "Task " << i << ": released = " << release << ", start = " << start << ", node = " << node->toString() << ", RT = " << maxRT << endl;
#endif
					return false;
				}

				lastC0 = node->wcet;
				lastC1 = node->nextWCET;

				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__	
			}
#endif
		}
	}

	return true;
}

void SchedAnalysis::asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int release, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet,start,release);
	maxRT = max(maxRT,rt);

#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
	cout << switchTime << ", " << rt << ", " << node->toString() << ", " << sum_wcet << ", " << sum_min_period << ", " << sum_max_period << endl;
#endif

	if (maxRT > maxTime) 
		return;

	vector<Node*> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto outEdge : node->out) {
		Node* nextNode = outEdge->snk_node;

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		//int pmin = mSEC_to_muSEC(outEdge->separationTime);
		//int pmax = mSEC_to_muSEC(outEdge->maxSeparationTime);

		int pmin = outEdge->separationTime;
		int pmax = outEdge->maxSeparationTime;

		//cout << pmin << ", " << pmax << "=>" << sum_min_period << ", " << sum_max_period << ", " << release + rt << endl;
		//exit(EXIT_FAILURE);

		int sum_min_period2 = sum_min_period + pmin;
		int sum_max_period2 = sum_max_period + pmax;

		vector<Node*> tmpExcluded;

		if (sum_min_period2 >= rt+release) {
			//cout << "Prunning!" << endl;
			continue;
		}
		else {
			sum_max_period2 = min(sum_max_period2,rt+release);
			if (sum_min_period2 < switchTime && sum_max_period2 > switchTime && nextNode->wcet !=  nextNode->nextWCET) {
				// before the switch time
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, sum_wcet + nextNode->wcet, sum_min_period2, switchTime,maxTime, start, release,toBeExcluded,tmpExcluded);
				
				// after the switch time, note these nodes cannot be pruned in the previous execution modes
				vector<Node*> emptySet0;
				vector<Node*> emptySet1;
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, sum_wcet + nextNode->nextWCET, switchTime, sum_max_period2,maxTime,start, release,emptySet0,emptySet1);					
			}
			else {
				int nodeWCET = 0;
				if (sum_max_period2 < switchTime) nodeWCET = nextNode->wcet;
				else nodeWCET = nextNode->nextWCET;
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, sum_wcet + nodeWCET, sum_min_period2, sum_max_period2,maxTime,start,release,toBeExcluded,tmpExcluded);
			}
		}

		if (maxRT > maxTime) 
			return;

		if (!tmpExcluded.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}

	for (auto outEdge:node->out) {
		Node* nextNode = outEdge->snk_node;
		if(find(returnedExcludeList.begin(),returnedExcludeList.end(),nextNode) != returnedExcludeList.end()) continue;
		returnedExcludeList.push_back(nextNode);
	}
}

void SchedAnalysis::asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int pWCET, int release, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet,start,pWCET,release);
	maxRT = max(maxRT,rt);

	if (maxRT > maxTime) 
		return;

	vector<Node*> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto outEdge : node->out) {
		Node* nextNode = outEdge->snk_node;

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		//int pmin = mSEC_to_muSEC(outEdge->separationTime);
		//int pmax = mSEC_to_muSEC(outEdge->maxSeparationTime);

		int pmin = outEdge->separationTime;
		int pmax = outEdge->maxSeparationTime;

		//cout << pmin << ", " << pmax << "=>" << sum_min_period << ", " << sum_max_period << ", " << release + rt << endl;
		//exit(EXIT_FAILURE);

		int sum_min_period2 = sum_min_period + pmin;
		int sum_max_period2 = sum_max_period + pmax;

		vector<Node*> tmpExcluded;

		if (sum_min_period2 >= rt+release) {
			//cout << "Prunning!" << endl;
			continue;
		}
		else {
			sum_max_period2 = min(sum_max_period2,rt+release);
			if (sum_min_period2 < switchTime && sum_max_period2 > switchTime && nextNode->wcet !=  nextNode->nextWCET) {
				// before the switch time
				//asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nextNode->wcet, sum_min_period2, switchTime,maxTime, start, release,toBeExcluded,tmpExcluded);
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, sum_wcet+nextNode->wcet, sum_min_period2, switchTime,maxTime, start, pWCET, release,toBeExcluded,tmpExcluded);

				// after the switch time, note these nodes cannot be pruned in the previous execution modes
				vector<Node*> emptySet0;
				vector<Node*> emptySet1;
				//asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nextNode->nextWCET, switchTime, sum_max_period2,maxTime,start, release,emptySet0,emptySet1);					
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, sum_wcet+nextNode->nextWCET, switchTime, sum_max_period2,maxTime,start,pWCET,release,emptySet0,emptySet1);	
			}
			else {
				int nodeWCET = 0;
				if (sum_max_period2 < switchTime) nodeWCET = nextNode->wcet;
				else nodeWCET = nextNode->nextWCET;
				//asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nodeWCET, sum_min_period2, sum_max_period2,maxTime,start,release,toBeExcluded,tmpExcluded);
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, sum_wcet+nodeWCET, sum_min_period2, sum_max_period2,maxTime,start,pWCET,release,toBeExcluded,tmpExcluded);
			}
		}

		if (maxRT > maxTime) 
			return;

		if (!tmpExcluded.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}

	for (auto outEdge:node->out) {
		Node* nextNode = outEdge->snk_node;
		if(find(returnedExcludeList.begin(),returnedExcludeList.end(),nextNode) != returnedExcludeList.end()) continue;
		returnedExcludeList.push_back(nextNode);
	}
}

void SchedAnalysis::asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, list<Node*> nodeList, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int release, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet,start,release);
	maxRT = max(maxRT,rt);

#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
	if (rt == 112422) {
		for (auto e : nodeList) cout << e->toString() << "=>";
		cout << endl;
		cout << switchTime << ", " << rt << ", " << sum_wcet << ", " << sum_min_period << ", " << sum_max_period << endl;
	}
#endif

	if (maxRT > maxTime) 
		return;

	vector<Node*> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto outEdge : node->out) {
		Node* nextNode = outEdge->snk_node;

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		// I'm thinking why Biondi can do this
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		//int pmin = mSEC_to_muSEC(outEdge->separationTime);
		//int pmax = mSEC_to_muSEC(outEdge->maxSeparationTime);

		int pmin = outEdge->separationTime;
		int pmax = outEdge->maxSeparationTime;

		//cout << pmin << ", " << pmax << "=>" << sum_min_period << ", " << sum_max_period << ", " << release + rt << endl;
		//exit(EXIT_FAILURE);

		int sum_min_period2 = sum_min_period + pmin;
		int sum_max_period2 = sum_max_period + pmax;

		vector<Node*> tmpExcluded;

		if (sum_min_period2 >= rt+release) {
			//cout << "Prunning!" << endl;
			continue;
		}
		else {
			nodeList.push_back(nextNode);
			sum_max_period2 = min(sum_max_period2,rt+release);
			if (sum_min_period2 < switchTime && sum_max_period2 > switchTime && nextNode->wcet !=  nextNode->nextWCET) {
				// before the switch time
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nextNode->wcet, sum_min_period2, switchTime,maxTime, start, release,toBeExcluded,tmpExcluded);

				// after the switch time, note these nodes cannot be pruned in the previous execution modes
				vector<Node*> emptySet0;
				vector<Node*> emptySet1;
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nextNode->nextWCET, switchTime, sum_max_period2,maxTime,start, release,emptySet0,emptySet1);					
			}
			else {
				int nodeWCET = 0;
				if (sum_max_period2 < switchTime) nodeWCET = nextNode->wcet;
				else nodeWCET = nextNode->nextWCET;
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nodeWCET, sum_min_period2, sum_max_period2,maxTime,start,release,toBeExcluded,tmpExcluded);
			}
		}

		if (maxRT > maxTime) 
			return;

		if (!tmpExcluded.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}

	for (auto outEdge:node->out) {
		Node* nextNode = outEdge->snk_node;
		if(find(returnedExcludeList.begin(),returnedExcludeList.end(),nextNode) != returnedExcludeList.end()) continue;
		returnedExcludeList.push_back(nextNode);
	}
}

void SchedAnalysis::asynchronous_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, Node* node, list<Node*> nodeList, int sum_wcet, int sum_min_period, int sum_max_period, int maxTime, int start, int pWCET, int release, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet,start,pWCET,release);
	maxRT = max(maxRT,rt);

#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
	if (rt == 131562) {
		cout << "+++++++++++++++++++" << endl;
		for (auto e : nodeList) cout << e->toString() << endl;
		cout << "+++++++++++++++++++" << endl;
		cout << start << "," << release << ", " << switchTime << ", " << rt << ", " << sum_wcet << ", " << sum_min_period << ", " << sum_max_period << ", " << pWCET << endl;
		exit(EXIT_FAILURE);
	}
#endif

	if (maxRT > maxTime) 
		return;

	vector<Node*> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto outEdge : node->out) {
		Node* nextNode = outEdge->snk_node;

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		//int pmin = mSEC_to_muSEC(outEdge->separationTime);
		//int pmax = mSEC_to_muSEC(outEdge->maxSeparationTime);

		int pmin = outEdge->separationTime;
		int pmax = outEdge->maxSeparationTime;

		//cout << pmin << ", " << pmax << "=>" << sum_min_period << ", " << sum_max_period << ", " << release + rt << endl;
		//exit(EXIT_FAILURE);

		int sum_min_period2 = sum_min_period + pmin;
		int sum_max_period2 = sum_max_period + pmax;

		vector<Node*> tmpExcluded;

		if (sum_min_period2 >= rt+release) {
			//cout << "Prunning!" << endl;
			continue;
		}
		else {
			nodeList.push_back(nextNode);
			sum_max_period2 = min(sum_max_period2,rt+release);
			if (sum_min_period2 < switchTime && sum_max_period2 > switchTime && nextNode->wcet !=  nextNode->nextWCET) {
				// before the switch time
				//asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nextNode->wcet, sum_min_period2, switchTime,maxTime, start, release,toBeExcluded,tmpExcluded);
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet+nextNode->wcet, sum_min_period2, switchTime,maxTime, start,pWCET , release,toBeExcluded,tmpExcluded);

				// after the switch time, note these nodes cannot be pruned in the previous execution modes
				vector<Node*> emptySet0;
				vector<Node*> emptySet1;
				//asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nextNode->nextWCET, switchTime, sum_max_period2,maxTime,start, release,emptySet0,emptySet1);					
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet+nextNode->nextWCET, switchTime, sum_max_period2,maxTime,start,pWCET,release,emptySet0,emptySet1);	
			}
			else {
				int nodeWCET = 0;
				if (sum_max_period2 < switchTime) nodeWCET = nextNode->wcet;
				else nodeWCET = nextNode->nextWCET;
				//asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet + nodeWCET, sum_min_period2, sum_max_period2,maxTime,start,release,toBeExcluded,tmpExcluded);
				asynchronous_sched_response_time_analysis(tasks, i, digraph, switchTime, maxRT, nextNode, nodeList, sum_wcet+nodeWCET, sum_min_period2, sum_max_period2,maxTime,start,pWCET,release,toBeExcluded,tmpExcluded);
			}
		}

		if (maxRT > maxTime) 
			return;

		if (!tmpExcluded.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}

	for (auto outEdge:node->out) {
		Node* nextNode = outEdge->snk_node;
		if(find(returnedExcludeList.begin(),returnedExcludeList.end(),nextNode) != returnedExcludeList.end()) continue;
		returnedExcludeList.push_back(nextNode);
	}
}

bool SchedAnalysis::asynchronous_sched_analysis_index_with_avrtask_refinement(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, AVRTask mixedAvrTask, int switchTime) {
	AsynchronousPeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int period = task.period;
	int deadline = task.deadline;
	int offset = task.offset;

	Engine engine = mixedAvrTask.engine;
	vector<double> dominants = mixedAvrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);

	PeriodicTask pAVR(digraph->minVertexC,digraph->minVertexT);

	int right = ceil((1.0*switchTime-offset)/period);
	int left = right -1;

	Timer timer;

	for (int l=left; l <= right; l++) {
		int release = l*task.period + task.offset;

		if (release+deadline <= switchTime) continue;

		int prevRelease = (l-1)*task.period + task.offset;
		//cout << "period = " << period << ", offset=" << offset << ", release = " << release << endl;

		/// Calculate the response time of job instance tau_{i,l-1} without the AVR task
		int prevPrevRelease = max(0,(l-2)*task.period + task.offset);
		vector<int> prevAsynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevPrevRelease,prevRelease);
		int prevRT = 0;
		for (auto start: prevAsynchronousCriticalInstants) {
			int rt = calculate_response_time(tasks,i-1,pAVR,0,start,wcet,prevRelease);
			prevRT = max(prevRT,rt);
		}
		//cout << "i = " << i << ", prevRelease = " << prevRelease << ", wcet = " << wcet << ", prevRT = " << prevRT << endl;

		vector<int> asynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevRelease,release);

		/*
		Utility::output_one_vector(cout,"CriticalInstants",asynchronousCriticalInstants);
		cerr << "exit" << endl;
		exit(EXIT_FAILURE);
		*/

		for (auto start : asynchronousCriticalInstants) {
			if (start <= prevRT+prevRelease) {
				//cout << "Prunning: prevRT + prevRelease = " << prevRT + prevRelease << ", start = " << start << endl;
				continue;
			}

#ifdef __USING_MIXED_AVRTASK__
			vector<double> initialOmegas;
			vector<double> toBeExcludedOmegas;
			int lastC = -1;

			for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
				int maxRT = INT_MIN;
				int currWCET = mixedAvrTask.getWCET(*iter);

				// If the mode is changed we clean the toBeExcluded list
				if (currWCET != lastC) toBeExcludedOmegas.clear();

				vector<double> tmpExcludeOmegas;
				//asynchronous_exact_response_time_analysis(tasks,i-1,avrTask,maxRT,*iter,wcet+currWCET,0,deadline,start,release,toBeExcluded,tmpExclude);
				asynchronous_exact_response_time_analysis(tasks,i-1,mixedAvrTask,maxRT,*iter,currWCET,0,deadline,start, wcet,release,toBeExcludedOmegas,tmpExcludeOmegas);

				lastC = currWCET;

				if (maxRT > deadline) {
					initialOmegas.push_back(*iter);
					toBeExcludedOmegas.clear();
				} else {
					toBeExcludedOmegas.insert(toBeExcludedOmegas.end(),tmpExcludeOmegas.begin(),tmpExcludeOmegas.end());
				}
			}

			if (initialOmegas.empty()) continue;
#endif
			vector<NodeItem> toBeExcludedNodeItems;
			int lastC0 = -1;
			int lastC1 = -1;

#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__
 			for (auto node : digraph->node_vec) {
#else
			auto node = digraph->node_vec.back();
#endif

#ifdef __USING_MIXED_AVRTASK__
				bool hasVerified = true;
				for (auto omega : initialOmegas) {
					//if (W_LEQ_TOL(node->omegaL,omega) && W_LEQ_TOL(omega,node->omegaH)) {
					if (W_LEQ_TOL(node->omegaL,omega)) {
						hasVerified = false;
						//cout << "Has verified: " << RPmSEC_to_RPM(omega) << "," << RPmSEC_to_RPM(node->omegaL) << "," << RPmSEC_to_RPM(node->omegaH) << endl;
						break;
					}
				}
				if (hasVerified) continue;
#endif

				//cout << "here" << endl;
				//exit(EXIT_FAILURE);

				int maxWCET = max(node->wcet,node->nextWCET);
				int nodeWCET = 0;
				if (start < switchTime) 
					nodeWCET = node->wcet;
				else if (start-switchTime < maxWCET) 
					nodeWCET = maxWCET;
				else 
					nodeWCET = node->nextWCET;

				// If the mode is changed we clean the toBeExcluded list
				if (lastC0 != node->wcet || lastC1 != node->nextWCET) toBeExcludedNodeItems.clear();

				//cout << "i = " << i << ", release = " << release << ", start = " << start << ", node is " << node->toString() << endl;

				int minT = INT_MAX;
				for (auto inEdge : node->in) {
					minT = min(minT,inEdge->separationTime);
				}

				int maxRT = INT_MIN;

				int tMinus = start-nodeWCET;
				int tPlus = start+minT;

				int tMinusStar = tMinus; 
				while (tMinusStar < tPlus) {
					int tPlusStar = min(switchTime,tPlus);
					if (tMinusStar >= switchTime)
						tPlusStar = tPlus;

					int CSum = 0;
					if (tMinusStar < switchTime) CSum = node->wcet;
					else CSum = node->nextWCET;

					vector<NodeItem> tmpExcludedNodeItems;

					ODRTSearchPath path;
					path.addNode(node,tMinusStar,tPlusStar);
					
					Timer timer;
					timer.start();
					//asynchronous_sched_response_time_analysis_refinement(tasks,i-1,digraph,switchTime,maxRT,path,CSum,deadline,start,wcet,release);
					asynchronous_sched_response_time_analysis_refinement(tasks,i-1,digraph,switchTime,maxRT,path,CSum,deadline,start,wcet,release,toBeExcludedNodeItems,tmpExcludedNodeItems);
					timer.end();
					//cout << timer.getTime() << endl;

					tMinusStar = tPlusStar;
					if (maxRT > deadline) {
						//cout << maxRT << endl;
						return false;
					}

					toBeExcludedNodeItems.insert(toBeExcludedNodeItems.end(),tmpExcludedNodeItems.begin(),tmpExcludedNodeItems.end());
				}

				lastC0 = node->wcet;
				lastC1 = node->nextWCET;
#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__	
			}
#endif
		}
	}

	return true;
}

void SchedAnalysis::asynchronous_sched_response_time_analysis_refinement(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime, int start, int pWCET, int release)
{
	Node* bNode = path.nodes.front();
	int tBMinus = path.tMinusVec.front();
	int tBPlus = path.tPlusVec.front();

	Node* eNode = path.nodes.back();
	int tEMinus = path.tMinusVec.back();
	int tEPlus = path.tPlusVec.back();

	int AVRStart = -1;
	if (tBPlus < start) 
		AVRStart = tBPlus;
	else if (tBMinus > start) 
		AVRStart = tBMinus;
	else
		AVRStart = start;

	int rt = calculate_response_time(tasks,i, CSum + max(start-tBPlus,0),start,pWCET,release);

	maxRT = max(maxRT,rt);
	//cout << path.nodes.size() << endl;

	if (maxRT > maxTime) {
#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
		cout << start << ", " << release << ", " << CSum << ", " << pWCET << ", " << maxTime << endl;
		cout << AVRStart << ", " << rt << ", " << maxRT << endl;
		path.output(cout);
		exit(EXIT_FAILURE);
#endif
		return;
	}

	for (auto edge : eNode->out) {
		Node* nextNode = edge->snk_node;

		// To prune this vertex
		if (path.getSumMinSeparationTime(AVRStart) + edge->separationTime >= rt + release) {
			//cout << maxRT << endl;
			//cout << "Pruning!" << endl;
			continue;
		}

		int tLB = minimum_binary_search(tasks,i,digraph,path,CSum,start,pWCET,release,edge,tBMinus,AVRStart);
		int tUB = maximum_binary_search(tasks,i,digraph,path,CSum,start,pWCET,release,edge,AVRStart,tBPlus);

#if 0
		if (tLB > path.tMinusVec.front()) {
			cout << "Happen MinBinSearch!" << endl;
		}

		if (tUB < path.tPlusVec.front()) {
			cout << "Happen MaxBinSearch!" << endl;
		}
#endif

		int tMinus = path.getSumMinSeparationTime(tLB) + edge->separationTime;
		int tPlus = min(path.getSumMaxSeparationTime(tUB)+edge->maxSeparationTime,rt+release);

#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
		cout << "Info:" << endl;
		cout << tLB << "," << tUB << "," <<  path.getSumMinSeparationTime(AVRStart)+ edge->separationTime << "," << rt + release << endl;
		cout << tMinus << "," << tPlus << "," << path.getSumMinSeparationTime(tLB) << ", " << edge->separationTime << "," << edge->maxSeparationTime << endl;
#endif

		int tMinusStar = tMinus;
		while (tMinusStar < tPlus) {
			int tPlusStar = min(switchTime,tPlus);
			if (tMinusStar >= switchTime)
				tPlusStar = tPlus;

			//cout << tMinusStar << "," << tPlusStar << "," << tMinus << "," << tPlus << "," << switchTime << endl;

			int nextCSum = CSum;
			if (tMinusStar < switchTime) nextCSum += nextNode->wcet;
			else nextCSum += nextNode->nextWCET;

			ODRTSearchPath pathStarStar = generate_ODRT_search_path(path,tLB,tUB,tMinusStar,tPlusStar,nextNode,edge);
			asynchronous_sched_response_time_analysis_refinement(tasks,i,digraph,switchTime,maxRT,pathStarStar,nextCSum,maxTime,start,pWCET,release);

			tMinusStar = tPlusStar;
			if (maxRT > maxTime) return;
		}
	}
}

void SchedAnalysis::asynchronous_sched_response_time_analysis_refinement(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime, int start, int pWCET, int release,vector<NodeItem> excludedList, vector<NodeItem>& returnExcludedList)
{
	Node* bNode = path.nodes.front();
	int tBMinus = path.tMinusVec.front();
	int tBPlus = path.tPlusVec.front();

	Node* eNode = path.nodes.back();
	int tEMinus = path.tMinusVec.back();
	int tEPlus = path.tPlusVec.back();

	int AVRStart = -1;
	if (tBPlus < start) 
		AVRStart = tBPlus;
	else if (tBMinus > start) 
		AVRStart = tBMinus;
	else
		AVRStart = start;

	int rt = calculate_response_time(tasks,i, CSum + max(start-tBPlus,0),start,pWCET,release);

	maxRT = max(maxRT,rt);
	//cout << path.nodes.size() << endl;

	if (maxRT > maxTime) {
#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
		cout << start << ", " << release << ", " << CSum << ", " << pWCET << ", " << maxTime << endl;
		cout << AVRStart << ", " << rt << ", " << maxRT << endl;
		path.output(cout);
		exit(EXIT_FAILURE);
#endif
		return;
	}

	vector<NodeItem> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto edge : eNode->out) {
		Node* nextNode = edge->snk_node;

		// To prune this vertex
		if (path.getSumMinSeparationTime(AVRStart) + edge->separationTime >= rt + release) {
			//cout << maxRT << endl;
			//cout << "Pruning!" << endl;
			continue;
		}

		int tLB = minimum_binary_search(tasks,i,digraph,path,CSum,start,pWCET,release,edge,tBMinus,AVRStart);
		int tUB = maximum_binary_search(tasks,i,digraph,path,CSum,start,pWCET,release,edge,AVRStart,tBPlus);

#if 0
		if (tLB > path.tMinusVec.front()) {
			cout << "Happen MinBinSearch!" << endl;
		}

		if (tUB < path.tPlusVec.front()) {
			cout << "Happen MaxBinSearch!" << endl;
		}
#endif

		int tMinus = path.getSumMinSeparationTime(tLB) + edge->separationTime;
		int tPlus = min(path.getSumMaxSeparationTime(tUB)+edge->maxSeparationTime,rt+release);

#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
		cout << "Info:" << endl;
		cout << tLB << "," << tUB << "," <<  path.getSumMinSeparationTime(AVRStart)+ edge->separationTime << "," << rt + release << endl;
		cout << tMinus << "," << tPlus << "," << path.getSumMinSeparationTime(tLB) << ", " << edge->separationTime << "," << edge->maxSeparationTime << endl;
#endif

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning:
		NodeItem currItem(nextNode,tMinus,tPlus);
		bool hasVerified = false;
		for (auto item : excludedList) {
			hasVerified = item.check(currItem,tMinus,tPlus);
			if (hasVerified) 
				break;
		}
		if (hasVerified) {
			//cout << "Additional Pruning!" << endl;
			continue;
		}

		// reset current node item
		currItem.lValue = tMinus;
		currItem.rValue = tPlus;
		returnExcludedList.push_back(currItem);

		int tMinusStar = tMinus;
		while (tMinusStar < tPlus) {
			int tPlusStar = min(switchTime,tPlus);
			if (tMinusStar >= switchTime)
				tPlusStar = tPlus;

			//cout << tMinusStar << "," << tPlusStar << "," << tMinus << "," << tPlus << "," << switchTime << endl;

			int nextCSum = CSum;
			if (tMinusStar < switchTime) nextCSum += nextNode->wcet;
			else nextCSum += nextNode->nextWCET;

			vector<NodeItem> tmpExcluded;
			ODRTSearchPath pathStarStar = generate_ODRT_search_path(path,tLB,tUB,tMinusStar,tPlusStar,nextNode,edge);
			asynchronous_sched_response_time_analysis_refinement(tasks,i,digraph,switchTime,maxRT,pathStarStar,nextCSum,maxTime,start,pWCET,release,toBeExcluded,tmpExcluded);

			if (!tmpExcluded.empty())
				toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

			tMinusStar = tPlusStar;
			if (maxRT > maxTime) return;
		}

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}
}

int SchedAnalysis::minimum_binary_search(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, int start, int pWCET, int release, Edge* edge, int tLB, int tUB)
{
	if (tLB == tUB) return tLB;

	// Check where tLB can be pruned
	int rt = calculate_response_time(tasks,i, CSum + max(start-tLB,0),start,pWCET,release);
	// cannot be pruned
	if (path.getSumMinSeparationTime(tLB)+edge->separationTime < rt + release) 
		return tLB;

	while (tUB-tLB > MAX_ERROR) {
		int tB = (tLB+tUB)/2;
		rt = calculate_response_time(tasks,i, CSum + max(start-tB,0),start,pWCET,release);
		if (path.getSumMinSeparationTime(tB)+edge->separationTime >= rt + release)
			tLB = tB;
		else
			tUB = tB;
	}

	return tLB;
}

int SchedAnalysis::maximum_binary_search(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, int start, int pWCET, int release, Edge* edge, int tLB, int tUB)
{
	if (tLB == tUB) return tUB;

	// Check where tUB can be pruned
	int rt = calculate_response_time(tasks,i, CSum + max(start-tUB,0),start,pWCET,release);
	// cannot be pruned
	if (path.getSumMinSeparationTime(tUB)+edge->separationTime < rt + release) 
		return tUB;

	while (tUB-tLB > MAX_ERROR) {
		int tB = (tLB+tUB)/2;
		rt = calculate_response_time(tasks,i, CSum + max(start-tB,0),start,pWCET,release);
		if (path.getSumMinSeparationTime(tB)+edge->separationTime >= rt + release)
			tUB = tB;
		else
			tLB = tB;
	}

	return tUB;
}

ODRTSearchPath SchedAnalysis::generate_ODRT_search_path(ODRTSearchPath path, int tLB, int tUB, int tMinus, int tPlus, Node* node, Edge* edge)
{
	ODRTSearchPath newPath;

	// Add edges
	for(auto e:path.edges) {
		newPath.addEdge(e);
	}
	newPath.addEdge(edge);

	// Generate the \pi_{l}^- and \pi_{l}^+
	list<int> lValues;
	list<int> rValues;

	lValues.push_front(tMinus);
	rValues.push_front(tPlus);

	for (int l=newPath.edges.size()-1; l>=0; l--) {
		Edge* currEdge = newPath.edges[l];
		int lValue = max(lValues.front()-currEdge->maxSeparationTime,path.tMinusVec[l]);
		int rValue = min(rValues.front()-currEdge->separationTime,path.tPlusVec[l]);

		if (l==0) {
			lValue = max(lValue,tLB);
			rValue = min(rValue,tUB);
		}

		lValues.push_front(lValue);
		rValues.push_front(rValue);
	}

	list<int>::iterator lIter = lValues.begin();
	list<int>::iterator rIter = rValues.begin();

	// Add nodes
	for (int i=0; i<path.nodes.size(); i++) {
		newPath.addNode(path.nodes[i],*lIter,*rIter);
		lIter++;
		rIter++;
	}
	newPath.addNode(node,*lIter,*rIter);

	return newPath;
}

bool SchedAnalysis::check_next_vertex(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath& path, int CSum, int start, int pWCET, int release, int fMinus, int fPlus, Node* node, int& tMinus, int& tPlus)
{

	Node* bNode = path.nodes.front();
	int tBMinus = path.tMinusVec.front();
	int tBPlus = path.tPlusVec.front();

	int tBLB = tBMinus;
	int tBUB = tBPlus;

	bool legal = true;
	if (fPlus <= tMinus)
		goto PRUNING;

	

	while ((fMinus < tMinus) && (tBUB-tBLB > MAX_ERROR)) {
		int tB = (tBLB + tBUB)/2;

		int CSumMinus = CSum - max(start-tB,0);
		int fB = calculate_response_time(tasks,i,CSumMinus,start,pWCET,release) + release;

		int tMinusStar = 0;
		legal = path.checkPathLB(node,tMinus,tPlus,tB,tMinusStar);

		if (!legal) {
			tBUB = tB;
			tBPlus = tB;
			fPlus = fB;
			if (fPlus <= tMinus)
				goto PRUNING;
			continue;
		}

		if (fB < tMinusStar) {
			tBLB = tB;
			tMinus = tMinusStar;
			if (fPlus <= tMinus)
				goto PRUNING;
		}
		else
			tBUB = tB;
	}

	if (!legal)
		goto PRUNING;

	tBMinus = tBLB;
	if (!path.update(tBMinus,tBPlus))
		goto PRUNING;

	tBUB = tBPlus;


	while ((fPlus < tPlus) && (tBUB-tBLB > MAX_ERROR)) {
		int tB = (tBLB + tBUB)/2;

		int tMinusStar = 0, tPlusStar = 0;
		legal = path.checkPathUB(node,tMinus,tPlus,tB,tMinusStar,tPlusStar);

		if (!legal) {
			tBLB = tB;
			tBMinus = tB;
			tMinus = tMinusStar;
			if (fPlus <= tMinus)
				goto PRUNING;
			continue;
		}

		int CSumMinus = CSum - max(start-tB,0);
		int fB = calculate_response_time(tasks,i,CSumMinus,start,pWCET,release) + release;

		if (fB < tPlusStar) {
			tBUB = tB;
			fPlus = fB;
			if (fPlus <= tMinus)
				goto PRUNING;
		}
		else
			tBLB = tB;
	}

	if (!legal)
		goto PRUNING;

	tBPlus = tBUB;
	if (!path.update(tBMinus,tBPlus))
		goto PRUNING;

	return true;
PRUNING:
	return false;
}

bool SchedAnalysis::sched_analysis_with_avrtask_refinement(vector<PeriodicTask> tasks, Digraph* digraph, AVRTask mixedAvrTask, AVRTask avrTask, int avrTaskIndex, int switchTime)
{
	for (int i=0; i<tasks.size()+1; i++) {
		if (i<= avrTaskIndex) continue;
#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
		if (i != tasks.size()) continue;
#endif
		bool schedulable = sched_analysis_index_with_avrtask_refinement(tasks,i-1,digraph,mixedAvrTask, avrTask, switchTime);
		if (!schedulable) 
			return false;
	}
	return true;
}

bool SchedAnalysis::sched_analysis_index_with_avrtask_refinement(vector<PeriodicTask> tasks, int i, Digraph* digraph, AVRTask mixedAvrTask, AVRTask avrTask, int switchTime) {
	PeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int period = task.period;
	int deadline = task.deadline;
	int lastC = -1;

#ifdef __USING_MIXED_AVRTASK__
	// determine the initial engine speeds which cannot be scheduled under the mixed AVR task
	double firstUnSchededOmega = NULL;
	Engine engine = avrTask.engine;
	
	vector<double> dominants;  
	/// First round : using the transition speeds as the initial speeds
	// dominants = mixedAvrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,deadline);
	for (auto e: mixedAvrTask.omegas) dominants.push_back(e.second);
	sort(dominants.begin(),dominants.end(),greater<double>());

#if 0
	Utility::output_one_vector(cout,"DominantSpeeds", dominants);
	exit(EXIT_FAILURE);
#endif

	// collect the unscheduled initial engine speeds
	vector<double> toBeExcluded;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		int maxRT = INT_MIN;

		int currWCET = mixedAvrTask.getWCET(*iter);

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();
		vector<double> tmpExclude;
		exact_response_time_analysis(tasks,i-1,mixedAvrTask,maxRT,*iter,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		lastC = currWCET;

		if (maxRT > deadline) {
			firstUnSchededOmega = *iter;
			break;
		}
		else {
			toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
		}
	}

	if (firstUnSchededOmega == NULL) {
		//cout << "Mixed AVR task Pruning!" << endl;
		return true;
	}

#if 0
	cout << "First unschedulable speed : " << RPmSEC_to_RPM(firstUnSchededOmega) << endl;
	exit(EXIT_FAILURE);
#endif

	// Second round: check the speed interval from firstUnSchededOmega to SPEED_MIN
	dominants = mixedAvrTask.getDominants(engine.SPEED_MIN, firstUnSchededOmega,deadline);
	vector<int> unSchededOmegaIndexes;
	unSchededOmegaIndexes.push_back(0);

	// collect the unscheduled initial engine speeds
	toBeExcluded.clear();
	lastC = -1;

	for (int omegaIndex = 1; omegaIndex < dominants.size(); omegaIndex++) {
		int maxRT = INT_MIN;
		double omega = dominants[omegaIndex];

		int currWCET = mixedAvrTask.getWCET(omega);

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();
		vector<double> tmpExclude;
		exact_response_time_analysis(tasks,i-1,mixedAvrTask,maxRT,omega,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		lastC = currWCET;

		if (maxRT > deadline) {
			unSchededOmegaIndexes.push_back(omegaIndex);
			toBeExcluded.clear();
		}
		else {
			toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
		}
	}

#if 0
	Utility::output_one_vector(cout,"unSchededOmegaIndexes",unSchededOmegaIndexes);
	Utility::output_one_vector(cout,"DominantSpeeds",dominants);
	exit(EXIT_FAILURE);
#endif

#endif

#ifdef __USING_MAX_DIGRAPH__
	// determine the initial nodes which cannot be scheduled under the max digraph-based task
	vector<Node*> unSchededInitialNodes;

	vector<Node*> toBeExcluded;
	for (vector<Node*>::iterator iter = digraph->node_vec.begin(); iter != digraph->node_vec.end(); iter++) {
		int maxRT = INT_MIN;
		Node* node = *iter;

		int currWCET = max(node->wcet,node->nextWCET);

		// If the mode is changed we clean the toBeExcluded list
		if (currWCET != lastC) toBeExcluded.clear();
		vector<Node*> tmpExclude;
		exact_response_time_analysis_digraph_max(tasks,i-1,digraph,maxRT,node,wcet+currWCET,0,deadline,toBeExcluded,tmpExclude);

		lastC = currWCET;

		if (maxRT > deadline) {
			unSchededInitialNodes.push_back(node);
			toBeExcluded.clear();
		}
		else {
			toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
		}
	}

	if (unSchededInitialNodes.empty()) 
		return true;

#endif

	int tMinus = switchTime-period;
	int tPlus = switchTime;

	int maxRT = INT_MIN;
	map<Node*,AVLTree*> record;

	vector<NodeItem> toBeExcludedNodeItems;
	lastC = -1;

#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__
	for (auto node : digraph->node_vec) {
#else
	auto node = digraph->node_vec.back();
#endif
		//cout << "Task" << i << ":Node=>" << node->toString() << endl;
#ifdef __USING_MIXED_AVRTASK__
		// check whether we need to verify this node
		if (W_GEQ_TOL(node->omegaL,firstUnSchededOmega))
			continue;

		bool hasVerified = true;
		for (auto omegaIndex : unSchededOmegaIndexes) {
			double ubOmega = dominants[omegaIndex];
			double lbOmega = 0;
			if (omegaIndex != dominants.size()-1) 
				lbOmega = dominants[omegaIndex+1];
			else 
				lbOmega = engine.SPEED_MIN;

			if (W_GEQ_TOL(lbOmega,node->omegaH) || W_LEQ_TOL(ubOmega,node->omegaL)) continue;

			hasVerified = false;
			//cout << "Has verified: " << RPmSEC_to_RPM(lbOmega) << "/" << RPmSEC_to_RPM(ubOmega) << " , " << RPmSEC_to_RPM(node->omegaL) << "/" << RPmSEC_to_RPM(node->omegaH) << endl;
			//exit(EXIT_FAILURE);
			break;
		}
		if (hasVerified) continue;
#endif

#ifdef __USING_MAX_DIGRAPH__
		// Check the extra pruning
		if (find(unSchededInitialNodes.begin(),unSchededInitialNodes.end(), node) == unSchededInitialNodes.end()) 
			continue;
#endif

		int CSum = wcet + node->wcet;

		// If the mode is changed we clean the toBeExcluded list
		if (lastC != node->wcet) toBeExcludedNodeItems.clear();

		vector<NodeItem> tmpExcludedNodeItems;

#if 0
		DigraphSearchItem currItem(tMinus,tPlus,CSum);

		if (record.find(node) == record.end()) {
			AVLTree* tree = new AVLTree();
			tree->insert(currItem);
			record[node] = tree;
		}
		else {
			vector<DigraphSearchItem> vec;
			bool check = record[node]->check(vec,currItem);
			if (check) {
#if 0
				cout << node->toString() << endl;
				cout << "AVL tree Pruning!" << endl;
				cout << currItem << endl;
				record[node]->write_graphviz(cout);
				exit(EXIT_FAILURE);
#endif
				continue;
			}
		}
#endif

		ODRTSearchPath path;
		path.addNode(node,tMinus,tPlus);
#if 0
		Timer timer;
		timer.start();
#endif
		//sched_response_time_analysis_refinement(tasks,i-1,digraph,avrTask,switchTime,maxRT,path,CSum,deadline,record);
		sched_response_time_analysis_refinement(tasks,i-1,digraph,avrTask,switchTime,maxRT,path,CSum,deadline,record,toBeExcludedNodeItems,tmpExcludedNodeItems);
#if 0
		timer.end();
		//cout << timer.getTime() << endl;
#endif

		if (maxRT > deadline) {
			//cout << maxRT << endl;
			return false;
		}

		lastC = node->wcet;
		toBeExcludedNodeItems.insert(toBeExcludedNodeItems.end(),tmpExcludedNodeItems.begin(),tmpExcludedNodeItems.end());

#ifndef __DEBUG_SCHED_ANALYSIS_DAVR__
	}
#endif
				
	return true;
}

void SchedAnalysis::sched_response_time_analysis_refinement(vector<PeriodicTask> tasks, int i, Digraph* digraph, AVRTask avrTask, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime,map<Node*,AVLTree*>& record)
{
	Node* bNode = path.nodes.front();
	int tBMinus = path.tMinusVec.front();
	int tBPlus = path.tPlusVec.front();

	Node* eNode = path.nodes.back();

	int rt = calculate_response_time(tasks,i, CSum);

	maxRT = max(maxRT,rt);
	//cout << path.nodes.size() << endl;

	if (maxRT > maxTime) {
		return;
	}

	for (auto edge : eNode->out) {
		Node* nextNode = edge->snk_node;

		// To prune this vertex
		if (path.getSumMinSeparationTime(tBPlus) + edge->separationTime >= rt + tBPlus) {
			continue;
		}

		int tLB = minimum_binary_search(tasks,i,digraph,path,CSum,edge,tBMinus,tBPlus);

		int tMinus = path.getSumMinSeparationTime(tLB) + edge->separationTime;
		int tPlus = min(path.getSumMaxSeparationTime(tBPlus)+edge->maxSeparationTime,rt+tBPlus);

		//cout << tMinus << ", " << tPlus << ": " << edge->separationTime << ", " << rt+tBPlus << endl;

		// if tMinus >= switchTime, we can only use the current AVR task to do the analysis
		if (tMinus >= switchTime) {
			//cout << "After switch time!" << endl;
			vector<double> excludedList;
			vector<double> returnExcludeList;
			exact_response_time_analysis(tasks,i,avrTask,maxRT,nextNode->omegaH,CSum+nextNode->nextWCET,tMinus,maxTime,excludedList,returnExcludeList);
		}
		else {
			int tMinusStar = tMinus;
			while (tMinusStar < tPlus) {
				int tPlusStar = min(switchTime,tPlus);
				if (tMinusStar >= switchTime)
					tPlusStar = tPlus;

				int nextCSum = CSum;
				if (tMinusStar < switchTime) nextCSum += nextNode->wcet;
				else nextCSum += nextNode->nextWCET;

#if 0
				DigraphSearchItem currItem(tMinusStar,tPlusStar,nextCSum);

				if (record.find(nextNode) == record.end()) {
					AVLTree* tree = new AVLTree();
					tree->insert(currItem);
					record[nextNode] = tree;
				}
				else {
					vector<DigraphSearchItem> vec;
					bool check = record[nextNode]->check(vec,currItem);
					if (check) {
#if 0
						cout << nextNode->toString() << endl;
						cout << "AVL tree Pruning!" << endl;
						cout << currItem << endl;
						record[nextNode]->write_graphviz(cout);
						exit(EXIT_FAILURE);
#endif
						continue;
					}
				}
#endif

				ODRTSearchPath pathStarStar = generate_ODRT_search_path(path,tLB,tBPlus,tMinusStar,tPlusStar,nextNode,edge);

				//pathStarStar.output(cout);

				sched_response_time_analysis_refinement(tasks,i,digraph, avrTask, switchTime,maxRT,pathStarStar,nextCSum,maxTime,record);

				tMinusStar = tPlusStar;
				if (maxRT > maxTime) return;
			}
		}
	}
}

void SchedAnalysis::sched_response_time_analysis_refinement(vector<PeriodicTask> tasks, int i, Digraph* digraph, AVRTask avrTask, int switchTime, int &maxRT, ODRTSearchPath path, int CSum, int maxTime,map<Node*,AVLTree*>& record,vector<NodeItem> excludedList, vector<NodeItem>& returnExcludedList)
{
	Node* bNode = path.nodes.front();
	int tBMinus = path.tMinusVec.front();
	int tBPlus = path.tPlusVec.front();

	Node* eNode = path.nodes.back();

	int rt = calculate_response_time(tasks,i, CSum);

	maxRT = max(maxRT,rt);
	//cout << path.nodes.size() << endl;

	if (maxRT > maxTime) {
		return;
	}

	vector<NodeItem> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto edge : eNode->out) {
		Node* nextNode = edge->snk_node;

		// To prune this vertex
		if (path.getSumMinSeparationTime(tBPlus) + edge->separationTime >= rt + tBPlus) {
			continue;
		}

		int tLB = minimum_binary_search(tasks,i,digraph,path,CSum,edge,tBMinus,tBPlus);

		int tMinus = path.getSumMinSeparationTime(tLB) + edge->separationTime;
		int tPlus = min(path.getSumMaxSeparationTime(tBPlus)+edge->maxSeparationTime,rt+tBPlus);

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning:
		NodeItem currItem(nextNode,tMinus,tPlus);
		bool hasVerified = false;
		for (auto item : excludedList) {
			hasVerified = item.check(currItem,tMinus,tPlus);
			if (hasVerified) 
				break;
		}
		if (hasVerified) {
			//cout << "Additional Pruning!" << endl;
			continue;
		}

		// reset current node item
		currItem.lValue = tMinus;
		currItem.rValue = tPlus;
		returnExcludedList.push_back(currItem);
		//cout << tMinus << ", " << tPlus << ": " << edge->separationTime << ", " << rt+tBPlus << endl;

		// if tMinus >= switchTime, we can only use the current AVR task to do the analysis
		if (tMinus >= switchTime) {
			//cout << "After switch time!" << endl;
			vector<double> excludedList2;
			vector<double> returnExcludeList2;
			exact_response_time_analysis(tasks,i,avrTask,maxRT,nextNode->omegaH,CSum+nextNode->nextWCET,tMinus,maxTime,excludedList2,returnExcludeList2);
		}
		else {
			int tMinusStar = tMinus;
			while (tMinusStar < tPlus) {
				int tPlusStar = min(switchTime,tPlus);
				if (tMinusStar >= switchTime)
					tPlusStar = tPlus;

				int nextCSum = CSum;
				if (tMinusStar < switchTime) nextCSum += nextNode->wcet;
				else nextCSum += nextNode->nextWCET;

#if 0
				DigraphSearchItem currItem(tMinusStar,tPlusStar,nextCSum);

				if (record.find(nextNode) == record.end()) {
					AVLTree* tree = new AVLTree();
					tree->insert(currItem);
					record[nextNode] = tree;
				}
				else {
					vector<DigraphSearchItem> vec;
					bool check = record[nextNode]->check(vec,currItem);
					if (check) {
#if 0
						cout << nextNode->toString() << endl;
						cout << "AVL tree Pruning!" << endl;
						cout << currItem << endl;
						record[nextNode]->write_graphviz(cout);
						exit(EXIT_FAILURE);
#endif
						continue;
					}
				}
#endif
				vector<NodeItem> tmpExcluded;
				ODRTSearchPath pathStarStar = generate_ODRT_search_path(path,tLB,tBPlus,tMinusStar,tPlusStar,nextNode,edge);

				//pathStarStar.output(cout);

				sched_response_time_analysis_refinement(tasks,i,digraph, avrTask, switchTime,maxRT,pathStarStar,nextCSum,maxTime,record,toBeExcluded,tmpExcluded);

				if (!tmpExcluded.empty())
					toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

				tMinusStar = tPlusStar;
				if (maxRT > maxTime) return;
			}
		}

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}
}

int SchedAnalysis::minimum_binary_search(vector<PeriodicTask> tasks, int i, Digraph* digraph, ODRTSearchPath path, int CSum, Edge* edge, int tLB, int tUB)
{
	if (tLB == tUB) return tLB;

	// Check where tLB can be pruned
	int rt = calculate_response_time(tasks,i, CSum);
	// cannot be pruned
	if (path.getSumMinSeparationTime(tLB)+edge->separationTime < rt + tLB) 
		return tLB;

	while (tUB-tLB > MAX_ERROR) {
		int tB = (tLB+tUB)/2;
		//rt = calculate_response_time(tasks,i, CSum);
		if (path.getSumMinSeparationTime(tB)+edge->separationTime >= rt + tB)
			tLB = tB;
		else
			tUB = tB;
	}

	return tLB;
}

void SchedAnalysis::exact_response_time_analysis_digraph_max(vector<PeriodicTask> tasks,int i, Digraph* digraph, int &maxRT, Node* node, int sum_wcet, int sum_period, int maxTime, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {	
	int rt = calculate_response_time(tasks,i,sum_wcet);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;

	vector<Node*> toBeExcluded;
	int lastC = -1;

	// Require: list node->out->snk_node sorted in the descending order of engine speed
	for (auto edge : node->out) {
		Node* nextNode = edge->snk_node;
		// If the mode is changed we clean the toBeExcluded list
		int maxC = max(nextNode->wcet,nextNode->nextWCET);
		if (lastC != maxC) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		int tNext = edge->separationTime;

		if (sum_period+tNext >= rt) continue;

		vector<Node*> tmpExclude;
		int sum_period2 = sum_period+ tNext;
		int sum_wcet2 = sum_wcet + maxC;
		exact_response_time_analysis_digraph_max(tasks,i,digraph,maxRT,nextNode,sum_wcet2,sum_period2,maxTime,toBeExcluded,tmpExclude);

		if (!tmpExclude.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExclude.begin(), tmpExclude.end());
		lastC = maxC;
	}

	returnedExcludeList.clear();
	for (auto edge : node->out) {
		returnedExcludeList.push_back(edge->snk_node);
	}
}

bool SchedAnalysis::asynchronous_sufficient_sched_analysis(vector<AsynchronousPeriodicTask> tasks, Digraph* digraph , int avrTaskIndex, int switchTime)
{
	for (int i=0; i<tasks.size()+1; i++) {
		if (i<= avrTaskIndex) continue;
		//if (i != tasks.size()) continue;
		bool schedulable = asynchronous_sufficient_sched_analysis_index_with_avrtask(tasks,i-1,digraph, switchTime);
		if (!schedulable) 
			return false;
	}
	return true;
}

bool SchedAnalysis::asynchronous_sufficient_sched_analysis_index_with_avrtask(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime) {
	AsynchronousPeriodicTask task = tasks[i];
	int wcet = task.wcet;
	int period = task.period;
	int deadline = task.deadline;
	int offset = task.offset;

	PeriodicTask pAVR(digraph->minVertexC,digraph->minVertexT);

	int right = ceil((1.0*switchTime-offset)/period);
	int left = right -1;

	Timer timer;

	for (int l=left; l <= right; l++) {
		int release = l*task.period + task.offset;

		if (release+deadline <= switchTime) continue;

		int prevRelease = (l-1)*task.period + task.offset;
		//cout << "period = " << period << ", offset=" << offset << ", release = " << release << endl;

		/// Calculate the response time of job instance tau_{i,l-1} without the AVR task
		int prevPrevRelease = max(0,(l-2)*task.period + task.offset);
		vector<int> prevAsynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevPrevRelease,prevRelease);
		int prevRT = 0;
		for (auto start: prevAsynchronousCriticalInstants) {
			int rt = calculate_response_time(tasks,i-1,pAVR,wcet,start,prevRelease);
			prevRT = max(prevRT,rt);
		}
		//cout << "i = " << i << ", prevRelease = " << prevRelease << ", wcet = " << wcet << ", prevRT = " << prevRT << endl;

		vector<int> asynchronousCriticalInstants = computing_asynchronous_critial_instants(tasks,i,prevRelease,release);

		for (auto start : asynchronousCriticalInstants) {
			if (start <= prevRT+prevRelease) {
				//cout << "Prunning: prevRT + prevRelease = " << prevRT + prevRelease << ", start = " << start << endl;
				continue;
			}

			//int start = release;
			vector<Node*> toBeExcluded;
			int lastC0 = -1;
			int lastC1 = -1;

 			for (auto node : digraph->node_vec) {
				bool afterSwitch = true;
				int nodeWCET = 0;
				if (start < switchTime) {
					afterSwitch = false;
					nodeWCET = node->wcet;
				}
				else if (start == switchTime || start - switchTime < node->wcet ) 
					nodeWCET = max(node->wcet,node->nextWCET);
				else 
					nodeWCET = node->nextWCET;

				//cout << "i = " << i << ", release = " << release << ", start = " << start << ", node is " << node->toString() << endl;

				// If the mode is changed we clean the toBeExcluded list
				if (lastC0 != node->wcet || lastC1 != node->nextWCET) toBeExcluded.clear();
				vector<Node*> tmpExclude;
				
				int maxRT = INT_MIN;
				//timer.start();
				asynchronous_sufficient_sched_response_time_analysis(tasks, i-1, digraph, switchTime, afterSwitch, maxRT, node, wcet + nodeWCET, start,deadline,start,release,toBeExcluded, tmpExclude);
				//timer.end();
				//cout << "P0: " << timer.getTime() << endl;
				if (maxRT > deadline) {
					return false;
				}

				lastC0 = node->wcet;
				lastC1 = node->nextWCET;

				toBeExcluded.insert(toBeExcluded.end(),tmpExclude.begin(),tmpExclude.end());
			}
		}
	}

	return true;
}

void SchedAnalysis::asynchronous_sufficient_sched_response_time_analysis(vector<AsynchronousPeriodicTask> tasks, int i, Digraph* digraph, int switchTime, bool afterSwitch, int &maxRT, Node* node, int sum_wcet, int sum_period, int maxTime, int start, int release, vector<Node*> excludeList, vector<Node*>& returnedExcludeList) {
	int rt = calculate_response_time(tasks,i,sum_wcet,start,release);
	maxRT = max(maxRT,rt);
	if (maxRT > maxTime) 
		return;
	//cout << "Searched vertex number = " << searchedVertexNum << endl;
	//cout << sum_wcet << " " << sum_period << " " << switchTime << " " << afterSwitch << " "  << rt << endl;
	//if (sum_period == switchTime) exit(EXIT_FAILURE);

	vector<Node*> toBeExcluded;
	int lastC0 = -1;
	int lastC1 = -1;

	for (auto outEdge : node->out) {
		Node* nextNode = outEdge->snk_node;

		// If the mode is changed we clean the toBeExcluded list
		if (lastC0 != nextNode->wcet || lastC1 != nextNode->nextWCET) toBeExcluded.clear();

		// Additional pruning: if the dominant is in exclude list then SKIP IT!
		// I'm thinking why Biondi can do this
		if(find(excludeList.begin(),excludeList.end(),nextNode) != excludeList.end()) continue;

		int pmin = mSEC_to_muSEC(outEdge->separationTime);
		int sum_period2 = sum_period + pmin;

		vector<Node*> tmpExcluded;

		if (sum_period2 >= rt+release) {
			//cout << "Prunning!" << endl;
			continue;
		}
		else {
			/// happen switch
			if (sum_period2 >= switchTime && !afterSwitch) {
				afterSwitch = true;
				//exit(EXIT_FAILURE);
				int nodeWCET = max(nextNode->wcet,nextNode->nextWCET);
				asynchronous_sufficient_sched_response_time_analysis(tasks, i, digraph, switchTime, afterSwitch, maxRT, nextNode, sum_wcet + nodeWCET, switchTime,maxTime,start,release,toBeExcluded,tmpExcluded);
			}
			else {
				int nodeWCET = 0;
				if (afterSwitch) nodeWCET = nextNode->nextWCET;
				else nodeWCET = nextNode->wcet;
				asynchronous_sufficient_sched_response_time_analysis(tasks, i, digraph, switchTime, afterSwitch, maxRT, nextNode, sum_wcet + nodeWCET, sum_period2,maxTime,start,release,toBeExcluded,tmpExcluded);
			}
		}

		if (!tmpExcluded.empty())
			toBeExcluded.insert(toBeExcluded.end(), tmpExcluded.begin(), tmpExcluded.end());

		lastC0 = nextNode->wcet;
		lastC1 = nextNode->nextWCET;
	}

	for (auto outEdge:node->out) {
		Node* nextNode = outEdge->snk_node;
		//if(find(returnedExcludeList.begin(),returnedExcludeList.end(),nextNode) != returnedExcludeList.end()) continue;
		returnedExcludeList.push_back(nextNode);
	}
}

int SchedAnalysis::computingMaxReleaseTime(vector<AsynchronousPeriodicTask> tasks, int i)
{
	int ret = 0;
	for (int j=0; j<=i; j++) ret = max(ret,tasks[j].offset);
	return ret;
}

int SchedAnalysis::computingPeriodLCM(vector<AsynchronousPeriodicTask> tasks, int i)
{
	int lcm = 1;
	for (int j=0; j<=i; j++) {
		lcm = Utility::math_lcm(lcm,tasks[j].period);
	}
	return lcm;
}

int SchedAnalysis::computingLit(vector<AsynchronousPeriodicTask> tasks, int i, int t, int last_known_idle)
{
	if (i < 0) return t; // there are no higher priority tasks

	int last_idle_instant = 0, total_idle_time = 0, iterator = 0;

	if (last_known_idle != 0) {
		last_idle_instant = last_known_idle;

		int sum = 0;
		for (int j=0; j<=i; j++) sum += tasks[j].getWorkload(last_known_idle);
		total_idle_time = last_known_idle - sum;

		iterator = last_known_idle;
	}

	while(1) {
		int last_iterator = iterator;

		int sum = 0;
		for (int j=0; j<=i; j++) sum += tasks[j].getWorkload(iterator);
		iterator = sum + total_idle_time;

		if (iterator > t)
			return last_idle_instant;

		int rho = INT_MAX;
		for (int j=0; j<=i; j++) rho = min(rho,tasks[j].getNextComputingInstant(iterator));

		if (last_iterator == iterator ) { // arrived at an idle instant
			if (iterator <= t && t <= rho)
				return t;
			last_idle_instant = rho;
			total_idle_time += rho - iterator;
			int fac0  = 0, fac1 = 0;
			for (int j=0; j<=i; j++) {
				fac0 += tasks[j].getWorkloadBar(iterator);
				fac1 += tasks[j].getWorkload(iterator);
			}
			iterator = rho + fac0 - fac1;
		}
	}
}

std::vector<int> SchedAnalysis::computing_asynchronous_critial_instants(vector<AsynchronousPeriodicTask> tasks, int i, int t_min, int t_max, int last_known_idle /*= 0*/)
{
	int next_work_instant = computingLit(tasks,i,t_min);
	vector<int> work_instants;
	while (1) {
		int next_idle_instant = computing_next_idle_instants(tasks,i,next_work_instant);

		int rho = INT_MAX;
		for (int j=0; j<=i; j++) rho = min(rho,tasks[j].getNextComputingInstant(next_idle_instant));

		next_work_instant = rho;

		if (next_work_instant > t_max)
			return work_instants;

		work_instants.push_back(next_work_instant);
	}
}

int SchedAnalysis::computing_next_idle_instants(vector<AsynchronousPeriodicTask> tasks, int i, int work_instant)
{
	int sum_workload = 0, sum_workloadBar = 0;
	for (int j=0; j<=i; j++) {
		sum_workload += tasks[j].getWorkload(work_instant);
		sum_workloadBar += tasks[j].getWorkloadBar(work_instant);
	}

	int total_idle_time = work_instant - sum_workload;
	int iterator = work_instant + sum_workloadBar - sum_workload;

	while (1) {
		int last_iterator = iterator;

		sum_workload = 0;
		for (int j=0; j<=i; j++) sum_workload += tasks[j].getWorkload(iterator);
		iterator = sum_workload + total_idle_time;

		if (iterator == last_iterator) return iterator;
	}
}

double SchedAnalysis::calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, int wcet, int start, int release) {
	double curRT = wcet + start;
	int L = computingLit(tasks,n,start);
	//int times = 0;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			AsynchronousPeriodicTask task = tasks[i];
			nextRT += task.getWorkload(curRT);
			nextRT -= task.getWorkload(L);
		}
		nextRT += L;

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}

	if (release == NULL)
		return curRT-start;
	else
		return curRT-release;
}

double SchedAnalysis::calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, int wcet, int start, int pWCET, int release) {
	double curRT = wcet + start;
	int L = computingLit(tasks,n,start);
	//int times = 0;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			AsynchronousPeriodicTask task = tasks[i];
			nextRT += task.getWorkload(curRT);
			nextRT -= task.getWorkload(L);
		}
		nextRT += L;
		if (curRT >= release) 
			nextRT += pWCET;

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}

	return curRT-release;
}

double SchedAnalysis::calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, PeriodicTask pTask, int wcet, int start, int release) {
	double curRT = wcet + start;
	int L = computingLit(tasks,n,start);
	//int times = 0;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			AsynchronousPeriodicTask task = tasks[i];
			nextRT += task.getWorkload(curRT);
			nextRT -= task.getWorkload(L);
		}
		nextRT += L;
		nextRT += ceil((curRT-start)/pTask.period) * pTask.wcet;

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}

	if (release == NULL)
		return curRT-start;
	else
		return curRT-release;
}

double SchedAnalysis::calculate_response_time(vector<AsynchronousPeriodicTask> tasks, int n, PeriodicTask pTask, int wcet, int start, int pWCET, int release) {
	double curRT = wcet + start;
	int L = computingLit(tasks,n,start);
	//int times = 0;
	while (true) {
		double nextRT = wcet;
		for (int i=0; i<=n; i++) {
			AsynchronousPeriodicTask task = tasks[i];
			nextRT += task.getWorkload(curRT);
			nextRT -= task.getWorkload(L);
		}
		nextRT += L;
		nextRT += ceil((curRT-start)/pTask.period) * pTask.wcet;
		if (curRT >= release)
			nextRT += pWCET;

		if (abs(curRT - nextRT) <= 0.01) break;
		curRT = nextRT;
	}

	return curRT-release;
}
