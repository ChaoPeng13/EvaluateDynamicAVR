#include "RunTest.h"

using namespace std;

/*
* An example comes from the ECRTS2014 paper 
* "Exact Interference of Adaptive Variable-Rate Tasks under Fixed-Priority Scheduling"
*/
void RunTest::test(int max, int scale) {
	Timer timer;

	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.621 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets[] = {965,576,424,343,277,246};

	//static const double _speeds[] = {5500,6500};
	//static const int _wcets[] = {277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	int modeNum = speeds.size();

	AVRTask avrTask(engine,period,speeds,wcets);

#if 1
	avrTask.buildDigraph();

	avrTask.prepareLinearUppperBound();
#endif


	int granularity = 200;
	vector<double> speedPartition = SchedAnalysis::getSpeedPartitionWithConstLength(avrTask,granularity);
	Digraph* digraph = SchedAnalysis::generateOffsetBasedDRT(avrTask,speedPartition);
	digraph->calculate_period_gcd();
	digraph->calculate_all_gcd();

	cout << "#node = " << digraph->node_vec.size() << ", #edge = " << digraph->edge_vec.size() << endl;
	//digraph->write_graphviz(cout);
	// Pre-processing
	/*
	digraph->generate_strongly_connected_components();
	digraph->check_strongly_connected();
	
	if (!digraph->strongly_connected) {
		cerr << "Not strongly connected!" << endl;
		exit(EXIT_FAILURE);
	}

	digraph->calculate_linear_factor();

	digraph->linear_factor = 
	*/

	cout << avrTask._maxU << endl;
	Utility::output_one_map(cout,"modePeriods",avrTask.modePeriods);
	Utility::output_one_map(cout,"modeUtilizations",avrTask.modeUtils);

	//digraph->unit_digraph = new UnitDigraph(digraph);
	//digraph->unit_digraph->prepare3(false);

	map<int,int> rbfs0;
	map<int,int> rbfs1;

	timer.start();
	//digraph->calculate_rbf_with_periodicity_DP(max*scale,avrTask.modeUtils[modeNum-1], avrTask.modePeriods[modeNum-1], rbfs0);
	timer.end();
	double t0 = timer.getTime();

	timer.start();
	digraph->calculate_rbf_without_periodicity_DP(max*scale, rbfs1);
	timer.end();
	double t1 = timer.getTime();

	Utility::output_one_map(cout,"rbfs0",rbfs0);
	Utility::output_one_map(cout,"rbfs1",rbfs1);

	cout << t0 << "\t" << t1 << endl;

	//vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN, engine.SPEED_MAX,max*scale);
	vector<double> dominants = avrTask.getAllDominants();

#if 1
	//digraph->calculate_rbf_without_periodicity_DP(max*scale,digraph->rbf_map2_DP);

	map<int,int> rbf_envelope;
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		map<int,int> ret = avrTask.calRequestBoundFunction(*iter,max*scale);
		rbf_envelope = Utility::merge(rbf_envelope,ret);
	}

	Utility::output_one_map(cout,"rbfs2",rbf_envelope);

#if 0

	for (int t = scale; t <= max*scale; t += scale) {
		int digraph_rbf = avrTask.getRequestBoundFunction(rbfs1,t);
		int avrtask_rbf = avrTask.getRequestBoundFunction(rbf_envelope,t);
		cout << t << "=>" << digraph_rbf << "\t" << avrtask_rbf << endl;

		if (digraph_rbf != avrtask_rbf) {
			cerr << "Error calculating rbf!" << endl;
			exit(EXIT_FAILURE);
		}
	}

#endif

#if 1 // RBF
	vector<int> vec_time;
	vector<double> vec_rbf1;
	vector<double> vec_rbf2;

	for (int t=scale; t<=max*scale; t+=scale) {
		int rbf1 = avrTask.getRequestBoundFunction(rbf_envelope,t);
		int rbf2 = avrTask.getRequestBoundFunction(rbfs1,t);

		vec_time.push_back(t/scale);
		vec_rbf1.push_back(rbf1);
		vec_rbf2.push_back(rbf2);
	}

	Utility::output_one_vector(cout,"time",vec_time);
	Utility::output_one_vector(cout,"rbf1",vec_rbf1);
	Utility::output_one_vector(cout,"rbf2",vec_rbf2);
#endif

#endif


#if 0
	map<double,map<int,int>> digraph_rbfs;
	digraph->calculate_rbf_without_periodicity_DP(max*scale,digraph_rbfs);

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		//if (!W_EQ_TOL(*iter,RPM_to_RPmSEC(6384.74))) continue;
		map<int,int> ret = avrTask.calRequestBoundFunction(*iter,max*scale);

		map<int,int> ret2 = digraph_rbfs[*iter];
		//digraph->calculate_rbf_without_periodicity_DP(*iter,max*scale,ret2);

		cout << "===========" << *iter << "==========" << endl;

		for (int t = scale; t <= max*scale; t += scale) {
		//for (int t = 0; t <= scale; t ++) {
			//int digraph_rbf = digraph->rbf2(t,ret2);
			int digraph_rbf = avrTask.getRequestBoundFunction(ret2,t);
			int avrtask_rbf = avrTask.getRequestBoundFunction(ret,t);
			cout << RPmSEC_to_RPM(*iter) << "," << t << "=>" << digraph_rbf << "\t" << avrtask_rbf << endl;

			if (digraph_rbf != avrtask_rbf) {
				cerr << "Error calculating rbf with initial speed!" << endl;
				Utility::output_one_map(cout,"Digraph",ret2);
				Utility::output_one_map(cout,"AVR",ret);
				exit(EXIT_FAILURE);
			}
		}
	}
#endif
	
	

#if 0
	vector<int> vec_rpm;
	vector<double> vec_rbf0;
	vector<double> vec_trbf0;

	for (int rpm = 1500; rpm <=6500; rpm+=100) {
		timer.start();
		map<int,int> ret = avrTask.calRequestBoundFunction(RPM_to_RPmSEC(rpm),max*scale);
		timer.end();
		double t0 = timer.getTime();

		int rbf0 = avrTask.getRequestBoundFunction(ret,max*scale);

		avrTask.clearAll();

		//cout << rpm << "=>" << rbf0 << "\t" << t0 << endl;
		//avrTask.outputRequestFunctions(cout,RPM_to_RPmSEC(rpm),max);

		vec_rpm.push_back(rpm);
		vec_rbf0.push_back(rbf0);
		vec_trbf0.push_back(t0);
	}

	Utility::output_one_vector(cout,"rpm",vec_rpm);
	Utility::output_one_vector(cout,"rbf0",vec_rbf0);
	Utility::output_one_vector(cout,"trbf0",vec_trbf0);
#endif

#if 0
	#ifdef __USING_ILPCPLEX__
	vector<int> vec_ILPCON;
	vector<double> vec_tILPCON;

	for (int rpm = 1500; rpm <=6500; rpm+=100) {
	//for (int rpm = 1600; rpm <=1600; rpm+=100) {
		timer.start();
		int ILPCON = avrTask.calILPCON(RPM_to_RPmSEC(rpm),max*scale);
		timer.end();
		double tILPCON = timer.getTime();

		//cout << rpm << "=>" << ILPCON << "\t" << tILPCON << endl;
		vec_ILPCON.push_back(ILPCON);
		vec_tILPCON.push_back(tILPCON);
	}

	Utility::output_one_vector(cout,"ILPCON",vec_ILPCON);
	Utility::output_one_vector(cout,"tILPCON",vec_tILPCON);
	#endif
#endif

#if 0
	#ifdef __USING_ILPCPLEX__
	timer.start();
	int rILP = avrTask.calILP(max*scale);
	timer.end();

	vector<int> vec_ILP;
	vector<double> vec_tILP;

	for (int rpm = 1500; rpm <=6500; rpm+=100) {
		vec_ILP.push_back(rILP);
		vec_tILP.push_back(timer.getTime());
	}

	Utility::output_one_vector(cout,"ILP",vec_ILP);
	Utility::output_one_vector(cout,"tILP",vec_tILP);
	#endif
#endif

#if 0
	#ifdef __USING_ILPCPLEX__
	// test
	for (int i=0; i<vec_ILP.size(); i++) {
		int rbfi = vec_rbf0[i];
		int ILPCONi = vec_ILPCON[i];

		if (rbfi > ILPCONi) {
			cerr << "Error: rpm = " << vec_rpm[i] << ", rbf = " << rbfi << ", ILPCON = " << ILPCONi << endl;
			exit(EXIT_FAILURE);
		}
	}
	#endif
#endif
}

void RunTest::testRBFPeriodicity(int max, int scale) {
	Timer timer;

	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.621 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	//static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	//static const int _wcets[] = {965,576,424,343,277,246};

	static const double _speeds[] = {5500,6500};
	static const int _wcets[] = {277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	int modeNum = speeds.size();

	AVRTask avrTask(engine,period,speeds,wcets);
	avrTask.buildDigraph();

	avrTask.prepareLinearUppperBound();

	Digraph* digraph = avrTask.digraph;

	cout << "#node = " << digraph->node_vec.size() << ", #edge = " << digraph->edge_vec.size() << endl;
	//digraph->write_graphviz(cout);
	// Pre-processing
	/*
	digraph->generate_strongly_connected_components();
	digraph->check_strongly_connected();
	
	if (!digraph->strongly_connected) {
		cerr << "Not strongly connected!" << endl;
		exit(EXIT_FAILURE);
	}

	digraph->calculate_linear_factor();

	digraph->linear_factor = 
	*/

	cout << avrTask._maxU << endl;
	Utility::output_one_map(cout,"modePeriods",avrTask.modePeriods);
	Utility::output_one_map(cout,"modeUtilizations",avrTask.modeUtils);

	//digraph->unit_digraph = new UnitDigraph(digraph);
	//digraph->unit_digraph->prepare3(false);

	map<double, map<int,int>> rbfs0;
	map<double, map<int,int>> rbfs1;

	timer.start();
	digraph->calculate_rbf_with_periodicity_DP(max*scale,avrTask.modeUtils[modeNum-1], avrTask.modePeriods[modeNum-1], rbfs0);
	timer.end();
	double t0 = timer.getTime();

	timer.start();
	digraph->calculate_rbf_without_periodicity_DP(max*scale, rbfs1);
	timer.end();
	double t1 = timer.getTime();

	Utility::output_map_map(cout,"rbfs0",rbfs0);
	Utility::output_map_map(cout,"rbfs1",rbfs1);

	cout << t0 << "\t" << t1 << endl;
}

void RunTest::testLUB(int max, int scale) {
	Timer timer;

	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.621 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	//static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	//static const int _wcets[] = {965,576,424,343,277,246};

	static const double _speeds[] = {5500,6500};
	static const int _wcets[] = {277,246};

	//static const double _speeds[] = {1500,6500};
	//static const int _wcets[] = {965,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	int modeNum = speeds.size();

	AVRTask avrTask(engine,period,speeds,wcets);
	avrTask.buildDigraph();

	avrTask.prepareLinearUppperBound();

	Digraph* digraph = avrTask.digraph;

	cout << "#node = " << digraph->node_vec.size() << ", #edge = " << digraph->edge_vec.size() << endl;
	//digraph->write_graphviz(cout);

	cout << avrTask._maxU << "\t" << (1.0-avrTask._maxU) * avrTask._maxC << endl;
	Utility::output_one_map(cout,"modePeriods",avrTask.modePeriods);
	Utility::output_one_map(cout,"modeUtilizations",avrTask.modeUtils);

	//digraph->unit_digraph = new UnitDigraph(digraph);
	//digraph->unit_digraph->prepare3(false);

	map<int,int> rbfs0;

	timer.start();
	//digraph->calculate_rbf_without_periodicity_DP(max*scale, rbfs0);
	timer.end();
	double t0 = timer.getTime();

	Utility::output_one_map(cout,"rbf0",rbfs0);

	cout << "Generating sccs ... " << endl;
	timer.start();
	digraph->generate_strongly_connected_components();
	digraph->check_strongly_connected();

	if (!digraph->strongly_connected) {
		cerr << "Not strongly connected!" << endl;
		exit(EXIT_FAILURE);
	}
	timer.end();
	cout << "tGSCCS = " << timer.getTime() << endl;

	cout << "End! @Generating sccs." << endl;

	cout << "Calculating linear factor ... " << endl;
	timer.start();
	//digraph->calculate_linear_factor();
	digraph->linear_factor = avrTask._maxU;
	timer.end();

	cout << "tCallfac = " << timer.getTime() << endl;
	cout << "End! @Calculating linear factor." << endl;


	cout << "Calculating linear upper bounds ... " << endl;
	timer.start();
	digraph->calculate_linear_upper_bounds();
	timer.end();
	cout << "tCallubs = " << timer.getTime() << endl;
	cout << "End! @Calculating linear upper bounds ... " << endl;

	double t1 = timer.getTime();

	cout << "\t lfac=" << digraph->linear_factor << "\tCibf = " << digraph->c_ibf << endl;

#ifdef __USING_ILPCPLEX__
	map<int,int> rbfs1;
	for (int i=0; i<=max*scale; i+=scale) {
	//for (int i=scale; i<=1*scale; i+=scale) {
		int ret = testCplexILP(avrTask.wcets[0], avrTask.modePeriods[0], avrTask.wcets[1], avrTask.modePeriods[1] ,digraph->linear_factor * i + digraph->c_ibf,i);
		rbfs1[i] = ret;
	}

	Utility::output_one_map(cout,"rbf1",rbfs1);
#endif

	cout << t0 << "\t" << t1 << endl;
}

void RunTest::test(int rpm, int max, int step, int scale) {
	max *= scale;
	step *= scale;

	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	//static const int _wcets[] = {965,576,424,343,277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	//vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	vector<int> wcets;
	int s =8;
	wcets.push_back(s*150);
	wcets.push_back(s*278);
	wcets.push_back(s*344);
	wcets.push_back(s*425);
	wcets.push_back(s*576);
	wcets.push_back(s*966);
	sort(wcets.begin(),wcets.end(),greater<int>());

	AVRTask avrTask(engine,period,speeds,wcets);
	Utility::output_one_map(cout,"Periods", avrTask.modePeriods);
	Utility::output_one_map(cout,"Utils",avrTask.modeUtils);
	Utility::output_one_vector(cout,"wcets",wcets);
	
	Timer timer;
#if 0 // RBF
	vector<int> vec_time;
	vector<double> vec_rbf0;
	vector<double> vec_trbf0;

	for (int t=scale; t<=max; t+=step) {
		timer.start();
		map<int,int> temp = avrTask.calRequestBoundFunction(RPM_to_RPmSEC(rpm),t);
		timer.end();
		double trbf0 = timer.getTime();

		int rbf0 = avrTask.getRequestBoundFunction(temp,t);

		avrTask.clearAll();

		vec_time.push_back(t/scale);
		vec_rbf0.push_back(rbf0);
		vec_trbf0.push_back(trbf0);
	}

	Utility::output_one_vector(cout,"time",vec_time);
	Utility::output_one_vector(cout,"rbf0",vec_rbf0);
	Utility::output_one_vector(cout,"trbf0",vec_trbf0);
#endif

#if 0 // ILP
	#ifdef __USING_ILPCPLEX__
	vector<int> vec_ILP;
	vector<double> vec_tILP;

	for (int t = scale; t <= max; t+=step) {
		timer.start();
		int rILP = avrTask.calILP(t);
		timer.end();

		//cout << t << " " << rILP << " " << timer.getTime() << endl;
		vec_ILP.push_back(rILP);
		vec_tILP.push_back(timer.getTime());
	}

	Utility::output_one_vector(cout,"ILP",vec_ILP);
	Utility::output_one_vector(cout,"tILP",vec_tILP);
	#endif	
#endif

#if 0 // ILPCON
	#ifdef __USING_ILPCPLEX__
	vector<int> vec_ILPCON;
	vector<double> vec_tILPCON;

	for (int t = scale; t <= max; t+=step) {
		timer.start();
		int rILPCON = avrTask.calILPCON(RPM_to_RPmSEC(rpm),t);
		timer.end();

		//cout << t << " " << rILPCON << " " << timer.getTime() << endl;
		vec_ILPCON.push_back(rILPCON);
		vec_tILPCON.push_back(timer.getTime());
	}

	Utility::output_one_vector(cout,"ILPCON",vec_ILPCON);
	Utility::output_one_vector(cout,"tILPCON",vec_tILPCON);
	#endif
#endif

#if 0 // DIGRAPH_RBF
	vector<double> vec_digraph_rbf0;
	vector<double> vec_t_digraph_rbf0;

	avrTask.buildDigraph();
	Digraph* digraph = avrTask.digraph;

	for (int t=scale; t<=max; t+=step) {
		timer.start();
		map<int,int> temp; 
		digraph->calculate_rbf_without_periodicity_DP(RPM_to_RPmSEC(rpm),t,temp);
		timer.end();
		double t_digraph_rbf0 = timer.getTime();

		int digraph_rbf0 = avrTask.getRequestBoundFunction(temp,t);

		avrTask.clearAll();

		vec_digraph_rbf0.push_back(digraph_rbf0);
		vec_t_digraph_rbf0.push_back(t_digraph_rbf0);
	}

	Utility::output_one_vector(cout,"digraph_rbf0",vec_digraph_rbf0);
	Utility::output_one_vector(cout,"t_digraph_rbf0",vec_t_digraph_rbf0);
#endif
	
#if 0
	#ifdef __USING_ILPCPLEX__
	map<int,int> ret = avrTask.calRequestBoundFunction(RPM_to_RPmSEC(rpm),max);
	Utility::output_one_map(cout,"rbf"+Utility::int_to_string(max/scale), ret);

	// test
	for (int i=0; i<vec_ILP.size(); i++) {
		int rbfi = vec_rbf0[i];
		int ILPi = vec_ILP[i];
		int ILPCONi = vec_ILPCON[i];

		if (rbfi > ILPCONi) {
			cerr << "Error: time = " << vec_time[i] << ", rbf = " << rbfi << ", ILPCON = " << ILPCONi << endl;
			exit(EXIT_FAILURE);
		}
	}
	#endif
#endif
}

void RunTest::test(int rpm, int tMin, int tMax, int tStep, int scale)
{
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.621 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets[] = {965,576,424,343,277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	AVRTask avrTask(engine,period,speeds,wcets);
	//Utility::output_one_map(cout,"Periods", avrTask.modePeriods);

	Timer timer;
#if 1 // RBF
	vector<int> vec_time;
	vector<double> vec_rbf0;
	vector<double> vec_trbf0;

	for (int t=tMin*scale; t<=tMax*scale; t+=tStep*scale) {
		timer.start();
		map<int,int> temp = avrTask.calRequestBoundFunction(RPM_to_RPmSEC(rpm),t);
		timer.end();
		double trbf0 = timer.getTime();

		int rbf0 = avrTask.getRequestBoundFunction(temp,t);

		avrTask.clearAll();

		vec_time.push_back(t/scale);
		vec_rbf0.push_back(rbf0);
		vec_trbf0.push_back(trbf0);
	}

	Utility::output_one_vector(cout,"time",vec_time);
	Utility::output_one_vector(cout,"rbf0",vec_rbf0);
	Utility::output_one_vector(cout,"trbf0",vec_trbf0);
#endif

#if 1 // ILP
	#ifdef __USING_ILPCPLEX__
	vector<int> vec_ILP;
	vector<double> vec_tILP;

	for (int t=tMin*scale; t<=tMax*scale; t+=tStep*scale) {
		timer.start();
		int rILP = avrTask.calILP(t);
		timer.end();

		//cout << t << " " << rILP << " " << timer.getTime() << endl;
		vec_ILP.push_back(rILP);
		vec_tILP.push_back(timer.getTime());
	}

	Utility::output_one_vector(cout,"ILP",vec_ILP);
	Utility::output_one_vector(cout,"tILP",vec_tILP);
	#endif
#endif

#if 1 // ILPCON
	#ifdef __USING_ILPCPLEX__
	vector<int> vec_ILPCON;
	vector<double> vec_tILPCON;

	for (int t=tMin*scale; t<=tMax*scale; t+=tStep*scale) {
		timer.start();
		int rILPCON = avrTask.calILPCON(RPM_to_RPmSEC(rpm),t);
		timer.end();

		//cout << t << " " << rILPCON << " " << timer.getTime() << endl;
		vec_ILPCON.push_back(rILPCON);
		vec_tILPCON.push_back(timer.getTime());
	}

	Utility::output_one_vector(cout,"ILPCON",vec_ILPCON);
	Utility::output_one_vector(cout,"tILPCON",vec_tILPCON);
	#endif
#endif

#if 1 // DIGRAPH_RBF
	vector<double> vec_digraph_rbf0;
	vector<double> vec_t_digraph_rbf0;

	avrTask.buildDigraph();
	Digraph* digraph = avrTask.digraph;

	for (int t=tMin*scale; t<=tMax*scale; t+=tStep*scale) {
		timer.start();
		map<int,int> temp; 
		digraph->calculate_rbf_without_periodicity_DP(RPM_to_RPmSEC(rpm),t,temp);
		timer.end();
		double t_digraph_rbf0 = timer.getTime();

		int digraph_rbf0 = avrTask.getRequestBoundFunction(temp,t);

		avrTask.clearAll();

		vec_digraph_rbf0.push_back(digraph_rbf0);
		vec_t_digraph_rbf0.push_back(t_digraph_rbf0);
	}

	Utility::output_one_vector(cout,"digraph_rbf0",vec_digraph_rbf0);
	Utility::output_one_vector(cout,"t_digraph_rbf0",vec_t_digraph_rbf0);
#endif
}

void RunTest::test(string inputDir,string dcName)
{
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	static const double _UBSpeeds[] = {1214,1871,2996,3629,4285,6500};
	static const double _BBSpeeds[] = {1044,1858,2778,3556,4274,6500};
	static const double _BSSpeeds[] = {1050,1868,2887,3194,4282,6500};

	vector<double> UBSpeeds(_UBSpeeds,_UBSpeeds+sizeof(_UBSpeeds)/sizeof(_UBSpeeds[0]));
	vector<double> BBSpeeds(_BBSpeeds,_BBSpeeds+sizeof(_BBSpeeds)/sizeof(_BBSpeeds[0]));
	vector<double> BSSpeeds(_BSSpeeds,_BSSpeeds+sizeof(_BSSpeeds)/sizeof(_BSSpeeds[0]));

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	double pDCUB = OptimizationAlgorithm::getPerformanceDC(dc,UBSpeeds,k);
	double pDCBB = OptimizationAlgorithm::getPerformanceDC(dc,BBSpeeds,k);
	double pDCBS = OptimizationAlgorithm::getPerformanceDC(dc,BSSpeeds,k);

	cout << "pDCBB/pDCUB = " << pDCBB/pDCUB * 100 << "%" << endl;
	cout << "pDCBS/pDCUB = " << pDCBS/pDCUB * 100 << "%" << endl;
}

#ifdef __USING_ILPCPLEX__
void RunTest::testCplexILP()
{
	IloEnv env;
	stringstream logfile;

	try {
		IloModel model(env);
		IloNumVarArray vars(env);
		IloRangeArray con(env);
		IloCplex cplex(model);

		// Set options of IP solver
		cplex.setParam(IloCplex::TiLim, 100.000);
		cplex.setParam(IloCplex::Threads, 1);
		cplex.setParam(IloCplex::EpGap, 0.0);
		cplex.setParam(IloCplex::EpAGap, 0.0);
		cplex.setOut(logfile);

		// Variables
		vars.add(IloNumVar(env, 0.0, 40.0));
		vars.add(IloNumVar(env));
		vars.add(IloNumVar(env));
		model.add(IloMaximize(env, vars[0] + 2 * vars[1] + 3 * vars[2]));
		model.add( - vars[0] +     vars[1] + vars[2] <= 20);
		model.add(   vars[0] - 3 * vars[1] + vars[2] <= 30);

		// Solve
		cplex.solve();
		if (cplex.getStatus() == IloAlgorithm::Optimal) {
			IloNumArray vals(env);
			cplex.getValues(vals, vars);
			env.out() << "Values = " << vals << endl;
			cout << "Max=" << cplex.getObjValue() << endl;
		}
	} catch (IloException& e) {
		cerr << "C-Exp: " << e << endl;
	} catch (...) {
		cerr << "Unknown Exception" << endl;
	}

	const string str = logfile.str();
	cout << str << endl;
	env.end();
}

int RunTest::testCplexILP(int wcet0, int period0, int wcet1, int period1, int con0, int con1)
{
	int ret = -1;

	IloEnv env;
	stringstream logfile;

	try {
		IloModel model(env);
		IloNumVarArray vars(env);
		IloCplex cplex(model);

		// Set options of IP solver
		cplex.setParam(IloCplex::TiLim, 100.000);
		cplex.setParam(IloCplex::Threads, 1);
		cplex.setParam(IloCplex::EpGap, 0.0);
		cplex.setParam(IloCplex::EpAGap, 0.0);
		cplex.setOut(logfile);

		// Variables
		vars.add(IloNumVar(env, 0.0, IloInfinity,ILOINT));
		vars.add(IloNumVar(env, 0.0, IloInfinity,ILOINT));
		model.add(IloMaximize(env, wcet0 * vars[0] + wcet1 * vars[1]));
		model.add( wcet0 * vars[0] + wcet1 * vars[1] <= con0);
		model.add( period0 * vars[0] + period1 * vars[1] <= con1+period0);

		// Solve
		cplex.solve();

#if 0
		string lpname = "Models"+Utility::linkNotation()+"model.lp";
		cplex.exportModel(lpname.c_str());
#endif

		if (cplex.getStatus() == IloAlgorithm::Optimal) {
			IloNumArray vals(env);
			ret = cplex.getObjValue();
#if 0
			cplex.getValues(vals, vars);
			env.out() << "Values = " << vals << endl;
			cout << "Max=" << cplex.getObjValue() << endl;
#endif
		}
	} catch (IloException& e) {
		cerr << "C-Exp: " << e << endl;
	} catch (...) {
		cerr << "Unknown Exception" << endl;
	}

#if 0
	const string str = logfile.str();
	cout << str << endl;
#endif

	env.end();
	return ret;
}

void RunTest::testVRBILPExample()
{
	IloEnv env;
	stringstream logfile;

	// Task Setting
	int Cax = 20;
	int Tax = 90;
	int Dax = 45;
	int Cay = 50;
	int Tay = 200;
	int Day = 100;

	int Cb = 270;
	int Tb = 500;
	int Db = 400;

	int w = 270;
	int iter = 0;

	while (true) {

		try {
			IloModel model(env);
			IloNumVarArray vars(env);
			IloRangeArray con(env);
			IloCplex cplex(model);

			// Set options of IP solver
			cplex.setParam(IloCplex::TiLim, 100.000);
			cplex.setParam(IloCplex::Threads, 1);
			cplex.setParam(IloCplex::EpGap, 0.0);
			cplex.setParam(IloCplex::EpAGap, 0.0);
			cplex.setOut(logfile);

			// Variables
			vars.add(IloNumVar(env ,0.0, IloInfinity, ILOINT));
			vars.add(IloNumVar(env ,0.0, IloInfinity, ILOINT));
			model.add(IloMaximize(env, Cax * vars[0] + Cay * vars[1]));
			model.add( Tax * vars[0] + Tay * vars[1] <= w + Tay -1);

			// Solve
			Timer timer;
			timer.start();
			cplex.solve();
			timer.end();

			cout << "time:" <<timer.getTime() << endl;
			if (cplex.getStatus() == IloAlgorithm::Optimal) {
				IloNumArray vals(env);
				cplex.getValues(vals, vars);
				env.out() << "Values = " << vals << endl;
				cout << "Max=" << cplex.getObjValue() << ", Time = " << cplex.getTime() << endl;

				int w_n = Cb + cplex.getObjValue();
				if (w_n == w) {
					cout << "Iteration " << iter << ": w^q = " << w << ", kx = " << vals[0] << ", ky = " << vals[1] << ", w^q+1 = " << w_n << endl;
					break;
				}
				else {
					cout << "Iteration " << iter << ": w^q = " << w << ", kx = " << vals[0] << ", ky = " << vals[1] << ", w^q+1 = " << w_n << endl;
					w = w_n;
					iter++;
				}
			}
			cplex.printTime();
		} catch (IloException& e) {
			cerr << "C-Exp: " << e << endl;
		} catch (...) {
			cerr << "Unknown Exception" << endl;
		}
	}

	const string str = logfile.str();
	cout << str << endl;
	env.end();
}
#endif

void RunTest::testInitialDominantSpeeds()
{
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.620 * 0.0001;
	//double alpha = 1.667 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets[] = {965,576,424,343,277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

#if 0

	AVRTask avrTask(engine,period,speeds,wcets);
	vector<double> dominants = avrTask.getAllDominants();
	cout << dominants.size() << endl;

	for (auto e : dominants) {
		cout << RPmSEC_to_RPM(e) << endl;
	}

	cout << "=============================" << endl;
	
	vector<double> ranges = avrTask.getAllSpeedRanges();
	cout << ranges.size() << endl;

	for (auto e : ranges) {
		cout << RPmSEC_to_RPM(e) << endl;
	}
#endif

#if 0
	vector<int> intervals;
	vector<int> nums;

	for (int interval = 1; interval <=1000; interval++) {
		vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN,engine.SPEED_MAX,interval*1000);
		cout << interval << "=>" << dominants.size() << endl;
		intervals.push_back(interval);
		nums.push_back(dominants.size());
	}
	Utility::output_one_vector(cout,"intervals",intervals);
	Utility::output_one_vector(cout,"nums",nums);
#endif
	
#if 0
	vector<double> dominants = avrTask.getDominants(engine.SPEED_MIN,engine.SPEED_MAX);
	vector<double> allDominants = avrTask.getAllDominants();
	cout << dominants.size() << "\t" << allDominants.size() << endl;
	
	for (auto e : allDominants) 
		cout << RPmSEC_to_RPM(e) << endl;
#endif
	
}


void RunTest::testMinMaxTime()
{
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.620 * 0.0001;
	//double alpha = 1.667 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets[] = {965,576,424,343,277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	static const double _intervals3[] = {500,2500,4500,6500};
	vector<double> intervals3(_intervals3,_intervals3+sizeof(_intervals3)/sizeof(_intervals3[0]));

	static const double _intervals6[] = {500,1500,2500,3500,4500,5500,6500};
	vector<double> intervals6(_intervals6,_intervals6+sizeof(_intervals6)/sizeof(_intervals6[0]));

	// Calculate ODRT with three vertices
	for (int i=0; i<3; i++) {
		double low0 = intervals3[i];
		double high0 = intervals3[i+1];

		for (int j=0; j<3; j++) {
			double low1 = intervals3[j];
			double high1 = intervals3[j+1];

			double minTime = engine.getMinTimeInterReleaseTime(RPM_to_RPmSEC(low0),RPM_to_RPmSEC(high0),RPM_to_RPmSEC(low1),RPM_to_RPmSEC(high1),period);
			double maxTime = engine.getMaxTimeInterReleaseTime(RPM_to_RPmSEC(low0),RPM_to_RPmSEC(high0),RPM_to_RPmSEC(low1),RPM_to_RPmSEC(high1),period);
		
			cout << "[" << low0 << "," << high0 << ") => " << "[" << low1 << "," << high1 << ")" << endl;
			cout << "minTime = " << minTime << ", maxTime = " << maxTime << endl;
		}
	}

	cout << "============================" << endl;

	// Calculate ODRT with six vertices
	for (int i=0; i<6; i++) {
		double low0 = intervals6[i];
		double high0 = intervals6[i+1];

		for (int j=0; j<6; j++) {
			double low1 = intervals6[j];
			double high1 = intervals6[j+1];

			double minTime = engine.getMinTimeInterReleaseTime(RPM_to_RPmSEC(low0),RPM_to_RPmSEC(high0),RPM_to_RPmSEC(low1),RPM_to_RPmSEC(high1),period);
			double maxTime = engine.getMaxTimeInterReleaseTime(RPM_to_RPmSEC(low0),RPM_to_RPmSEC(high0),RPM_to_RPmSEC(low1),RPM_to_RPmSEC(high1),period);

			cout << "[" << low0 << "," << high0 << ") => " << "[" << low1 << "," << high1 << ")" << endl;
			cout << "minTime = " << minTime << ", maxTime = " << maxTime << endl;
		}
	}

	cout << engine.getMaxInterArrivalTime(RPM_to_RPmSEC(4500),period) << endl;
	cout << engine.getConstantPeriodWithArbitraryAcceleration(RPM_to_RPmSEC(4500),period) << endl;
}


void RunTest::testJobSequences()
{
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets[] = {965,576,424,343,277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	AVRTask avrTask(engine,period,speeds,wcets);

	vector<int> intervals;
	vector<int> nums;
	vector<int> nums_NO1;
	vector<int> nums_NO2;
	vector<int> nums_NO3;

	for (int interval = 1; interval <=100; interval++) {
		int totalNum = avrTask.getJobSequencesNumber(interval*1000);
		int totalNum_NO1 = avrTask.getJobSequencesNumberNO1(interval*1000);
		int totalNum_NO2 = avrTask.getJobSequencesNumberNO2(interval*1000);
		int totalNum_NO3 = avrTask.getJobSequencesNumberNO3(interval*1000);

		cout << interval << "=>" << totalNum << "\t" << totalNum_NO1 << "\t" << totalNum_NO2 <<"\t" << totalNum_NO3  << endl;

		intervals.push_back(interval);
		nums.push_back(totalNum);
		nums_NO1.push_back(totalNum_NO1);
		nums_NO2.push_back(totalNum_NO2);
		nums_NO3.push_back(totalNum_NO3);
	}
	Utility::output_one_vector(cout,"intervals",intervals);
	Utility::output_one_vector(cout,"nums",nums);
	Utility::output_one_vector(cout,"nums2",nums_NO1);
	Utility::output_one_vector(cout,"nums3",nums_NO2);
	Utility::output_one_vector(cout,"nums4",nums_NO3);
}

void RunTest::seekMotivatedExample(string dcFile)
{
	static const double _k1[] = {1.0,1.0,1.0,1.0,1.0,1.0};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	static const double _k2[] = {0,200,500,1000,1500,2000};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	generateBiondiTestedSystems("MotivatedExamples",2000,5,75,6);
}

void RunTest::testSpeedIntervals()
{
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	//double alpha = 0.001/6; // ECRTS2017 DIGRAPH
	//cout << alpha << endl;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	static const double _speeds1[] = {2500,4500,6500};
	static const int _wcets1[] = {600,400,150};

	vector<double> speeds1(_speeds1,_speeds1+sizeof(_speeds1)/sizeof(_speeds1[0]));
	vector<int> wcets1(_wcets1,_wcets1+sizeof(_wcets1)/sizeof(_wcets1[0]));

	AVRTask avrTask1(engine,period,speeds1,wcets1);
	vector<double> speedSet1 = avrTask1.getAllSpeedRanges();

	cout << "SpeedSet1 size = " << speedSet1.size() << endl;

	cout << "x1 = [";
	for (int i=0; i<speedSet1.size(); i++) {
		cout << "1";
		if (i<speedSet1.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds1 = [";
	for (int i=0; i<speedSet1.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet1[i]);
		if (i<speedSet1.size()-1) cout << ",";
	}
	cout << "]" << endl;

	static const double _speeds2[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets2[] = {965,576,424,343,277,246};

	vector<double> speeds2(_speeds2,_speeds2+sizeof(_speeds2)/sizeof(_speeds2[0]));
	vector<int> wcets2(_wcets2,_wcets2+sizeof(_wcets2)/sizeof(_wcets2[0]));

	AVRTask avrTask2(engine,period,speeds2,wcets2);
	vector<double> speedSet2 = avrTask2.getAllSpeedRanges();

	cout << "SpeedSet2 size = " << speedSet2.size() << endl;

	cout << "x2 = [";
	for (int i=0; i<speedSet2.size(); i++) {
		cout << "2";
		if (i<speedSet2.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds2 = [";
	for (int i=0; i<speedSet2.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet2[i]);
		if (i<speedSet2.size()-1) cout << ",";
	}
	cout << "]" << endl;

	static const double _speeds3[] = {2000,3500,5000,6500};
	static const int _wcets3[] = {600,450,300,150};

	vector<double> speeds3(_speeds3,_speeds3+sizeof(_speeds3)/sizeof(_speeds3[0]));
	vector<int> wcets3(_wcets3,_wcets3+sizeof(_wcets3)/sizeof(_wcets3[0]));

	AVRTask avrTask3(engine,period,speeds3,wcets3);
	vector<double> speedSet3 = avrTask3.getAllSpeedRanges();

	cout << "SpeedSet3 size = " << speedSet3.size() << endl;

	cout << "x3 = [";
	for (int i=0; i<speedSet3.size(); i++) {
		cout << "3";
		if (i<speedSet3.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds3 = [";
	for (int i=0; i<speedSet3.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet3[i]);
		if (i<speedSet3.size()-1) cout << ",";
	}
	cout << "]" << endl;

	vector<double> speedSet4 = Utility::merge(speedSet1,speedSet2);
	speedSet4 = Utility::merge(speedSet3,speedSet4);

	cout << "SpeedSet4 size = " << speedSet4.size() << endl;

	cout << "x4 = [";
	for (int i=0; i<speedSet4.size(); i++) {
		cout << "4";
		if (i<speedSet4.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds4 = [";
	for (int i=0; i<speedSet4.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet4[i]);
		if (i<speedSet4.size()-1) cout << ",";
	}
	cout << "]" << endl;

	static const double _speeds5[] = {1500,2000,2500,3500,4500,5000,5500,6500};
	static const int _wcets5[] = {965,600,576,450,424,343,277,246};

	vector<double> speeds5(_speeds5,_speeds5+sizeof(_speeds5)/sizeof(_speeds5[0]));
	vector<int> wcets5(_wcets5,_wcets5+sizeof(_wcets5)/sizeof(_wcets5[0]));

	AVRTask avrTask5(engine,period,speeds5,wcets5);
	vector<double> speedSet5 = avrTask5.getAllSpeedRanges();

	cout << "SpeedSet5 size = " << speedSet5.size() << endl;

	cout << "x5 = [";
	for (int i=0; i<speedSet5.size(); i++) {
		cout << "5";
		if (i<speedSet5.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds5 = [";
	for (int i=0; i<speedSet5.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet5[i]);
		if (i<speedSet5.size()-1) cout << ",";
	}
	cout << "]" << endl;

	avrTask5.buildSimpleODRT();
	//avrTask5.digraph->write_graphviz_ODRT(cout);
	avrTask5.digraph->write_tikz_ODRT(cout);

	static const double _speeds6[] = {1500};
	static const int _wcets6[] = {965};

	vector<double> speeds6(_speeds6,_speeds6+sizeof(_speeds6)/sizeof(_speeds6[0]));
	vector<int> wcets6(_wcets6,_wcets6+sizeof(_wcets6)/sizeof(_wcets6[0]));

	Engine engine2(rpm_min,1500,alpha,alpha);
	
	AVRTask avrTask6(engine2,period,speeds6,wcets6);
	vector<double> speedSet6 = avrTask6.getAllSpeedRanges();

	cout << "SpeedSet6 size = " << speedSet6.size() << endl;

	cout << "x6 = [";
	for (int i=0; i<speedSet6.size(); i++) {
		cout << "6";
		if (i<speedSet6.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds6 = [";
	for (int i=0; i<speedSet6.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet6[i]);
		if (i<speedSet6.size()-1) cout << ",";
	}
	cout << "]" << endl;

	avrTask6.buildComplexODRT();
	//avrTask5.digraph->write_graphviz_ODRT(cout);
	avrTask6.digraph->write_tikz_ODRT(cout);

	/*
	cout << engine2.getMaxTimeInterReleaseTime(RPM_to_RPmSEC(500),RPM_to_RPmSEC(1041),RPM_to_RPmSEC(1041),RPM_to_RPmSEC(1190),period) << endl;
	cout << engine2.getMinInterArrivalTimeWithArbitraryAcceleration(RPM_to_RPmSEC(500),RPM_to_RPmSEC(1041),period) << endl;
	cout << engine2.getMaxInterArrivalTimeWithArbitraryAcceleration(RPM_to_RPmSEC(500),RPM_to_RPmSEC(1041),period) << endl;
	cout << engine2.getInterArrivalTimeWithConstantAcceleration(RPM_to_RPmSEC(500),RPM_to_RPmSEC(1041),period) << endl;
	*/

	static const double _speeds7[] = {505,1500};
	static const int _wcets7[] = {1200,965};

	vector<double> speeds7(_speeds7,_speeds7+sizeof(_speeds7)/sizeof(_speeds7[0]));
	vector<int> wcets7(_wcets7,_wcets7+sizeof(_wcets7)/sizeof(_wcets7[0]));

	//Engine engine2(rpm_min,1500,alpha,alpha);

	AVRTask avrTask7(engine2,period,speeds7,wcets7);
	vector<double> speedSet7 = avrTask7.getAllSpeedRanges();

	cout << "SpeedSet7 size = " << speedSet7.size() << endl;

	cout << "x7 = [";
	for (int i=0; i<speedSet7.size(); i++) {
		cout << "7";
		if (i<speedSet7.size()-1) cout << ",";
	}
	cout << "]" << endl;

	cout << "speeds7 = [";
	for (int i=0; i<speedSet7.size(); i++) {
		cout << RPmSEC_to_RPM(speedSet7[i]);
		if (i<speedSet7.size()-1) cout << ",";
	}
	cout << "]" << endl;

	avrTask7.buildComplexODRT();
	avrTask7.digraph->write_tikz_ODRT(cout);
}

void RunTest::testAsynchronousPeriodicTasks()
{
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.667 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	static const double _speeds[] = {1500,2500,3500,4500,5500,6500};
	static const int _wcets[] = {965,576,424,343,277,246};

	vector<double> speeds(_speeds,_speeds+sizeof(_speeds)/sizeof(_speeds[0]));
	vector<int> wcets(_wcets,_wcets+sizeof(_wcets)/sizeof(_wcets[0]));

	AVRTask avrTask(engine,period,speeds,wcets);

	/// Asynchronous Periodic Tasks
#if 1
	AsynchronousPeriodicTask atask0(2000,2000,10000,17000);
	AsynchronousPeriodicTask atask1(1000,2000,15000,0);
	AsynchronousPeriodicTask atask2(5000,10000,22000,1000);
	AsynchronousPeriodicTask atask3(5000,20000,33000,6000);
	AsynchronousPeriodicTask atask4(5000,42000,42000,1000);
	AsynchronousPeriodicTask atask5(7000,47000,57000,19000);
	AsynchronousPeriodicTask atask6(2000,90000,90000,34000);
	AsynchronousPeriodicTask atask7(3000,120000,120000,36000);
	AsynchronousPeriodicTask atask8(17000,340000,345000,0);
	AsynchronousPeriodicTask atask9(2000,700000,700000,0);
#else
	AsynchronousPeriodicTask atask0(2,2,10,17);
	AsynchronousPeriodicTask atask1(1,2,15,0);
	AsynchronousPeriodicTask atask2(5,10,22,1);
	AsynchronousPeriodicTask atask3(5,20,33,6);
	AsynchronousPeriodicTask atask4(5,42,42,1);
	AsynchronousPeriodicTask atask5(7,47,57,19);
	AsynchronousPeriodicTask atask6(2,90,90,34);
	AsynchronousPeriodicTask atask7(3,120,120,36);
	AsynchronousPeriodicTask atask8(17,340,345,0);
	AsynchronousPeriodicTask atask9(2,700,700,0);
#endif

	vector<AsynchronousPeriodicTask> atasks;
	atasks.push_back(atask0);
	atasks.push_back(atask1);
	atasks.push_back(atask2);
	/*
	atasks.push_back(atask3);
	atasks.push_back(atask4);
	atasks.push_back(atask5);
	atasks.push_back(atask6);
	atasks.push_back(atask7);
	atasks.push_back(atask8);
	atasks.push_back(atask9);
	*/

	Timer timer;
	timer.start();
	bool sched0 = SchedAnalysis::asynchronous_exact_analysis(atasks,avrTask,11);
	timer.end();

	Utility::output_one_map(cout,"rts",SchedAnalysis::rts);

	vector<int> asynchronousCriticalInstants = SchedAnalysis::computing_asynchronous_critial_instants(atasks,2,38000,(39+330)*1000);
	Utility::output_one_vector(cout,"instants",asynchronousCriticalInstants);

	cout << asynchronousCriticalInstants.size() << endl;

	/// calculating the response times
	int wcet = 10000;
	for (int j=0; j<10; j++) {
		int rt = SchedAnalysis::calculate_response_time(atasks,2,wcet,asynchronousCriticalInstants[j]);
		cout << rt << endl;
	}

	/// general periodic tasks
	PeriodicTask gtask0(2000,2000,10000);
	PeriodicTask gtask1(1000,2000,15000);

	vector<PeriodicTask> gtasks;
	gtasks.push_back(gtask0);
	gtasks.push_back(gtask1);

	bool sched1 = SchedAnalysis::exact_analysis(gtasks,avrTask,3);

	cout << sched0 << "\t" << timer.getTime() << "\t" << sched1 << endl;
}

void RunTest::testAsynchronousPeriodicTasks2()
{
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s =8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	Timer timer;

	cout << "Computing UBs...";
	timer.start();
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
	cout << pUB << "\t" << timer.getTime() << endl;

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << pH1 << "\t" << timer.getTime() << endl;

	AsynchronousPeriodicTask atask0(task0);
	AsynchronousPeriodicTask atask1(task1);
	AsynchronousPeriodicTask atask2(task2);
	AsynchronousPeriodicTask atask3(task3);

	vector<AsynchronousPeriodicTask> atasks;
	atasks.push_back(atask0);
	atasks.push_back(atask1);
	atasks.push_back(atask2);
	atasks.push_back(atask3);
	
	cout << "Computing asynchronous UBs...";
	timer.start();
	vector<double> aMaxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,atasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"aMax Speeds", aMaxSpeeds);

	double apUB = OptimizationAlgorithm::getPerformance(aMaxSpeeds, k,engine);
	cout << apUB << "\t" << timer.getTime() << endl;

	cout << "Computing asynchronous H1...";
	timer.start();
	vector<double> aBSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, atasks, engine, period);
	timer.end();
	double atimeH1 = timer.getTime();
	cout << "done." << endl;
	double apH1 = OptimizationAlgorithm::getPerformance(aBSSpeeds, k, engine);
	Utility::output_one_vector(cout, "aBS Speeds", aBSSpeeds);

	cout << apH1 << "\t" << timer.getTime() << endl;
}

void RunTest::testAsynchronousPeriodicTasks3()
{
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s =8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	Timer timer;

	cout << "Computing UBs...";
	timer.start();
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
	cout << pUB << "\t" << timer.getTime() << endl;

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << pH1 << "\t" << timer.getTime() << endl;

	AsynchronousPeriodicTask atask0(1000,5000,0);
	AsynchronousPeriodicTask atask1(6500,20000,3000);
	AsynchronousPeriodicTask atask2(10000,50000,20000);
	AsynchronousPeriodicTask atask3(10000,100000,80000);

	vector<AsynchronousPeriodicTask> atasks;
	atasks.push_back(atask0);
	atasks.push_back(atask1);
	atasks.push_back(atask2);
	atasks.push_back(atask3);

	cout << "Computing asynchronous UBs...";
	timer.start();
	vector<double> aMaxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,atasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"aMax Speeds", aMaxSpeeds);

	double apUB = OptimizationAlgorithm::getPerformance(aMaxSpeeds, k,engine);
	cout << apUB << "\t" << timer.getTime() << endl;

	cout << "Computing asynchronous H1...";
	timer.start();
	vector<double> aBSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, atasks, engine, period);
	timer.end();
	double atimeH1 = timer.getTime();
	cout << "done." << endl;
	double apH1 = OptimizationAlgorithm::getPerformance(aBSSpeeds, k, engine);
	Utility::output_one_vector(cout, "aBS Speeds", aBSSpeeds);

	cout << apH1 << "\t" << timer.getTime() << endl;
}


void RunTest::testAsynchronousBiondiExample() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	vector<int> WCETs;
	double s =9;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);

	//Utility::output_one_vector(cout,"WCETs",WCETs);
	//exit(EXIT_FAILURE);

	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	Timer timer;

	for (int i=0; i<5; i++) {
		int offset2 = i*10000;
		for (int j=0; j<10; j++) {
			int offset3 = j*10000;
			cout << "+++++++++++++++++++++++++" << endl;
			cout << offset2 << "\t" << offset3 << endl;
			AsynchronousPeriodicTask aTask0(1000,5000,5000,0);
			AsynchronousPeriodicTask aTask1(6500,20000,20000,0);
			AsynchronousPeriodicTask aTask2(10000,50000,50000,offset2);
			AsynchronousPeriodicTask aTask3(5000,95000,100000,offset3);
			AsynchronousPeriodicTask aTask4(5000,100000,100000,0);

			vector<AsynchronousPeriodicTask> aTasks;
			aTasks.push_back(aTask0);
			aTasks.push_back(aTask1);
			aTasks.push_back(aTask2);
			aTasks.push_back(aTask3);
			aTasks.push_back(aTask4);

			cout << "Computing Asynchronous UBs...";
			timer.start();
			vector<double> aMaxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
			timer.end();
			cout << "done." << endl;
			Utility::output_one_vector(cout,"aMaxSpeeds", aMaxSpeeds);

			double aPUB = OptimizationAlgorithm::getPerformance(aMaxSpeeds, k,engine);
			cout << aPUB << "\t" << timer.getTime() << endl;

			cout << "Computing Asynchronous BS...";
			timer.start();
			vector<double> aBSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, aTasks, engine, period);
			timer.end();
			double aTimeBS = timer.getTime();
			cout << "done." << endl;
			OptimizationAlgorithm::enforceModeGuards(aBSSpeeds);
			double aPBS = OptimizationAlgorithm::getPerformance(aBSSpeeds, k, engine);
			Utility::output_one_vector(cout, "aBSSpeeds", aBSSpeeds);

			cout << "Computing Asynchronous B&B...";
			timer.start();
			//vector<double> aOPTSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, aTasks, engine, period);
			vector<double> aOPTSpeeds = aMaxSpeeds;
			timer.end();
			double aTimeOPT = timer.getTime();
			cout << "done." << endl;
			double aPOPT = OptimizationAlgorithm::getPerformance(aOPTSpeeds, k, engine);
			Utility::output_one_vector(cout, "aOPTSpeeds", aOPTSpeeds);

			cout << endl;
			cout << "aPUB = " << aPUB << endl;
			cout << "aPBS = " << aPBS << endl;
			cout << "aPOPT = " << aPOPT << endl;
			cout << endl;
			cout << "Ratio Asynchronous BS/UB = " << aPBS/aPUB*100.0 << "%" << endl;
			cout << "Ratio Asynchronous OPT/UB = " << aPOPT/aPUB*100.0 << "%" << endl;
			cout << "Ratio Asynchronous BS/OPT = " << aPBS/aPOPT*100.0 << "%" << endl;
			cout << endl;
			cout << "timeH1 = " << aTimeBS << endl;
			cout << "timeOPT = " << aTimeOPT << endl;

			// Calculate the performance for the driving cycles
			string INPUT_PREFIX = "Inputs5"+Utility::linkNotation();

			//static const string _dcName[] = {"UDC","NEDC", "ftp75", "Ja1015", "Highway", "IDC", "EUDC", "US_SC03", "Artemis_Road", "Artemis_Urban"};
			//static const string _dcName[] = {"UDC","NEDC"};
			static const string _dcName[] = {"NEDC", "ftp75","Highway"};
			vector<string> dcFiles(_dcName,_dcName+sizeof(_dcName)/sizeof(_dcName[0]));


			vector<list<double>> dcListVec;
			for (auto dcFile : dcFiles) {
				dcFile = INPUT_PREFIX + dcFile + ".info";
				const char *dcFilePointer = dcFile.c_str();
				list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
				dcListVec.push_back(dcList);
			}

			int index = 0;
			for (auto dcList : dcListVec) {
				double pUBDC =  OptimizationAlgorithm::getPerformanceDC(dcList,aMaxSpeeds, k);
				double pBSDC =  OptimizationAlgorithm::getPerformanceDC(dcList,aBSSpeeds, k);
				double pOPTDC = OptimizationAlgorithm::getPerformanceDC(dcList,aOPTSpeeds, k);
				list<int> dcResult;

#if 0
				timer.start();
				dcResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,aTasks,engine,period,maxSpeeds);
				timer.end();
#endif

				double pDAVRDC = OptimizationAlgorithm::getPerformanceDC(dcResult,k);
				double tDAVRDC = timer.getTime();

				cout << "DC - " << dcFiles[index++] << endl;
				cout << "Ratio DC H1/UB = " << pBSDC/pUBDC*100.0 << "%" << endl;
				cout << "Ratio DC OPT/UB = " << pOPTDC/pUBDC*100.0 << "%" << endl;
				cout << "Ratio DC H1/OPT = " << pBSDC/pOPTDC*100.0 << "%" << endl;
				cout << "Ratio DC DAVR/UB = " << pDAVRDC/pUBDC*100.0 << "%" << endl; 
				cout << "TimeDAVR = " << tDAVRDC << endl;
			}
		}
	}
}

void RunTest::testAsynchronousBiondiExample2() {
#define __ZERO_OFFESTS__ // set zero offsets
#define __LOCAL_DEBUG__ // omit BB and BS algorithms
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	vector<int> WCETs;
	double s =9;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);

	//Utility::output_one_vector(cout,"WCETs",WCETs);
	//exit(EXIT_FAILURE);

	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	Timer timer;

#ifndef __ZERO_OFFESTS__
	AsynchronousPeriodicTask aTask0(1000,5000,5000,0);
	AsynchronousPeriodicTask aTask1(6500,20000,20000,0);
	AsynchronousPeriodicTask aTask2(10000,50000,50000,40000);
	AsynchronousPeriodicTask aTask3(5000,95000,100000,50000);
	AsynchronousPeriodicTask aTask4(5000,100000,100000,0);
#else
	AsynchronousPeriodicTask aTask0(1000,5000,5000,0);
	AsynchronousPeriodicTask aTask1(6500,20000,20000,0);
	AsynchronousPeriodicTask aTask2(10000,50000,50000,0);
	AsynchronousPeriodicTask aTask3(10000,100000,100000,0);
#endif

	vector<AsynchronousPeriodicTask> aTasks;
	aTasks.push_back(aTask0);
	aTasks.push_back(aTask1);
	aTasks.push_back(aTask2);
	aTasks.push_back(aTask3);

#ifndef __ZERO_OFFESTS__
	aTasks.push_back(aTask4);
#endif

#ifdef __USING_STATIC_OFFSETS__
	cout << "Computing Asynchronous UBs...";
	timer.start();
	vector<double> MaxSOSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
	timer.end();
	double UBSOTime = timer.getTime();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"MaxSOSpeeds", MaxSOSpeeds);

	double UBSOPerf = OptimizationAlgorithm::getPerformance(MaxSOSpeeds, k,engine);
	cout << UBSOPerf << "\t" << timer.getTime() << endl;

	cout << "Computing Asynchronous BS...";
	timer.start();
#ifndef __LOCAL_DEBUG__
	vector<double> BSSOSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, aTasks, engine, period);
#else
	vector<double> BSSOSpeeds = MaxSOSpeeds;
#endif
	timer.end();
	double BSSOTime = timer.getTime();
	cout << "done." << endl;
	OptimizationAlgorithm::enforceModeGuards(BSSOSpeeds);
	double BSSOPerf = OptimizationAlgorithm::getPerformance(BSSOSpeeds, k, engine);
	Utility::output_one_vector(cout, "BSSOSpeeds", BSSOSpeeds);

	cout << "Computing Asynchronous B&B...";
	timer.start();
#ifndef __LOCAL_DEBUG__
	vector<double> OPTSOSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, aTasks, engine, period);
#else
	vector<double> OPTSOSpeeds = MaxSOSpeeds;
#endif
	timer.end();
	double OPTSOTime = timer.getTime();
	cout << "done." << endl;
	double OPTSOPerf = OptimizationAlgorithm::getPerformance(OPTSOSpeeds, k, engine);
	Utility::output_one_vector(cout, "OPTSOSpeeds", OPTSOSpeeds);

	cout << endl;
	cout << "BSOPerf = " << UBSOPerf << endl;
	cout << "BSSOPerf = " << BSSOPerf << endl;
	cout << "OPTSOPerf = " << OPTSOPerf << endl;
	cout << endl;
	cout << "Ratio Asynchronous BS/UB = " << BSSOPerf/UBSOPerf*100.0 << "%" << endl;
	cout << "Ratio Asynchronous OPT/UB = " << OPTSOPerf/UBSOPerf*100.0 << "%" << endl;
	cout << "Ratio Asynchronous BS/OPT = " << BSSOPerf/OPTSOPerf*100.0 << "%" << endl;
	cout << endl;
	cout << "UBSOTime = " << UBSOTime << endl;
	cout << "BSSOTime = " << BSSOTime << endl;
	cout << "OPTSOTime = " << OPTSOTime << endl;
	cout << endl;
#endif

	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////

#ifndef __ZERO_OFFESTS__
	PeriodicTask pTask0(1000,5000);
	PeriodicTask pTask1(6500,20000);
	PeriodicTask pTask2(10000,50000);
	PeriodicTask pTask3(5000,95000,100000);
	PeriodicTask pTask4(5000,100000);
#else
	PeriodicTask pTask0(1000,5000);
	PeriodicTask pTask1(6500,20000);
	PeriodicTask pTask2(10000,50000);
	PeriodicTask pTask3(10000,100000);
#endif

	vector<PeriodicTask> pTasks;
	pTasks.push_back(pTask0);
	pTasks.push_back(pTask1);
	pTasks.push_back(pTask2);
	pTasks.push_back(pTask3);

#ifndef __ZERO_OFFESTS__
	pTasks.push_back(pTask4);
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
	cout << "Computing Synchronous UBs...";
	timer.start();
	vector<double> MaxAOSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
	timer.end();
	double UBAOTime = timer.getTime();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"MaxAOSpeeds", MaxAOSpeeds);

	double UBAOPerf = OptimizationAlgorithm::getPerformance(MaxAOSpeeds, k,engine);
	cout << UBAOPerf << "\t" << timer.getTime() << endl;

	cout << "Computing Synchronous BS...";
	timer.start();
#ifndef __LOCAL_DEBUG__
	vector<double> BSAOSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, pTasks, engine, period);
#else
	vector<double> BSAOSpeeds = MaxAOSpeeds;
#endif
	timer.end();
	double BSAOTime = timer.getTime();
	cout << "done." << endl;
	OptimizationAlgorithm::enforceModeGuards(BSAOSpeeds);
	double BSAOPerf = OptimizationAlgorithm::getPerformance(BSAOSpeeds, k, engine);
	Utility::output_one_vector(cout, "BSAOSpeeds", BSAOSpeeds);

	cout << "Computing Synchronous B&B...";
	timer.start();
#ifndef __LOCAL_DEBUG__
	vector<double> OPTAOSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, pTasks, engine, period);
#else
	vector<double> OPTAOSpeeds = MaxAOSpeeds;
#endif
	timer.end();
	double OPTAOTime = timer.getTime();
	cout << "done." << endl;
	double OPTAOPerf = OptimizationAlgorithm::getPerformance(OPTAOSpeeds, k, engine);
	Utility::output_one_vector(cout, "OPTAOSpeeds", OPTAOSpeeds);

	cout << endl;
	cout << "UBAOPerf = " << UBAOPerf << endl;
	cout << "BSAOPerf = " << BSAOPerf << endl;
	cout << "OPTAOPerf = " << OPTAOPerf << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << BSAOPerf/UBAOPerf*100.0 << "%" << endl;
	cout << "Ratio OPT/UB = " << OPTAOPerf/UBAOPerf*100.0 << "%" << endl;
	cout << "Ratio H1/OPT = " << BSAOPerf/OPTAOPerf*100.0 << "%" << endl;
	cout << endl;
	cout << "UBAOTime = " << UBAOTime << endl;
	cout << "BSAOTime = " << BSAOTime << endl;
	cout << "OPTAOTime = " << OPTAOTime << endl;
	cout << endl;
#endif

	// Calculate the performance for the driving cycles
	string INPUT_PREFIX = "Inputs5"+Utility::linkNotation();

	//static const string _dcName[] = {"UDC","NEDC", "ftp75", "Ja1015", "Highway", "IDC", "EUDC", "US_SC03", "Artemis_Road", "Artemis_Urban"};
	//static const string _dcName[] = {"UDC","NEDC"};
	//static const string _dcName[] = {"me"};
	//static const string _dcName[] = {"NEDC", "ftp75","Highway"};
	static const string _dcName[] = {"NEDC"};
	//static const string _dcName[] = { "ftp75","Highway"};
	vector<string> dcFiles(_dcName,_dcName+sizeof(_dcName)/sizeof(_dcName[0]));

	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = INPUT_PREFIX + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

	int index = 0;
	for (auto dcList : dcListVec) {
#ifdef __USING_STATIC_OFFSETS__
		double UBSODCPerf =  OptimizationAlgorithm::getPerformanceDC(dcList,MaxSOSpeeds, k);
		double BSSODCPerf =  OptimizationAlgorithm::getPerformanceDC(dcList,BSSOSpeeds, k);
		double OPTSODCPerf = OptimizationAlgorithm::getPerformanceDC(dcList,OPTSOSpeeds, k);
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
		double UBAODCPerf =  OptimizationAlgorithm::getPerformanceDC(dcList,MaxAOSpeeds, k);
		double BSAODCPerf =  OptimizationAlgorithm::getPerformanceDC(dcList,BSAOSpeeds, k);
		double OPTAODCPerf = OptimizationAlgorithm::getPerformanceDC(dcList,OPTAOSpeeds, k);
#endif

		list<int> dcResultSO, dcResultAO;

#ifdef __USING_STATIC_OFFSETS__
		timer.start();
#ifndef __USING_FAST_DAVR__
		dcResultSO = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,aTasks,engine,period,MaxSOSpeeds);
#else
		dcResultSO = OptimizationAlgorithm::computeUBPerfDCFast(dcList,WCETs,aTasks,engine,period,MaxSOSpeeds);
#endif
		timer.end();
		double DAVRSODCPerf = OptimizationAlgorithm::getPerformanceDC(dcResultSO,k);
		double DAVRSODCTime = timer.getTime();
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
		int optNum = 0;
		double optTime = 0.0;
		double maxOptTime = 0.0;
		int granularity = -1;
		timer.start();
#ifndef __USING_FAST_DAVR__
		dcResultAO = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,pTasks,engine,period,MaxAOSpeeds,optNum,optTime,maxOptTime,granularity);
#else
		dcResultAO = OptimizationAlgorithm::computeUBPerfDCFast(dcList,WCETs,pTasks,engine,period,MaxAOSpeeds);
#endif
		timer.end();
		double DAVRAODCPerf = OptimizationAlgorithm::getPerformanceDC(dcResultAO,k);
		double DAVRAODCTime = timer.getTime()+UBAOTime;
#endif

		cout << "DC - " << dcFiles[index++] << endl;
#ifdef __USING_STATIC_OFFSETS__
		cout << "Ratio Asynchronous DC BS/UB = " << BSSODCPerf/UBSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Asynchronous DC OPT/UB = " << OPTSODCPerf/UBSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Asynchronous DC BS/OPT = " << BSSODCPerf/OPTSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Asynchronous DC DAVR/UB = " << DAVRSODCPerf/UBSODCPerf*100.0 << "%" << endl;
		cout << "DAVRSODCTime = " << DAVRSODCTime << endl;
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
		cout << "Ratio Synchronous DC BS/UB = " << BSAODCPerf/UBAODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous DC OPT/UB = " << OPTAODCPerf/UBAODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous DC BS/OPT = " << BSAODCPerf/OPTAODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous DC DAVR/UB = " << DAVRAODCPerf/UBAODCPerf*100.0 << "%" << endl;
		cout << "DAVRAODCTime = " << DAVRAODCTime << endl;
		cout << "OPTNum = " << optNum << ", " << "OPTTime = " << optTime << ", " << maxOptTime << endl;
#endif

#ifdef __USING_STATIC_OFFSETS__
#ifdef __USING_ARBITRARY_OFFSETS__
		cout << "Ratio Synchronous/Asynchronous DC UB/UB = " << UBAODCPerf/UBSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous/Asynchronous DC BS/UB = " << BSAODCPerf/UBSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous/Asynchronous DC OPT/UB = " << OPTAODCPerf/UBSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous/Asynchronous DC BS/OPT = " << BSAODCPerf/OPTSODCPerf*100.0 << "%" << endl;
		cout << "Ratio Synchronous/Asynchronous DC DAVR/UB = " << DAVRAODCPerf/UBSODCPerf*100.0 << "%" << endl;
#endif
#endif
	}
}

void RunTest::testAsynchronousBiondiExample3() {
	//#define __ZERO_OFFESTS__
#define __LOCAL_DEBUG__
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	vector<int> WCETs;
	double s =9;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);

	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	Timer timer;

#ifndef __ZERO_OFFESTS__
	AsynchronousPeriodicTask aTask0(1000,5000,5000,0);
	AsynchronousPeriodicTask aTask1(6500,20000,20000,0);
	AsynchronousPeriodicTask aTask2(10000,50000,50000,40000);
	AsynchronousPeriodicTask aTask3(5000,95000,100000,50000);
	AsynchronousPeriodicTask aTask4(5000,100000,100000,0);
#else
	AsynchronousPeriodicTask aTask0(1000,5000,5000,0);
	AsynchronousPeriodicTask aTask1(6500,20000,20000,0);
	AsynchronousPeriodicTask aTask2(10000,50000,50000,0);
	AsynchronousPeriodicTask aTask3(10000,100000,100000,0);
#endif

	vector<AsynchronousPeriodicTask> aTasks;
	aTasks.push_back(aTask0);
	aTasks.push_back(aTask1);
	aTasks.push_back(aTask2);
	aTasks.push_back(aTask3);

#ifndef __ZERO_OFFESTS__
	aTasks.push_back(aTask4);
#endif

	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////

#ifndef __ZERO_OFFESTS__
	PeriodicTask pTask0(1000,5000);
	PeriodicTask pTask1(6500,20000);
	PeriodicTask pTask2(10000,50000);
	PeriodicTask pTask3(5000,95000,100000);
	PeriodicTask pTask4(5000,100000);
#else
	PeriodicTask pTask0(1000,5000);
	PeriodicTask pTask1(6500,20000);
	PeriodicTask pTask2(10000,50000);
	PeriodicTask pTask3(10000,100000);
#endif

	vector<PeriodicTask> pTasks;
	pTasks.push_back(pTask0);
	pTasks.push_back(pTask1);
	pTasks.push_back(pTask2);
	pTasks.push_back(pTask3);

#ifndef __ZERO_OFFESTS__
	pTasks.push_back(pTask4);
#endif

	///////////////////////////////////////////////////////////////

	double prevSpeed = 1039;
	double nextSpeed = 1120;

	int prevIndex = 0;
	for (int nextIndex = 1; nextIndex < 5; nextIndex ++) {
		// Prepare AVR task with two modes
		vector<double> avr_speeds;
		vector<int> avr_wcets;
		avr_speeds.push_back(prevSpeed);
		avr_speeds.push_back(nextSpeed);
		avr_speeds.push_back(engine.RPM_MAX);

		avr_wcets.push_back(WCETs[prevIndex]);
		avr_wcets.push_back(WCETs[nextIndex]);
		avr_wcets.push_back(WCETs.back());

		AVRTask avrTask(engine,period,avr_speeds,avr_wcets);

		bool aSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,aTasks,avrTask);
		bool pSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,pTasks,avrTask);
		cout << nextIndex << " => " << aSchedulable << ", " << pSchedulable << endl;
	}
}


void RunTest::AVLTest()
{

	AVLTree* tree=new AVLTree();

	DigraphSearchItem item0(5,10,2);
	DigraphSearchItem item1(2,4,2);
	DigraphSearchItem item2(12,15,2);
	DigraphSearchItem item3(20,21,2);
	DigraphSearchItem item4(16,18,2);

	cout << "== ÒÀ´ÎÌí¼Ó: ";
	tree->insert(item0);
	tree->insert(item1);
	tree->insert(item2);
	tree->insert(item3);
	tree->insert(item4);

	cout << "\n== preOrder: ";
	tree->preOrder();

	cout << "\n== inOrder: ";
	tree->inOrder();

	cout << "\n== postOrder: ";
	tree->postOrder();
	cout << endl;

	cout << "== height: " << tree->height() << endl;
	cout << "== minimum: " << tree->minimum() << endl;
	cout << "== maximum: " << tree->maximum() << endl;
	cout << "== tree info: " << endl;
	tree->write_graphviz(cout);


	DigraphSearchItem sItem0(16,18,3);
	AVLTreeNode* foundItem = tree->iterativeSearch(sItem0);
	cout << "== find node: ";
	if (foundItem == NULL)
		cout << "NULL" << endl;
	else
		cout << foundItem->key << endl;

	cout << "== find all nodes: " << endl;
	vector<DigraphSearchItem> vec;
	cout << tree->check(vec,sItem0) << endl;
	for (auto e : vec) {
		cout << e << endl;
	}
	tree->write_graphviz(cout);

	cout << "\n== remove node: " << 1;
	tree->remove(item0);

	cout << "\n== height: " << tree->height() ;
	cout << "\n== inOrder: " ;
	tree->inOrder();
	cout << "\n== tree info: " << endl;
	tree->write_graphviz(cout);

	tree->destroy();
}

void RunTest::testBiondiExample() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(5000,95000,100000);
	PeriodicTask task4(5000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);

	vector<int> WCETs;
	double s =8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);

	//Utility::output_one_vector(cout,"WCETs",WCETs);
	//exit(EXIT_FAILURE);

	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	Timer timer;

	cout << "Computing UBs...";
	timer.start();
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
	cout << pUB << "\t" << timer.getTime() << endl;

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	OptimizationAlgorithm::enforceModeGuards(BSSpeeds);
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformance(OPTSpeeds, k, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio H1/OPT = " << pH1/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeOPT = " << timeOPT << endl;

#if 0
	// Calculate the performance for the driving cycles
	string INPUT_PREFIX = "Inputs5"+Utility::linkNotation();

	//static const string _dcName[] = {"UDC","NEDC", "ftp75", "Ja1015", "Highway", "IDC", "EUDC", "US_SC03", "Artemis_Road", "Artemis_Urban"};
	//static const string _dcName[] = {"UDC","NEDC"};
	static const string _dcName[] = {"NEDC", "ftp75","Highway"};
	vector<string> dcFiles(_dcName,_dcName+sizeof(_dcName)/sizeof(_dcName[0]));


	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = INPUT_PREFIX + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

	vector<AsynchronousPeriodicTask> aTasks;
	for (auto pTask: tasks) {
		aTasks.push_back(AsynchronousPeriodicTask(pTask));
	}

	int index = 0;
	for (auto dcList : dcListVec) {
		double pUBDC =  OptimizationAlgorithm::getPerformanceDC(dcList,maxSpeeds, k);
		double pBSDC =  OptimizationAlgorithm::getPerformanceDC(dcList,BSSpeeds, k);
		double pOPTDC = OptimizationAlgorithm::getPerformanceDC(dcList,OPTSpeeds, k);
		list<int> dcResult;

#if 0
		timer.start();
		dcResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,aTasks,engine,period,maxSpeeds);
		timer.end();
#endif

		double pDAVRDC = OptimizationAlgorithm::getPerformanceDC(dcResult,k);
		double tDAVRDC = timer.getTime();

		cout << "DC - " << dcFiles[index++] << endl;
		cout << "Ratio DC H1/UB = " << pBSDC/pUBDC*100.0 << "%" << endl;
		cout << "Ratio DC OPT/UB = " << pOPTDC/pUBDC*100.0 << "%" << endl;
		cout << "Ratio DC H1/OPT = " << pBSDC/pOPTDC*100.0 << "%" << endl;
		cout << "Ratio DC DAVR/UB = " << pDAVRDC/pUBDC*100.0 << "%" << endl; 
		cout << "TimeDAVR = " << tDAVRDC << endl;
	}

#endif
}

void RunTest::testBiondiExampleSchedAnalysis() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	double s =8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	//static const double _SPEEDs[] = {1214,1871,2996,3629,4285,6500}; // UB
	static const double _SPEEDs[] = {1050,1868,2887,3194,4282,6500}; // BS
	vector<double> SPEEDs(_SPEEDs,_SPEEDs+sizeof(_SPEEDs)/sizeof(_SPEEDs[0]));

	vector<AVRTask> avrTasks;
	for (int i=0; i<5; i++) {
		vector<int> lWCETs;
		vector<double> lSpeeds;

		lWCETs.push_back(WCETs[i]);
		lWCETs.push_back(WCETs.back());

		lSpeeds.push_back(SPEEDs[i]);
		lSpeeds.push_back(SPEEDs.back());

		AVRTask lAvrTask(engine,period,lSpeeds,lWCETs);
		avrTasks.push_back(lAvrTask);
	}

	AVRTask avrTask(engine,period,SPEEDs,WCETs);
	bool schedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTask,1);
	cout << "AllInOne = " << schedulable << endl;

	for (int i=1; i < avrTasks.size(); i++) {
		AVRTask prevAvrTask = avrTasks[i-1];
		AVRTask avrTask = avrTasks[i];
		vector<AVRTask> two;
		two.push_back(prevAvrTask);
		two.push_back(avrTask);
		AVRTask mixedTwo(engine,period,two);

		bool sAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedTwo,1);
		bool dAVRSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,mixedTwo, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME);
		cout << "i=" << i << "=>" << sAVRSchedulable << "\t" << dAVRSchedulable << endl;
	}
}

void RunTest::testBiondiExampleRelationTMinTMax() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	/*
	double limitedOmega = 1.5*sqrt(1.0*alpha*period/2.0);
	cout << RPmSEC_to_RPM(limitedOmega) << endl;
	*/

	double limitedOmega = 1.45*sqrt(1.0*alpha*period/2.0);
	cout << RPmSEC_to_RPM(limitedOmega) << endl;

	vector<int> RPMs;
	vector<double> TMins;
	vector<double> TMaxs;
	vector<double> T2Mins;

	for (int rpm = rpm_min; rpm <= rpm_max; rpm++) {
		double omega = RPM_to_RPmSEC(rpm);

		double TMin = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega,period);
		double TMax = engine.getMaxInterArrivalTimeWithArbitraryAcceleration(omega,omega,period);
		//double T2Min = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega,2.0*period);
		double T2Min = 2.0*TMin;

		//cout << rpm << "\t" << TMin << "\t" << TMax << "\t" << T2Min << endl;
		
		if (TMax > T2Min) {
			cout << rpm << "\t" << TMin << "\t" << TMax << "\t" << T2Min << endl;
		}
		

		RPMs.push_back(rpm);
		TMins.push_back(TMin);
		TMaxs.push_back(TMax);
		T2Mins.push_back(T2Min);
	}

	
	Utility::output_one_vector(cout,"RPM",RPMs);
	Utility::output_one_vector(cout,"TMin",TMins);
	Utility::output_one_vector(cout,"TMax",TMaxs);
	Utility::output_one_vector(cout,"T2Min",T2Mins);
	
}

void RunTest::testBiondiExampleStateGraph() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s =8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	string dcName = "Inputs5"+Utility::linkNotation()+"NEDC.info";
	list<double> dc = FileReader::ReadDrivingCycleToList(dcName.c_str());

	Timer timer;

	cout << "Computing UBs...";
	timer.start();
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
	cout << pUB << "\t" << timer.getTime() << endl;

#if 1
	cout << "Generating State Graphs ..." << endl;
	timer.start();
	for (int i=0; i<5; i++) {
		//if (i < 2) continue;
		// Prepare AVR task with two modes
		vector<double> avr_speeds;
		vector<int> avr_wcets;

		avr_speeds.push_back(maxSpeeds[i]);
		avr_speeds.push_back(engine.RPM_MAX);

		avr_wcets.push_back(WCETs[i]);
		avr_wcets.push_back(WCETs[5]);

		AVRTask prevAvrTask(engine,period,avr_speeds,avr_wcets);
		vector<double> prevMaxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period,&prevAvrTask);
		cout << i << ": " << maxSpeeds[i] << endl;
		Utility::output_one_vector(cout,"Prev Max Speeds", prevMaxSpeeds);
	}
	timer.end();
	cout << "done." << endl;
	cout << timer.getTime() << endl;
#endif

	cout << "Generating stategraph...";
	timer.start();
	vector<vector<double>> ks;
	ks.push_back(k);
	StateGraph* sg = OptimizationAlgorithm::generateStateGraph(EXACT,PERF_CONSTANT,ks,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	cout << timer.getTime() << endl;

	sg->write_graphviz(cout);

	cout << "======================="  << endl;
	double pUBDC = OptimizationAlgorithm::getPerformanceDC(dc,maxSpeeds,k);
	cout << "=======================" << endl;
	double pUBDC2 = sg->getTotalPerformance(dc);
	cout << "=======================" << endl;
	cout << pUBDC << "\t" << pUBDC2 << endl;

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformance(OPTSpeeds, k, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio H1/OPT = " << pH1/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeOPT = " << timeOPT << endl;
}

void RunTest::testBiondiExampleStateGraph2() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	AsynchronousPeriodicTask task0(1000,5000);
	AsynchronousPeriodicTask task1(6500,20000);
	AsynchronousPeriodicTask task2(10000,50000);
	AsynchronousPeriodicTask task3(10000,100000);

	vector<AsynchronousPeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s =8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	string dcName = "Inputs5"+Utility::linkNotation()+"NEDC.info";
	list<double> dc = FileReader::ReadDrivingCycleToList(dcName.c_str());

	Timer timer;

	cout << "Computing UBs...";
	timer.start();
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
	cout << pUB << "\t" << timer.getTime() << endl;

#if 1
	cout << "Generating State Graphs ..." << endl;
	timer.start();
	for (int i=0; i<5; i++) {

		if (i == 0) continue;
		// Prepare AVR task with two modes
		vector<double> avr_speeds;
		vector<int> avr_wcets;

		avr_speeds.push_back(maxSpeeds[i]);
		avr_speeds.push_back(engine.RPM_MAX);

		avr_wcets.push_back(WCETs[i]);
		avr_wcets.push_back(WCETs[5]);

		AVRTask prevAvrTask(engine,period,avr_speeds,avr_wcets);
		vector<double> nextMaxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period,prevAvrTask,500000);
		cout << i << ": " << maxSpeeds[i] << endl;
		Utility::output_one_vector(cout,"Prev Max Speeds", nextMaxSpeeds);
	}
	timer.end();
	cout << "done." << endl;
	cout << timer.getTime() << endl;
#endif

#if 1
	cout << "Generating stategraph...";
	timer.start();
	vector<vector<double>> ks;
	ks.push_back(k);
	StateGraph* sg = OptimizationAlgorithm::generateStateGraph(EXACT,PERF_CONSTANT,ks,WCETs,tasks,engine,period);
	timer.end();
	cout << "done." << endl;
	cout << timer.getTime() << endl;

	sg->write_graphviz(cout);

	cout << "======================="  << endl;
	double pUBDC = OptimizationAlgorithm::getPerformanceDC(dc,maxSpeeds,k);
	cout << "=======================" << endl;
	double pUBDC2 = sg->getTotalPerformance(dc);
	cout << "=======================" << endl;
	cout << pUBDC << "\t" << pUBDC2 << endl;

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	double pH1DC = OptimizationAlgorithm::getPerformanceDC(dc,BSSpeeds, k);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);
#if 0
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformance(OPTSpeeds, k, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);
#endif

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH1DC = " << pH1DC << endl;
	//cout << "pOPT = " << pOPT << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	//cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	//cout << "Ratio H1/OPT = " << pH1/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	//cout << "timeOPT = " << timeOPT << endl;
#endif
}

void RunTest::testBiondiExampleJobSequence() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	// Generating a job sequence which starts at 1214 RPM
	double startOmega = RPM_to_RPmSEC(1214.0);
	double interval = 0.0;
	cout << interval << "\t" << RPmSEC_to_RPM(startOmega) << endl;
	for (int i=1; i<=10; i++) {
		double nextOmega = engine.getHigherSpeed(startOmega,period,i);
		interval += engine.getInterArrivalTimeWithConstantAcceleration(startOmega,nextOmega,period);
		cout << interval << "\t" << RPmSEC_to_RPM(nextOmega) << endl;
	}
}

void RunTest::testBiondiExampleWithoutBB() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
			
	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBS_NECESSARYONLY_WithoutDebug(k,WCETs,tasks,engine,period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformance(BSSpeeds2, k, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pH2/pUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
}

void RunTest::testBiondiExampleWithoutBBMapping() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000,5000);
	PeriodicTask task1(6500,20000,20000);
	PeriodicTask task2(10000,50000,50000);
	PeriodicTask task3(5000,95000,100000);
	PeriodicTask task4(5000,100000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);

	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBS_NECESSARYONLY_WithoutDebug(k,WCETs,tasks,engine,period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformance(BSSpeeds2, k, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pH2/pUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
}

void RunTest::testBiondiExampleWithoutBBColllectInfo() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBS_NECESSARYONLY_CollectInfo(k,WCETs, tasks, engine, period);
	OptimizationAlgorithm::save_results();
	OptimizationAlgorithm::outputCollectInfo(cout);
}

void RunTest::testBiondiExampleRBFBased() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
			
	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(RBF,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
}

void RunTest::testBiondiExampleExp() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k1[] = {1,1,1,1,1,1};
	//static const double _k1[] = {10,7,5,4,3,2};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {0.0,0.2,0.3,0.4,0.5,0.7};
	//static const double _k2[] = {0.0,0.1,1,10,100,1000};
	//static const double _k2[] = {0.0,100,300,500,700,1000};
	static const double _k2[] = {0,0.873475,1.67477,1.9488,3.34087,3.35556};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExp(maxSpeeds, k1,k2,engine);
	cout << "pUB = " << pUB << endl;
			
	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExp(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug(k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformanceExp(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExp(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExp(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pH2/pUB*100.0 << "%" << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio H1/OPT = " << pH1/pOPT*100.0 << "%" << endl;
	cout << "Ratio H2/OPT = " << pH2/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH1 << endl;
	cout << "timeOPT = " << timeOPT << endl;
}

void RunTest::testBiondiExampleExpWithoutBB() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k1[] = {1,1,1,1,1,1};
	//static const double _k1[] = {10,7,5,4,3,2};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {0.0,0.2,0.3,0.4,0.5,0.7};
	//static const double _k2[] = {0.0,0.1,1,10,100,1000};
	//static const double _k2[] = {0.0,100,300,500,700,1000};
	static const double _k2[] = {0,0.873475,1.67477,1.9488,3.34087,3.35556};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExp(maxSpeeds, k1,k2,engine);

	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExp(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug(k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformanceExp(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pH2/pUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
}

void RunTest::testBiondiExamplePoly() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	//WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k1[] = {-1.57e-07,3.59E-07,7.57E-07};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	static const double _k2[] = {0.0009045, -0.0004595, -0.00272};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));
	
	static const double _k3[] = {2.00E-05, 1.91E-05, 2.03E-05};
	vector<double> k3 (_k3,_k3+sizeof(_k3)/sizeof(_k3[0]));
	
	static const double _k4[] = {-0.1014, 1.203, 3.938};
	vector<double> k4 (_k4,_k4+sizeof(_k4)/sizeof(_k4[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	// Assumming T=0, k3 should not be used here.
	double pUB = OptimizationAlgorithm::getPerformancePoly(maxSpeeds, k1,k2,k4,engine);
			
	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSPoly3(EXACT,k1,k2,k4,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformancePoly(BSSpeeds, k1,k2, k4, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing B&B without torque...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBPoly3(EXACT,k1,k2,k4,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformancePoly(OPTSpeeds, k1,k2,k4, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	cout << "Computing B&B with torque...";
	timer.start();
	vector<double> OPTSpeeds2 = OptimizationAlgorithm::computeBiondiBBPoly4(EXACT,k1,k2,k3,k4,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT2 = timer.getTime();
	cout << "done." << endl;
	double pOPT2 = OptimizationAlgorithm::getPerformancePoly(OPTSpeeds, k1,k2,k3, k4, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio H1/OPT = " << pH1/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeOPT = " << timeOPT << endl;
}

void RunTest::testMultiInjectionsExp() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	
	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	Timer timer;

	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug_Min(k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << "Ratio H1/UB = " << pUB/pH1*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pUB/pH2*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
}

void RunTest::testMultiInjectionsExpWithoutBB() {
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());


	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	Timer timer;

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug_Min(k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pUB/pH1*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pUB/pH2*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
}

void RunTest::testBiondiExampleDC(string inputDir, string dcName, vector<double>& perf_results, vector<double>& time_results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDC(dc, maxSpeeds, k);
			
	cout << "Computing H1...";
	Timer timer;
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeChaoBS(dc,EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);
	double pDCH1 = OptimizationAlgorithm::getPerformanceDC(dc, BSSpeeds, k);

	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeDrivingCycleBB(dc,EXACT,k,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformance(OPTSpeeds, k, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);
	double pDCOPT = OptimizationAlgorithm::getPerformanceDC(dc,OPTSpeeds,k);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pDCH1 = " << pDCH1 << endl;
	cout << "pDCOPT = " << pDCOPT << endl;
	cout << endl;
	cout << "Ratio H1/UB = " << pH1/pUB*100.0 << "%" << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio H1/OPT = " << pH1/pOPT*100.0 << "%" << endl;
	cout << "Ratio DC H1/UB = " << pDCH1/pDCUB*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCOPT/pDCUB*100.0 << "%" << endl;
	cout << "Ratio DC H1/OPT = " << pDCH1/pDCOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeOPT = " << timeOPT << endl;

	perf_results.push_back(pDCH1/pDCOPT);
	time_results.push_back(timeH1);
}

void RunTest::generateBiondiTestedSystems(string directory, int numSystem, int numPeriodicTask, int tUtil, int numMode) {
	// mkdir directory
	Utility::makeDir(directory);

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		TaskSystemGenerator::generate_random_system_for_engine_optimization(tasks,numPeriodicTask,1.0*tUtil/100, avrTask,numMode);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask);

		delete avrTask;

		#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
		#endif
	}
}

void RunTest::doBBForBiondiTestedSystems(string resultDir, string inputDir, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK) {
	// mkdir directory
	Utility::makeDir(resultDir);
	
	string resultUBSpeeds = resultDir + Utility::linkNotation()+"UBSpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBBSpeeds = resultDir + Utility::linkNotation()+"BBSpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultUBPerformance = resultDir + Utility::linkNotation()+"UBPerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBBPerformance = resultDir + Utility::linkNotation()+"BBPerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultCollection = resultDir + Utility::linkNotation()+"BBResultCollection"+Utility::int_to_string(tUtil)+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBBSpeeds(resultBBSpeeds, ios::out | ios::trunc);
	if (!foutBBSpeeds.is_open()) {
		cerr << "Can't open "<<resultBBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutUBPerformance(resultUBPerformance, ios::out | ios::trunc);
	if (!foutUBPerformance.is_open()) {
		cerr << "Can't open "<<resultUBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBBPerformance(resultBBPerformance, ios::out | ios::trunc);
	if (!foutBBPerformance.is_open()) {
		cerr << "Can't open "<<resultBBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedPerfs;
	vector<double> normalizedTimes;

	double maxTime = INT_MIN;
	double minTime = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		foutUBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutUBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutBBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		double totalPerf = 0.0;
		double totalTime = 0.0;
		int numUnFeasible = 0;

		for (int i=0; i<numK; i++) {
			vector<double> k = TaskSystemGenerator::generate_coefficients(sizeK);
			Utility::output_one_vector(foutUBSpeeds, "k", k);
			Utility::output_one_vector(foutBBSpeeds, "k", k);
			Utility::output_one_vector(foutUBPerformance, "k", k);
			Utility::output_one_vector(foutBBPerformance, "k", k);

			for (int run=minSystem; run <= maxSystem; run++) {
				cout << "factor = " << factor << ", No.k = " << i << ", run = " << run << " => ";
				vector<PeriodicTask> tasks;
				vector<int> WCETs;

				string name = inputDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
				const char *p = name.c_str();
				FileReader::ReadTaskSystem(p,tasks,WCETs);


				for (int i=0; i<WCETs.size(); i++)
					WCETs[i] *= factor;

				vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);

				Timer timer;
				timer.start();
				vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBB(EXACT,k,WCETs, tasks, engine, period);
				timer.end();

				if (OPTSpeeds.empty()) {
					numUnFeasible ++;
					continue;
				}

				totalTime += timer.getTime();

				cout << " time = " << timer.getTime();

				minTime = min(minTime, timer.getTime());
				maxTime = max(maxTime, timer.getTime());

				Utility::output_one_vector(foutUBSpeeds,"Max Speeds", maxSpeeds);
				double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
				foutUBPerformance << pUB << endl;

				Utility::output_one_vector(foutBBSpeeds, "OPT Speeds", OPTSpeeds);
				double pOPT = OptimizationAlgorithm::getPerformance(OPTSpeeds, k, engine);
				foutBBPerformance << pOPT << endl;

				totalPerf += pOPT/pUB;
				cout << " normalizedPerf = " << pOPT/pUB << endl;
			}
		}

		int norm = (maxSystem-minSystem+1)*numK-numUnFeasible;
		normalizedPerfs.push_back(totalPerf/norm);
		normalizedTimes.push_back(totalTime/norm);

		cout << "minTime = " << minTime << endl;
		cout << "maxTime = " << maxTime << endl;
		cout << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);
	}

	cout << "minTime = " << minTime << endl;
	cout << "maxTime = " << maxTime << endl;
	
	Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

	foutResultCollection << "minTime = " << minTime << endl;
	foutResultCollection << "maxTime = " << maxTime << endl;

	foutUBSpeeds.close();
	foutBBSpeeds.close();
	foutUBPerformance.close();
	foutBBPerformance.close();
	foutResultCollection.close();
}

void RunTest::doBSForBiondiTestedSystems(string resultDir, string inputDir, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK) {
	// mkdir directory
	Utility::makeDir(resultDir);

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"UBSpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBSSpeeds = resultDir + Utility::linkNotation()+"BSSpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBS2Speeds = resultDir + Utility::linkNotation()+"BS2SpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultUBPerformance = resultDir + Utility::linkNotation()+"UBPerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBSPerformance = resultDir + Utility::linkNotation()+"BSPerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBS2Performance = resultDir + Utility::linkNotation()+"BS2PerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultCollection = resultDir + Utility::linkNotation()+"BSResultCollection"+Utility::int_to_string(tUtil)+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBSSpeeds(resultBSSpeeds, ios::out | ios::trunc);
	if (!foutBSSpeeds.is_open()) {
		cerr << "Can't open "<<resultBSSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBS2Speeds(resultBS2Speeds, ios::out | ios::trunc);
	if (!foutBS2Speeds.is_open()) {
		cerr << "Can't open "<<resultBS2Speeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutUBPerformance(resultUBPerformance, ios::out | ios::trunc);
	if (!foutUBPerformance.is_open()) {
		cerr << "Can't open "<<resultUBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBSPerformance(resultBSPerformance, ios::out | ios::trunc);
	if (!foutBSPerformance.is_open()) {
		cerr << "Can't open "<<resultBSPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBS2Performance(resultBS2Performance, ios::out | ios::trunc);
	if (!foutBS2Performance.is_open()) {
		cerr << "Can't open "<<resultBS2Performance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedPerfs;
	vector<double> normalizedTimes;

	vector<double> normalizedPerfs2;
	vector<double> normalizedTimes2;

	double maxTime = INT_MIN;
	double minTime = INT_MAX;

	double maxTime2 = INT_MIN;
	double minTime2 = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		foutUBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBSSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBS2Speeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutUBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutBSPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBS2Performance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 

		double totalPerf = 0.0;
		double totalPerf2 = 0.0;

		double totalTime = 0.0;
		double totalTime2 = 0.0;
		int numUnFeasible = 0;

		for (int i=0; i<numK; i++) {
			vector<double> k = TaskSystemGenerator::generate_coefficients(sizeK);
			Utility::output_one_vector(foutUBSpeeds, "k", k);
			Utility::output_one_vector(foutBSSpeeds, "k", k);
			Utility::output_one_vector(foutBS2Speeds, "k", k);
			Utility::output_one_vector(foutUBPerformance, "k", k);
			Utility::output_one_vector(foutBSPerformance, "k", k);
			Utility::output_one_vector(foutBS2Performance, "k", k);

			for (int run=minSystem; run <= maxSystem; run++) {
				cout << "factor = " << factor << ", No.k = " << i << ", run = " << run << " => ";
				foutResultCollection << "factor = " << factor << ", No.k = " << i << ", run = " << run << " => ";
				vector<PeriodicTask> tasks;
				vector<int> WCETs;

				string name = inputDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
				const char *p = name.c_str();
				FileReader::ReadTaskSystem(p,tasks,WCETs);


				for (int i=0; i<WCETs.size(); i++)
					WCETs[i] *= factor;

				vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
				OptimizationAlgorithm::enforceModeGuards(maxSpeeds);

				Timer timer;
				timer.start();
				vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
				timer.end();
				double tBS = timer.getTime();

				if (BSSpeeds.empty()) {
					numUnFeasible ++;
					cout << endl;
					continue;
				}

				timer.start();
				vector<double> BS2Speeds = OptimizationAlgorithm::computeBS_NECESSARYONLY_WithoutDebug(k,WCETs, tasks, engine, period);
				timer.end();
				double tBS2 = timer.getTime();

				
				if (BS2Speeds.empty()) {
					numUnFeasible ++;
					cout << endl;
					continue;
				}
				

				totalTime += tBS;
				totalTime2 += tBS2;

				cout << " time = " << tBS << ", " << tBS2;
				foutResultCollection << " time = " << tBS << ", " << tBS2;

				minTime = min(minTime, tBS);
				maxTime = max(maxTime, tBS);

				minTime2 = min(minTime2, tBS2);
				maxTime2 = max(maxTime2, tBS2);

				Utility::output_one_vector(foutUBSpeeds,"Max Speeds", maxSpeeds);
				double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
				foutUBPerformance << pUB << endl;

				Utility::output_one_vector(foutBSSpeeds, "BS Speeds", BSSpeeds);
				double pBS = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
				foutBSPerformance << pBS << endl;

				Utility::output_one_vector(foutBS2Speeds, "BS2 Speeds", BS2Speeds);
				double pBS2 = OptimizationAlgorithm::getPerformance(BS2Speeds, k, engine);
				foutBS2Performance << pBS2 << endl;

				totalPerf += pBS/pUB;
				totalPerf2 += pBS2/pUB;

				cout << " normalizedPerf = " << pBS/pUB << ", " << pBS2/pUB << endl;
				foutResultCollection << " normalizedPerf = " << pBS/pUB << ", " << pBS2/pUB << endl;
			}
		}

		int norm = (maxSystem-minSystem+1)*numK-numUnFeasible;
		normalizedPerfs.push_back(totalPerf/norm);
		normalizedTimes.push_back(totalTime/norm);

		normalizedPerfs2.push_back(totalPerf2/norm);
		normalizedTimes2.push_back(totalTime2/norm);

		cout << "minTime = " << minTime << endl;
		cout << "maxTime = " << maxTime << endl;

		cout << "minTime2 = " << minTime2 << endl;
		cout << "maxTime2 = " << maxTime2 << endl;
		cout << "numUnFeasible = " << numUnFeasible << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

		Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);
	}

	cout << "minTime = " << minTime << endl;
	cout << "maxTime = " << maxTime << endl;

	cout << "minTime2 = " << minTime2 << endl;
	cout << "maxTime2 = " << maxTime2 << endl;

	Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);

	foutResultCollection << "minTime = " << minTime << endl;
	foutResultCollection << "maxTime = " << maxTime << endl;

	foutResultCollection << "minTime2 = " << minTime2 << endl;
	foutResultCollection << "maxTime2 = " << maxTime2 << endl;

	foutUBSpeeds.close();
	foutBSSpeeds.close();
	foutBS2Speeds.close();
	foutUBPerformance.close();
	foutBSPerformance.close();
	foutBS2Performance.close();
	foutResultCollection.close();
}

void RunTest::doBSForBiondiTestedSystemsCollectInfo(string resultDir, string inputDir, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK) {
	// mkdir directory
	Utility::makeDir(resultDir);

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"UBSpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBS2Speeds = resultDir + Utility::linkNotation()+"BS2SpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultUBPerformance = resultDir + Utility::linkNotation()+"UBPerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBS2Performance = resultDir + Utility::linkNotation()+"BS2PerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultCollection = resultDir + Utility::linkNotation()+"BSResultCollection"+Utility::int_to_string(tUtil)+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBS2Speeds(resultBS2Speeds, ios::out | ios::trunc);
	if (!foutBS2Speeds.is_open()) {
		cerr << "Can't open "<<resultBS2Speeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutUBPerformance(resultUBPerformance, ios::out | ios::trunc);
	if (!foutUBPerformance.is_open()) {
		cerr << "Can't open "<<resultUBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBS2Performance(resultBS2Performance, ios::out | ios::trunc);
	if (!foutBS2Performance.is_open()) {
		cerr << "Can't open "<<resultBS2Performance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedPerfs2;
	vector<double> normalizedTimes2;

	double maxTime2 = INT_MIN;
	double minTime2 = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		foutUBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBS2Speeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutUBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutBS2Performance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 

		double totalPerf2 = 0.0;

		double totalTime2 = 0.0;
		int numUnFeasible = 0;

		for (int i=0; i<numK; i++) {
			vector<double> k = TaskSystemGenerator::generate_coefficients(sizeK);
			Utility::output_one_vector(foutUBSpeeds, "k", k);
			Utility::output_one_vector(foutBS2Speeds, "k", k);
			Utility::output_one_vector(foutUBPerformance, "k", k);
			Utility::output_one_vector(foutBS2Performance, "k", k);

			for (int run=minSystem; run <= maxSystem; run++) {
				cout << "factor = " << factor << ", No.k = " << i << ", run = " << run << " => ";
				foutResultCollection << "factor = " << factor << ", No.k = " << i << ", run = " << run << " => ";
				vector<PeriodicTask> tasks;
				vector<int> WCETs;

				string name = inputDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
				const char *p = name.c_str();
				FileReader::ReadTaskSystem(p,tasks,WCETs);


				for (int i=0; i<WCETs.size(); i++)
					WCETs[i] *= factor;

				vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
				OptimizationAlgorithm::enforceModeGuards(maxSpeeds);

				Timer timer;
				timer.start();
				vector<double> BS2Speeds = OptimizationAlgorithm::computeBS_NECESSARYONLY_CollectInfo(k,WCETs, tasks, engine, period);
				timer.end();
				double tBS2 = timer.getTime();

				if (BS2Speeds.empty()) {
					numUnFeasible ++;
					cout << endl;
					continue;
				}

				totalTime2 += tBS2;

				cout << " time = " << tBS2;
				foutResultCollection << " time = " << tBS2;

				minTime2 = min(minTime2, tBS2);
				maxTime2 = max(maxTime2, tBS2);

				Utility::output_one_vector(foutUBSpeeds,"Max Speeds", maxSpeeds);
				double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
				foutUBPerformance << pUB << endl;

				Utility::output_one_vector(foutBS2Speeds, "BS2 Speeds", BS2Speeds);
				double pBS2 = OptimizationAlgorithm::getPerformance(BS2Speeds, k, engine);
				foutBS2Performance << pBS2 << endl;

				totalPerf2 += pBS2/pUB;

				cout << " normalizedPerf = " << pBS2/pUB << endl;
				foutResultCollection << " normalizedPerf = " << pBS2/pUB << endl;
			}
		}

		int norm = (maxSystem-minSystem+1)*numK-numUnFeasible;

		normalizedPerfs2.push_back(totalPerf2/norm);
		normalizedTimes2.push_back(totalTime2/norm);


		cout << "minTime2 = " << minTime2 << endl;
		cout << "maxTime2 = " << maxTime2 << endl;
		cout << "numUnFeasible = " << numUnFeasible << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);

		OptimizationAlgorithm::set_scale(norm);

		OptimizationAlgorithm::save_results();
		OptimizationAlgorithm::set_zero();

		OptimizationAlgorithm::outputCollectInfo(cout);
		OptimizationAlgorithm::outputCollectInfo(foutResultCollection);
	}

	cout << "minTime2 = " << minTime2 << endl;
	cout << "maxTime2 = " << maxTime2 << endl;


	Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);


	Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);


	foutResultCollection << "minTime2 = " << minTime2 << endl;
	foutResultCollection << "maxTime2 = " << maxTime2 << endl;

	OptimizationAlgorithm::outputCollectInfo(cout);
	OptimizationAlgorithm::outputCollectInfo(foutResultCollection);

	foutUBSpeeds.close();
	foutBS2Speeds.close();
	foutUBPerformance.close();
	foutBS2Performance.close();
	foutResultCollection.close();
}

void RunTest::doBSForSingleBiondiTestedSystem(string inputDir, int tUtil, int noSystem, int noFactor, vector<double> k) {
	vector<double> normalizedPerfs;
	vector<double> normalizedTimes;

	double maxTime = INT_MIN;
	double minTime = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = noFactor; factor <= noFactor; factor++) {
		double totalPerf = 0.0;
		double totalTime = 0.0;
		int numUnFeasible = 0;

		for (int run=noSystem; run <= noSystem; run++) {
			vector<PeriodicTask> tasks;
			vector<int> WCETs;

			string name = inputDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,tasks,WCETs);


			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;


			vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
			OptimizationAlgorithm::enforceModeGuards(maxSpeeds);

			Timer timer;
			timer.start();
			vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, tasks, engine, period);
			timer.end();
			double tBS = timer.getTime();

			if (BSSpeeds.empty()) {
				numUnFeasible ++;
				continue;
			}

			timer.start();
			vector<double> BS2Speeds = OptimizationAlgorithm::computeBS_NECESSARYONLY_WithoutDebug(k,WCETs, tasks, engine, period);
			timer.end();
			double tBS2 = timer.getTime();

			cout << " time = " << tBS << ", " << tBS2;

			
			double pUB = OptimizationAlgorithm::getPerformance(maxSpeeds, k,engine);
			double pBS = OptimizationAlgorithm::getPerformance(BSSpeeds, k, engine);
			double pBS2 = OptimizationAlgorithm::getPerformance(BS2Speeds, k, engine);

			cout << " normalizedPerf = " << pBS/pUB << ", " << pBS2/pUB << endl;
		}
		cout << "numUnFeasible = " << numUnFeasible << endl;
	}
}

void RunTest::doOptimizationOverBiondiTestedSystemsExp(bool isMax, vector<bool> vecTest, string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, int numK, int sizeK, double k2Min, double k2Max, double k2Step) {
	// mkdir directory
	Utility::makeDir(resultDir);
	
	if (vecTest.size()!=4) {
		cerr << "Error vecTest size = "<<vecTest.size() << endl;
		exit(EXIT_FAILURE);
	}

	const char *dcFilePointer = dcFile.c_str();
	vector<map<int,int>> dcVector = FileReader::ReadDrivingCycleVector(dcFilePointer);

	//map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	map<int,int> dc;
	for (auto dcElement:dcVector) {
		for (auto e : dcElement) {
			if (dc.find(e.first)!=dc.end()) {
				dc[e.first] += e.second; 
			}
			else dc[e.first] = e.second;
		}
	}

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"ExpConstKUBSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultOPTSpeeds = resultDir + Utility::linkNotation()+"ExpConstKOPTSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultBSSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBSSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultOPTDCSpeeds = resultDir + Utility::linkNotation()+"ExpConstKOPTDCSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultBSDCSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBSDCSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";

	string resultCollection = resultDir + Utility::linkNotation()+"ExpConstKBSResultCollectionDC"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";

	vector<string> speedsFileNames;
	speedsFileNames.push_back(resultUBSpeeds);
	speedsFileNames.push_back(resultOPTSpeeds);
	speedsFileNames.push_back(resultBSSpeeds);
	speedsFileNames.push_back(resultOPTDCSpeeds);
	speedsFileNames.push_back(resultBSDCSpeeds);

	vector<ofstream*> speedsOfstreams;
	for (auto fileName:speedsFileNames) {
		ofstream fout(fileName,ios::out | ios::trunc);
		if (!fout.is_open()) {
			cerr << "Can't open "<<fileName<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		speedsOfstreams.push_back(&fout);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedOPTPerfs;
	vector<double> normalizedOPTTimes;

	vector<double> normalizedBSPerfs;
	vector<double> normalizedBSTimes;

	vector<double> normalizedOPTDCPerfs;
	vector<double> normalizedOPTDCTimes;

	vector<double> normalizedBSDCPerfs;
	vector<double> normalizedBSDCTimes;

	vector<double> normalizedBSDCSecPerfs;
	vector<double> normalizedBSDCSecTimes;

	vector<double> OPTDCImpRatios;
	vector<double> BSDCImpRatios;
	vector<double> BSDCSecImpRatios;

	double maxOPTTime = INT_MIN;
	double minOPTTime = INT_MAX;

	double maxBSTime = INT_MIN;
	double minBSTime = INT_MAX;

	double maxOPTDCTime = INT_MIN;
	double minOPTDCTime = INT_MAX;

	double maxBSDCTime = INT_MIN;
	double minBSDCTime = INT_MAX;

	double maxBSDCSecTime = INT_MIN;
	double minBSDCSecTime = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {

		// record factor
		for (int i=0; i<speedsOfstreams.size(); i++) {
			if (i==0) (*speedsOfstreams[i]) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
			else {
				if (vecTest[i-1]) (*speedsOfstreams[i]) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
			}
		}

		vector<double> k1(sizeK,1.0);

		vector<double> temp = TaskSystemGenerator::generate_coefficients_k2(sizeK,k2Min,k2Max,k2Step);
		vector<double> k2; // reverse vector temp
		if (isMax) k2 = temp;
		else { // reverse vector temp
			for (int ii = temp.size()-1; ii >=0; ii--) {
				k2.push_back(temp[ii]);
			}
		}

		// record k1 and k2
		for (int i=0; i<speedsOfstreams.size(); i++) {
			if (i==0) {
				Utility::output_one_vector((*speedsOfstreams[i]), "k1",k1);
				Utility::output_one_vector((*speedsOfstreams[i]), "k2",k1);
			}
			else {
				if (vecTest[i-1]) {
					Utility::output_one_vector((*speedsOfstreams[i]), "k1",k1);
					Utility::output_one_vector((*speedsOfstreams[i]), "k2",k1);
				}
			}
		}

		double totalOPTPerf = 0.0;
		double totalBSPerf = 0.0;
		double totalOPTDCPerf = 0.0;
		double totalBSDCPerf = 0.0;
		double totalBSDCSecPerf = 0.0;

		double totalOPTTime = 0.0;
		double totalBSTime = 0.0;
		double totalOPTDCTime = 0.0;
		double totalBSDCTime = 0.0;
		double totalBSDCSecTime = 0.0;

		double totalOPTDCImpRatio = 0.0;
		double totalBSDCImpRatio = 0.0;
		double totalBSDCSecImpRatio = 0.0;

		int numUnFeasible = 0;

		int run = 0;
		int numSucc = 0;

		while (numSucc < numSystem) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> tasks;
			vector<int> WCETs;

			string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,tasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

			bool bOPT = vecTest[0];
			bool bBS = vecTest[1];
			bool bOPTDC = vecTest[2];
			bool bBSDC = vecTest[3];
			bool bBSDCSec = vecTest[4];

			vector<double> UBSpeeds,OPTSpeeds,BSSpeeds, OPTDCSpeeds, BSDCSpeeds;
			vector<vector<double>> BSDCSecSpeedsVec;
			double pUB = 0.0, pOPT = 0.0, pBS = 0.0, pOPTDC = 0.0, pBSDC = 0.0, pBSDCSec = 0.0;
			double tUB = 0.0, tOPT = 0.0, tBS = 0.0, tOPTDC = 0.0, tBSDC = 0.0, tBSDCSec = 0.0;

			bool found = false;
			if (isMax)
				found = OptimizationAlgorithm::doAllOptimizationsExpMax(UBSpeeds,pUB,tUB,
				bOPT,OPTSpeeds,pOPT,tOPT,
				bBS, BSSpeeds,pBS,tBS,
				bOPTDC,OPTDCSpeeds,pOPTDC,tOPTDC,
				bBSDC,BSDCSpeeds,pBSDC,tBSDC,
				bBSDCSec,BSDCSecSpeedsVec,pBSDCSec,tBSDCSec,
				dc,dcVector,k1,k2,WCETs,tasks,engine,period);
			else
				found = OptimizationAlgorithm::doAllOptimizationsExpMin(UBSpeeds,pUB,tUB,
				bOPT,OPTSpeeds,pOPT,tOPT,
				bBS, BSSpeeds,pBS,tBS,
				bOPTDC,OPTDCSpeeds,pOPTDC,tOPTDC,
				bBSDC,BSDCSpeeds,pBSDC,tBSDC,
				bBSDCSec,BSDCSecSpeedsVec,pBSDCSec,tBSDCSec,
				dc,dcVector,k1,k2,WCETs,tasks,engine,period);

			if (found) numSucc++;
			else {
				numUnFeasible++;
				continue;
			}

			vector<vector<double>> allSpeeds;
			allSpeeds.push_back(UBSpeeds);
			allSpeeds.push_back(OPTSpeeds);
			allSpeeds.push_back(BSSpeeds);
			allSpeeds.push_back(OPTDCSpeeds);
			allSpeeds.push_back(BSDCSpeeds);

			minOPTTime = min(minOPTTime, tOPT);
			maxOPTTime = max(maxOPTTime, tOPT);

			minBSTime = min(minBSTime, tBS);
			maxBSTime = max(maxBSTime, tBS);

			minOPTDCTime = min(minOPTDCTime, tOPTDC);
			maxOPTDCTime = max(maxOPTDCTime, tOPTDC);

			minBSDCTime = min(minBSDCTime, tBSDC);
			maxBSDCTime = max(maxBSDCTime, tBSDC);

			minBSDCSecTime = min(minBSDCSecTime, tBSDCSec);
			maxBSDCSecTime = max(maxBSDCSecTime, tBSDCSec);

			// record speed results
			for (int i=0; i<speedsOfstreams.size(); i++) {
				if (i==0) {
					Utility::output_one_vector((*speedsOfstreams[i]),"Max Speeds", allSpeeds[i]);
					//Utility::output_one_vector(cout,"Max Speeds", allSpeeds[i]);
				}
				else {
					if (vecTest[i-1]) {
						Utility::output_one_vector((*speedsOfstreams[i]),"Result Speeds", allSpeeds[i]);
						//Utility::output_one_vector(cout,"Result Speeds", allSpeeds[i]);
					}
				}
			}

			double OPTPerf = 0.0; 
			double BSPerf = 0.0; 
			double OPTDCPerf = 0.0; 
			double BSDCPerf = 0.0;
			double BSDCSecPerf = 0.0;

			double OPTDCImpRatio = 0.0;
			double BSDCImpRatio = 0.0;
			double BSDCSecImpRatio = 0.0;

			if (isMax) {
				OPTPerf = pOPT/pUB;
				BSPerf = pBS/pUB;
				OPTDCPerf = pOPTDC/pUB;
				BSDCPerf = pBSDC/pUB;
				BSDCSecPerf = pBSDCSec/pUB;

				OPTDCImpRatio = (pOPTDC-pOPT)/pOPT;
				BSDCImpRatio = (pBSDC-pBS)/pBS;
				BSDCSecImpRatio = (pBSDCSec-pBS)/pBS;
			} 
			else {
				OPTPerf = pUB/pOPT;
				BSPerf = pUB/pBS;
				OPTDCPerf = pUB/pOPTDC;
				BSDCPerf = pUB/pBSDC;

				OPTDCImpRatio = (pOPT-pOPTDC)/pOPT;
				BSDCImpRatio = (pBS-pBSDC)/pBS;
			}

			totalOPTPerf += OPTPerf;
			totalBSPerf += BSPerf;
			totalOPTDCPerf += OPTDCPerf;
			totalBSDCPerf += BSDCPerf;
			totalBSDCSecPerf += BSDCSecPerf;

			totalOPTTime += tOPT;
			totalBSTime += tBS;
			totalOPTDCTime += tOPTDC;
			totalBSDCTime += tBSDC;
			totalBSDCSecTime += tBSDCSec;

			totalOPTDCImpRatio += OPTDCImpRatio;
			totalBSDCImpRatio += BSDCImpRatio;
			totalBSDCSecImpRatio += BSDCSecImpRatio;

			cout << " Perf = " << OPTPerf << " " << BSPerf << " " << OPTDCPerf << " " << BSDCPerf << " " << BSDCSecPerf 
				<< ", Time = " << tOPT << " " << tBS << " " << tOPTDC << " " << tBSDC << " " << tBSDCSec
				<< ", Ratio = " << OPTDCImpRatio << " " << BSDCImpRatio << " " << BSDCSecImpRatio;

			foutResultCollection << " Perf = " << OPTPerf << " " << BSPerf << " " << OPTDCPerf << " " << BSDCPerf << " " << BSDCSecPerf 
				<< ", Time = " << tOPT << " " << tBS << " " << tOPTDC << " " << tBSDC << " " << tBSDCSec
				<< ", Ratio = " << OPTDCImpRatio << " " << BSDCImpRatio << " " << BSDCSecImpRatio;

			if (OPTDCPerf-OPTPerf > 0.05) {
				cout << " ++++";
				foutResultCollection << " ++++";
			}

			if (BSDCPerf-BSPerf > 0.05) {
				cout << " ####";
				foutResultCollection << " ####";
			}

			if (OPTDCPerf-BSDCPerf > 0.05) {
				cout << " $$$$";
				foutResultCollection << " $$$$";
			}

			cout << endl;
			foutResultCollection << endl;
		}

		int norm = numSystem;
		normalizedOPTPerfs.push_back(totalOPTPerf/norm);
		normalizedBSPerfs.push_back(totalBSPerf/norm);
		normalizedOPTDCPerfs.push_back(totalOPTDCPerf/norm);
		normalizedBSDCPerfs.push_back(totalBSDCPerf/norm);
		normalizedBSDCSecPerfs.push_back(totalBSDCSecPerf/norm);

		normalizedOPTTimes.push_back(totalOPTTime/norm);
		normalizedBSTimes.push_back(totalBSTime/norm);
		normalizedOPTDCTimes.push_back(totalOPTDCTime/norm);
		normalizedBSDCTimes.push_back(totalBSDCTime/norm);
		normalizedBSDCSecTimes.push_back(totalBSDCSecTime/norm);

		OPTDCImpRatios.push_back(totalOPTDCImpRatio/norm);
		BSDCImpRatios.push_back(totalBSDCImpRatio/norm);
		BSDCSecImpRatios.push_back(totalBSDCSecImpRatio/norm);

		cout << "minOPTTime = " << minOPTTime << endl;
		cout << "maxOPTTime = " << maxOPTTime << endl;

		cout << "minBSTime = " << minBSTime << endl;
		cout << "maxBSTime = " << maxBSTime << endl;

		cout << "minOPTDCTime = " << minOPTDCTime << endl;
		cout << "maxOPTDCTime = " << maxOPTDCTime << endl;

		cout << "minBSDCTime = " << minBSDCTime << endl;
		cout << "maxBSDCTime = " << maxBSDCTime << endl;

		cout << "minBSDCSecTime = " << minBSDCSecTime << endl;
		cout << "maxBSDCSecTime = " << maxBSDCSecTime << endl;

		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedOPTPerfs", normalizedOPTPerfs);
		Utility::output_one_vector(cout, "normalizedOPTTimes", normalizedOPTTimes);

		Utility::output_one_vector(cout, "normalizedBSPerfs", normalizedBSPerfs);
		Utility::output_one_vector(cout, "normalizedBSTimes", normalizedBSTimes);

		Utility::output_one_vector(cout, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
		Utility::output_one_vector(cout, "normalizedOPTDCTimes", normalizedOPTDCTimes);

		Utility::output_one_vector(cout, "normalizedBSDCPerfs", normalizedBSDCPerfs);
		Utility::output_one_vector(cout, "normalizedBSDCTimes", normalizedBSDCTimes);

		Utility::output_one_vector(cout, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
		Utility::output_one_vector(cout, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

		Utility::output_one_vector(cout, "OPTDCImpRatios", OPTDCImpRatios);
		Utility::output_one_vector(cout, "BSDCImpRatios", BSDCImpRatios);
		Utility::output_one_vector(cout, "BSDCSecImpRatios", BSDCSecImpRatios);

		Utility::output_one_vector(foutResultCollection, "normalizedOPTPerfs", normalizedOPTPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedOPTTimes", normalizedOPTTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedBSPerfs", normalizedBSPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedBSTimes", normalizedBSTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedOPTDCTimes", normalizedOPTDCTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedBSDCPerfs", normalizedBSDCPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedBSDCTimes", normalizedBSDCTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

		Utility::output_one_vector(foutResultCollection, "OPTDCImpRatios", OPTDCImpRatios);
		Utility::output_one_vector(foutResultCollection, "BSDCImpRatios", BSDCImpRatios);
		Utility::output_one_vector(foutResultCollection, "BSDCSecImpRatios", BSDCSecImpRatios);
	}

	cout << "minOPTTime = " << minOPTTime << endl;
	cout << "maxOPTTime = " << maxOPTTime << endl;

	cout << "minBSTime = " << minBSTime << endl;
	cout << "maxBSTime = " << maxBSTime << endl;

	cout << "minOPTDCTime = " << minOPTDCTime << endl;
	cout << "maxOPTDCTime = " << maxOPTDCTime << endl;

	cout << "minBSDCTime = " << minBSDCTime << endl;
	cout << "maxBSDCTime = " << maxBSDCTime << endl;

	cout << "minBSDCSecTime = " << minBSDCSecTime << endl;
	cout << "maxBSDCSecTime = " << maxBSDCSecTime << endl;

	Utility::output_one_vector(cout, "normalizedOPTPerfs", normalizedOPTPerfs);
	Utility::output_one_vector(cout, "normalizedOPTTimes", normalizedOPTTimes);

	Utility::output_one_vector(cout, "normalizedBSPerfs", normalizedBSPerfs);
	Utility::output_one_vector(cout, "normalizedBSTimes", normalizedBSTimes);

	Utility::output_one_vector(cout, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
	Utility::output_one_vector(cout, "normalizedOPTDCTimes", normalizedOPTDCTimes);

	Utility::output_one_vector(cout, "normalizedBSDCPerfs", normalizedBSDCPerfs);
	Utility::output_one_vector(cout, "normalizedBSDCTimes", normalizedBSDCTimes);

	Utility::output_one_vector(cout, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
	Utility::output_one_vector(cout, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

	Utility::output_one_vector(cout, "OPTDCImpRatios", OPTDCImpRatios);
	Utility::output_one_vector(cout, "BSDCImpRatios", BSDCImpRatios);
	Utility::output_one_vector(cout, "BSDCSecImpRatios", BSDCSecImpRatios);

	Utility::output_one_vector(foutResultCollection, "normalizedOPTPerfs", normalizedOPTPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedOPTTimes", normalizedOPTTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedBSPerfs", normalizedBSPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedBSTimes", normalizedBSTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedOPTDCTimes", normalizedOPTDCTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedBSDCPerfs", normalizedBSDCPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedBSDCTimes", normalizedBSDCTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

	Utility::output_one_vector(foutResultCollection, "OPTDCImpRatios", OPTDCImpRatios);
	Utility::output_one_vector(foutResultCollection, "BSDCImpRatios", BSDCImpRatios);
	Utility::output_one_vector(foutResultCollection, "BSDCSecImpRatios", BSDCSecImpRatios);

	for (int i=0; i<speedsOfstreams.size(); i++) {
		(*speedsOfstreams[i]).close();
		delete speedsOfstreams[i];
	}
	foutResultCollection.close();
}

void RunTest::doOptimizationOverBiondiTestedSystemsExpConstantK(bool isMax, vector<bool> vecTest,string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, vector<double> k1, vector<double> k2) {
	// mkdir directory
	Utility::makeDir(resultDir);

	if (vecTest.size()!=5) {
		cerr << "Error vecTest size = "<<vecTest.size() << endl;
		exit(EXIT_FAILURE);
	}

	const char *dcFilePointer = dcFile.c_str();
#ifdef __USING_CONSTANT_SEPARATION__
	vector<map<int,int>> dcVector = FileReader::ReadDrivingCycleVectorWithConstantStep(dcFilePointer);
#else
	vector<map<int,int>> dcVector = FileReader::ReadDrivingCycleVector(dcFilePointer);
#endif

	//map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	map<int,int> dc;
	for (auto dcElement:dcVector) {
		for (auto e : dcElement) {
			if (dc.find(e.first)!=dc.end()) {
				dc[e.first] += e.second; 
			}
			else dc[e.first] = e.second;
		}
	}

	list<double> dcList = FileReader::ReadDrivingCycleToList(dcFilePointer);

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"ExpConstKUBSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultOPTSpeeds = resultDir + Utility::linkNotation()+"ExpConstKOPTSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultBSSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBSSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultOPTDCSpeeds = resultDir + Utility::linkNotation()+"ExpConstKOPTDCSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultBSDCSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBSDCSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";

	string resultCollection = resultDir + Utility::linkNotation()+"ExpConstKBSResultCollectionDC"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";

	vector<string> speedsFileNames;
	speedsFileNames.push_back(resultUBSpeeds);
	speedsFileNames.push_back(resultOPTSpeeds);
	speedsFileNames.push_back(resultBSSpeeds);
	speedsFileNames.push_back(resultOPTDCSpeeds);
	speedsFileNames.push_back(resultBSDCSpeeds);
	
	vector<ofstream*> speedsOfstreams;
	for (auto fileName:speedsFileNames) {
		ofstream* fout = new ofstream(fileName,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<fileName<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		speedsOfstreams.push_back(fout);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedUBPerfs;
	vector<double> normalizedUBTimes;

	vector<double> normalizedUBDPerfs;
	vector<double> normalizedUBDTimes;

	vector<double> normalizedOPTPerfs;
	vector<double> normalizedOPTTimes;

	vector<double> normalizedBSPerfs;
	vector<double> normalizedBSTimes;

	vector<double> normalizedOPTDCPerfs;
	vector<double> normalizedOPTDCTimes;

	vector<double> normalizedBSDCPerfs;
	vector<double> normalizedBSDCTimes;

	vector<double> normalizedBSDCSecPerfs;
	vector<double> normalizedBSDCSecTimes;

	vector<double> OPTDCImpRatios;
	vector<double> BSDCImpRatios;
	vector<double> BSDCSecImpRatios;

	double maxOPTTime = INT_MIN;
	double minOPTTime = INT_MAX;

	double maxBSTime = INT_MIN;
	double minBSTime = INT_MAX;

	double maxOPTDCTime = INT_MIN;
	double minOPTDCTime = INT_MAX;

	double maxBSDCTime = INT_MIN;
	double minBSDCTime = INT_MAX;

	double maxBSDCSecTime = INT_MIN;
	double minBSDCSecTime = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {

		// record factor
		for (int i=0; i<speedsOfstreams.size(); i++) {
			if (i==0) (*speedsOfstreams[i]) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
			else {
				if (vecTest[i-1]) (*speedsOfstreams[i]) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
			}
		}

		// record k1 and k2
		for (int i=0; i<speedsOfstreams.size(); i++) {
			if (i==0) {
				Utility::output_one_vector((*speedsOfstreams[i]), "k1",k1);
				Utility::output_one_vector((*speedsOfstreams[i]), "k2",k2);
			}
			else {
				if (vecTest[i-1]) {
					Utility::output_one_vector((*speedsOfstreams[i]), "k1",k1);
					Utility::output_one_vector((*speedsOfstreams[i]), "k2",k2);
				}
			}
		}

		double totalUBPerf = 0.0;
		double totalUBDPerf = 0.0;
		double totalOPTPerf = 0.0;
		double totalBSPerf = 0.0;
		double totalOPTDCPerf = 0.0;
		double totalBSDCPerf = 0.0;
		double totalBSDCSecPerf = 0.0;

		double totalUBTime = 0.0;
		double totalUBDTime = 0.0;
		double totalOPTTime = 0.0;
		double totalBSTime = 0.0;
		double totalOPTDCTime = 0.0;
		double totalBSDCTime = 0.0;
		double totalBSDCSecTime = 0.0;

		double totalOPTDCImpRatio = 0.0;
		double totalBSDCImpRatio = 0.0;
		double totalBSDCSecImpRatio = 0.0;
		
		int numUnFeasible = 0;

		int run = 0;
		int numSucc = 0;

		while (numSucc < numSystem) {
		//for (run=94; run <=94; run ++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> tasks;
			vector<int> WCETs;

			string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,tasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

#ifdef __USING_PERFORMANCE_FACTOR__
			//k1 = OptimizationAlgorithm::generatePerformanceFunctionsByWCET(WCETs);
			k1 = OptimizationAlgorithm::generatePerformanceFunctionsByWCETExponential(WCETs);
#endif

			bool bOPT = vecTest[0];
			bool bBS = vecTest[1];
			bool bOPTDC = vecTest[2];
			bool bBSDC = vecTest[3];
			bool bBSDCSec = vecTest[4];

			vector<double> UBSpeeds,OPTSpeeds,BSSpeeds, OPTDCSpeeds, BSDCSpeeds;
			vector<vector<double>> BSDCSecSpeedsVec;
			double pUB = 0.0, pUBd = 0.0, pOPT = 0.0, pBS = 0.0, pOPTDC = 0.0, pBSDC = 0.0, pBSDCSec = 0.0;
			double tUB = 0.0, tUBd = 0.0, tOPT = 0.0, tBS = 0.0, tOPTDC = 0.0, tBSDC = 0.0, tBSDCSec = 0.0;
			
			bool found = false;
			if (isMax) {
#if 0
				found = OptimizationAlgorithm::doAllOptimizationsExpMax(UBSpeeds,pUB,tUB,
				bOPT,OPTSpeeds,pOPT,tOPT,
				bBS, BSSpeeds,pBS,tBS,
				bOPTDC,OPTDCSpeeds,pOPTDC,tOPTDC,
				bBSDC,BSDCSpeeds,pBSDC,tBSDC,
				bBSDCSec,BSDCSecSpeedsVec,pBSDCSec,tBSDCSec,
				dc,dcVector,k1,k2,WCETs,tasks,engine,period);
#else
				found = OptimizationAlgorithm::doAllOptimizationsExpMax(UBSpeeds,pUB,tUB,pUBd,tUBd,
				bOPT,OPTSpeeds,pOPT,tOPT,
				bBS, BSSpeeds,pBS,tBS,
				bOPTDC,OPTDCSpeeds,pOPTDC,tOPTDC,
				bBSDC,BSDCSpeeds,pBSDC,tBSDC,
				bBSDCSec,BSDCSecSpeedsVec,pBSDCSec,tBSDCSec,
				dcList,dcVector,k1,k2,WCETs,tasks,engine,period);
#endif
			}
			else
				found = OptimizationAlgorithm::doAllOptimizationsExpMin(UBSpeeds,pUB,tUB,
				bOPT,OPTSpeeds,pOPT,tOPT,
				bBS, BSSpeeds,pBS,tBS,
				bOPTDC,OPTDCSpeeds,pOPTDC,tOPTDC,
				bBSDC,BSDCSpeeds,pBSDC,tBSDC,
				bBSDCSec,BSDCSecSpeedsVec,pBSDCSec,tBSDCSec,
				dc,dcVector,k1,k2,WCETs,tasks,engine,period);
			
			if (found) numSucc++;
			else {
				numUnFeasible++;
				continue;
			}

			vector<vector<double>> allSpeeds;
			allSpeeds.push_back(UBSpeeds);
			allSpeeds.push_back(OPTSpeeds);
			allSpeeds.push_back(BSSpeeds);
			allSpeeds.push_back(OPTDCSpeeds);
			allSpeeds.push_back(BSDCSpeeds);

			minOPTTime = min(minOPTTime, tOPT);
			maxOPTTime = max(maxOPTTime, tOPT);

			minBSTime = min(minBSTime, tBS);
			maxBSTime = max(maxBSTime, tBS);

			minOPTDCTime = min(minOPTDCTime, tOPTDC);
			maxOPTDCTime = max(maxOPTDCTime, tOPTDC);

			minBSDCTime = min(minBSDCTime, tBSDC);
			maxBSDCTime = max(maxBSDCTime, tBSDC);

			minBSDCSecTime = min(minBSDCSecTime, tBSDCSec);
			maxBSDCSecTime = max(maxBSDCSecTime, tBSDCSec);

			// record speed results
			for (int i=0; i<speedsOfstreams.size(); i++) {
				if (i==0) {
					Utility::output_one_vector((*speedsOfstreams[i]),"Max Speeds", allSpeeds[i]);
					//Utility::output_one_vector(cout,"Max Speeds", allSpeeds[i]);
				}
				else {
					if (vecTest[i-1]) {
						Utility::output_one_vector((*speedsOfstreams[i]),"Result Speeds", allSpeeds[i]);
						//Utility::output_one_vector(cout,"Result Speeds", allSpeeds[i]);
					}
				}
			}

			double OPTPerf = 0.0; 
			double BSPerf = 0.0; 
			double OPTDCPerf = 0.0; 
			double BSDCPerf = 0.0;
			double BSDCSecPerf = 0.0;

			double OPTDCImpRatio = 0.0;
			double BSDCImpRatio = 0.0;
			double BSDCSecImpRatio = 0.0;

			if (isMax) {
				pUBd = pUBd/pUB;
				OPTPerf = pOPT/pUB;
				BSPerf = pBS/pUB;
				OPTDCPerf = pOPTDC/pUB;
				BSDCPerf = pBSDC/pUB;
				BSDCSecPerf = pBSDCSec/pUB;

				OPTDCImpRatio = (pOPTDC-pOPT)/pOPT;
				BSDCImpRatio = (pBSDC-pBS)/pBS;
				BSDCSecImpRatio = (pBSDCSec-pBS)/pBS;
			} 
			else {
				pUBd = pUB/pUBd;
				OPTPerf = pUB/pOPT;
				BSPerf = pUB/pBS;
				OPTDCPerf = pUB/pOPTDC;
				BSDCPerf = pUB/pBSDC;
				BSDCSecPerf = pUB/pBSDCSec;

				OPTDCImpRatio = (pOPT-pOPTDC)/pOPT;
				BSDCImpRatio = (pBS-pBSDC)/pBS;
				BSDCSecImpRatio = (pBS-pBSDCSec)/pBS;
			}

			totalUBPerf += pUB;
			totalUBDPerf += pUBd;
			totalOPTPerf += OPTPerf;
			totalBSPerf += BSPerf;
			totalOPTDCPerf += OPTDCPerf;
			totalBSDCPerf += BSDCPerf;
			totalBSDCSecPerf += BSDCSecPerf;

			totalUBTime += tUB;
			totalUBDTime += tUBd;
			totalOPTTime += tOPT;
			totalBSTime += tBS;
			totalOPTDCTime += tOPTDC;
			totalBSDCTime += tBSDC;
			totalBSDCSecTime += tBSDCSec;

			totalOPTDCImpRatio += OPTDCImpRatio;
			totalBSDCImpRatio += BSDCImpRatio;
			totalBSDCSecImpRatio += BSDCSecImpRatio;

			cout << " Perf = " << pUB << " " << pUBd << " " << OPTPerf << " " << BSPerf << " " << OPTDCPerf << " " << BSDCPerf << " " << BSDCSecPerf 
				<< ", Time = " << tUB << " " << tUBd << " " << tOPT << " " << tBS << " " << tOPTDC << " " << tBSDC << " " << tBSDCSec
				<< ", Ratio = " << OPTDCImpRatio << " " << BSDCImpRatio << " " << BSDCSecImpRatio;

			foutResultCollection << " Perf = " << pUB << " " << pUBd << " " << OPTPerf << " " << BSPerf << " " << OPTDCPerf << " " << BSDCPerf << " " << BSDCSecPerf 
				<< ", Time = " << tUB << " " << tUBd << " " << tOPT << " " << tBS << " " << tOPTDC << " " << tBSDC << " " << tBSDCSec
				<< ", Ratio = " << OPTDCImpRatio << " " << BSDCImpRatio << " " << BSDCSecImpRatio;

			if (OPTDCPerf-OPTPerf > 0.05) {
				cout << " ++++";
				foutResultCollection << " ++++";
			}

			if (BSDCPerf-BSPerf > 0.05) {
				cout << " ####";
				foutResultCollection << " ####";
			}

			if (OPTDCPerf-BSDCPerf > 0.05) {
				cout << " $$$$";
				foutResultCollection << " $$$$";
			}

			cout << endl;
			foutResultCollection << endl;
		}

		int norm = numSystem;
		normalizedUBPerfs.push_back(totalUBPerf/norm);
		normalizedUBDPerfs.push_back(totalUBDPerf/norm);
		normalizedOPTPerfs.push_back(totalOPTPerf/norm);
		normalizedBSPerfs.push_back(totalBSPerf/norm);
		normalizedOPTDCPerfs.push_back(totalOPTDCPerf/norm);
		normalizedBSDCPerfs.push_back(totalBSDCPerf/norm);
		normalizedBSDCSecPerfs.push_back(totalBSDCSecPerf/norm);

		normalizedUBTimes.push_back(totalUBTime/norm);
		normalizedUBDTimes.push_back(totalUBDTime/norm);
		normalizedOPTTimes.push_back(totalOPTTime/norm);
		normalizedBSTimes.push_back(totalBSTime/norm);
		normalizedOPTDCTimes.push_back(totalOPTDCTime/norm);
		normalizedBSDCTimes.push_back(totalBSDCTime/norm);
		normalizedBSDCSecTimes.push_back(totalBSDCSecTime/norm);

		OPTDCImpRatios.push_back(totalOPTDCImpRatio/norm);
		BSDCImpRatios.push_back(totalBSDCImpRatio/norm);
		BSDCSecImpRatios.push_back(totalBSDCSecImpRatio/norm);
		
		cout << "minOPTTime = " << minOPTTime << endl;
		cout << "maxOPTTime = " << maxOPTTime << endl;

		cout << "minBSTime = " << minBSTime << endl;
		cout << "maxBSTime = " << maxBSTime << endl;

		cout << "minOPTDCTime = " << minOPTDCTime << endl;
		cout << "maxOPTDCTime = " << maxOPTDCTime << endl;

		cout << "minBSDCTime = " << minBSDCTime << endl;
		cout << "maxBSDCTime = " << maxBSDCTime << endl;

		cout << "minBSDCSecTime = " << minBSDCSecTime << endl;
		cout << "maxBSDCSecTime = " << maxBSDCSecTime << endl;

		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedUBPerfs", normalizedUBPerfs);
		Utility::output_one_vector(cout, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(cout, "normalizedUBDPerfs", normalizedUBDPerfs);
		Utility::output_one_vector(cout, "normalizedUBDTimes", normalizedUBDTimes);

		Utility::output_one_vector(cout, "normalizedOPTPerfs", normalizedOPTPerfs);
		Utility::output_one_vector(cout, "normalizedOPTTimes", normalizedOPTTimes);

		Utility::output_one_vector(cout, "normalizedBSPerfs", normalizedBSPerfs);
		Utility::output_one_vector(cout, "normalizedBSTimes", normalizedBSTimes);

		Utility::output_one_vector(cout, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
		Utility::output_one_vector(cout, "normalizedOPTDCTimes", normalizedOPTDCTimes);

		Utility::output_one_vector(cout, "normalizedBSDCPerfs", normalizedBSDCPerfs);
		Utility::output_one_vector(cout, "normalizedBSDCTimes", normalizedBSDCTimes);

		Utility::output_one_vector(cout, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
		Utility::output_one_vector(cout, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

		Utility::output_one_vector(cout, "OPTDCImpRatios", OPTDCImpRatios);
		Utility::output_one_vector(cout, "BSDCImpRatios", BSDCImpRatios);
		Utility::output_one_vector(cout, "BSDCSecImpRatios", BSDCSecImpRatios);

		Utility::output_one_vector(foutResultCollection, "normalizedUBPerfs", normalizedUBPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBDPerfs", normalizedUBDPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedUBDTimes", normalizedUBDTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedOPTPerfs", normalizedOPTPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedOPTTimes", normalizedOPTTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedBSPerfs", normalizedBSPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedBSTimes", normalizedBSTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedOPTDCTimes", normalizedOPTDCTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedBSDCPerfs", normalizedBSDCPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedBSDCTimes", normalizedBSDCTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

		Utility::output_one_vector(foutResultCollection, "OPTDCImpRatios", OPTDCImpRatios);
		Utility::output_one_vector(foutResultCollection, "BSDCImpRatios", BSDCImpRatios);
		Utility::output_one_vector(foutResultCollection, "BSDCSecImpRatios", BSDCSecImpRatios);
	}

	cout << "minOPTTime = " << minOPTTime << endl;
	cout << "maxOPTTime = " << maxOPTTime << endl;

	cout << "minBSTime = " << minBSTime << endl;
	cout << "maxBSTime = " << maxBSTime << endl;

	cout << "minOPTDCTime = " << minOPTDCTime << endl;
	cout << "maxOPTDCTime = " << maxOPTDCTime << endl;

	cout << "minBSDCTime = " << minBSDCTime << endl;
	cout << "maxBSDCTime = " << maxBSDCTime << endl;

	cout << "minBSDCSecTime = " << minBSDCSecTime << endl;
	cout << "maxBSDCSecTime = " << maxBSDCSecTime << endl;

	Utility::output_one_vector(cout, "normalizedUBPerfs", normalizedUBPerfs);
	Utility::output_one_vector(cout, "normalizedUBTimes", normalizedUBTimes);

	Utility::output_one_vector(cout, "normalizedUBDPerfs", normalizedUBDPerfs);
	Utility::output_one_vector(cout, "normalizedUBDTimes", normalizedUBDTimes);

	Utility::output_one_vector(cout, "normalizedOPTPerfs", normalizedOPTPerfs);
	Utility::output_one_vector(cout, "normalizedOPTTimes", normalizedOPTTimes);

	Utility::output_one_vector(cout, "normalizedBSPerfs", normalizedBSPerfs);
	Utility::output_one_vector(cout, "normalizedBSTimes", normalizedBSTimes);

	Utility::output_one_vector(cout, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
	Utility::output_one_vector(cout, "normalizedOPTDCTimes", normalizedOPTDCTimes);

	Utility::output_one_vector(cout, "normalizedBSDCPerfs", normalizedBSDCPerfs);
	Utility::output_one_vector(cout, "normalizedBSDCTimes", normalizedBSDCTimes);

	Utility::output_one_vector(cout, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
	Utility::output_one_vector(cout, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

	Utility::output_one_vector(cout, "OPTDCImpRatios", OPTDCImpRatios);
	Utility::output_one_vector(cout, "BSDCImpRatios", BSDCImpRatios);
	Utility::output_one_vector(cout, "BSDCSecImpRatios", BSDCSecImpRatios);

	Utility::output_one_vector(foutResultCollection, "normalizedUBPerfs", normalizedUBPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedUBTimes", normalizedUBTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedUBDPerfs", normalizedUBDPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedUBDTimes", normalizedUBDTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedOPTPerfs", normalizedOPTPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedOPTTimes", normalizedOPTTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedBSPerfs", normalizedBSPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedBSTimes", normalizedBSTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedOPTDCPerfs", normalizedOPTDCPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedOPTDCTimes", normalizedOPTDCTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedBSDCPerfs", normalizedBSDCPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedBSDCTimes", normalizedBSDCTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecPerfs", normalizedBSDCSecPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedBSDCSecTimes", normalizedBSDCSecTimes);

	Utility::output_one_vector(foutResultCollection, "OPTDCImpRatios", OPTDCImpRatios);
	Utility::output_one_vector(foutResultCollection, "BSDCImpRatios", BSDCImpRatios);
	Utility::output_one_vector(foutResultCollection, "BSDCSecImpRatios", BSDCSecImpRatios);

	for (int i=0; i<speedsOfstreams.size(); i++) {
		(*speedsOfstreams[i]).close();
		delete speedsOfstreams[i];
	}
	foutResultCollection.close();
}

void RunTest::doOptimizationZeroOffsetExpConstantK(string resultDir, string resultDAVRDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, vector<double> k1, vector<double> k2) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultDAVRDir = resultDir + Utility::linkNotation() + resultDAVRDir;
	Utility::makeDir(resultDAVRDir);

	const char *dcFilePointer = dcFile.c_str();
#ifdef __USING_CONSTANT_SEPARATION__
	vector<map<int,int>> dcVector = FileReader::ReadDrivingCycleVectorWithConstantStep(dcFilePointer);
#else
	vector<map<int,int>> dcVector = FileReader::ReadDrivingCycleVector(dcFilePointer);
#endif

	//map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	map<int,int> dc;
	for (auto dcElement:dcVector) {
		for (auto e : dcElement) {
			if (dc.find(e.first)!=dc.end()) {
				dc[e.first] += e.second; 
			}
			else dc[e.first] = e.second;
		}
	}

	list<double> dcList = FileReader::ReadDrivingCycleToList(dcFilePointer);

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"ExpConstKUBSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";
	string resultSAVRSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBSSpeedsUtil"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";

	string resultCollection = resultDir + Utility::linkNotation()+"ExpConstKBSResultCollectionDC"+Utility::int_to_string(tUtil)+"suffix"+suffix+".result";

	vector<string> speedsFileNames;
	speedsFileNames.push_back(resultUBSpeeds);
	speedsFileNames.push_back(resultSAVRSpeeds);

	vector<ofstream*> speedsOfstreams;
	for (auto fileName:speedsFileNames) {
		ofstream* fout = new ofstream(fileName,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<fileName<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		speedsOfstreams.push_back(fout);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedUBTimes;

	vector<double> normalizedSAVRPerfs;
	vector<double> normalizedSAVRTimes;

	vector<double> normalizedDAVRPerfs;
	vector<double> normalizedDAVRTimes;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		string resultDAVRFactorDir = resultDAVRDir + Utility::linkNotation()+"Factor" + Utility::int_to_string(factor);
		Utility::makeDir(resultDAVRFactorDir);

		// record factor, k1 and k2
		for (int i=0; i<speedsOfstreams.size(); i++) {
			 (*speedsOfstreams[i]) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
			 Utility::output_one_vector((*speedsOfstreams[i]), "k1",k1);
			 Utility::output_one_vector((*speedsOfstreams[i]), "k2",k2);
		}

		double totalSAVRPerf = 0.0;
		double totalDAVRPerf = 0.0;

		double totalUBTime = 0.0;
		double totalSAVRTime = 0.0;
		double totalDAVRTime = 0.0;

		int numUnFeasible = 0;

		int run = 0;
		int numSucc = 0;

		while (numSucc < numSystem) {
			//for (run=94; run <=94; run ++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> tasks;
			vector<int> WCETs;

			string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,tasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

			/// Perform the optimization procedures
			vector<double> UBSpeeds, SAVRSpeeds;

			double pUB = 0.0, pSAVR = 0.0, pDAVR = 0.0;
			double tUB = 0.0, tSAVR = 0.0, tDAVR = 0.0;

			bool found = false;

			/// Calculate the upper bound speeds
			timer.start();
			UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
			OptimizationAlgorithm::enforceModeGuards(UBSpeeds);
			timer.end();

			if (UBSpeeds.empty()) {
				found = false;
				goto CHECK_SUCCESS;
			}
			
			pUB = OptimizationAlgorithm::getPerformanceDCExp(dc,UBSpeeds, k1, k2);
			tUB = timer.getTime();

			/// Calculate the transition speeds for the static AVR task with Biondi's BS algorithm
			timer.start();
			SAVRSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, tasks, engine, period);
			timer.end();

			if (SAVRSpeeds.empty()) {
				found = false;
				goto CHECK_SUCCESS;
			}

			pSAVR = OptimizationAlgorithm::getPerformanceDCExp(dc,SAVRSpeeds, k1, k2);
			tSAVR = timer.getTime();

			{
				string resultDAVR = resultDAVRFactorDir + Utility::linkNotation()+"Run" + Utility::int_to_string(run) + ".result";
				ofstream foutDAVR(resultDAVR,ios::out | ios::trunc);
				if (!foutDAVR.is_open()) {
					cerr << "Can't open "<<resultDAVR<<" file for output" <<endl;
					exit(EXIT_FAILURE);
				}

				foutDAVR << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
				Utility::output_one_vector(foutDAVR, "k1",k1);
				Utility::output_one_vector(foutDAVR, "k2",k2);

				/// Calculate the transition speeds for the dynamic AVR task with our optimization algorithm for each point
				timer.start();
				pDAVR = OptimizationAlgorithm::computeUBPerfDC(foutDAVR,dcList,EXACT,WCETs,k1,k2,tasks,engine,period);
				timer.end();

				foutDAVR.close();
			}
			
			found = true;
CHECK_SUCCESS:
			if (found) numSucc++;
			else {
				numUnFeasible++;
				continue;
			}

			vector<vector<double>> allSpeeds;
			allSpeeds.push_back(UBSpeeds);
			allSpeeds.push_back(SAVRSpeeds);

			// record speed results
			for (int i=0; i<speedsOfstreams.size(); i++) {
				Utility::output_one_vector((*speedsOfstreams[i]),"Result Speeds", allSpeeds[i]);	
			}

			double pSAVRPerfRatio = pSAVR/pUB; 
			double pDAVRPerfRatio = pDAVR/pUB; 

			totalSAVRPerf += pSAVRPerfRatio;
			totalDAVRPerf += pDAVRPerfRatio;

			totalUBTime += tUB;
			totalSAVRTime += tSAVR;
			totalDAVRTime += tDAVR;

			cout << " Perf = " << pUB << " " << pSAVRPerfRatio << " " << pDAVRPerfRatio 
				<< ", Time = " << tUB << " " << tSAVR << " " << tDAVR << endl;

			foutResultCollection << " Perf = " << pUB << " " << pSAVRPerfRatio << " " << pDAVRPerfRatio 
				<< ", Time = " << tUB << " " << tSAVR << " " << tDAVR << endl;
		}

		int norm = numSystem;
		
		normalizedSAVRPerfs.push_back(totalSAVRPerf/norm);
		normalizedDAVRPerfs.push_back(totalDAVRPerf/norm);

		normalizedUBTimes.push_back(totalUBTime/norm);
		normalizedSAVRTimes.push_back(totalSAVRTime/norm);
		normalizedDAVRTimes.push_back(totalDAVRTime/norm);

		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(cout, "normalizedSAVRPerfs", normalizedSAVRPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRTimes", normalizedSAVRTimes);

		Utility::output_one_vector(cout, "normalizedDAVRPerfs", normalizedDAVRPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRTimes", normalizedDAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRPerfs", normalizedSAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRTimes", normalizedSAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRPerfs", normalizedDAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRTimes", normalizedDAVRTimes);
	}

	for (int i=0; i<speedsOfstreams.size(); i++) {
		(*speedsOfstreams[i]).close();
		delete speedsOfstreams[i];
	}
	foutResultCollection.close();
}

void RunTest::doDCBBForBiondiTestedSystems(string resultDir, string inputDir, string dcFile, int tUtil, int minSystem, int maxSystem, int minFactor, int maxFactor, int numK, int sizeK) {
	// mkdir directory
	Utility::makeDir(resultDir);

	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	
	string resultBBSpeeds = resultDir + Utility::linkNotation()+"DCBBSpeedsUtil"+Utility::int_to_string(tUtil)+".result";
	string resultBBPerformance = resultDir + Utility::linkNotation()+"DCBBPerformanceUtil"+Utility::int_to_string(tUtil)+".result";
	string resultCollection = resultDir + Utility::linkNotation()+"DCResultCollection"+Utility::int_to_string(tUtil)+".result";

	ofstream foutBBSpeeds(resultBBSpeeds, ios::out | ios::trunc);
	if (!foutBBSpeeds.is_open()) {
		cerr << "Can't open "<<resultBBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBBPerformance(resultBBPerformance, ios::out | ios::trunc);
	if (!foutBBPerformance.is_open()) {
		cerr << "Can't open "<<resultBBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedPerfs;
	vector<double> normalizedTimes;

	double maxTime = INT_MIN;
	double minTime = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		foutBBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutBBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		double totalPerf = 0.0;
		double totalTime = 0.0;
		int numUnFeasible = 0;

		for (int i=0; i<numK; i++) {
			vector<double> k = TaskSystemGenerator::generate_coefficients(sizeK);
			Utility::output_one_vector(foutBBSpeeds, "k", k);
			Utility::output_one_vector(foutBBPerformance, "k", k);

			for (int run=minSystem; run <= maxSystem; run++) {
				cout << "factor = " << factor << ", No.k = " << i << ", run = " << run << " => ";
				vector<PeriodicTask> tasks;
				vector<int> WCETs;

				string name = inputDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
				const char *p = name.c_str();
				FileReader::ReadTaskSystem(p,tasks,WCETs);


				for (int i=0; i<WCETs.size(); i++)
					WCETs[i] *= factor;

				vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
				double pUB = OptimizationAlgorithm::getPerformanceDC(dc,maxSpeeds, k);

				Timer timer;
				timer.start();
				vector<double> OPTSpeeds = OptimizationAlgorithm::computeDrivingCycleBB(dc,EXACT,k,WCETs, tasks, engine, period);
				timer.end();

				if (OPTSpeeds.empty()) {
					numUnFeasible ++;
					continue;
				}

				totalTime += timer.getTime();

				cout << " time = " << timer.getTime();

				minTime = min(minTime, timer.getTime());
				maxTime = max(maxTime, timer.getTime());

				Utility::output_one_vector(foutBBSpeeds, "OPT Speeds", OPTSpeeds);
				double pOPT = OptimizationAlgorithm::getPerformanceDC(dc,OPTSpeeds, k);
				foutBBPerformance << pOPT << endl;

				totalPerf += pOPT/pUB;
				cout << " normalizedPerf = " << pOPT/pUB << endl;
			}
		}

		int norm = (maxSystem-minSystem+1)*numK-numUnFeasible;
		normalizedPerfs.push_back(totalPerf/norm);
		normalizedTimes.push_back(totalTime/norm);

		cout << "minTime = " << minTime << endl;
		cout << "maxTime = " << maxTime << endl;
		cout << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);
	}

	cout << "minTime = " << minTime << endl;
	cout << "maxTime = " << maxTime << endl;
	
	Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

	foutResultCollection << "minTime = " << minTime << endl;
	foutResultCollection << "maxTime = " << maxTime << endl;

	foutBBSpeeds.close();
	foutBBPerformance.close();
	foutResultCollection.close();
}

void RunTest::computePerformanceDC(string inputDir, string dcName, string BiondiResultDir, string DCResultDir, int tUtil) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	string UBSpeedsFile = BiondiResultDir + Utility::linkNotation() + "UBSpeedsUtil"+Utility::int_to_string(75)+".result";
	string BBSpeedsFile = BiondiResultDir + Utility::linkNotation() + "BBSpeedsUtil"+Utility::int_to_string(75)+".result";
	string DCBBSpeedsFile = DCResultDir + Utility::linkNotation() + dcName + "results"+Utility::linkNotation() + "DCBBSpeedsUtil"+Utility::int_to_string(75)+".result";

	map<int,vector<double>> UBPerf = FileReader::ComputePerformanceResult(dc,UBSpeedsFile,"Max");
	map<int,vector<double>> BBPerf = FileReader::ComputePerformanceResult(dc,BBSpeedsFile,"OPT");
	map<int,vector<double>> DCBBPerf = FileReader::ComputePerformanceResult(dc,DCBBSpeedsFile,"OPT");

	vector<double> normalizedBiondi;
	vector<double> normalizedDC;

	for (auto iter : UBPerf) {
		int factor = iter.first;
		vector<double> pUB_vec = iter.second;
		vector<double> pBB_vec = BBPerf[factor];
		vector<double> pDCBB_vec = DCBBPerf[factor];

		double pBB = 0;
		double pDCBB = 0;

		int size = pUB_vec.size();
		for (int i=0; i<size; i++) {
			pBB += pBB_vec[i]/pUB_vec[i];
			pDCBB += pDCBB_vec[i]/pUB_vec[i];
		}

		normalizedBiondi.push_back(pBB/size);
		normalizedDC.push_back(pDCBB/size);
	}

	Utility::output_one_vector(cout,"BiondiBB", normalizedBiondi);
	Utility::output_one_vector(cout,"DCBB", normalizedDC);
}

void RunTest::computePerformanceDCExp(string inputDir, string dcName, string BiondiResultDir, string DCResultDir, int tUtil) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	string UBSpeedsFile = BiondiResultDir + Utility::linkNotation() + "UBSpeedsExpUtil"+Utility::int_to_string(75)+".result";
	string BBSpeedsFile = BiondiResultDir + Utility::linkNotation() + "BBSpeedsExpUtil"+Utility::int_to_string(75)+".result";
	string DCBBSpeedsFile = DCResultDir + Utility::linkNotation() + dcName + "results"+Utility::linkNotation() + "DCBBSpeedsExpUtil"+Utility::int_to_string(75)+".result";

	map<int,map<int,map<int,double>>> UBPerf = FileReader::ComputePerformanceExpResult(dc,UBSpeedsFile,"Max");
	map<int,map<int,map<int,double>>> BBPerf = FileReader::ComputePerformanceExpResult(dc,BBSpeedsFile,"OPT");
	map<int,map<int,map<int,double>>> DCBBPerf = FileReader::ComputePerformanceExpResult(dc,DCBBSpeedsFile,"OPT");

	vector<double> normalizedBiondi;
	vector<double> normalizedDC;

	for (auto iter : UBPerf) {
		int factor = iter.first;
		map<int,map<int,double>> pUB_map2 = iter.second;
		map<int,map<int,double>> pBB_map2 = BBPerf[factor];
		map<int,map<int,double>> pDCBB_map2 = DCBBPerf[factor];

		double pBB = 0;
		double pDCBB = 0;
		int sumNum = 0;

		for(auto iter1 : pUB_map2) {
			int kNum = iter1.first;
			if (pBB_map2.find(kNum) == pBB_map2.end() || pDCBB_map2.find(kNum) == pDCBB_map2.end())
				continue;
			map<int,double> pUB_map = iter1.second;
			map<int,double> pBB_map = pBB_map2[kNum];
			map<int,double> pDCBB_map = pDCBB_map2[kNum];

			for (auto iter2 : pUB_map) {
				int tasksetNum = iter2.first;
				if (pBB_map.find(tasksetNum) == pBB_map.end() || pDCBB_map.find(tasksetNum) == pDCBB_map.end())
					continue;
				double ratioA = pBB_map[tasksetNum] / iter2.second;
				double ratioB = pDCBB_map[tasksetNum] / iter2.second;
				/*
				if (fabs(ratioA-ratioB) > 0.001) {
					cout << "================" << endl;
					cout << "RatioA = " << ratioA << " RatioB = " << ratioB << endl; 
				}
				*/
				pBB += ratioA;
				pDCBB += ratioB;
				sumNum ++;
			}
		}

		normalizedBiondi.push_back(pBB/sumNum);
		normalizedDC.push_back(pDCBB/sumNum);
	}

	Utility::output_one_vector(cout,"BiondiBB"+dcName, normalizedBiondi);
	Utility::output_one_vector(cout,"DCBB"+dcName, normalizedDC);
}

void RunTest::computePerformanceTwoDCsExp(string inputDir, string dcAFileName, string dcBFileName, string BiondiResultDir, string DCResultDir, int tUtil) {
	string dcFileB = inputDir + Utility::linkNotation() + dcBFileName + ".info";
	const char *dcFilePointerB = dcFileB.c_str();
	map<int,int> dcB = FileReader::ReadDrivingCycle(dcFilePointerB);
	
	string UBSpeedsFile = BiondiResultDir + Utility::linkNotation() + "UBSpeedsExpUtil"+Utility::int_to_string(75)+".result";
	string DCBBSpeedsFileA = DCResultDir + Utility::linkNotation() + dcAFileName + "results"+Utility::linkNotation() + "DCBBSpeedsExpUtil"+Utility::int_to_string(75)+".result";
	string DCBBSpeedsFileB = DCResultDir + Utility::linkNotation() + dcBFileName + "results"+Utility::linkNotation() + "DCBBSpeedsExpUtil"+Utility::int_to_string(75)+".result";

	map<int,map<int,map<int,double>>> UBPerf = FileReader::ComputePerformanceExpResult(dcB,UBSpeedsFile,"Max");
	map<int,map<int,map<int,double>>> DCBBPerfA = FileReader::ComputePerformanceExpResult(dcB,DCBBSpeedsFileA,"OPT");
	map<int,map<int,map<int,double>>> DCBBPerfB = FileReader::ComputePerformanceExpResult(dcB,DCBBSpeedsFileB,"OPT");

	vector<double> normalizedDCA;
	vector<double> normalizedDCB;

	vector<double> results;

	for (auto iter : UBPerf) {
		int factor = iter.first;
		map<int,map<int,double>> pUB_map2 = iter.second;
		map<int,map<int,double>> pDCBBA_map2 = DCBBPerfA[factor];
		map<int,map<int,double>> pDCBBB_map2 = DCBBPerfB[factor];

		double pDCBBA = 0;
		double pDCBBB = 0;
		double perfRatio = 0;
		int sumNum = 0;
		int largeNum = 0;

		for(auto iter1 : pUB_map2) {
			int kNum = iter1.first;
			if (pDCBBA_map2.find(kNum) == pDCBBA_map2.end() || pDCBBB_map2.find(kNum) == pDCBBB_map2.end())
				continue;
			map<int,double> pUB_map = iter1.second;
			map<int,double> pDCBBA_map = pDCBBA_map2[kNum];
			map<int,double> pDCBBB_map = pDCBBB_map2[kNum];

			for (auto iter2 : pUB_map) {
				int tasksetNum = iter2.first;
				if (pDCBBA_map.find(tasksetNum) == pDCBBA_map.end() || pDCBBB_map.find(tasksetNum) == pDCBBB_map.end())
					continue;
				double ratioA = pDCBBA_map[tasksetNum] / iter2.second;
				double ratioB = pDCBBB_map[tasksetNum] / iter2.second;
				/*
				if (fabs(ratioA-ratioB) > 0.001) {
					cout << "================" << endl;
					cout << "RatioA = " << ratioA << " RatioB = " << ratioB << endl; 
				}
				*/
				pDCBBA += ratioA;
				pDCBBB += ratioB;
				double improvement = 1.0 - pDCBBA_map[tasksetNum]/pDCBBB_map[tasksetNum];
				if (improvement > 0.01) {
					largeNum++;
					//cout << "================" << endl;
					//cout << "Improvement Ratio = " << improvement << endl; 
				}
				perfRatio += improvement;
				sumNum ++;
			}
		}

		cout << "largeNum = " << largeNum << endl;

		normalizedDCA.push_back(pDCBBA/sumNum);
		normalizedDCB.push_back(pDCBBB/sumNum);
		results.push_back(perfRatio/sumNum);
	}

	/*
	for (int i=0; i<normalizedDCA.size(); i++) {
		results.push_back(normalizedDCB[i]-normalizedDCA[i]);
	}
	*/
	//Utility::output_one_vector(cout,"DCBBA"+dcAFileName, normalizedDCA);
	//Utility::output_one_vector(cout,"DCBBB"+dcBFileName, normalizedDCB);
	Utility::output_one_vector(cout,dcBFileName, results);
}

void RunTest::computePerformanceTwoDCsExpBS(string inputDir, string dcAFileName, string dcBFileName, string speedsResultDir, string dataOutputDir, int tUtil) {
	// mkdir directory
	Utility::makeDir(dataOutputDir);
	
	string dcFileB = inputDir + Utility::linkNotation() + dcBFileName + ".info";
	const char *dcFilePointerB = dcFileB.c_str();
	map<int,int> dcB = FileReader::ReadDrivingCycle(dcFilePointerB);

	string UBSpeedsFile = speedsResultDir + Utility::linkNotation() + "resultsExpConstk" + Utility::int_to_string(tUtil)  + Utility::linkNotation() + "ExpConstKUBSpeedsUtil"+Utility::int_to_string(tUtil)+"Factorf8.result";
	string DCBBSpeedsFileA = speedsResultDir + Utility::linkNotation() + "resultsExpConstk" + Utility::int_to_string(tUtil) + Utility::linkNotation() + "ExpConstKBBSpeedsUtil"+Utility::int_to_string(tUtil)+"Factorf8.result";
	//string DCBBSpeedsFileA = speedsResultDir + Utility::linkNotation() + "resultsBBExpConstk" + Utility::int_to_string(tUtil) + dcAFileName + Utility::linkNotation() + "ExpConstKBBSpeedsDCUtil"+Utility::int_to_string(tUtil) + "FactorfALL.result";
	string DCBBSpeedsFileB = speedsResultDir + Utility::linkNotation() + "resultsExpConstk" + Utility::int_to_string(tUtil) + dcBFileName + Utility::linkNotation() + "ExpConstKBBSpeedsDCUtil"+Utility::int_to_string(tUtil) + "Factorf8.result";

	/*
	string UBSpeedsFile = speedsResultDir + Utility::linkNotation() + "resultsBSExpConstk" + Utility::int_to_string(tUtil) + dcBFileName  + Utility::linkNotation() + "ExpConstKUBSpeedsDCUtil"+Utility::int_to_string(tUtil)+".result";
	string DCBBSpeedsFileA = speedsResultDir + Utility::linkNotation() + "resultsBSExpConstk" + Utility::int_to_string(tUtil) + dcAFileName+ Utility::linkNotation() + "ExpConstKBS2SpeedsDCUtil"+Utility::int_to_string(tUtil)+".result";
	string DCBBSpeedsFileB = speedsResultDir + Utility::linkNotation() + "resultsBSExpConstk" + Utility::int_to_string(tUtil) + dcBFileName+ Utility::linkNotation() + "ExpConstKBS2SpeedsDCUtil"+Utility::int_to_string(tUtil)+".result";
	*/

	/*
	string UBSpeedsFile = speedsResultDir + Utility::linkNotation() + "resultsBSExpRandomk" + Utility::int_to_string(tUtil) + dcBFileName  + Utility::linkNotation() + "ExpConstKUBSpeedsDCUtil"+Utility::int_to_string(tUtil)+".result";
	string DCBBSpeedsFileA = speedsResultDir + Utility::linkNotation() + "resultsBSExpRandomk" + Utility::int_to_string(tUtil) + dcAFileName+ Utility::linkNotation() + "ExpConstKBS2SpeedsDCUtil"+Utility::int_to_string(tUtil)+".result";
	string DCBBSpeedsFileB = speedsResultDir + Utility::linkNotation() + "resultsBSExpRandomk" + Utility::int_to_string(tUtil) + dcBFileName+ Utility::linkNotation() + "ExpConstKBS2SpeedsDCUtil"+Utility::int_to_string(tUtil)+".result";
	*/

	map<int,map<int,map<int,double>>> UBPerf = FileReader::ComputePerformanceExpResult(dcB,UBSpeedsFile,"Max");
	map<int,map<int,map<int,double>>> DCBBPerfA = FileReader::ComputePerformanceExpResult(dcB,DCBBSpeedsFileA,"BS2");
	map<int,map<int,map<int,double>>> DCBBPerfB = FileReader::ComputePerformanceExpResult(dcB,DCBBSpeedsFileB,"OPT");

	vector<double> normalizedDCA;
	vector<double> normalizedDCB;

	vector<double> resultsDiff;
	vector<double> resultsBiondi;
	vector<double> resultsDC;

	for (auto iter : UBPerf) {
		int factor = iter.first;
		map<int,map<int,double>> pUB_map2 = iter.second;
		map<int,map<int,double>> pDCBBA_map2 = DCBBPerfA[factor];
		map<int,map<int,double>> pDCBBB_map2 = DCBBPerfB[factor];

		double pDCBBA = 0;
		double pDCBBB = 0;
		double perfDiff = 0;
		
		int sumNum = 0;
		int largeNum = 0;

		string outputFileName = dataOutputDir + Utility::linkNotation() + dcBFileName + "Factor" + Utility::int_to_string(factor) + ".txt";
		ofstream fout(outputFileName, ios::out | ios::trunc);
		if (!fout.is_open()) {
			cerr << "Can't open " << outputFileName << " file for output" <<endl;
			exit(EXIT_FAILURE);
		}

		for(auto iter1 : pUB_map2) {
			int kNum = iter1.first;
			if (pDCBBA_map2.find(kNum) == pDCBBA_map2.end() || pDCBBB_map2.find(kNum) == pDCBBB_map2.end())
				continue;
			map<int,double> pUB_map = iter1.second;
			map<int,double> pDCBBA_map = pDCBBA_map2[kNum];
			map<int,double> pDCBBB_map = pDCBBB_map2[kNum];

			for (auto iter2 : pUB_map) {
				int tasksetNum = iter2.first;
				if (pDCBBA_map.find(tasksetNum) == pDCBBA_map.end() || pDCBBB_map.find(tasksetNum) == pDCBBB_map.end())
					continue;
				// MIN
				double ratioA = iter2.second / pDCBBA_map[tasksetNum];
				double ratioB = iter2.second / pDCBBB_map[tasksetNum];

				// MAX
				//double ratioA = pDCBBA_map[tasksetNum] / iter2.second;
				//double ratioB = pDCBBB_map[tasksetNum] / iter2.second;
				/*
				if (fabs(ratioA-ratioB) > 0.001) {
					cout << "================" << endl;
					cout << "RatioA = " << ratioA << " RatioB = " << ratioB << endl; 
				}
				*/

				//cout << "RatioA = " << ratioA << " RatioB = " << ratioB << endl; 

				pDCBBA += ratioA;
				pDCBBB += ratioB;
				double improvement = ratioB-ratioA;
				fout << improvement << endl;
				
				
				if (improvement > 0.05) {
					largeNum++;
					cout << "factor = " << factor << ", kNum = " << kNum << ", TaskSetNum = " << tasksetNum
						 << ", RatioA = " << ratioA << " RatioB = " << ratioB << endl; 
				}
				
				perfDiff += improvement;
				sumNum ++;
			}
		}

		//cout << "largeNum = " << largeNum << endl;
		fout.close();

		normalizedDCA.push_back(pDCBBA/sumNum);
		normalizedDCB.push_back(pDCBBB/sumNum);
		resultsDiff.push_back(perfDiff/sumNum);
	}

	/*
	for (int i=0; i<normalizedDCA.size(); i++) {
		results.push_back(normalizedDCB[i]-normalizedDCA[i]);
	}
	*/
	/*
	Utility::output_one_vector(cout,dcBFileName+"_Biondi", normalizedDCA);
	Utility::output_one_vector(cout,dcBFileName+"_DC", normalizedDCB);
	Utility::output_one_vector(cout,dcBFileName+"_Diff", resultsDiff);
	*/

	Utility::output_one_vector(cout,dcBFileName, resultsDiff);
	
	cout << endl;
}

void RunTest::testBiondiExampleExp(string inputDir, string dcName, vector<double> &results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	/*
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	*/
	//WCETs.push_back(s*727);
	//WCETs.push_back(s*842);
	//WCETs.push_back(s*999);
	
	
	WCETs.push_back(s*119);
	WCETs.push_back(s*419);
	WCETs.push_back(s*613);
	WCETs.push_back(s*727);
	WCETs.push_back(s*842);
	WCETs.push_back(s*999);
	

	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k1[] = {1,1,1,1,1,1};
	//static const double _k1[] = {10,7,5,4,3,2};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {0.0,0.2,0.3,0.4,0.5,0.7};
	//static const double _k2[] = {0.0,0.1,1,10,100,1000};
	//static const double _k2[] = {0.0,100,300,500,700,1000};
	//static const double _k2[] = {0,158,382,529,722,872};
	//static const double _k2[] = {0,0.873475*100,1.67477*100,1.9488*100,3.34087*100,3.35556*100};
	//static const double _k2[] = {0,0.873475,1.67477,1.9488,3.34087,3.35556};
	static const double _k2[] = {0,200,500,1000,1500,2000};

	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);

	cout << "Computing B&B...";
	Timer timer;
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExp(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);

	cout << "Computing BS...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeBS = timer.getTime();
	cout << "done." << endl;
	double pBS = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCBS = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pBS = " << pBS << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pOPTDC = " << pDCOPT << endl;
	cout << "pBSDC = " << pDCBS << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio BS/UB = " << pBS/pUB*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCOPT/pDCUB*100.0 << "%" << endl;
	cout << "Ratio DC BS/UB = " << pDCBS/pDCUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;
	cout << "timeBS = " << timeBS << endl;

	results.push_back(pDCBS/pDCUB);
}

void RunTest::testBiondiExampleDCExp(string inputDir, string dcName, vector<double>& results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	//WCETs.push_back(s*727);
	//WCETs.push_back(s*842);
	//WCETs.push_back(s*999);
	
	/*
	WCETs.push_back(s*119);
	WCETs.push_back(s*419);
	WCETs.push_back(s*613);
	WCETs.push_back(s*727);
	WCETs.push_back(s*842);
	WCETs.push_back(s*999);
	*/

	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k1[] = {1,1,1,1,1,1};
	//static const double _k1[] = {10,7,5,4,3,2};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {0.0,0.2,0.3,0.4,0.5,0.7};
	//static const double _k2[] = {0.0,0.1,1,10,100,1000};
	//static const double _k2[] = {0.0,100,300,500,700,1000};
	//static const double _k2[] = {0,158,382,529,722,872};
	//static const double _k2[] = {0,0.873475*100,1.67477*100,1.9488*100,3.34087*100,3.35556*100};
	//static const double _k2[] = {0,0.873475,1.67477,1.9488,3.34087,3.35556};
	static const double _k2[] = {0,200,500,1000,1500,2000};

	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	Utility::output_one_vector(cout,"k2",k2);

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);

	cout << "Computing B&B...";
	Timer timer;
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCExp(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);

	cout << "Computing BS...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug(dc,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeBS = timer.getTime();
	cout << "done." << endl;
	double pBS = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCBS = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	cout << "Computing BS2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeBS2 = timer.getTime();
	cout << "done." << endl;
	double pBS2 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	double pDCBS2 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds2, k1,k2);

	cout << "Computing BS NecessaryOnly...";
	timer.start();
	vector<double> BSSpeeds3 = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(dc,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeBS3 = timer.getTime();
	cout << "done." << endl;
	double pBS3 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds3, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds3", BSSpeeds3);

	double pDCBS3 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds3, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pBS = " << pBS << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pDCOPT = " << pDCOPT << endl;
	cout << "pDCBS = " << pDCBS << endl;
	cout << "pDCBS2 = " << pDCBS2 << endl;
	cout << "pDCBS3 = " << pDCBS3 << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio BS/UB = " << pBS/pUB*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCOPT/pDCUB*100.0 << "%" << endl;
	cout << "Ratio DC BS/UB = " << pDCBS/pDCUB*100.0 << "%" << endl;
	cout << "Ratio DC BS2/UB = " << pDCBS2/pDCUB*100.0 << "%" << endl;
	cout << "Ratio DC BS3/UB = " << pDCBS3/pDCUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;
	cout << "timeBS = " << timeBS << endl;
	cout << "timeBS2 = " << timeBS2 << endl;
	cout << "timeBS3 = " << timeBS3 << endl;

	results.push_back(pDCBS3/pDCUB);
}

void RunTest::testBiondiExampleEmissionExp(string inputDir, string dcName, vector<double> &results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	//PeriodicTask task0(1000,5000);
	//PeriodicTask task1(6500,20000);
	//PeriodicTask task2(10000,50000);
	//PeriodicTask task3(10000,100000);

	PeriodicTask task0(103,5000);
	PeriodicTask task1(2688,10000);
	PeriodicTask task2(2033,50000);
	PeriodicTask task3(33583,80000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	//WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	//WCETs.push_back(s*425);
	//WCETs.push_back(s*576);
	//WCETs.push_back(s*966);
	WCETs.push_back(s*174);
	WCETs.push_back(s*274);
	WCETs.push_back(s*389);
	WCETs.push_back(s*518);
	WCETs.push_back(s*754);
	WCETs.push_back(s*959);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	//static const double _k1[] = {1,1,1,1,1,1};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	//static const double _k2[] = {1000,700,500,300,100,0.0};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);

	Timer timer;

#ifdef __TEST_LOCAL_BB__
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);
#endif

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCH1 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug_Min(k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	double pDCH2 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds2, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPT = " << pOPT << endl;
#endif
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "pDCUB = " << pDCUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPTDC = " << pDCOPT << endl;
#endif
	cout << "pDCH1 = " << pDCH1 << endl;
	cout << "pDCH2 = " << pDCH2 << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
#endif
	cout << "Ratio H1/UB = " << pUB/pH1*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pUB/pH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio DC OPT/UB = " << pDCUB/pDCOPT*100.0 << "%" << endl;
#endif
	cout << "Ratio DC H1/UB = " << pDCUB/pDCH1*100.0 << "%" << endl;
	cout << "Ratio DC H2/UB = " << pDCUB/pDCH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "timeOPT = " << timeOPT << endl;
#endif
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
	cout << endl;

	results.push_back(pDCUB/pDCH1);
}

void RunTest::testBiondiExampleEmissionDCExp(string inputDir, string dcName, vector<double>& results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	//PeriodicTask task0(1000,5000);
	//PeriodicTask task1(6500,20000);
	//PeriodicTask task2(10000,50000);
	//PeriodicTask task3(10000,100000);

	PeriodicTask task0(103,5000);
	PeriodicTask task1(2688,10000);
	PeriodicTask task2(2033,50000);
	PeriodicTask task3(33583,80000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	//WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	//WCETs.push_back(s*425);
	//WCETs.push_back(s*576);
	//WCETs.push_back(s*966);
	WCETs.push_back(s*174);
	WCETs.push_back(s*274);
	WCETs.push_back(s*389);
	WCETs.push_back(s*518);
	WCETs.push_back(s*754);
	WCETs.push_back(s*959);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	//static const double _k1[] = {1,1,1,1,1,1};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	//static const double _k2[] = {1000,700,500,300,100,0.0};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);
	Timer timer;

#ifdef __TEST_LOCAL_BB__
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);
#endif

	cout << "Computing H1...";
	timer.start();
	//vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	vector<double> BSSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCH1 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPT = " << pOPT << endl;
#endif
	cout << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pDCH1 = " << pDCH1 << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPTDC = " << pDCOPT << endl;
#endif
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCUB/pDCOPT*100.0 << "%" << endl;
#endif
	cout << "Ratio DC H1/UB = " << pDCUB/pDCH1*100.0 << "%" << endl; 
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "timeOPT = " << timeOPT << endl;
#endif
	cout << "timeH1 = " << timeH1 << endl;
	cout << endl;

	results.push_back(pDCUB/pDCH1);
}

void RunTest::testBiondiExampleEmissionExp(string inputDir, string dcName, string systemDir, int tUtil, int factor, int run, vector<double> &results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	vector<PeriodicTask> tasks;
	vector<int> WCETs;

	string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
	const char *p = name.c_str();
	FileReader::ReadTaskSystem(p,tasks,WCETs);

	for (int i=0; i<WCETs.size(); i++)
		WCETs[i] *= factor;

	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	//static const double _k1[] = {1,1,1,1,1,1};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	//static const double _k2[] = {1000,700,500,300,100,0.0};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);

	Timer timer;

#ifdef __TEST_LOCAL_BB__
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);
#endif

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCH1 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	cout << "Computing H2...";
	timer.start();
	vector<double> BSSpeeds2 = OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug_Min(k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH2 = timer.getTime();
	cout << "done." << endl;
	double pH2 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds2, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", BSSpeeds2);

	double pDCH2 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds2, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPT = " << pOPT << endl;
#endif
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "pDCUB = " << pDCUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPTDC = " << pDCOPT << endl;
#endif
	cout << "pDCH1 = " << pDCH1 << endl;
	cout << "pDCH2 = " << pDCH2 << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
#endif
	cout << "Ratio H1/UB = " << pUB/pH1*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pUB/pH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio DC OPT/UB = " << pDCUB/pDCOPT*100.0 << "%" << endl;
#endif
	cout << "Ratio DC H1/UB = " << pDCUB/pDCH1*100.0 << "%" << endl;
	cout << "Ratio DC H2/UB = " << pDCUB/pDCH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "timeOPT = " << timeOPT << endl;
#endif
	cout << "timeH1 = " << timeH1 << endl;
	cout << "timeH2 = " << timeH2 << endl;
	cout << endl;

	results.push_back(pDCUB/pDCH1);
}

void RunTest::testBiondiExampleEmissionDCExp(string inputDir, string dcName, string systemDir, int tUtil, int factor, int run, vector<double>& results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	vector<PeriodicTask> tasks;
	vector<int> WCETs;

	string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
	const char *p = name.c_str();
	FileReader::ReadTaskSystem(p,tasks,WCETs);

	for (int i=0; i<WCETs.size(); i++)
		WCETs[i] *= factor;

	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	//static const double _k1[] = {1,1,1,1,1,1};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	//static const double _k2[] = {1000,700,500,300,100,0.0};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);
	Timer timer;

#ifdef __TEST_LOCAL_BB__
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);
#endif

	cout << "Computing H1...";
	timer.start();
	//vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	vector<double> BSSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,WCETs, tasks, engine, period);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCH1 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pH1 = " << pH1 << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPT = " << pOPT << endl;
#endif
	cout << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pDCH1 = " << pDCH1 << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPTDC = " << pDCOPT << endl;
#endif
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCUB/pDCOPT*100.0 << "%" << endl;
#endif
	cout << "Ratio DC H1/UB = " << pDCUB/pDCH1*100.0 << "%" << endl; 
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "timeOPT = " << timeOPT << endl;
#endif
	cout << "timeH1 = " << timeH1 << endl;
	cout << endl;

	results.push_back(pDCUB/pDCH1);
}

void RunTest::testBiondiExampleEmissionExpMapping(string inputDir, string dcName, vector<double> &results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(5000,95000,100000);
	PeriodicTask task4(5000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	//static const double _k1[] = {1,1,1,1,1,1};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	//static const double _k2[] = {1000,700,500,300,100,0.0};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);

	Timer timer;

#ifdef __TEST_LOCAL_BB__
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	CollectionResult cR0 = OptimizationAlgorithm::doOptimalAlgorithmDFSMappingAndBnBMin(k1,k2,WCETs, tasks, engine, period,4);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);

	double pOPT2 = OptimizationAlgorithm::getPerformanceExpGsl(cR0.speeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds2", cR0.speeds);

	double pDCOPT2 = OptimizationAlgorithm::getPerformanceDCExp(dc, cR0.speeds, k1,k2);
#endif

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,WCETs, tasks, engine, period);
	CollectionResult cR1 = OptimizationAlgorithm::doHeuristicAlgorithmBFSMappingAndBSMin(k1,k2,WCETs, tasks, engine, period,4);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCH1 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	double pH2 = OptimizationAlgorithm::getPerformanceExpGsl(cR1.speeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", cR1.speeds);

	double pDCH2 = OptimizationAlgorithm::getPerformanceDCExp(dc, cR1.speeds, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPT = " << pOPT << endl;
	cout << "pOPT2 = " << pOPT2 << endl;
#endif
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "pDCUB = " << pDCUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPTDC = " << pDCOPT << endl;
	cout << "pOPTDC2 = " << pDCOPT2 << endl;
#endif
	cout << "pDCH1 = " << pDCH1 << endl;
	cout << "pDCH2 = " << pDCH2 << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << "Ratio OPT2/UB = " << pUB/pOPT2*100.0 << "%" << endl;
#endif
	cout << "Ratio H1/UB = " << pUB/pH1*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pUB/pH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio DC OPT/UB = " << pDCUB/pDCOPT*100.0 << "%" << endl;
	cout << "Ratio DC OPT2/UB = " << pDCUB/pDCOPT2*100.0 << "%" << endl;
#endif
	cout << "Ratio DC H1/UB = " << pDCUB/pDCH1*100.0 << "%" << endl;
	cout << "Ratio DC H2/UB = " << pDCUB/pDCH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "timeOPT = " << timeOPT << endl;
#endif
	cout << "timeH1 = " << timeH1 << endl;
	cout << endl;

	results.push_back(pDCUB/pDCH1);
}

void RunTest::testBiondiExampleEmissionDCExpMapping(string inputDir, string dcName, vector<double>& results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);

	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(5000,95000,100000);
	PeriodicTask task4(5000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);
	tasks.push_back(task4);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	WCETs.push_back(s*278);
	WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	//static const double _k1[] = {6877,9853,1.19E+04};
	//static const double _k1[] = {1.19E+04,9853,6877};
	//static const double _k1[] = {-6877,-9853,-1.19E+04};
	//static const double _k1[] = {1,1,1,1,1,1};
	static const double _k1[] = {6877,9853,1.19E+04,1.3E+04,1.4E+04,1.5E+04};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	//static const double _k2[] = {5763,5621,5686};
	//static const double _k2[] = {5686,5621,5763};
	static const double _k2[] = {5763,5621,5686,5600,5500,5400};
	//static const double _k2[] = {1000,700,500,300,100,0.0};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformanceExpGsl(maxSpeeds, k1,k2,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCExp(dc, maxSpeeds, k1,k2);
	Timer timer;

#ifdef __TEST_LOCAL_BB__
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	CollectionResult cR0 = OptimizationAlgorithm::doOptimalAlgorithmDFSMappingAndBnBMin(dc,k1,k2,WCETs, tasks, engine, period,4);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformanceExpGsl(OPTSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCExp(dc, OPTSpeeds, k1,k2);

	double pOPT2 = OptimizationAlgorithm::getPerformanceExpGsl(cR0.speeds, k1,k2, engine);
	Utility::output_one_vector(cout, "OPT Speeds2", cR0.speeds);

	double pDCOPT2 = OptimizationAlgorithm::getPerformanceDCExp(dc, cR0.speeds, k1,k2);
#endif

	cout << "Computing H1...";
	timer.start();
	vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBSDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
	CollectionResult cR = OptimizationAlgorithm::doHeuristicAlgorithmBFSMappingAndBSMin(dc,k1,k2,WCETs, tasks, engine, period,4);
	timer.end();
	double timeH1 = timer.getTime();
	cout << "done." << endl;
	
	double pH1 = OptimizationAlgorithm::getPerformanceExpGsl(BSSpeeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds", BSSpeeds);

	double pDCH1 = OptimizationAlgorithm::getPerformanceDCExp(dc, BSSpeeds, k1,k2);

	double pH2 = OptimizationAlgorithm::getPerformanceExpGsl(cR.speeds, k1,k2, engine);
	Utility::output_one_vector(cout, "BS Speeds2", cR.speeds);

	double pDCH2 = OptimizationAlgorithm::getPerformanceDCExp(dc, cR.speeds, k1,k2);

	cout << endl;
	cout << "pUB = " << pUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPT = " << pOPT << endl;
	cout << "pOPT2 = " << pOPT2 << endl;
#endif
	cout << "pH1 = " << pH1 << endl;
	cout << "pH2 = " << pH2 << endl;
	cout << endl;
	cout << "pDCUB = " << pDCUB << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "pOPTDC = " << pDCOPT << endl;
	cout << "pOPTDC2 = " << pDCOPT2 << endl;
#endif
	cout << "pDCH1 = " << pDCH1 << endl;
	cout << "pDCH2 = " << pDCH2 << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << "Ratio OPT2/UB = " << pUB/pOPT2*100.0 << "%" << endl;
#endif
	cout << "Ratio H1/UB = " << pUB/pH1*100.0 << "%" << endl;
	cout << "Ratio H2/UB = " << pUB/pH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "Ratio DC OPT/UB = " << pDCUB/pDCOPT*100.0 << "%" << endl;
	cout << "Ratio DC OPT2/UB = " << pDCUB/pDCOPT2*100.0 << "%" << endl;
#endif
	cout << "Ratio DC H1/UB = " << pDCUB/pDCH1*100.0 << "%" << endl;
	cout << "Ratio DC H2/UB = " << pDCUB/pDCH2*100.0 << "%" << endl;
	cout << endl;
#ifdef __TEST_LOCAL_BB__
	cout << "timeOPT = " << timeOPT << endl;
#endif
	cout << "timeH1 = " << timeH1 << endl;
	cout << endl;

	results.push_back(pDCUB/pDCH1);
}


void RunTest::testBiondiExamplePoly(string inputDir, string dcName, vector<double> &results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	//int rpm_max = 2650;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	//WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

#if 0 // based on the real data
	static const double _k1[] = {-8.743E-07, -7.193E-07 ,-6.972E-07};
	static const double _k2[] = {0.01155, 0.0111 ,0.01106};
	static const double _k3[] = {-8.502, -7.659 ,-7.651};
#elif 0 // based on the simulation data
	static const double _k1[] = {-1.952e-06, -1.864e-06 ,-1.843e-06};
	static const double _k2[] = {0.02035, 0.02018 ,0.02013};
	static const double _k3[] = {-8.502, 0.07438 ,0.1205};
#else // based on the simulation data, 2016.11.29
	static const double _k1[] = { -0.0001378, -6.122e-05 ,-6.548e-05  };
	static const double _k2[] = {7.04, 6.647 ,6.677};
	static const double _k3[] = {-1068, -598.9 , -605.1};
#endif

	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));
	vector<double> k3 (_k3,_k3+sizeof(_k3)/sizeof(_k3[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformancePoly(maxSpeeds, k1,k2,k3,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCPoly3(dc, maxSpeeds, k1,k2,k3);

	cout << "Computing B&B...";
	time_t start = time(0);
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBPoly3(EXACT,k1,k2,k3,WCETs, tasks, engine, period);
	double timeOPT = difftime(time(0),start);
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformancePoly(OPTSpeeds, k1,k2,k3, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCPoly3(dc, OPTSpeeds, k1,k2,k3);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pOPTDC = " << pDCOPT << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCOPT/pDCUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;

	results.push_back(pDCOPT/pDCUB);
}

void RunTest::testBiondiExampleDCPoly(string inputDir, string dcName, vector<double>& results) {
	string dcFile = inputDir + Utility::linkNotation() + dcName + ".info";
	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);
	
	// AVR task
	int rpm_min = 500;
	int rpm_max = 6500;
	//int rpm_max = 2650;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	//WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

#if 0 // based on the real data
	static const double _k1[] = {-8.743E-07, -7.193E-07 ,-6.972E-07};
	static const double _k2[] = {0.01155, 0.0111 ,0.01106};
	static const double _k3[] = {-8.502, -7.659 ,-7.651};
#elif 0 // based on the simulation data
	static const double _k1[] = {-1.952e-06, -1.864e-06 ,-1.843e-06};
	static const double _k2[] = {0.02035, 0.02018 ,0.02013};
	static const double _k3[] = {-8.502, 0.07438 ,0.1205};
#else // based on the simulation data, 2016.11.29
	static const double _k1[] = { -0.0001378, -6.122e-05 ,-6.548e-05  };
	static const double _k2[] = {7.04, 6.647 ,6.677};
	static const double _k3[] = {-1068, -598.9 , -605.1};
#endif

	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));
	vector<double> k3 (_k3,_k3+sizeof(_k3)/sizeof(_k3[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformancePoly(maxSpeeds, k1,k2,k3,engine);
	double pDCUB = OptimizationAlgorithm::getPerformanceDCPoly3(dc, maxSpeeds, k1,k2,k3);

	cout << "Computing B&B...";
	time_t start = time(0);
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCPoly3(dc,EXACT,k1,k2,k3,WCETs, tasks, engine, period);
	double timeOPT = difftime(time(0),start);
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformancePoly(OPTSpeeds, k1,k2,k3, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCPoly3(dc, OPTSpeeds, k1,k2,k3);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pDCUB = " << pDCUB << endl;
	cout << "pOPTDC = " << pDCOPT << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pOPT/pUB*100.0 << "%" << endl;
	cout << "Ratio DC OPT/UB = " << pDCOPT/pDCUB*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;

	results.push_back(pDCOPT/pDCUB);
}

void RunTest::testFuelConsumptionExample(vector<double>& speeds_results, vector<double>& dc_perfs, string inputDir, string dcName) {
	string speedsFile = inputDir + Utility::linkNotation() + dcName + ".speeds";
	string torquesFile = inputDir + Utility::linkNotation() + dcName + ".torques";

	vector<double> dc_speeds = FileReader::ReadVectorFromFile(speedsFile);
	vector<double> dc_torques = FileReader::ReadVectorFromFile(torquesFile);

	// AVR task
	int rpm_min = 500;
	//int rpm_max = 6500;
	int rpm_max = 2650;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	//WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	static const double _k1[] = {-1.57e-07,3.59E-07,7.57E-07};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	static const double _k2[] = {0.0009045, -0.0004595, -0.00272};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	static const double _k3[] = {2.00E-05, 1.91E-05, 2.03E-05};
	vector<double> k3 (_k3,_k3+sizeof(_k3)/sizeof(_k3[0]));

	static const double _k4[] = {-0.1014, 1.203, 3.938};
	vector<double> k4 (_k4,_k4+sizeof(_k4)/sizeof(_k4[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformancePoly(maxSpeeds, k1,k2,k3,k4,engine);
			
	cout << "Computing B&B...";
	time_t start = time(0);
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBPoly3(EXACT,k1,k2,k4,WCETs, tasks, engine, period);
	double timeOPT = difftime(time(0),start);
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformancePoly(OPTSpeeds, k1,k2,k3,k4, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCPoly(dc_speeds,dc_torques, OPTSpeeds, k1,k2,k3,k4);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pDCOPT = " << pDCOPT << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;

	if (speeds_results.empty()) speeds_results = OPTSpeeds;
	dc_perfs.push_back(pDCOPT);
}

void RunTest::testFuelConsumptionExampleDC(vector<vector<double>>& speeds_results, vector<double>& dc_perfs, vector<double>& dc_times, string inputDir, string dcName) {
	string speedsFile = inputDir + Utility::linkNotation() + dcName + ".speeds";
	string torquesFile = inputDir + Utility::linkNotation() + dcName + ".torques";

	vector<double> dc_speeds = FileReader::ReadVectorFromFile(speedsFile);
	vector<double> dc_torques = FileReader::ReadVectorFromFile(torquesFile);

	// AVR task
	int rpm_min = 500;
	//int rpm_max = 6500;
	int rpm_max = 2650;
	double alpha = 1.62 * 0.0001;
	double period = 1.0;
	Engine engine(rpm_min,rpm_max,alpha,alpha);
	
	PeriodicTask task0(1000,5000);
	PeriodicTask task1(6500,20000);
	PeriodicTask task2(10000,50000);
	PeriodicTask task3(10000,100000);

	vector<PeriodicTask> tasks;
	tasks.push_back(task0);
	tasks.push_back(task1);
	tasks.push_back(task2);
	tasks.push_back(task3);

	vector<int> WCETs;
	int s = 8;
	//WCETs.push_back(s*150);
	//WCETs.push_back(s*278);
	//WCETs.push_back(s*344);
	WCETs.push_back(s*425);
	WCETs.push_back(s*576);
	WCETs.push_back(s*966);
	sort(WCETs.begin(),WCETs.end(),greater<int>());

	static const double _k[] = {10,7,5,4,3,2};
	vector<double> k (_k,_k+sizeof(_k)/sizeof(_k[0]));

	static const double _k1[] = {-1.57e-07,3.59E-07,7.57E-07};
	vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

	static const double _k2[] = {0.0009045, -0.0004595, -0.00272};
	vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

	static const double _k3[] = {2.00E-05, 1.91E-05, 2.03E-05};
	vector<double> k3 (_k3,_k3+sizeof(_k3)/sizeof(_k3[0]));

	static const double _k4[] = {-0.1014, 1.203, 3.938};
	vector<double> k4 (_k4,_k4+sizeof(_k4)/sizeof(_k4[0]));

	cout << "Computing UBs...";
	vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
	cout << "done." << endl;
	Utility::output_one_vector(cout,"Max Speeds", maxSpeeds);

	double pUB = OptimizationAlgorithm::getPerformancePoly(maxSpeeds, k1,k2,k3,k4,engine);
	Timer timer;		
	cout << "Computing B&B...";
	timer.start();
	vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCPoly4(dc_speeds,dc_torques,EXACT,k1,k2,k3,k4,WCETs, tasks, engine, period);
	timer.end();
	double timeOPT = timer.getTime();
	cout << "done." << endl;
	double pOPT = OptimizationAlgorithm::getPerformancePoly(OPTSpeeds, k1,k2,k3,k4, engine);
	Utility::output_one_vector(cout, "OPT Speeds", OPTSpeeds);

	double pDCOPT = OptimizationAlgorithm::getPerformanceDCPoly(dc_speeds,dc_torques, OPTSpeeds, k1,k2,k3,k4);

	cout << endl;
	cout << "pUB = " << pUB << endl;
	cout << "pOPT = " << pOPT << endl;
	cout << "pDCOPT = " << pDCOPT << endl;
	cout << endl;
	cout << "Ratio OPT/UB = " << pUB/pOPT*100.0 << "%" << endl;
	cout << endl;
	cout << "timeOPT = " << timeOPT << endl;

	speeds_results.push_back(OPTSpeeds);
	dc_perfs.push_back(pDCOPT);
	dc_times.push_back(timeOPT);
}

void RunTest::generateBiondiRTATestedSystems(string directory, int numSystem, int numPeriodicTask, int tUtil, double rho, int minModeNum, int maxModeNum) {
	// mkdir directory
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		double U_A = rho*tUtil/1000;
		double U_P = (1.0-rho)*tUtil/1000;

		int avrTaskIndex;
		TaskSystemGenerator::generate_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTask,U_A,minModeNum,maxModeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::generateBiondiRTATestedSystems2(string directory, int numSystem, int numPeriodicTask, int U_P, int U_A, int minModeNum, int maxModeNum) {
	// mkdir directory
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		int avrTaskIndex;
		TaskSystemGenerator::generate_random_system_for_response_time_analysis(tasks,numPeriodicTask,1.0*U_P/100, avrTask,1.0*U_A/100,minModeNum,maxModeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(U_P+U_A)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << U_P+U_A << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::generateBiondiRTATestedSystems3(string directory, int numSystem, int numPeriodicTask, double tUtil, int rho, int minModeNum, int maxModeNum) {
	// mkdir directory
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		double U_A = tUtil*rho/1000;
		double U_P = tUtil*(1000-rho)/1000;

		int avrTaskIndex;
		TaskSystemGenerator::generate_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTask,U_A,minModeNum,maxModeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"Rho"+Utility::int_to_string(rho)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::generateBiondiRTATestedSystems4(string directory, int numSystem, int numPeriodicTask, double tUtil, double rho, int modeNum) {
	// mkdir directory
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		double U_A = tUtil*rho;
		double U_P = tUtil*(1.0-rho);

		int avrTaskIndex;
		TaskSystemGenerator::generate_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTask,U_A,modeNum,modeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"ModeNum"+Utility::int_to_string(modeNum)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", modeNum = " << modeNum << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::testBiondiRTATestedSystems(vector<bool> vecTest, ofstream& fout, string directory, int minSystem, int maxSystem, int tUtil, vector<int> & tUtils, vector<vector<double>> & pResults, vector<vector<double>> & tResults) {
	if (vecTest.size() != 10) {
		cerr << "Size error: " << vecTest.size() << endl;
		exit(EXIT_FAILURE);
	}
	
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // Exact
	int sum_p1 = 0; // LUB
	int sum_p2 = 0; // Necessary Only 1 
	int sum_p3 = 0; // Necessary Only 1A
	int sum_p4 = 0; // Necessary Only 2A
	int sum_p5 = 0; // RBF
	int sum_p6 = 0; // RBF_Envelope
	int sum_p7 = 0; // ILP_CON
	int sum_p8 = 0; // ILP
	int sum_p9 = 0; // SEG_ILP_CON

	double sum_t0 = 0; // Exact
	double sum_t1 = 0; // LUB
	double sum_t2 = 0; // Necessary Only 1 
	double sum_t3 = 0; // Necessary Only 1A
	double sum_t4 = 0; // Necessary Only 2A
	double sum_t5 = 0; // RBF
	double sum_t6 = 0; // RBF_Envelope
	double sum_t7 = 0; // ILP_CON
	double sum_t8 = 0; // ILP
	double sum_t9 = 0; // SEG_ILP_CON

	for (int run=minSystem; run <= maxSystem; run++) {
	//for (int run=80; run <= 80; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<int> WCETs;
		vector<double> speeds;
		int avrTaskIndex;

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex,WCETs,speeds);

		//cout << avrTaskIndex << ", => ";

		AVRTask avrTask(engine,period,speeds,WCETs);

		Timer timer;
		bool schedulable = false;
		double t0 = 0.0;

		if (vecTest[0]) {
			timer.start();
			schedulable =  SchedAnalysis::exact_analysis(tasks,avrTask,avrTaskIndex);
			timer.end();
			t0 = timer.getTime();
			sum_t0 += t0;
		}

		bool lub_schedulable = false;
		double t1 = 0.0;

		if (vecTest[1]) {
			timer.start();
			//lub_schedulable = SchedAnalysis::lub_analysis(tasks,avrTask,avrTaskIndex);
			lub_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,1000);
			timer.end();
			t1 = timer.getTime();
			sum_t1 += t1;
		}

		bool nec_schedulable1 = false;
		double t2 = 0.0;

		if (vecTest[2]) {
			timer.start();
			//nec_schedulable1 = SchedAnalysis::necessary_only_analysis(tasks,avrTask,avrTaskIndex);
			//nec_schedulable1 = SchedAnalysis::segmental_necessary_only_analysis(tasks,avrTask,avrTaskIndex);
			nec_schedulable1 = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,500);
			timer.end();
			t2 = timer.getTime();
			sum_t2 += t2;
		}

		bool nec_schedulable1A = false;
		double t3 = 0.0;

		if (vecTest[3]) {
			timer.start();
			//nec_schedulable1A = SchedAnalysis::necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
			//nec_schedulable1A = SchedAnalysis::segmental_necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
			nec_schedulable1A = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,300);
			timer.end();
			t3 = timer.getTime();
			sum_t3 += t3;
		}

		bool nec_schedulable2A = false;
		double t4 = 0.0;

		if (vecTest[4]) {
			timer.start();
			//nec_schedulable2A = SchedAnalysis::necessary_only_analysis2A(tasks,avrTask,avrTaskIndex);
			nec_schedulable2A = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,200);
			timer.end();
			t4 = timer.getTime();
			sum_t4 += t4;
		}

		avrTask.buildDigraph();

		bool rbf_schedulable = false;
		double t5 = 0.0;

		if (vecTest[5]) {
			timer.start();
#ifdef __USING_DIGRAPH_RBF__
			//rbf_schedulable = SchedAnalysis::digraph_rbf_analysis(tasks,avrTask,avrTaskIndex);
			rbf_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,100);
#else
			rbf_schedulable = SchedAnalysis::rbf_analysis(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t5 = timer.getTime();
			sum_t5 += t5;
		}

		bool rbf_envelope_schedulable = false;
		double t6 = 0.0;

		if (vecTest[6]) {
			timer.start();
#ifdef __USING_DIGRAPH_RBF__
			//rbf_envelope_schedulable = SchedAnalysis::digraph_rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
			rbf_envelope_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,50);
#else
			rbf_envelope_schedulable = SchedAnalysis::rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t6 = timer.getTime();
			sum_t6 += t6;
		}

		bool ilp_con_schedulable = false;
		double t7 = 0.0;

		if (vecTest[7]) {
			timer.start();
			#ifdef __USING_ILPCPLEX__
			ilp_con_schedulable = SchedAnalysis::ilp_con_analysis(tasks,avrTask,avrTaskIndex);
			#else
			//ilp_con_schedulable = true;
			ilp_con_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex);
			#endif
			timer.end();
			t7 = timer.getTime();
			sum_t7 += t7;
		}

		bool ilp_schedulable = false;
		double t8 = 0.0;

		if (vecTest[8]) {
			timer.start();
			#ifdef __USING_ILPCPLEX__
			ilp_schedulable = SchedAnalysis::ilp_analysis(tasks,avrTask,avrTaskIndex);
			#else
			ilp_schedulable = true;
			#endif
			timer.end();
			t8 = timer.getTime();
			sum_t8 += t8;
		}

		bool seg_ilp_con_schedulable = false;
		double t9 = 0.0;

		if (vecTest[9]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			seg_ilp_con_schedulable = SchedAnalysis::segmental_ilp_con_exact_analysis(tasks,avrTask,avrTaskIndex);
#else
			seg_ilp_con_schedulable = true;
#endif
			timer.end();
			t9 = timer.getTime();
			sum_t9 += t9;
		}

		// delete digraph
		if (avrTask.digraph != NULL)
			delete avrTask.digraph;	

#if 1
		fout << "Util = " << tUtil << ", run = " << run << endl;
		fout << schedulable << "\t" << lub_schedulable << "\t" << nec_schedulable1 << "\t" << nec_schedulable1A << "\t" << nec_schedulable2A << "\t" 
			 << rbf_schedulable << "\t" << rbf_envelope_schedulable << "\t" << ilp_con_schedulable << "\t" << ilp_schedulable << "\t" << seg_ilp_con_schedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << "\t" << t3 << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 << "\t" << t8 << "\t" << t9 << endl;
#endif

#if 0 // Need to consider all the schedulability tests
		bool foundError = false;
		if (!schedulable && lub_schedulable) {
			cout << "lub_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && rbf_schedulable) {
			cout << "rbf_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && rbf_envelope_schedulable) {
			cout << "rbf_envelope_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && ilp_con_schedulable) {
			cout << "ilp_con_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && ilp_schedulable) {
			cout << "ilp_schedulable" << endl;
			foundError = true;
		}

		if (!rbf_schedulable && ilp_con_schedulable) {
			cout << "rbf/ilp_con_schedulable" << endl;
			foundError = true;
		}

		if (!rbf_envelope_schedulable && ilp_schedulable) {
			cout << "rbf_envelope/ilp_schedulable" << endl;
			foundError = true;
		}

		if (foundError) {
			cerr << "Util = " << tUtil << ", run = " << run << endl;
			cerr << schedulable << "\t" << lub_schedulable << "\t" << nec_schedulable1 << "\t" << nec_schedulable1A << "\t" << nec_schedulable2A << "\t" 
				<< rbf_schedulable << "\t" << rbf_envelope_schedulable << "\t" << ilp_con_schedulable << "\t" << ilp_schedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << "\t" << t3 << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 << "\t" << t8 << endl;
			exit(EXIT_FAILURE);
		}
#endif

		if (schedulable) sum_p0++;
		if (lub_schedulable) sum_p1++;
		if (nec_schedulable1) sum_p2++;
		if (nec_schedulable1A) sum_p3++;
		if (nec_schedulable2A) sum_p4++;
		if (rbf_schedulable) sum_p5++;
		if (rbf_envelope_schedulable) sum_p6++;
		if (ilp_con_schedulable) sum_p7++;
		if (ilp_schedulable) sum_p8++;
		if (seg_ilp_con_schedulable) sum_p9++;
	}

	fout << "==================total=================" << endl;
	fout << tUtil << " => " 
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_p3 << "\t" << sum_p4 << "\t" << sum_p5 << "\t" 
		<< sum_p6 << "\t" << sum_p7 << "\t" << sum_p8 << "\t" << sum_p9 << "\t"
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << "\t" 
		<< sum_t3 << "\t" << sum_t4 << "\t" << sum_t5 << "\t" 
		<< sum_t6 << "\t" << sum_t7 << "\t" << sum_t8 << "\t" << sum_t9 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	tUtils.push_back(tUtil);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;
		vector<double> vec_p3;
		vector<double> vec_p4;
		vector<double> vec_p5;
		vector<double> vec_p6;
		vector<double> vec_p7;
		vector<double> vec_p8;
		vector<double> vec_p9;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);
		vec_p3.push_back(1.0*sum_p3/size);
		vec_p4.push_back(1.0*sum_p4/size);
		vec_p5.push_back(1.0*sum_p5/size);
		vec_p6.push_back(1.0*sum_p6/size);
		vec_p7.push_back(1.0*sum_p7/size);
		vec_p8.push_back(1.0*sum_p8/size);
		vec_p9.push_back(1.0*sum_p9/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
		pResults.push_back(vec_p3);
		pResults.push_back(vec_p4);
		pResults.push_back(vec_p5);
		pResults.push_back(vec_p6);
		pResults.push_back(vec_p7);
		pResults.push_back(vec_p8);
		pResults.push_back(vec_p9);
	} else {
		if (pResults.size() != 10) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
		pResults[3].push_back(1.0*sum_p3/size);
		pResults[4].push_back(1.0*sum_p4/size);
		pResults[5].push_back(1.0*sum_p5/size);
		pResults[6].push_back(1.0*sum_p6/size);
		pResults[7].push_back(1.0*sum_p7/size);
		pResults[8].push_back(1.0*sum_p8/size);
		pResults[9].push_back(1.0*sum_p9/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;
		vector<double> vec_t3;
		vector<double> vec_t4;
		vector<double> vec_t5;
		vector<double> vec_t6;
		vector<double> vec_t7;
		vector<double> vec_t8;
		vector<double> vec_t9;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);
		vec_t3.push_back(1.0*sum_t3/size);
		vec_t4.push_back(1.0*sum_t4/size);
		vec_t5.push_back(1.0*sum_t5/size);
		vec_t6.push_back(1.0*sum_t6/size);
		vec_t7.push_back(1.0*sum_t7/size);
		vec_t8.push_back(1.0*sum_t8/size);
		vec_t9.push_back(1.0*sum_t9/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
		tResults.push_back(vec_t3);
		tResults.push_back(vec_t4);
		tResults.push_back(vec_t5);
		tResults.push_back(vec_t6);
		tResults.push_back(vec_t7);
		tResults.push_back(vec_t8);
		tResults.push_back(vec_t9);
	} else {
		if (tResults.size() != 10) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
		tResults[3].push_back(1.0*sum_t3/size);
		tResults[4].push_back(1.0*sum_t4/size);
		tResults[5].push_back(1.0*sum_t5/size);
		tResults[6].push_back(1.0*sum_t6/size);
		tResults[7].push_back(1.0*sum_t7/size);
		tResults[8].push_back(1.0*sum_t8/size);
		tResults[9].push_back(1.0*sum_t9/size);
	}
}

void RunTest::testBiondiRTATestedSystems2(vector<bool> vecTest, ofstream& fout, string directory, int minSystem, int maxSystem, int rho, vector<int> & rhos, vector<vector<double>> & pResults, vector<vector<double>> & tResults) {
	if (vecTest.size() != 10) {
		cerr << "Size error: " << vecTest.size() << endl;
		exit(EXIT_FAILURE);
	}
	
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // Exact
	int sum_p1 = 0; // LUB
	int sum_p2 = 0; // Necessary Only 1 
	int sum_p3 = 0; // Necessary Only 1A
	int sum_p4 = 0; // Necessary Only 2A
	int sum_p5 = 0; // RBF
	int sum_p6 = 0; // RBF_Envelope
	int sum_p7 = 0; // ILP_CON
	int sum_p8 = 0; // ILP
	int sum_p9 = 0; // SEG_ILP_CON

	double sum_t0 = 0; // Exact
	double sum_t1 = 0; // LUB
	double sum_t2 = 0; // Necessary Only 1 
	double sum_t3 = 0; // Necessary Only 1A
	double sum_t4 = 0; // Necessary Only 2A
	double sum_t5 = 0; // RBF
	double sum_t6 = 0; // RBF_Envelope
	double sum_t7 = 0; // ILP_CON
	double sum_t8 = 0; // ILP
	double sum_t9 = 0; // SEG_ILP_CON

	for (int run=minSystem; run <= maxSystem; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<int> WCETs;
		vector<double> speeds;
		int avrTaskIndex;

		string name = directory+Utility::linkNotation()+"Rho"+Utility::int_to_string(rho)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex,WCETs,speeds);

		//cout << avrTaskIndex << ", => ";

		AVRTask avrTask(engine,period,speeds,WCETs);

		Timer timer;
		bool schedulable = false;
		double t0 = 0.0;

		if (vecTest[0]) {
			timer.start();
			schedulable =  SchedAnalysis::exact_analysis(tasks,avrTask,avrTaskIndex);
			timer.end();
			t0 = timer.getTime();
			sum_t0 += t0;
		}

		bool lub_schedulable = false;
		double t1 = 0.0;

		if (vecTest[1]) {
			timer.start();
			//lub_schedulable = SchedAnalysis::lub_analysis(tasks,avrTask,avrTaskIndex);
			lub_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,1000);
			timer.end();
			t1 = timer.getTime();
			sum_t1 += t1;
		}

		bool nec_schedulable1 = false;
		double t2 = 0.0;

		if (vecTest[2]) {
			timer.start();
			//nec_schedulable1 = SchedAnalysis::necessary_only_analysis(tasks,avrTask,avrTaskIndex);
			//nec_schedulable1 = SchedAnalysis::segmental_necessary_only_analysis(tasks,avrTask,avrTaskIndex);
			nec_schedulable1 = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,500);
			timer.end();
			t2 = timer.getTime();
			sum_t2 += t2;
		}

		bool nec_schedulable1A = false;
		double t3 = 0.0;

		if (vecTest[3]) {
			timer.start();
			//nec_schedulable1A = SchedAnalysis::necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
			//nec_schedulable1A = SchedAnalysis::segmental_necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
			nec_schedulable1A = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,300);
			timer.end();
			t3 = timer.getTime();
			sum_t3 += t3;
		}

		bool nec_schedulable2A = false;
		double t4 = 0.0;

		if (vecTest[4]) {
			timer.start();
			//nec_schedulable2A = SchedAnalysis::necessary_only_analysis2A(tasks,avrTask,avrTaskIndex);
			nec_schedulable2A = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,200);
			timer.end();
			t4 = timer.getTime();
			sum_t4 += t4;
		}

		avrTask.buildDigraph();

		bool rbf_schedulable = false;
		double t5 = 0.0;

		if (vecTest[5]) {
			timer.start();
#ifdef __USING_DIGRAPH_RBF__
			//rbf_schedulable = SchedAnalysis::digraph_rbf_analysis(tasks,avrTask,avrTaskIndex);
			rbf_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,100);
#else
			rbf_schedulable = SchedAnalysis::rbf_analysis(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t5 = timer.getTime();
			sum_t5 += t5;
		}

		bool rbf_envelope_schedulable = false;
		double t6 = 0.0;

		if (vecTest[6]) {
			timer.start();
#ifdef __USING_DIGRAPH_RBF__
			//rbf_envelope_schedulable = SchedAnalysis::digraph_rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
			rbf_envelope_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,50);
#else
			rbf_envelope_schedulable = SchedAnalysis::rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t6 = timer.getTime();
			sum_t6 += t6;
		}

		bool ilp_con_schedulable = false;
		double t7 = 0.0;

		if (vecTest[7]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			ilp_con_schedulable = SchedAnalysis::ilp_con_analysis(tasks,avrTask,avrTaskIndex);
#else
			//ilp_con_schedulable = true;
			ilp_con_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t7 = timer.getTime();
			sum_t7 += t7;
		}

		bool ilp_schedulable = false;
		double t8 = 0.0;

		if (vecTest[8]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			ilp_schedulable = SchedAnalysis::ilp_analysis(tasks,avrTask,avrTaskIndex);
#else
			ilp_schedulable = true;
#endif
			timer.end();
			t8 = timer.getTime();
			sum_t8 += t8;
		}

		bool seg_ilp_con_schedulable = false;
		double t9 = 0.0;

		if (vecTest[9]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			seg_ilp_con_schedulable = SchedAnalysis::segmental_ilp_con_exact_analysis(tasks,avrTask,avrTaskIndex);
#else
			seg_ilp_con_schedulable = true;
#endif
			timer.end();
			t9 = timer.getTime();
			sum_t9 += t9;
		}

		// delete digraph
		if (avrTask.digraph != NULL)
			delete avrTask.digraph;	

#if 1
		fout << "rho = " << rho << ", run = " << run << endl;
		fout << schedulable << "\t" << lub_schedulable << "\t" << nec_schedulable1 << "\t" << nec_schedulable1A << "\t" << nec_schedulable2A << "\t" 
			<< rbf_schedulable << "\t" << rbf_envelope_schedulable << "\t" << ilp_con_schedulable << "\t" << ilp_schedulable << "\t" << seg_ilp_con_schedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << "\t" << t3 << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 << "\t" << t8 << "\t" << t9 << endl;
#endif

#if 0 // Need to consider all the schedulability tests
		bool foundError = false;
		if (!schedulable && lub_schedulable) {
			cout << "lub_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && rbf_schedulable) {
			cout << "rbf_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && rbf_envelope_schedulable) {
			cout << "rbf_envelope_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && ilp_con_schedulable) {
			cout << "ilp_con_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && ilp_schedulable) {
			cout << "ilp_schedulable" << endl;
			foundError = true;
		}

		if (foundError) {
			cerr << "rho = " << rho << ", run = " << run << endl;
			cerr << schedulable << "\t" << lub_schedulable << "\t" << nec_schedulable1 << "\t" << nec_schedulable1A << "\t" << nec_schedulable2A << "\t" 
				<< rbf_schedulable << "\t" << rbf_envelope_schedulable << "\t" << ilp_con_schedulable << "\t" << ilp_schedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << "\t" << t3 << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 << "\t" << t8 << endl;
			exit(EXIT_FAILURE);
		}
#endif

		if (schedulable) sum_p0++;
		if (lub_schedulable) sum_p1++;
		if (nec_schedulable1) sum_p2++;
		if (nec_schedulable1A) sum_p3++;
		if (nec_schedulable2A) sum_p4++;
		if (rbf_schedulable) sum_p5++;
		if (rbf_envelope_schedulable) sum_p6++;
		if (ilp_con_schedulable) sum_p7++;
		if (ilp_schedulable) sum_p8++;
		if (seg_ilp_con_schedulable) sum_p9++;
	}

	fout << "===============total=============" << endl;
	fout << rho << " => "
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_p3 << "\t" << sum_p4 << "\t" << sum_p5 << "\t" 
		<< sum_p6 << "\t" << sum_p7 << "\t" << sum_p8 << "\t" << sum_p9 << "\t"
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << "\t" 
		<< sum_t3 << "\t" << sum_t4 << "\t" << sum_t5 << "\t" 
		<< sum_t6 << "\t" << sum_t7 << "\t" << sum_t8 << "\t" << sum_t9 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	rhos.push_back(rho);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;
		vector<double> vec_p3;
		vector<double> vec_p4;
		vector<double> vec_p5;
		vector<double> vec_p6;
		vector<double> vec_p7;
		vector<double> vec_p8;
		vector<double> vec_p9;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);
		vec_p3.push_back(1.0*sum_p3/size);
		vec_p4.push_back(1.0*sum_p4/size);
		vec_p5.push_back(1.0*sum_p5/size);
		vec_p6.push_back(1.0*sum_p6/size);
		vec_p7.push_back(1.0*sum_p7/size);
		vec_p8.push_back(1.0*sum_p8/size);
		vec_p9.push_back(1.0*sum_p9/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
		pResults.push_back(vec_p3);
		pResults.push_back(vec_p4);
		pResults.push_back(vec_p5);
		pResults.push_back(vec_p6);
		pResults.push_back(vec_p7);
		pResults.push_back(vec_p8);
		pResults.push_back(vec_p9);
	} else {
		if (pResults.size() != 10) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
		pResults[3].push_back(1.0*sum_p3/size);
		pResults[4].push_back(1.0*sum_p4/size);
		pResults[5].push_back(1.0*sum_p5/size);
		pResults[6].push_back(1.0*sum_p6/size);
		pResults[7].push_back(1.0*sum_p7/size);
		pResults[8].push_back(1.0*sum_p8/size);
		pResults[9].push_back(1.0*sum_p9/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;
		vector<double> vec_t3;
		vector<double> vec_t4;
		vector<double> vec_t5;
		vector<double> vec_t6;
		vector<double> vec_t7;
		vector<double> vec_t8;
		vector<double> vec_t9;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);
		vec_t3.push_back(1.0*sum_t3/size);
		vec_t4.push_back(1.0*sum_t4/size);
		vec_t5.push_back(1.0*sum_t5/size);
		vec_t6.push_back(1.0*sum_t6/size);
		vec_t7.push_back(1.0*sum_t7/size);
		vec_t8.push_back(1.0*sum_t8/size);
		vec_t9.push_back(1.0*sum_t9/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
		tResults.push_back(vec_t3);
		tResults.push_back(vec_t4);
		tResults.push_back(vec_t5);
		tResults.push_back(vec_t6);
		tResults.push_back(vec_t7);
		tResults.push_back(vec_t8);
		tResults.push_back(vec_t9);
	} else {
		if (tResults.size() != 10) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
		tResults[3].push_back(1.0*sum_t3/size);
		tResults[4].push_back(1.0*sum_t4/size);
		tResults[5].push_back(1.0*sum_t5/size);
		tResults[6].push_back(1.0*sum_t6/size);
		tResults[7].push_back(1.0*sum_t7/size);
		tResults[8].push_back(1.0*sum_t8/size);
		tResults[9].push_back(1.0*sum_t9/size);
	}
}

void RunTest::testBiondiRTATestedSystems3(vector<bool> vecTest, ofstream& fout, string directory, int minSystem, int maxSystem, int modeNum, vector<int> & modeNums, vector<vector<double>> & pResults, vector<vector<double>> & tResults) {
	if (vecTest.size() != 10) {
		cerr << "Size error: " << vecTest.size() << endl;
		exit(EXIT_FAILURE);
	}

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // Exact
	int sum_p1 = 0; // LUB
	int sum_p2 = 0; // Necessary Only 1 
	int sum_p3 = 0; // Necessary Only 1A
	int sum_p4 = 0; // Necessary Only 2A
	int sum_p5 = 0; // RBF
	int sum_p6 = 0; // RBF_Envelope
	int sum_p7 = 0; // ILP_CON
	int sum_p8 = 0; // ILP
	int sum_p9 = 0; // SEG_ILP_CON

	double sum_t0 = 0; // Exact
	double sum_t1 = 0; // LUB
	double sum_t2 = 0; // Necessary Only 1 
	double sum_t3 = 0; // Necessary Only 1A
	double sum_t4 = 0; // Necessary Only 2A
	double sum_t5 = 0; // RBF
	double sum_t6 = 0; // RBF_Envelope
	double sum_t7 = 0; // ILP_CON
	double sum_t8 = 0; // ILP
	double sum_t9 = 0; // SEG_ILP_CON

	for (int run=minSystem; run <= maxSystem; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<int> WCETs;
		vector<double> speeds;
		int avrTaskIndex;

		string name = directory+Utility::linkNotation()+"ModeNum"+Utility::int_to_string(modeNum)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex,WCETs,speeds);

		//cout << avrTaskIndex << ", => ";

		AVRTask avrTask(engine,period,speeds,WCETs);

		Timer timer;
		bool schedulable = false;
		double t0 = 0.0;

		if (vecTest[0]) {
			timer.start();
			schedulable =  SchedAnalysis::exact_analysis(tasks,avrTask,avrTaskIndex);
			timer.end();
			t0 = timer.getTime();
			sum_t0 += t0;
		}

		bool lub_schedulable = false;
		double t1 = 0.0;

		if (vecTest[1]) {
			timer.start();
			//lub_schedulable = SchedAnalysis::lub_analysis(tasks,avrTask,avrTaskIndex);
			lub_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,1000);
			timer.end();
			t1 = timer.getTime();
			sum_t1 += t1;
		}

		bool nec_schedulable1 = false;
		double t2 = 0.0;

		if (vecTest[2]) {
			timer.start();
			//nec_schedulable1 = SchedAnalysis::necessary_only_analysis(tasks,avrTask,avrTaskIndex);
			//nec_schedulable1 = SchedAnalysis::segmental_necessary_only_analysis(tasks,avrTask,avrTaskIndex);
			nec_schedulable1 = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,500);
			timer.end();
			t2 = timer.getTime();
			sum_t2 += t2;
		}

		bool nec_schedulable1A = false;
		double t3 = 0.0;

		if (vecTest[3]) {
			timer.start();
			//nec_schedulable1A = SchedAnalysis::necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
			//nec_schedulable1A = SchedAnalysis::segmental_necessary_only_analysis1A(tasks,avrTask,avrTaskIndex);
			nec_schedulable1A = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,300);
			timer.end();
			t3 = timer.getTime();
			sum_t3 += t3;
		}

		bool nec_schedulable2A = false;
		double t4 = 0.0;

		if (vecTest[4]) {
			timer.start();
			//nec_schedulable2A = SchedAnalysis::necessary_only_analysis2A(tasks,avrTask,avrTaskIndex);
			nec_schedulable2A = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,200);
			timer.end();
			t4 = timer.getTime();
			sum_t4 += t4;
		}

		avrTask.buildDigraph();

		bool rbf_schedulable = false;
		double t5 = 0.0;

		if (vecTest[5]) {
			timer.start();
#ifdef __USING_DIGRAPH_RBF__
			//rbf_schedulable = SchedAnalysis::digraph_rbf_analysis(tasks,avrTask,avrTaskIndex);
			rbf_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,100);
#else
			rbf_schedulable = SchedAnalysis::rbf_analysis(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t5 = timer.getTime();
			sum_t5 += t5;
		}

		bool rbf_envelope_schedulable = false;
		double t6 = 0.0;

		if (vecTest[6]) {
			timer.start();
#ifdef __USING_DIGRAPH_RBF__
			//rbf_envelope_schedulable = SchedAnalysis::digraph_rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
			rbf_envelope_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex,50);
#else
			rbf_envelope_schedulable = SchedAnalysis::rbf_envelope_analysis(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t6 = timer.getTime();
			sum_t6 += t6;
		}

		bool ilp_con_schedulable = false;
		double t7 = 0.0;

		if (vecTest[7]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			ilp_con_schedulable = SchedAnalysis::ilp_con_analysis(tasks,avrTask,avrTaskIndex);
#else
			//ilp_con_schedulable = true;
			ilp_con_schedulable = SchedAnalysis::exact_analysis_avrtask_to_digraph(tasks,avrTask,avrTaskIndex);
#endif
			timer.end();
			t7 = timer.getTime();
			sum_t7 += t7;
		}

		bool ilp_schedulable = false;
		double t8 = 0.0;

		if (vecTest[8]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			ilp_schedulable = SchedAnalysis::ilp_analysis(tasks,avrTask,avrTaskIndex);
#else
			ilp_schedulable = true;
#endif
			timer.end();
			t8 = timer.getTime();
			sum_t8 += t8;
		}

		bool seg_ilp_con_schedulable = false;
		double t9 = 0.0;

		if (vecTest[9]) {
			timer.start();
#ifdef __USING_ILPCPLEX__
			seg_ilp_con_schedulable = SchedAnalysis::segmental_ilp_con_exact_analysis(tasks,avrTask,avrTaskIndex);
#else
			seg_ilp_con_schedulable = true;
#endif
			timer.end();
			t9 = timer.getTime();
			sum_t9 += t9;
		}

		// delete digraph
		if (avrTask.digraph != NULL)
			delete avrTask.digraph;		

#if 1
		fout << "modeNum = " << modeNum << ", run = " << run << endl;
		fout << schedulable << "\t" << lub_schedulable << "\t" << nec_schedulable1 << "\t" << nec_schedulable1A << "\t" << nec_schedulable2A << "\t" 
			<< rbf_schedulable << "\t" << rbf_envelope_schedulable << "\t" << ilp_con_schedulable << "\t" << ilp_schedulable << "\t" << seg_ilp_con_schedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << "\t" << t3 << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 << "\t" << t8 << "\t" << t9 << endl;
#endif

#if 0 // Need to consider all the schedulability tests
		bool foundError = false;
		if (!schedulable && lub_schedulable) {
			cout << "lub_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && rbf_schedulable) {
			cout << "rbf_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && rbf_envelope_schedulable) {
			cout << "rbf_envelope_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && ilp_con_schedulable) {
			cout << "ilp_con_schedulable" << endl;
			foundError = true;
		}

		if (!schedulable && ilp_schedulable) {
			cout << "ilp_schedulable" << endl;
			foundError = true;
		}

		if (!rbf_schedulable && ilp_con_schedulable) {
			cout << "rbf/ilp_con_schedulable" << endl;
			foundError = true;
		}

		if (!rbf_envelope_schedulable && ilp_schedulable) {
			cout << "rbf_envelope/ilp_schedulable" << endl;
			foundError = true;
		}

		if (foundError) {
			cerr << "modeNum = " << modeNum << ", run = " << run << endl;
			cerr << schedulable << "\t" << lub_schedulable << "\t" << nec_schedulable1 << "\t" << nec_schedulable1A << "\t" << nec_schedulable2A << "\t" 
				<< rbf_schedulable << "\t" << rbf_envelope_schedulable << "\t" << ilp_con_schedulable << "\t" << ilp_schedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << "\t" << t3 << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 << "\t" << t8 << endl;
			exit(EXIT_FAILURE);
		}
#endif

		if (schedulable) sum_p0++;
		if (lub_schedulable) sum_p1++;
		if (nec_schedulable1) sum_p2++;
		if (nec_schedulable1A) sum_p3++;
		if (nec_schedulable2A) sum_p4++;
		if (rbf_schedulable) sum_p5++;
		if (rbf_envelope_schedulable) sum_p6++;
		if (ilp_con_schedulable) sum_p7++;
		if (ilp_schedulable) sum_p8++;
		if (seg_ilp_con_schedulable) sum_p9++;
	}
	fout << "================total==============" << endl;
	fout << modeNum << " => "
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_p3 << "\t" << sum_p4 << "\t" << sum_p5 << "\t" 
		<< sum_p6 << "\t" << sum_p7 << "\t" << sum_p8 << "\t" << sum_p9 << "\t"
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << "\t" 
		<< sum_t3 << "\t" << sum_t4 << "\t" << sum_t5 << "\t" 
		<< sum_t6 << "\t" << sum_t7 << "\t" << sum_t8 << "\t" << sum_t9 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	modeNums.push_back(modeNum);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;
		vector<double> vec_p3;
		vector<double> vec_p4;
		vector<double> vec_p5;
		vector<double> vec_p6;
		vector<double> vec_p7;
		vector<double> vec_p8;
		vector<double> vec_p9;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);
		vec_p3.push_back(1.0*sum_p3/size);
		vec_p4.push_back(1.0*sum_p4/size);
		vec_p5.push_back(1.0*sum_p5/size);
		vec_p6.push_back(1.0*sum_p6/size);
		vec_p7.push_back(1.0*sum_p7/size);
		vec_p8.push_back(1.0*sum_p8/size);
		vec_p9.push_back(1.0*sum_p9/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
		pResults.push_back(vec_p3);
		pResults.push_back(vec_p4);
		pResults.push_back(vec_p5);
		pResults.push_back(vec_p6);
		pResults.push_back(vec_p7);
		pResults.push_back(vec_p8);
		pResults.push_back(vec_p9);
	} else {
		if (pResults.size() != 10) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
		pResults[3].push_back(1.0*sum_p3/size);
		pResults[4].push_back(1.0*sum_p4/size);
		pResults[5].push_back(1.0*sum_p5/size);
		pResults[6].push_back(1.0*sum_p6/size);
		pResults[7].push_back(1.0*sum_p7/size);
		pResults[8].push_back(1.0*sum_p8/size);
		pResults[9].push_back(1.0*sum_p9/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;
		vector<double> vec_t3;
		vector<double> vec_t4;
		vector<double> vec_t5;
		vector<double> vec_t6;
		vector<double> vec_t7;
		vector<double> vec_t8;
		vector<double> vec_t9;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);
		vec_t3.push_back(1.0*sum_t3/size);
		vec_t4.push_back(1.0*sum_t4/size);
		vec_t5.push_back(1.0*sum_t5/size);
		vec_t6.push_back(1.0*sum_t6/size);
		vec_t7.push_back(1.0*sum_t7/size);
		vec_t8.push_back(1.0*sum_t8/size);
		vec_t9.push_back(1.0*sum_t9/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
		tResults.push_back(vec_t3);
		tResults.push_back(vec_t4);
		tResults.push_back(vec_t5);
		tResults.push_back(vec_t6);
		tResults.push_back(vec_t7);
		tResults.push_back(vec_t8);
		tResults.push_back(vec_t9);
	} else {
		if (tResults.size() != 10) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
		tResults[3].push_back(1.0*sum_t3/size);
		tResults[4].push_back(1.0*sum_t4/size);
		tResults[5].push_back(1.0*sum_t5/size);
		tResults[6].push_back(1.0*sum_t6/size);
		tResults[7].push_back(1.0*sum_t7/size);
		tResults[8].push_back(1.0*sum_t8/size);
		tResults[9].push_back(1.0*sum_t9/size);
	}
}

void RunTest::generateECRTS18RTATestedSystems(string directory, int numSystem, int numAvrTask, int numPeriodicTask, int tUtil, double rho, int minModeNum, int maxModeNum) {
	// mkdir directory
	directory += "Rho" + Utility::int_to_string(100*rho);
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		vector<AVRTask*> avrTasks;

		double U_A = rho*tUtil/1000;
		double U_P = (1.0-rho)*tUtil/1000;

		int avrTaskIndex;
		TaskSystemGenerator::generate_dynamic_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTasks,numAvrTask,U_A,minModeNum,maxModeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,avrTasks,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		for (auto avrTask : avrTasks)
			delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::generateECRTS18RTATestedSystems2(string directory, int numSystem, int numAvrTask, int numPeriodicTask, double tUtil, int rho, int minModeNum, int maxModeNum) {
	// mkdir directory
	directory += "Util" + Utility::int_to_string(100*tUtil);
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		vector<AVRTask*> avrTasks;

		
		double U_A = tUtil*rho/1000;
		double U_P = tUtil*(1000-rho)/1000;
		

		//double U_A = 0.1;
		//double U_P = 0.65;

		int avrTaskIndex;
		TaskSystemGenerator::generate_dynamic_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTasks,numAvrTask,U_A,minModeNum,maxModeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"Rho"+Utility::int_to_string(rho)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,avrTasks,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		for (auto avrTask : avrTasks)
			delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::generateECRTS18RTATestedSystems3(string directory, int numSystem, int numAvrTask, int numPeriodicTask, double tUtil, double rho, int modeNum) {
	// mkdir directory
	directory += "Util" + Utility::int_to_string(100*tUtil) + "Rho"+ Utility::int_to_string(100*rho);
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		vector<AVRTask*> avrTasks;

		double U_A = tUtil*rho;
		double U_P = tUtil*(1.0-rho);

		int avrTaskIndex;
		TaskSystemGenerator::generate_dynamic_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTasks,numAvrTask,U_A,modeNum,modeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"ModeNum"+Utility::int_to_string(modeNum)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,avrTasks,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		for (auto avrTask : avrTasks)
			delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", modeNum = " << modeNum << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::generateECRTS18RTATestedSystems4(string directory, int numSystem, int numAvrTask, int numPeriodicTask, double tUtil, double rho, int minModeNum, int maxModeNum) {
	// mkdir directory
	directory += "Util" + Utility::int_to_string(100*tUtil) + "Rho"+ Utility::int_to_string(100*rho);
	Utility::makeDir(directory);

	int numAVRTaskIndex = 0;

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		vector<AVRTask*> avrTasks;

		double U_A = tUtil*rho;
		double U_P = tUtil*(1.0-rho);

		int avrTaskIndex;
		TaskSystemGenerator::generate_dynamic_random_system_for_response_time_analysis(tasks,numPeriodicTask,U_P, avrTasks,numAvrTask,U_A,minModeNum,maxModeNum,avrTaskIndex);

		string name = directory+Utility::linkNotation()+"AVRTaskNum"+Utility::int_to_string(numAvrTask)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,avrTasks,avrTaskIndex);

		if (avrTaskIndex == 0) numAVRTaskIndex++;

		for (auto avrTask : avrTasks)
			delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}

	cout << "Util = " << tUtil << ", rho = " << rho << ", AvrTaskNum = " << numAvrTask << ", num = " << numAVRTaskIndex << endl;
}

void RunTest::testECRTS18RTATestedSystems(ofstream& fout, string directory, int minSystem, int maxSystem, int tUtil, vector<int> & tUtils, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double rho) {
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // DAVR
	int sum_p1 = 0; // SAVR
	int sum_p2 = 0; // fSAVR

	double sum_t0 = 0; // DAVR
	double sum_t1 = 0; // SAVR
	double sum_t2 = 0; // fSAVR

	for (int run=minSystem; run <= maxSystem; run++) {
		//for (int run=15; run <= 15; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<AVRTask> avrTasks;
		int avrTaskIndex;

		string name = directory + "Rho" + Utility::int_to_string(100*rho) +Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex, engine, period,avrTasks);
		//cout << avrTaskIndex << ", => ";

		Timer timer;

		timer.start();
		bool dAVRSchedulable = false;
#if 1
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
		  dAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
		  if (!dAVRSchedulable) break;
		}

		int granularity = 1000;

		// check each switch of the engine configurations
		if (dAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				dAVRSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,mixedTwo, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME,granularity);
				if (!dAVRSchedulable) break;
			}
		}
#endif
		timer.end();
		double t0 = timer.getTime();
		sum_t0 += t0;

		timer.start();
		AVRTask mixedAvrTask(engine,period,avrTasks);
		//bool sAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedAvrTask,avrTaskIndex);
		bool sAVRSchedulable = true;
		timer.end();
		double t1 = timer.getTime();
		sum_t1 += t1;
		
		timer.start();
		bool fSAVRSchedulable = false;
#if 0 // check fine-d2s
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!fSAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (fSAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedTwo,avrTaskIndex);
				if (!fSAVRSchedulable) break;
			}
		}
		
#else // check necessary test for generation
		// check two modes for each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			AVRTask avrTask = avrTasks[i];
			int numMode = avrTask.numMode;
			int minWCET = avrTask.wcets[numMode-1];
			double maxSpeed = avrTask.speeds[numMode-1];
			for (int j=0; j< numMode-1; j++) {
				vector<int> localWCETs;
				vector<double> localSpeeds; 

				localWCETs.push_back(avrTask.wcets[j]);
				localWCETs.push_back(minWCET);

				localSpeeds.push_back(avrTask.speeds[j]);
				localSpeeds.push_back(maxSpeed);

				AVRTask localAvrTask(engine,avrTask.period,localSpeeds,localWCETs);
				fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,localAvrTask,avrTaskIndex);
				if (!fSAVRSchedulable) break;
			}
		}
#endif
		timer.end();
		double t2 = timer.getTime();
		sum_t2 += t2;

#if 1
		fout << "Util = " << tUtil << ", run = " << run << endl;
		fout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << endl;
#endif

#if 1
		if (dAVRSchedulable != sAVRSchedulable) {
			cout << "Util = " << tUtil << ", run = " << run << endl;
			cout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << endl;
		}
#endif

		if (dAVRSchedulable) sum_p0++;
		if (sAVRSchedulable) sum_p1++;
		if (fSAVRSchedulable) sum_p2++;
	}

	fout << "==================total=================" << endl;
	fout << tUtil << " => " 
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	tUtils.push_back(tUtil);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
	} else {
		if (pResults.size() != 3) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
	} else {
		if (tResults.size() != 3) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
	}
}

void RunTest::testECRTS18RTATestedSystems2(ofstream& fout, string directory, int minSystem, int maxSystem, int rho, vector<int> & rhos, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double tUtil) {
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // DAVR
	int sum_p1 = 0; // SAVR
	int sum_p2 = 0; // fSAVR

	double sum_t0 = 0; // DAVR
	double sum_t1 = 0; // SAVR
	double sum_t2 = 0; // fSAVR

	int granularity = -1;

	for (int run=minSystem; run <= maxSystem; run++) {
	//for (int run=8; run <= 8; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<AVRTask> avrTasks;
		int avrTaskIndex;

		string name = directory + "Util" + Utility::int_to_string(100*tUtil) +Utility::linkNotation()+"Rho"+Utility::int_to_string(rho)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex, engine, period,avrTasks);

		//cout << avrTaskIndex << ", => ";

		Timer timer;

		timer.start();
		bool dAVRSchedulable = false;
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			dAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!dAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (dAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				dAVRSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,mixedTwo, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME,granularity);
				if (!dAVRSchedulable) break;
			}
		}
		timer.end();
		double t0 = timer.getTime();
		sum_t0 += t0;

		timer.start();
		AVRTask mixedAvrTask(engine,period,avrTasks);
		//bool sAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedAvrTask,avrTaskIndex);
		bool sAVRSchedulable = false;
		timer.end();
		double t1 = timer.getTime();
		sum_t1 += t1;

		timer.start();
		bool fSAVRSchedulable = false;
#if 0
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!fSAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (fSAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedTwo,avrTaskIndex);
				if (!fSAVRSchedulable) break;
			}
		}
#else // check necessary test for generation
		// check two modes for each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			AVRTask avrTask = avrTasks[i];
			int numMode = avrTask.numMode;
			int minWCET = avrTask.wcets[numMode-1];
			double maxSpeed = avrTask.speeds[numMode-1];
			for (int j=0; j< numMode-1; j++) {
				vector<int> localWCETs;
				vector<double> localSpeeds; 

				localWCETs.push_back(avrTask.wcets[j]);
				localWCETs.push_back(minWCET);

				localSpeeds.push_back(avrTask.speeds[j]);
				localSpeeds.push_back(maxSpeed);

				AVRTask localAvrTask(engine,avrTask.period,localSpeeds,localWCETs);
				fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,localAvrTask,avrTaskIndex);
				if (!fSAVRSchedulable) break;
			}
		}
#endif
		timer.end();
		double t2 = timer.getTime();
		sum_t2 += t2;

#if 1
		fout << "Rho = " << rho << ", run = " << run << endl;
		fout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << endl;
#endif

#if 1
		if (dAVRSchedulable != sAVRSchedulable) {
			cout << "Rho = " << rho << ", run = " << run << endl;
			cout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << endl;
		}
#endif

		if (dAVRSchedulable) sum_p0++;
		if (sAVRSchedulable) sum_p1++;
		if (fSAVRSchedulable) sum_p2++;
	}

	fout << "==================total=================" << endl;
	fout << rho << " => " 
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	rhos.push_back(rho);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
	} else {
		if (pResults.size() != 3) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
	} else {
		if (tResults.size() != 3) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
	}
}

void RunTest::testECRTS18RTATestedSystems3(ofstream& fout, string directory, int minSystem, int maxSystem, int modeNum, vector<int> & modeNums, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double tUtil, double rho) 
{
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // DAVR
	int sum_p1 = 0; // SAVR
	int sum_p2 = 0; // fSAVR

	double sum_t0 = 0; // DAVR
	double sum_t1 = 0; // SAVR
	double sum_t2 = 0; // fSAVR

	for (int run=minSystem; run <= maxSystem; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<AVRTask> avrTasks;
		int avrTaskIndex;

		string name = directory + "Util" + Utility::int_to_string(100*tUtil) + "Rho" + Utility::int_to_string(100*rho)+Utility::linkNotation()+"ModeNum"+Utility::int_to_string(modeNum)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex, engine, period,avrTasks);

		//cout << avrTaskIndex << ", => ";

		Timer timer;

		timer.start();
		bool dAVRSchedulable = false;
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			dAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!dAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (dAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				dAVRSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,mixedTwo, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME);
				if (!dAVRSchedulable) break;
			}
		}
		timer.end();
		double t0 = timer.getTime();
		sum_t0 += t0;

		timer.start();
		AVRTask mixedAvrTask(engine,period,avrTasks);
		bool sAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedAvrTask,avrTaskIndex);
		timer.end();
		double t1 = timer.getTime();
		sum_t1 += t1;

		timer.start();
		bool fSAVRSchedulable = false;
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!fSAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (fSAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedTwo,avrTaskIndex);
				if (!fSAVRSchedulable) break;
			}
		}
		timer.end();
		double t2 = timer.getTime();
		sum_t2 += t2;

#if 1
		fout << "ModeNum = " << modeNum << ", run = " << run << endl;
		fout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << endl;
#endif

#if 1
		if (dAVRSchedulable != sAVRSchedulable) {
			cout << "ModeNum = " << modeNum << ", run = " << run << endl;
			cout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << endl;
		}
#endif

		if (dAVRSchedulable) sum_p0++;
		if (sAVRSchedulable) sum_p1++;
		if (fSAVRSchedulable) sum_p2++;
	}

	fout << "==================total=================" << endl;
	fout << modeNum << " => " 
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	modeNums.push_back(modeNum);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
	} else {
		if (pResults.size() != 3) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
	} else {
		if (tResults.size() != 3) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
	}
}

void RunTest::testECRTS18RTATestedSystems4(ofstream& fout, string directory, int minSystem, int maxSystem, int avrTaskNum, vector<int> & avrTaskNums, vector<vector<double>> & pResults, vector<vector<double>> & tResults, double tUtil, double rho) 
{
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	int sum_p0 = 0; // DAVR
	int sum_p1 = 0; // SAVR
	int sum_p2 = 0; // fSAVR

	double sum_t0 = 0; // DAVR
	double sum_t1 = 0; // SAVR
	double sum_t2 = 0; // fSAVR

	for (int run=minSystem; run <= maxSystem; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<AVRTask> avrTasks;
		int avrTaskIndex;

		string name = directory + "Util" + Utility::int_to_string(100*tUtil) + "Rho" + Utility::int_to_string(100*rho) +
			Utility::linkNotation()+"AVRTaskNum"+Utility::int_to_string(avrTaskNum)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex, engine, period,avrTasks);

		//cout << avrTaskIndex << ", => ";

		Timer timer;

		timer.start();
		bool dAVRSchedulable = false;
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			dAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!dAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (dAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				dAVRSchedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,mixedTwo, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME);
				if (!dAVRSchedulable) break;
			}
		}
		timer.end();
		double t0 = timer.getTime();
		sum_t0 += t0;

		timer.start();
		AVRTask mixedAvrTask(engine,period,avrTasks);
		bool sAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedAvrTask,avrTaskIndex);
		timer.end();
		double t1 = timer.getTime();
		sum_t1 += t1;

		timer.start();
		bool fSAVRSchedulable = false;
		// check each engine configuration
		for (int i=0; i<avrTasks.size(); i++) {
			fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
			if (!fSAVRSchedulable) break;
		}

		// check each switch of the engine configurations
		if (fSAVRSchedulable && avrTasks.size() >= 2) {
			for (int i=1; i < avrTasks.size(); i++) {
				AVRTask prevAvrTask = avrTasks[i-1];
				AVRTask avrTask = avrTasks[i];
				vector<AVRTask> two;
				two.push_back(prevAvrTask);
				two.push_back(avrTask);
				AVRTask mixedTwo(engine,period,two);

				fSAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,mixedTwo,avrTaskIndex);
				if (!fSAVRSchedulable) break;
			}
		}
		timer.end();
		double t2 = timer.getTime();
		sum_t2 += t2;

#if 1
		fout << "AVRTaskNum = " << avrTaskNum << ", run = " << run << endl;
		fout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
			<< t0 << "\t" << t1 << "\t" << t2 << endl;
#endif

#if 1
		if (dAVRSchedulable != sAVRSchedulable) {
			cout << "AVRTaskNum = " << avrTaskNum << ", run = " << run << endl;
			cout << dAVRSchedulable << "\t" << sAVRSchedulable << "\t" << fSAVRSchedulable << "\t"
				<< t0 << "\t" << t1 << "\t" << t2 << endl;
		}
#endif

		if (dAVRSchedulable) sum_p0++;
		if (sAVRSchedulable) sum_p1++;
		if (fSAVRSchedulable) sum_p2++;
	}

	fout << "==================total=================" << endl;
	fout << avrTaskNum << " => " 
		<< sum_p0 << "\t" << sum_p1 << "\t" << sum_p2 << "\t" 
		<< sum_t0 << "\t" << sum_t1 << "\t" << sum_t2 << endl;
	int size = maxSystem-minSystem+1;
	// store the results
	avrTaskNums.push_back(avrTaskNum);

	if (pResults.empty()) {
		vector<double> vec_p0;
		vector<double> vec_p1;
		vector<double> vec_p2;

		vec_p0.push_back(1.0*sum_p0/size);
		vec_p1.push_back(1.0*sum_p1/size);
		vec_p2.push_back(1.0*sum_p2/size);

		pResults.push_back(vec_p0);
		pResults.push_back(vec_p1);
		pResults.push_back(vec_p2);
	} else {
		if (pResults.size() != 3) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		pResults[0].push_back(1.0*sum_p0/size);
		pResults[1].push_back(1.0*sum_p1/size);
		pResults[2].push_back(1.0*sum_p2/size);
	}

	if (tResults.empty()) {
		vector<double> vec_t0;
		vector<double> vec_t1;
		vector<double> vec_t2;

		vec_t0.push_back(1.0*sum_t0/size);
		vec_t1.push_back(1.0*sum_t1/size);
		vec_t2.push_back(1.0*sum_t2/size);

		tResults.push_back(vec_t0);
		tResults.push_back(vec_t1);
		tResults.push_back(vec_t2);
	} else {
		if (tResults.size() != 3) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		tResults[0].push_back(1.0*sum_t0/size);
		tResults[1].push_back(1.0*sum_t1/size);
		tResults[2].push_back(1.0*sum_t2/size);
	}
}

void RunTest::testECRTS18RTATestedSystems5(ofstream& fout, string directory, int minSystem, int maxSystem, int tUtil, vector<int> & tUtils, vector<vector<double>> & pResults, vector<vector<double>> & tResults, vector<vector<double>> & dResults, double rho) {
	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;
	
	// DAVR-Tight : -1
	// DAVR-1000
	// DAVR-500
	// DAVR-300
	// DAVR-200
	// DAVR-100
	// DAVR-50
	// DAVR-20
	//static const int _CONSTANTLENGTHs[] = {-1,1000,500,300,200,100,50,20}; 
	static const int _CONSTANTLENGTHs[] = {1000,500,300,200,100,50,20}; 
	//static const int _CONSTANTLENGTHs[] = {1000,500,100}; 
	vector<int> constantLengths(_CONSTANTLENGTHs,_CONSTANTLENGTHs+sizeof(_CONSTANTLENGTHs)/sizeof(_CONSTANTLENGTHs[0]));

	vector<int> sum_scheds(constantLengths.size(),0);
	vector<double> sum_times(constantLengths.size(),0);
	vector<int> sum_digraph_nums(constantLengths.size(),0);
	vector<int> sum_partition_nums(constantLengths.size(),0);

	for (int run=minSystem; run <= maxSystem; run++) {
		//for (int run=15; run <= 15; run++) {
		//cout << "run = " << run << ", tUtil = " << 1.0*tUtil/100 << " => ";
		vector<PeriodicTask> tasks;
		vector<AVRTask> avrTasks;
		int avrTaskIndex;

		string name = directory + "Rho" + Utility::int_to_string(100*rho) +Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
		const char *p = name.c_str();
		FileReader::ReadTaskSystem(p,tasks,avrTaskIndex, engine, period,avrTasks);
		//cout << avrTaskIndex << ", => ";

		fout << "Util = " << tUtil << ", run = " << run << endl;

		for (int lengthIndex=0; lengthIndex< constantLengths.size(); lengthIndex++) {
			Timer timer;
			timer.start();
			bool dAVRSchedulable = false;
			// check each engine configuration
			for (int i=0; i<avrTasks.size(); i++) {
				dAVRSchedulable = SchedAnalysis::necessary_only_analysis(tasks,avrTasks[i],avrTaskIndex);
				if (!dAVRSchedulable) break;
			}

			// check each switch of the engine configurations
			if (dAVRSchedulable && avrTasks.size() >= 2) {
				for (int i=1; i < avrTasks.size(); i++) {
					AVRTask prevAvrTask = avrTasks[i-1];
					AVRTask avrTask = avrTasks[i];
					vector<AVRTask> two;
					two.push_back(prevAvrTask);
					two.push_back(avrTask);
					AVRTask mixedTwo(engine,period,two);

					int partitionNum = 0;
					dAVRSchedulable = SchedAnalysis::checkSchedulabilityAllPrioritiesDebug(EXACT,tasks,mixedTwo, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME,partitionNum,constantLengths[lengthIndex]);
					
					sum_digraph_nums[lengthIndex] += 1;
					sum_partition_nums[lengthIndex] += partitionNum;

					if (!dAVRSchedulable) break;
				}
			}
			timer.end();

			if (dAVRSchedulable) sum_scheds[lengthIndex] += 1;

			double time = timer.getTime();
			sum_times[lengthIndex] += time;
			
#if 1
			fout << dAVRSchedulable << "\t" << time << "\t";
#endif
		}
		fout << endl;
	}

	fout << "==================total=================" << endl;
	fout << tUtil << " => " << endl;
	Utility::output_one_vector(fout,"Scheds",sum_scheds);
	Utility::output_one_vector(fout,"Times",sum_times);
	Utility::output_one_vector(fout,"DigraphNums",sum_digraph_nums);
	Utility::output_one_vector(fout,"ParitionNums",sum_partition_nums);

	int size = maxSystem-minSystem+1;
	// store the results
	tUtils.push_back(tUtil);

	if (pResults.empty()) {
		for (int lengthIndex = 0; lengthIndex < constantLengths.size(); lengthIndex++) {
			vector<double> vec;
			vec.push_back(1.0*sum_scheds[lengthIndex]/size);
			pResults.push_back(vec);
		}
	} else {
		if (pResults.size() != constantLengths.size()) {
			cerr << "Error pResults size = " << pResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		for (int lengthIndex = 0; lengthIndex < constantLengths.size(); lengthIndex++) {
			pResults[lengthIndex].push_back(1.0*sum_scheds[lengthIndex]/size);
		}
	}

	if (tResults.empty()) {

		for (int lengthIndex = 0; lengthIndex < constantLengths.size(); lengthIndex++) {
			vector<double> vec;
			vec.push_back(1.0*sum_times[lengthIndex]/size);
			tResults.push_back(vec);
		}
	} else {
		if (tResults.size() != constantLengths.size()) {
			cerr << "Error tResults size = " << tResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		for (int lengthIndex = 0; lengthIndex < constantLengths.size(); lengthIndex++) {
			tResults[lengthIndex].push_back(1.0*sum_times[lengthIndex]/size);
		}
	}

	if (dResults.empty()) {

		for (int lengthIndex = 0; lengthIndex < constantLengths.size(); lengthIndex++) {
			vector<double> vec;
			vec.push_back(1.0*sum_partition_nums[lengthIndex]/sum_digraph_nums[lengthIndex]);
			dResults.push_back(vec);
		}
	} else {
		if (dResults.size() != constantLengths.size()) {
			cerr << "Error tResults size = " << dResults.size() << endl;
			exit(EXIT_FAILURE);
		}

		for (int lengthIndex = 0; lengthIndex < constantLengths.size(); lengthIndex++) {
			dResults[lengthIndex].push_back(1.0*sum_partition_nums[lengthIndex]/sum_digraph_nums[lengthIndex]);
		}
	}


}

void RunTest::ICCPS15_Experiment1(vector<bool> vecTest, string resultDir, string resultFile, int numPeriodicTasks, double rho, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);
	
	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> utilResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int tUtil=300; tUtil<=950; tUtil+=25) {
	//for (int tUtil=700; tUtil<=950; tUtil+=25) {
	//for (int tUtil=400; tUtil<=400; tUtil+=25) {
		RunTest::generateBiondiRTATestedSystems("ICCPS15E1",numSystem,numPeriodicTasks,tUtil,rho,4,8);
		RunTest::testBiondiRTATestedSystems(vecTest,fout,"ICCPS15E1",0,numSystem-1,tUtil,utilResults,pResults,tResults);
	}

	//RunTest::testBiondiRTATestedSystems("ICCPS15E1",606,999,30,utilResults,pResults,tResults);

	Utility::output_one_vector(cout,"utilResults",utilResults);
	Utility::output_one_vector(fout,"utilResults",utilResults);

	vector<string> ratioNames;
	ratioNames.push_back("pExact");
	ratioNames.push_back("pLUB");
	ratioNames.push_back("pNEC1");
	ratioNames.push_back("pNEC1A");
	ratioNames.push_back("pNEC2A");
	ratioNames.push_back("pRBF");
	ratioNames.push_back("pRBF2");
	ratioNames.push_back("pILPCON");
	ratioNames.push_back("pILP");
	ratioNames.push_back("pSEGILPCON");

	vector<string> timeNames;
	timeNames.push_back("tExact");
	timeNames.push_back("tLUB");
	timeNames.push_back("tNEC1");
	timeNames.push_back("tNEC1A");
	timeNames.push_back("tNEC2A");
	timeNames.push_back("tRBF");
	timeNames.push_back("tRBF2");
	timeNames.push_back("tILPCON");
	timeNames.push_back("tILP");
	timeNames.push_back("tSEGILPCON");


	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::ICCPS15_Experiment2(vector<bool> vecTest, string resultDir, string resultFile, int numPeriodicTasks, double tUtil, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> rhoResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int rho=100; rho<=900; rho+=25) {
		//RunTest::generateBiondiRTATestedSystems3("ICCPS15E2",numSystem,numPeriodicTasks,tUtil,rho,4,8);
		RunTest::generateBiondiRTATestedSystems3("ICCPS15E2",numSystem,numPeriodicTasks,tUtil,rho,6,6);
		RunTest::testBiondiRTATestedSystems2(vecTest,fout,"ICCPS15E2",0,numSystem-1,rho,rhoResults,pResults,tResults);
	}

	//RunTest::testBiondiRTATestedSystems2("ICCPS15E2",327,327,900,rhoResults,pResults,tResults);

	Utility::output_one_vector(cout,"rhoResults",rhoResults);
	Utility::output_one_vector(fout,"rhoResults",rhoResults);

	vector<string> ratioNames;
	ratioNames.push_back("pExact");
	ratioNames.push_back("pLUB");
	ratioNames.push_back("pNEC1");
	ratioNames.push_back("pNEC1A");
	ratioNames.push_back("pNEC2A");
	ratioNames.push_back("pRBF");
	ratioNames.push_back("pRBF2");
	ratioNames.push_back("pILPCON");
	ratioNames.push_back("pILP");
	ratioNames.push_back("pSEGILPCON");

	vector<string> timeNames;
	timeNames.push_back("tExact");
	timeNames.push_back("tLUB");
	timeNames.push_back("tNEC1");
	timeNames.push_back("tNEC1A");
	timeNames.push_back("tNEC2A");
	timeNames.push_back("tRBF");
	timeNames.push_back("tRBF2");
	timeNames.push_back("tILPCON");
	timeNames.push_back("tILP");
	timeNames.push_back("tSEGILPCON");


	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::ICCPS15_Experiment3(vector<bool> vecTest, string resultDir, string resultFile, int numPeriodicTasks, double tUtil, double rho, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> modeNumResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int modeNum=2; modeNum<=12; modeNum++) {
		RunTest::generateBiondiRTATestedSystems4("ICCPS15E2",numSystem,numPeriodicTasks,tUtil,rho,modeNum);
		RunTest::testBiondiRTATestedSystems3(vecTest,fout,"ICCPS15E2",0,numSystem-1,modeNum,modeNumResults,pResults,tResults);
	}

	//RunTest::testBiondiRTATestedSystems2("ICCPS15E2",327,327,900,rhoResults,pResults,tResults);

	Utility::output_one_vector(cout,"modeNumResults",modeNumResults);
	Utility::output_one_vector(fout,"modeNumResults",modeNumResults);

	vector<string> ratioNames;
	ratioNames.push_back("pExact");
	ratioNames.push_back("pLUB");
	ratioNames.push_back("pNEC1");
	ratioNames.push_back("pNEC1A");
	ratioNames.push_back("pNEC2A");
	ratioNames.push_back("pRBF");
	ratioNames.push_back("pRBF2");
	ratioNames.push_back("pILPCON");
	ratioNames.push_back("pILP");
	ratioNames.push_back("pSEGILPCON");

	vector<string> timeNames;
	timeNames.push_back("tExact");
	timeNames.push_back("tLUB");
	timeNames.push_back("tNEC1");
	timeNames.push_back("tNEC1A");
	timeNames.push_back("tNEC2A");
	timeNames.push_back("tRBF");
	timeNames.push_back("tRBF2");
	timeNames.push_back("tILPCON");
	timeNames.push_back("tILP");
	timeNames.push_back("tSEGILPCON");

	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::mytest_Experiment1(vector<bool> vecTest, string resultDir, string resultFile, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	int U_p = 85;
	vector<int> utilResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int i=1; i<=10; i+=1) {
		RunTest::generateBiondiRTATestedSystems2("RTARandomSystemsUtil",numSystem,5,U_p,i,4,8);
		RunTest::testBiondiRTATestedSystems(vecTest,fout,"RTARandomSystemsUtil",0,numSystem-1,U_p+i,utilResults,pResults,tResults);
	}

	Utility::output_one_vector(cout,"utilResults",utilResults);

	Utility::output_one_vector(cout,"pExact",pResults[0]);
	Utility::output_one_vector(cout,"pLUB",pResults[1]);
	Utility::output_one_vector(cout,"pNEC1",pResults[2]);
	Utility::output_one_vector(cout,"pNEC1A",pResults[3]);
	Utility::output_one_vector(cout,"pNEC2A",pResults[4]);

	Utility::output_one_vector(cout,"tExact",tResults[0]);
	Utility::output_one_vector(cout,"tLUB",tResults[1]);
	Utility::output_one_vector(cout,"tNEC1",tResults[2]);
	Utility::output_one_vector(cout,"tNEC1A",tResults[3]);
	Utility::output_one_vector(cout,"tNEC2A",tResults[4]);

	Utility::output_one_vector(fout,"utilResults",utilResults);

	Utility::output_one_vector(fout,"pExact",pResults[0]);
	Utility::output_one_vector(fout,"pLUB",pResults[1]);
	Utility::output_one_vector(fout,"pNEC1",pResults[2]);
	Utility::output_one_vector(fout,"pNEC1A",pResults[3]);
	Utility::output_one_vector(fout,"pNEC2A",pResults[4]);

	Utility::output_one_vector(fout,"tExact",tResults[0]);
	Utility::output_one_vector(fout,"tLUB",tResults[1]);
	Utility::output_one_vector(fout,"tNEC1",tResults[2]);
	Utility::output_one_vector(fout,"tNEC1A",tResults[3]);
	Utility::output_one_vector(fout,"tNEC2A",tResults[4]);

	fout.close();
}

void RunTest::ECRTS18_Experiment1(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double rho, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> utilResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int tUtil=300; tUtil<=950; tUtil+=25) {
	//for (int tUtil=700; tUtil<=850; tUtil+=25) {
	//for (int tUtil=850; tUtil<=850; tUtil+=25) {
#ifdef __GENERATING_NEW_TASKSETS_4_SCHED_ANALYSIS__
		RunTest::generateECRTS18RTATestedSystems("ECRTS18E1",numSystem, numAVRTasks, numPeriodicTasks,tUtil,rho,4,8);
#endif
		RunTest::testECRTS18RTATestedSystems(fout,"ECRTS18E1",0,numSystem-1,tUtil,utilResults,pResults,tResults,rho);
	}

	Utility::output_one_vector(cout,"utilResults",utilResults);
	Utility::output_one_vector(fout,"utilResults",utilResults);

	vector<string> ratioNames;
	ratioNames.push_back("pDAVR");
	ratioNames.push_back("pSAVR");
	ratioNames.push_back("pFSAVR");

	vector<string> timeNames;
	timeNames.push_back("tDAVR");
	timeNames.push_back("tSAVR");
	timeNames.push_back("tFSAVR");

	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::ECRTS18_Experiment2(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double tUtil, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> rhoResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int rho=100; rho<=900; rho+=25) {
	//for (int rho=450; rho<=900; rho+=25) {
#ifdef __GENERATING_NEW_TASKSETS_4_SCHED_ANALYSIS__
		RunTest::generateECRTS18RTATestedSystems2("ECRTS18E2",numSystem,numAVRTasks,numPeriodicTasks,tUtil,rho,4,8);
#endif
		RunTest::testECRTS18RTATestedSystems2(fout,"ECRTS18E2",0,numSystem-1,rho,rhoResults,pResults,tResults,tUtil);
	}

	Utility::output_one_vector(cout,"rhoResults",rhoResults);
	Utility::output_one_vector(fout,"rhoResults",rhoResults);

	vector<string> ratioNames;
	ratioNames.push_back("pDAVR");
	ratioNames.push_back("pSAVR");
	ratioNames.push_back("pFSAVR");

	vector<string> timeNames;
	timeNames.push_back("tDAVR");
	timeNames.push_back("tSAVR");
	timeNames.push_back("tFSAVR");

	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::ECRTS18_Experiment3(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double tUtil, double rho, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> modeNumResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int modeNum=2; modeNum<=12; modeNum++) {
#ifdef __GENERATING_NEW_TASKSETS_4_SCHED_ANALYSIS__
		RunTest::generateECRTS18RTATestedSystems3("ECRTS18E3",numSystem,numAVRTasks, numPeriodicTasks,tUtil,rho,modeNum);
#endif
		RunTest::testECRTS18RTATestedSystems3(fout,"ECRTS18E3",0,numSystem-1,modeNum,modeNumResults,pResults,tResults,tUtil,rho);
	}

	Utility::output_one_vector(cout,"modeNumResults",modeNumResults);
	Utility::output_one_vector(fout,"modeNumResults",modeNumResults);

	vector<string> ratioNames;
	ratioNames.push_back("pDAVR");
	ratioNames.push_back("pSAVR");
	ratioNames.push_back("pFSAVR");

	vector<string> timeNames;
	timeNames.push_back("tDAVR");
	timeNames.push_back("tSAVR");
	timeNames.push_back("tFSAVR");

	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::ECRTS18_Experiment4(string resultDir, string resultFile, int numPeriodicTasks, double tUtil, double rho, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> AVRTaskNumResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;

	for (int numAVRTasks=5; numAVRTasks<=50; numAVRTasks+=5) {
#ifdef __GENERATING_NEW_TASKSETS_4_SCHED_ANALYSIS__
		RunTest::generateECRTS18RTATestedSystems4("ECRTS18E4",numSystem,numAVRTasks, numPeriodicTasks,tUtil,rho,4,8);
#endif
		RunTest::testECRTS18RTATestedSystems4(fout,"ECRTS18E4",0,numSystem-1,numAVRTasks,AVRTaskNumResults,pResults,tResults,tUtil,rho);
	}

	Utility::output_one_vector(cout,"AVRTaskNumResults",AVRTaskNumResults);
	Utility::output_one_vector(fout,"AVRTaskNumResults",AVRTaskNumResults);

	vector<string> ratioNames;
	ratioNames.push_back("pDAVR");
	ratioNames.push_back("pSAVR");
	ratioNames.push_back("pFSAVR");

	vector<string> timeNames;
	timeNames.push_back("tDAVR");
	timeNames.push_back("tSAVR");
	timeNames.push_back("tFSAVR");

	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);

	fout.close();
}

void RunTest::ECRTS18_Experiment5(string resultDir, string resultFile, int numAVRTasks, int numPeriodicTasks, double rho, int tUtil, int numSystem) {
	// mkdir directory
	Utility::makeDir(resultDir);

	resultFile = resultDir + Utility::linkNotation() + resultFile;

	ofstream fout(resultFile, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<resultFile<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<int> utilResults;
	vector<vector<double>> pResults;
	vector<vector<double>> tResults;
	vector<vector<double>> dResults;

	//for (int tUtil=300; tUtil<=950; tUtil+=25) {
	//for (int tUtil=800; tUtil<=900; tUtil+=25) {
	for (tUtil=300; tUtil<=850; tUtil+=25) {
#ifdef __GENERATING_NEW_TASKSETS_4_SCHED_ANALYSIS__
		RunTest::generateECRTS18RTATestedSystems("ECRTS18E1",numSystem, numAVRTasks, numPeriodicTasks,tUtil,rho,4,8);
#endif
		RunTest::testECRTS18RTATestedSystems5(fout,"ECRTS18E1",0,numSystem-1,tUtil,utilResults,pResults,tResults,dResults,rho);
	}

	Utility::output_one_vector(cout,"utilResults",utilResults);
	Utility::output_one_vector(fout,"utilResults",utilResults);

	// DAVR-Tight : -1
	// DAVR-1000
	// DAVR-500
	// DAVR-300
	// DAVR-200
	// DAVR-100
	// DAVR-50
	// DAVR-20
	vector<string> ratioNames;
	//ratioNames.push_back("pDAVR-tight");
	ratioNames.push_back("pDAVR-1000");
	ratioNames.push_back("pDAVR-500");
	ratioNames.push_back("pDAVR-300");
	ratioNames.push_back("pDAVR-200");
	ratioNames.push_back("pDAVR-100");
	ratioNames.push_back("pDAVR-50");
	ratioNames.push_back("pDAVR-20");

	vector<string> timeNames;
	//timeNames.push_back("tDAVR-tight");
	timeNames.push_back("tDAVR-1000");
	timeNames.push_back("tDAVR-500");
	timeNames.push_back("tDAVR-300");
	timeNames.push_back("tDAVR-200");
	timeNames.push_back("tDAVR-100");
	timeNames.push_back("tDAVR-50");
	timeNames.push_back("tDAVR-20");

	vector<string> partitionNumNames;
	//partitionNumNames.push_back("pnDAVR-tight");
	partitionNumNames.push_back("pnDAVR-1000");
	partitionNumNames.push_back("pnDAVR-500");
	partitionNumNames.push_back("pnDAVR-300");
	partitionNumNames.push_back("pnDAVR-200");
	partitionNumNames.push_back("pnDAVR-100");
	partitionNumNames.push_back("pnDAVR-50");
	partitionNumNames.push_back("pnDAVR-20");

	Utility::output_multiple_vectors(cout,ratioNames,pResults);
	Utility::output_multiple_vectors(cout,timeNames,tResults);
	Utility::output_multiple_vectors(cout,partitionNumNames,dResults);

	Utility::output_multiple_vectors(fout,ratioNames,pResults);
	Utility::output_multiple_vectors(fout,timeNames,tResults);
	Utility::output_multiple_vectors(fout,partitionNumNames,dResults);

	fout.close();
}

void RunTest::generateMappingRunnables(string directory, int numSystem, int numPeriodicTask, int tUtil, double d1, double d2, int numMode) {
	// mkdir directory
	Utility::makeDir(directory);

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		TaskSystemGenerator::generate_random_runnables_for_engine_optimization(tasks,numPeriodicTask,1.0*tUtil/100, d1,d2, avrTask,numMode);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask);

		delete avrTask;

		#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
		#endif
	}
}

void RunTest::doDCBSForMappingRunnablesExpMinConstantKNecessaryOnly(string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, int numConstraint, vector<double> k1, vector<double> k2) {
	// mkdir directory
	Utility::makeDir(resultDir);

	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"ExpConstKUBSpeedsDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultOPTSpeeds = resultDir + Utility::linkNotation()+"ExpConstKOPTSpeedsDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultBSSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBSSpeedsDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultUBPerformance = resultDir + Utility::linkNotation()+"ExpConstKUBPerformanceDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultOPTPerformance = resultDir + Utility::linkNotation()+"ExpConstKOPTPerformanceDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultBSPerformance = resultDir + Utility::linkNotation()+"ExpConstKBSPerformanceDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultCollection = resultDir + Utility::linkNotation()+"ExpConstKBSResultCollectionDC"+Utility::int_to_string(tUtil)+suffix+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutOPTSpeeds(resultOPTSpeeds, ios::out | ios::trunc);
	if (!foutOPTSpeeds.is_open()) {
		cerr << "Can't open "<<resultOPTSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBSSpeeds(resultBSSpeeds, ios::out | ios::trunc);
	if (!foutBSSpeeds.is_open()) {
		cerr << "Can't open "<<resultBSSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutUBPerformance(resultUBPerformance, ios::out | ios::trunc);
	if (!foutUBPerformance.is_open()) {
		cerr << "Can't open "<<resultUBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutOPTPerformance(resultOPTPerformance, ios::out | ios::trunc);
	if (!foutOPTPerformance.is_open()) {
		cerr << "Can't open "<<resultOPTPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBSPerformance(resultBSPerformance, ios::out | ios::trunc);
	if (!foutBSPerformance.is_open()) {
		cerr << "Can't open "<<resultBSPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedPerfs;
	vector<double> normalizedTimes;

	vector<double> normalizedPerfs2;
	vector<double> normalizedTimes2;

	double maxTime = INT_MIN;
	double minTime = INT_MAX;

	double maxTime2 = INT_MIN;
	double minTime2 = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		foutUBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutOPTSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBSSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutUBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutOPTPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBSPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 

		double totalPerf = 0.0;
		double totalPerf2 = 0.0;

		double totalTime = 0.0;
		double totalTime2 = 0.0;
		int numUnFeasible = 0;

		Utility::output_one_vector(foutUBSpeeds, "k1", k1);
		Utility::output_one_vector(foutOPTSpeeds, "k1", k1);
		Utility::output_one_vector(foutBSSpeeds, "k1", k1);
		Utility::output_one_vector(foutUBPerformance, "k1", k1);
		Utility::output_one_vector(foutOPTPerformance, "k1", k1);
		Utility::output_one_vector(foutBSPerformance, "k1", k1);

		Utility::output_one_vector(foutUBSpeeds, "k2", k2);
		Utility::output_one_vector(foutOPTSpeeds, "k2", k2);
		Utility::output_one_vector(foutBSSpeeds, "k2", k2);
		Utility::output_one_vector(foutUBPerformance, "k2", k2);
		Utility::output_one_vector(foutOPTPerformance, "k2", k2);
		Utility::output_one_vector(foutBSPerformance, "k2", k2);

		int run = 0;
		int numSucc = 0;

		while (numSucc < numSystem) {
		//for (run=20;run<=20; run++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> tasks;
			vector<int> WCETs;

			string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,tasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

			vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
			OptimizationAlgorithm::enforceModeGuards(maxSpeeds);

			Timer timer;
			timer.start();
			CollectionResult OPTCR = OptimizationAlgorithm::doOptimalAlgorithmDFSMappingAndBnBMin(dc,k1,k2,WCETs,tasks,engine,period,numConstraint);
			vector<double> OPTSpeeds = OPTCR.speeds;
			//vector<double> OPTSpeeds = maxSpeeds;
			timer.end();
			double tOPT = timer.getTime();

			if (OPTSpeeds.empty()) {
				numUnFeasible ++;
				cout << endl;
				continue;
			}

			timer.start();
			CollectionResult BSCR = OptimizationAlgorithm::doHeuristicAlgorithmBFSMappingAndBSMin(dc,k1,k2,WCETs,tasks,engine,period,numConstraint);
			vector<double> BSSpeeds = BSCR.speeds;
			timer.end();
			double tBS = timer.getTime();


			if (BSSpeeds.empty()) {
				numUnFeasible ++;
				cout << endl;
				continue;
			}

			numSucc++;

			totalTime += tOPT;
			totalTime2 += tBS;

			cout << " time = " << tOPT << ", " << tBS;
			foutResultCollection << " time = " << tOPT << ", " << tBS;

			minTime = min(minTime, tOPT);
			maxTime = max(maxTime, tOPT);

			minTime2 = min(minTime2, tBS);
			maxTime2 = max(maxTime2, tBS);

			foutUBSpeeds << "System " << run << " ";
			Utility::output_one_vector(foutUBSpeeds,"Max Speeds", maxSpeeds);
			double pUB = OptimizationAlgorithm::getPerformanceDCExp(dc,maxSpeeds, k1, k2);
			foutUBPerformance << pUB << endl;

			foutOPTSpeeds << "System " << run << " ";
			Utility::output_one_vector(foutOPTSpeeds, "BS Speeds", OPTSpeeds);
			double pOPT = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTSpeeds, k1, k2);
			foutOPTPerformance << pOPT << endl;

			foutBSSpeeds << "System " << run << " ";
			Utility::output_one_vector(foutBSSpeeds, "BS2 Speeds", BSSpeeds);
			double pBS = OptimizationAlgorithm::getPerformanceDCExp(dc,BSSpeeds, k1, k2);
			foutBSPerformance << pBS << endl;

			totalPerf += pUB/pOPT;
			totalPerf2 += pUB/pBS;

			double perf = pUB/pOPT;
			double perf2 = pUB/pBS;

			cout << " normalizedPerf = " << perf << ", " << perf2;
			foutResultCollection << " normalizedPerf = " << perf << ", " << perf2;

			if (perf2-perf > 0.05) {
				cout << " ++++";
				foutResultCollection << " ++++";
			}
			cout << endl;
			foutResultCollection << endl;
		}

		int norm = numSystem;
		normalizedPerfs.push_back(totalPerf/norm);
		normalizedTimes.push_back(totalTime/norm);

		normalizedPerfs2.push_back(totalPerf2/norm);
		normalizedTimes2.push_back(totalTime2/norm);

		cout << "minTime = " << minTime << endl;
		cout << "maxTime = " << maxTime << endl;

		cout << "minTime2 = " << minTime2 << endl;
		cout << "maxTime2 = " << maxTime2 << endl;
		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

		Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);
	}

	cout << "minTime = " << minTime << endl;
	cout << "maxTime = " << maxTime << endl;

	cout << "minTime2 = " << minTime2 << endl;
	cout << "maxTime2 = " << maxTime2 << endl;

	Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);

	foutResultCollection << "minTime = " << minTime << endl;
	foutResultCollection << "maxTime = " << maxTime << endl;

	foutResultCollection << "minTime2 = " << minTime2 << endl;
	foutResultCollection << "maxTime2 = " << maxTime2 << endl;

	foutUBSpeeds.close();
	foutOPTSpeeds.close();
	foutBSSpeeds.close();
	foutUBPerformance.close();
	foutOPTPerformance.close();
	foutBSPerformance.close();
	foutResultCollection.close();
}

void RunTest::doDCBSForMappingRunnablesExpMinConstantKNecessaryOnlyBaseline(string resultDir, string systemDir, string dcFile, string suffix, int tUtil, int numSystem, int minFactor, int maxFactor, int numConstraint, vector<double> k1, vector<double> k2) {
	// mkdir directory
	Utility::makeDir(resultDir);

	const char *dcFilePointer = dcFile.c_str();
	map<int,int> dc = FileReader::ReadDrivingCycle(dcFilePointer);

	string resultUBSpeeds = resultDir + Utility::linkNotation()+"ExpConstKUBSpeedsDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultOPTSpeeds = resultDir + Utility::linkNotation()+"ExpConstKOPTSpeedsDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultBaseSpeeds = resultDir + Utility::linkNotation()+"ExpConstKBaseSpeedsDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultUBPerformance = resultDir + Utility::linkNotation()+"ExpConstKUBPerformanceDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultOPTPerformance = resultDir + Utility::linkNotation()+"ExpConstKOPTPerformanceDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultBasePerformance = resultDir + Utility::linkNotation()+"ExpConstKBasePerformanceDCUtil"+Utility::int_to_string(tUtil)+suffix+".result";
	string resultCollection = resultDir + Utility::linkNotation()+"ExpConstKBaseResultCollectionDC"+Utility::int_to_string(tUtil)+suffix+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutOPTSpeeds(resultOPTSpeeds, ios::out | ios::trunc);
	if (!foutOPTSpeeds.is_open()) {
		cerr << "Can't open "<<resultOPTSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBaseSpeeds(resultBaseSpeeds, ios::out | ios::trunc);
	if (!foutBaseSpeeds.is_open()) {
		cerr << "Can't open "<<resultBaseSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutUBPerformance(resultUBPerformance, ios::out | ios::trunc);
	if (!foutUBPerformance.is_open()) {
		cerr << "Can't open "<<resultUBPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutOPTPerformance(resultOPTPerformance, ios::out | ios::trunc);
	if (!foutOPTPerformance.is_open()) {
		cerr << "Can't open "<<resultOPTPerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutBasePerformance(resultBasePerformance, ios::out | ios::trunc);
	if (!foutBasePerformance.is_open()) {
		cerr << "Can't open "<<resultBasePerformance<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> normalizedPerfs;
	vector<double> normalizedTimes;

	vector<double> normalizedPerfs2;
	vector<double> normalizedTimes2;

	double maxTime = INT_MIN;
	double minTime = INT_MAX;

	double maxTime2 = INT_MIN;
	double minTime2 = INT_MAX;

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		foutUBSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutOPTSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBaseSpeeds << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutUBPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 
		foutOPTPerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
		foutBasePerformance << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl; 

		double totalPerf = 0.0;
		double totalPerf2 = 0.0;

		double totalTime = 0.0;
		double totalTime2 = 0.0;
		int numUnFeasible = 0;

		Utility::output_one_vector(foutUBSpeeds, "k1", k1);
		Utility::output_one_vector(foutOPTSpeeds, "k1", k1);
		Utility::output_one_vector(foutBaseSpeeds, "k1", k1);
		Utility::output_one_vector(foutUBPerformance, "k1", k1);
		Utility::output_one_vector(foutOPTPerformance, "k1", k1);
		Utility::output_one_vector(foutBasePerformance, "k1", k1);

		Utility::output_one_vector(foutUBSpeeds, "k2", k2);
		Utility::output_one_vector(foutOPTSpeeds, "k2", k2);
		Utility::output_one_vector(foutBaseSpeeds, "k2", k2);
		Utility::output_one_vector(foutUBPerformance, "k2", k2);
		Utility::output_one_vector(foutOPTPerformance, "k2", k2);
		Utility::output_one_vector(foutBasePerformance, "k2", k2);

		int run = 0;
		int numSucc = 0;

		while (numSucc < numSystem) {
		//for (run=55;run<=55; run++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> tasks;
			vector<int> WCETs;

			string name = systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,tasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

			vector<double> maxSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,tasks,engine,period);
			OptimizationAlgorithm::enforceModeGuards(maxSpeeds);

			Timer timer;
			timer.start();
			vector<double> OPTSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,WCETs, tasks, engine, period);
			//vector<double> OPTSpeeds = OptimizationAlgorithm::computeBiondiBBDCExpMin(dc,EXACT,k1,k2,WCETs, tasks, engine, period);
			timer.end();
			double tOPT = timer.getTime();

			if (OPTSpeeds.empty()) {
				numUnFeasible ++;
				cout << endl;
				continue;
			}

			// Baseline:
			// Clustering the periodic tasks with the same period into one single task
			map<int,int> record0;
			map<int,int> record1;
			for (int i=0; i<tasks.size(); i++) {
				PeriodicTask taski = tasks[i];
				int wcet = taski.wcet;
				int period = taski.period;
				int deadline = taski.deadline;

				if (record0.find(period) != record0.end()) {
					record0[period] += wcet;
				}
				else
					record0[period] = wcet;

				if (record1.find(period) != record1.end()) {
					record1[period] = min(record1[period],deadline);
				}
				else
					record1[period] = deadline;
			}

			vector<PeriodicTask> clusteredTasks;
			for (auto e: record0) {
				PeriodicTask task(e.second, record1[e.first], e.first);
				clusteredTasks.push_back(task);
			}

			timer.start();
			vector<double> BSSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,WCETs, clusteredTasks, engine, period);
			//vector<double> BSSpeeds = OptimizationAlgorithm::computeBiondiBBDCExpMin(dc,EXACT,k1,k2,WCETs, clusteredTasks, engine, period);
			timer.end();
			double tBS = timer.getTime();

			if (BSSpeeds.empty()) {
				numUnFeasible ++;
				cout << endl;
				continue;
			}

			numSucc++;

			totalTime += tOPT;
			totalTime2 += tBS;

			cout << " time = " << tOPT << ", " << tBS;
			foutResultCollection << " time = " << tOPT << ", " << tBS;

			minTime = min(minTime, tOPT);
			maxTime = max(maxTime, tOPT);

			minTime2 = min(minTime2, tBS);
			maxTime2 = max(maxTime2, tBS);

			foutUBSpeeds << "System " << run << " ";
			Utility::output_one_vector(foutUBSpeeds,"Max Speeds", maxSpeeds);
			double pUB = OptimizationAlgorithm::getPerformanceDCExp(dc,maxSpeeds, k1, k2);
			foutUBPerformance << pUB << endl;

			foutOPTSpeeds << "System " << run << " ";
			Utility::output_one_vector(foutOPTSpeeds, "BS Speeds", OPTSpeeds);
			double pOPT = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTSpeeds, k1, k2);
			foutOPTPerformance << pOPT << endl;

			foutBaseSpeeds << "System " << run << " ";
			Utility::output_one_vector(foutBaseSpeeds, "BS2 Speeds", BSSpeeds);
			double pBS = OptimizationAlgorithm::getPerformanceDCExp(dc,BSSpeeds, k1, k2);
			foutBasePerformance << pBS << endl;

			totalPerf += pUB/pOPT;
			totalPerf2 += pUB/pBS;

			double perf = pUB/pOPT;
			double perf2 = pUB/pBS;

			cout << " normalizedPerf = " << perf << ", " << perf2;
			foutResultCollection << " normalizedPerf = " << perf << ", " << perf2;

			if (perf - perf2 > 0.01) {
				cout << " ++++";
				foutResultCollection << " ++++";
				//exit(EXIT_FAILURE);
			}
			cout << endl;
			foutResultCollection << endl;
		}

		int norm = numSystem;
		normalizedPerfs.push_back(totalPerf/norm);
		normalizedTimes.push_back(totalTime/norm);

		normalizedPerfs2.push_back(totalPerf2/norm);
		normalizedTimes2.push_back(totalTime2/norm);

		cout << "minTime = " << minTime << endl;
		cout << "maxTime = " << maxTime << endl;

		cout << "minTime2 = " << minTime2 << endl;
		cout << "maxTime2 = " << maxTime2 << endl;
		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

		Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
		Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);
	}

	cout << "minTime = " << minTime << endl;
	cout << "maxTime = " << maxTime << endl;

	cout << "minTime2 = " << minTime2 << endl;
	cout << "maxTime2 = " << maxTime2 << endl;

	Utility::output_one_vector(cout, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(cout, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(cout, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(cout, "normalizedTimes2", normalizedTimes2);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs", normalizedPerfs);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes", normalizedTimes);

	Utility::output_one_vector(foutResultCollection, "normalizedPerfs2", normalizedPerfs2);
	Utility::output_one_vector(foutResultCollection, "normalizedTimes2", normalizedTimes2);

	foutResultCollection << "minTime = " << minTime << endl;
	foutResultCollection << "maxTime = " << maxTime << endl;

	foutResultCollection << "minTime2 = " << minTime2 << endl;
	foutResultCollection << "maxTime2 = " << maxTime2 << endl;

	foutUBSpeeds.close();
	foutOPTSpeeds.close();
	foutBaseSpeeds.close();
	foutUBPerformance.close();
	foutOPTPerformance.close();
	foutBasePerformance.close();
	foutResultCollection.close();
}

void RunTest::generateFuelInjectionApplications(string directory, double tUtil, int numSystem, bool fixedWCET, int numMode) {
	// mkdir directory
	Utility::makeDir(directory);

	for (int i=0; i<numSystem; i++) {
		vector<PeriodicTask> tasks;
		AVRTask* avrTask;

		TaskSystemGenerator::generate_fuel_injection_applications_for_engine_optimization(tasks,avrTask,fixedWCET,numMode);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask);

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}
}

void RunTest::generateFuelInjectionApplications2(string directory, double tUtil, int numSystem, bool fixedWCET, int numMode) {
	// mkdir directory
	Utility::makeDir(directory);

	for (int i=0; i<numSystem; i++) {
		vector<AsynchronousPeriodicTask> tasks;
		AVRTask* avrTask;

		TaskSystemGenerator::generate_fuel_injection_applications_for_engine_optimization2(tasks,avrTask,fixedWCET,numMode);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask);

		/*
		string name2 = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".latex";
		const char *p2 = name2.c_str();
		FileWriter::WriteTaskSystemLatex(p2,tasks,*avrTask);
		*/

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}
}

void RunTest::generateExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK, double k2Min, double k2Max)
{
	// mkdir directory
	Utility::makeDir(director);
	filename = director + Utility::linkNotation() + filename + ".functions";

	ofstream fout(filename, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<filename<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << "#start numK " << numK << " sizeK " << sizeK << " k2Min " << k2Min << " k2Max " << k2Max << endl;

	for (int i=0; i< numK; i++) {
		vector<double> k1(sizeK,1.0);
		//vector<double> k2 = TaskSystemGenerator::generate_coefficients_k2(sizeK,k2Min,k2Max);
		vector<double> k2 = TaskSystemGenerator::generate_Biondi_coefficients_k2(sizeK,k2Min,k2Max);

		/*
		static const double _k2[] = {0,200,500,1000,1500,2000};
		vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));
		*/

		Utility::output_one_vector(fout,"k1",k1);
		Utility::output_one_vector(fout,"k2",k2);
	}

	fout << "#end" << endl; 

	fout.close();
}

void RunTest::generateExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK, double k2Min, double k2Max, double k2Step)
{
	// mkdir directory
	Utility::makeDir(director);
	filename = director + Utility::linkNotation() + filename + ".functions";

	ofstream fout(filename, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<filename<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << "#start numK " << numK << " sizeK " << sizeK << " k2Min " << k2Min << " k2Max " << k2Max << " k2Step " << k2Step << endl;

	for (int i=0; i< numK; i++) {
		vector<double> k1(sizeK,1.0);
		vector<double> k2 = TaskSystemGenerator::generate_coefficients_k2(sizeK,k2Min,k2Max,k2Step);

		/*
		static const double _k2[] = {0,200,500,1000,1500,2000};
		vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));
		*/

		Utility::output_one_vector(fout,"k1",k1);
		Utility::output_one_vector(fout,"k2",k2);
	}

	fout << "#end" << endl; 

	fout.close();
}

void RunTest::generateExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK, double k1Min, double k1Max, double k1Step, double k2Min, double k2Max)
{
	// mkdir directory
	Utility::makeDir(director);
	filename = director + Utility::linkNotation() + filename + ".functions";

	ofstream fout(filename, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<filename<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << "#start numK " << numK << " sizeK " << sizeK << " k2Min " << k2Min << " k2Max " << k2Max << endl;

	for (int i=0; i< numK; i++) {
		//vector<double> k1(sizeK,1.0);
		vector<double> k1 = Utility::uniforms(false,k1Min,k1Max,k1Step,sizeK);
		vector<double> k2 = TaskSystemGenerator::generate_coefficients_k2(sizeK,k2Min,k2Max);

		/*
		static const double _k2[] = {0,200,500,1000,1500,2000};
		vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));
		*/

		Utility::output_one_vector(fout,"k1",k1);
		Utility::output_one_vector(fout,"k2",k2);
	}

	fout << "#end" << endl; 

	fout.close();
}

void RunTest::generateConstantPerformanceFunctions(string director, string filename, int numK, int sizeK, double kMin, double kMax, double kStep)
{
	// mkdir directory
	Utility::makeDir(director);
	filename = director + Utility::linkNotation() + filename + ".functions";

	ofstream fout(filename, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<filename<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << "#start numK " << numK << " sizeK " << sizeK << " kMin " << kMin << " kMax " << kMax << " kStep " << kStep << endl;

	for (int i=0; i< numK; i++) {
		vector<double> k = Utility::uniforms(false,kMin,kMax,kStep,sizeK);
		Utility::output_one_vector(fout,"k",k);
	}

	fout << "#end" << endl; 

	fout.close();
}

void RunTest::generateEmissionExponentialPerformanceFunctions(string director, string filename, int numK, int sizeK)
{
	// mkdir directory
	Utility::makeDir(director);
	filename = director + Utility::linkNotation() + filename + ".functions";

	ofstream fout(filename, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<filename<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << "#start numK " << numK << " sizeK " << sizeK << endl;

	if (sizeK == 6) {
		static const double _k1[] = {1000, 2000, 3000, 6877,9853,11900};
		vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

		static const double _k2[] = {5300,5400,5500,5763,5621,5686};
		vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

		Utility::output_one_vector(fout,"k1",k1);
		Utility::output_one_vector(fout,"k2",k2);
	}
	else if (sizeK == 3) { // Simulated Emission Data
		static const double _k1[] = {6877,9853,11900};
		vector<double> k1 (_k1,_k1+sizeof(_k1)/sizeof(_k1[0]));

		static const double _k2[] = {5763,5621,5686};
		vector<double> k2 (_k2,_k2+sizeof(_k2)/sizeof(_k2[0]));

		Utility::output_one_vector(fout,"k1",k1);
		Utility::output_one_vector(fout,"k2",k2);
	}
	else {
		cerr << "Error: sizeK = " << sizeK << endl;
		exit(EXIT_FAILURE);
	}

	fout << "#end" << endl; 

	fout.close();
}

void RunTest::RTADAVRExp(bool checkCaseStudy, string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, string kRatio, int tUtil, int numSystem, int minFactor, int maxFactor, int granularity)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	factorName += "kRatio" + kRatio;

#ifndef __DISABLE_MOST_OUTPUT__
	// mkdir
	string SAVRAOSpeedsDir = director + Utility::linkNotation()+"SAVRAOSpeeds"+factorName;
	Utility::makeDir(SAVRAOSpeedsDir);

	string SAVRSOSpeedsDir = director + Utility::linkNotation()+"SAVRSOSpeeds"+factorName;
	Utility::makeDir(SAVRSOSpeedsDir);

	string DCResultsDir = director + Utility::linkNotation()+"DCResults"+factorName;
	Utility::makeDir(DCResultsDir);

	string DCAOIndexResultsDir = director + Utility::linkNotation()+"DCAOIndexResults"+factorName;
	Utility::makeDir(DCAOIndexResultsDir);

	string DCSOIndexResultsDir = director + Utility::linkNotation()+"DCSOIndexResults"+factorName;
	Utility::makeDir(DCSOIndexResultsDir);
#endif

	string ExpFunsResultsDir = director + Utility::linkNotation()+"ExpFunsResults"+factorName;
	Utility::makeDir(ExpFunsResultsDir);

	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

#ifndef __DISABLE_MOST_OUTPUT__
	string resultUBAOSpeeds = director + Utility::linkNotation()+"UBAOSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultUBSOSpeeds = director + Utility::linkNotation()+"UBSOSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
#endif

	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";

#ifndef __DISABLE_MOST_OUTPUT__
	ofstream foutUBAOSpeeds(resultUBAOSpeeds, ios::out | ios::trunc);
	if (!foutUBAOSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBAOSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutUBSOSpeeds(resultUBSOSpeeds, ios::out | ios::trunc);
	if (!foutUBSOSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSOSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}
#endif

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

#ifndef __DISABLE_MOST_OUTPUT__
	vector<ofstream*> dcResultOfstreams;
	for (auto dcFile:dcFiles) {
		dcFile = DCResultsDir + Utility::linkNotation() + dcFile + ".result";
		ofstream* fout = new ofstream(dcFile,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<dcFile<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		dcResultOfstreams.push_back(fout);
	}
#endif

	// read the exponential performance functions
	expFunsFile = director + Utility::linkNotation() + expFunsFile + ".functions";;
	int numK = 0;
	vector<vector<double>> k1s;
	vector<vector<double>> k2s;

	FileReader::ReadExponentialFunctions(expFunsFile.c_str(),numK,k1s,k2s);

	vector<ofstream*> expFunsResultOfstreams;
	for (int i=0; i<numK; i++) {
		string expFuncResult = ExpFunsResultsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(expFuncResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<expFuncResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		expFunsResultOfstreams.push_back(fout);
	}

#ifndef __DISABLE_MOST_OUTPUT__
	vector<ofstream*> SAVRAOSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string SAVRAOSpeedsResult = SAVRAOSpeedsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(SAVRAOSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<SAVRAOSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		SAVRAOSpeedsOfstreams.push_back(fout);
	}

	vector<ofstream*> SAVRSOSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string SAVRSOSpeedsResult = SAVRSOSpeedsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(SAVRSOSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<SAVRSOSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		SAVRSOSpeedsOfstreams.push_back(fout);
	}
#endif


	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedUBAOOPAPerfs;
	vector<double> normalizedUBAOFPPerfs;
	vector<double> normalizedSAVRAOPerfs;
	vector<double> normalizedSAVRSOPerfs;
	vector<double> normalizedDAVRAOPerfs;
	vector<double> normalizedDAVRAOFastPerfs;
	vector<double> normalizedDAVRSOPerfs;

	vector<double> normalizedMaxUBAOOPAPerfs;
	vector<double> normalizedMaxUBAOFPPerfs;
	vector<double> normalizedMaxSAVRAOPerfs;
	vector<double> normalizedMaxSAVRSOPerfs;
	vector<double> normalizedMaxDAVRAOPerfs;
	vector<double> normalizedMaxDAVRAOFastPerfs;
	vector<double> normalizedMaxDAVRSOPerfs;

	vector<double> normalizedMinUBAOOPAPerfs;
	vector<double> normalizedMinUBAOFPPerfs;
	vector<double> normalizedMinSAVRAOPerfs;
	vector<double> normalizedMinSAVRSOPerfs;
	vector<double> normalizedMinDAVRAOPerfs;
	vector<double> normalizedMinDAVRAOFastPerfs;
	vector<double> normalizedMinDAVRSOPerfs;

	vector<double> averageUBAOOPATimes;
	vector<double> averageUBAOFPTimes;
	vector<double> averageUBSOTimes;
	vector<double> averageSAVRAOTimes;
	vector<double> averageSAVRSOTimes;
	vector<double> averageDAVRAOTimes;
	vector<double> averageDAVRAOFastTimes;
	vector<double> averageDAVRSOTimes;
	vector<double> averageOPTTimes; 

	vector<double> maxUBAOOPATimes;
	vector<double> maxUBAOFPTimes;
	vector<double> maxUBSOTimes;
	vector<double> maxSAVRAOTimes;
	vector<double> maxSAVRSOTimes;
	vector<double> maxDAVRAOTimes;
	vector<double> maxDAVRAOFastTimes;
	vector<double> maxDAVRSOTimes;
	vector<double> maxOPTTimes;

	vector<int> UBAOOPAFailedNums;
	vector<int> UBAOFPFailedNums;
	vector<int> UBSOFailedNums;
	vector<int> SAVRAOFailedNums;
	vector<int> SAVRSOFailedNums;
	vector<int> DAVRAOFailedNums;
	vector<int> DAVRAOFastFailedNums;
	vector<int> DAVRSOFailedNums;
	
	vector<int> UBAOOPANums;
	vector<int> UBAOFPNums;
	vector<int> UBSONums;
	vector<int> SAVRAONums;
	vector<int> SAVRSONums;
	vector<int> DAVRAONums;
	vector<int> DAVRAOFastNums;
	vector<int> DAVRSONums;
	vector<int> OPTNums;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalUBAOOPAPerf = 0.0;
		double totalUBAOFPPerf = 0.0;
		double totalSAVRAOPerf = 0.0;
		double totalSAVRSOPerf = 0.0;
		double totalDAVRAOPerf = 0.0;
		double totalDAVRAOFastPerf = 0.0;
		double totalDAVRSOPerf = 0.0;

		double maxUBAOOPAPerf = 0.0;
		double maxUBAOFPPerf = 0.0;
		double maxSAVRAOPerf = 0.0;
		double maxSAVRSOPerf = 0.0;
		double maxDAVRAOPerf = 0.0;
		double maxDAVRAOFastPerf = 0.0;
		double maxDAVRSOPerf = 0.0;

		double minUBAOOPAPerf = INT_MAX;
		double minUBAOFPPerf = INT_MAX;
		double minSAVRAOPerf = INT_MAX;
		double minSAVRSOPerf = INT_MAX;
		double minDAVRAOPerf = INT_MAX;
		double minDAVRAOFastPerf = INT_MAX;
		double minDAVRSOPerf = INT_MAX;

		double totalUBAOOPATime = 0.0;
		double totalUBAOFPTime = 0.0;
		double totalUBSOTime = 0.0;
		double totalSAVRAOTime = 0.0;
		double totalSAVRSOTime = 0.0;
		double totalDAVRAOTime = 0.0;
		double totalDAVRAOFastTime = 0.0;
		double totalDAVRSOTime = 0.0;
		double totalOPTTime = 0.0;


		double maxUBAOOPATime = 0.0;
		double maxUBAOFPTime = 0.0;
		double maxUBSOTime = 0.0;
		double maxSAVRAOTime = 0.0;
		double maxSAVRSOTime = 0.0;
		double maxDAVRAOTime = 0.0;
		double maxDAVRAOFastTime = 0.0;
		double maxDAVRSOTime = 0.0;
		double maxOPTTime = 0.0;

		int UBAOOPANum = 0;
		int UBAOFPNum = 0;
		int UBSONum = 0;
		int SAVRAONum = 0;
		int SAVRSONum = 0;
		int DAVRAONum = 0;
		int DAVRAOFastNum = 0;
		int DAVRSONum = 0;

		int totalOPTNum = 0;

		int UBAOOPAFailedNum = 0;
		int UBAOFPFailedNum = 0;
		int UBSOFailedNum = 0;
		int SAVRAOFailedNum = 0;
		int SAVRSOFailedNum = 0;
		int DAVRAOFailedNum = 0;
		int DAVRAOFastFailedNum = 0;
		int DAVRSOFailedNum = 0;

		//int numSucc = 0;

		bool firstRecord = true;

		//while (numSucc < numSystem) {
		for (int run=0; run < numSystem; run ++) {
#ifndef __DISABLE_MOST_OUTPUT__
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
#endif
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks, WCETs);

			bool UBAOOPAFailed = false;
			bool UBAOFPFailed = false;
			bool UBSOFailed = false;
			bool SAVRAOFailed = false;
			bool SAVRSOFailed = false;
			bool DAVRAOFailed = false;
			bool DAVRAOFastFailed = false;
			bool DAVRSOFailed = false;

			if (checkCaseStudy) {
				for (int i=0; i<WCETs.size(); i++) {
					if (i!=WCETs.size()-1)
						WCETs[i] *= 0.1*factor;
					else WCETs[i] = 148+416+100+330+10;
				}
			}
			else {
				for (int i=0; i<WCETs.size(); i++) {
					WCETs[i] *= factor;
				}
			}
			
			vector<double> UBSOSpeeds; 
			vector<double> UBAOOPASpeeds;
			double tUBSO = 0.0, tUBAOOPA = 0.0;

#ifdef __USING_STATIC_OFFSETS__
			cout << "Start to calculate the upper bound speeds with offset" << endl;
			timer.start();
			UBSOSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
			//AUBSpeeds = PUBSpeeds;
			timer.end();
			tUBSO = timer.getTime();
			cout << "End. time = " << tUBSO << endl;

			UBSONum ++;
			totalUBSOTime += tUBSO;
			maxUBSOTime = max(maxUBSOTime,tUBSO);

			if (UBSOSpeeds.empty()) {
				UBSOFailed = true;
				UBSOFailedNum ++;
			}
			else {
				Utility::output_one_vector(foutUBSOSpeeds,"Result Speeds",UBSOSpeeds);
			}
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
#ifndef __DISABLE_MOST_OUTPUT__
			cout << "Start to calculate the upper bound speeds without offset" << endl;
#endif
			timer.start();
			//UBAOSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
			UBAOOPASpeeds = OptimizationAlgorithm::computeUBSpeeds(NECESSARY_ONLY1,WCETs,pTasks,engine,period);
			timer.end();
			tUBAOOPA = timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
			cout << "End. time = " << tUBAOOPA << endl;
#endif

			UBAOOPANum ++;
			totalUBAOOPATime += tUBAOOPA;
			maxUBAOOPATime = max(maxUBAOOPATime,tUBAOOPA);

			if (UBAOOPASpeeds.empty()) {
				UBAOOPAFailed = true;
				UBAOOPAFailedNum ++;
			}
			else {
#ifndef __DISABLE_MOST_OUTPUT__
				Utility::output_one_vector(foutUBAOSpeeds,"Result Speeds",UBAOOPASpeeds);
#endif
			}
#endif

			{
				map<int,list<int>> dcAOResults;
				map<int,list<int>> dcAOFastResults;
				map<int,list<int>> dcSOResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k1 = k1s[noK];
					vector<double> k2 = k2s[noK];

					vector<double> SAVRAOSpeeds;
#ifndef __DISABLE_MOST_OUTPUT__
					if (firstRecord) {
						// record factor, k1 and k2
						for (auto pointer : expFunsResultOfstreams) {
							(*pointer) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
							Utility::output_one_vector((*pointer), "k1",k1);
							Utility::output_one_vector((*pointer), "k2",k2);
						}

						firstRecord = false;
					}
#endif

#ifdef __USING_ARBITRARY_OFFSETS__

#ifndef __DISABLE_MOST_OUTPUT__
					cout << "Start to calculate the BS speeds without offset" << endl;
#endif
					//cout << "++++++++++++++" << endl;
					//Utility::output_one_vector(cout,"UB",UBAOSpeeds);
					//Utility::output_one_vector(cout,"WCETs",WCETs);
					//exit(EXIT_FAILURE);
					/// Perform the optimization procedures
					timer.start();
					//SAVRAOSpeeds = UBAOOPASpeeds;
					//SAVRAOSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, pTasks, engine, period);
					SAVRAOSpeeds = OptimizationAlgorithm::computeBiondiBSExp(NECESSARY_ONLY1,k1,k2,WCETs, pTasks, engine, period);
					timer.end();
					double tSAVRAO = timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
					cout << "End. time = " << tSAVRAO << endl;
#endif

					SAVRAONum ++;
					totalSAVRAOTime += tSAVRAO;
					maxSAVRAOTime = max(maxSAVRAOTime,tSAVRAO);
					
					if (SAVRAOSpeeds.empty()) {
						SAVRAOFailed = true;
						SAVRAOFailedNum++;
						//exit(EXIT_FAILURE);
					}
					else {
#ifndef __DISABLE_MOST_OUTPUT__
						Utility::output_one_vector(*SAVRAOSpeedsOfstreams[noK],"Result Speeds",SAVRAOSpeeds);
#endif
					}
#endif

					vector<double> SAVRSOSpeeds;
#ifdef __USING_STATIC_OFFSETS__
					cout << "Start to calculate the BS speeds with offset" << endl;
					/// Perform the optimization procedures
					timer.start();
					//SAVRSpeeds = PUBSpeeds;
					SAVRSOSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, aTasks, engine, period);
					timer.end();
					double tSAVRSO = timer.getTime();
					cout << "End. time = " << tSAVRSO << endl;

					SAVRSONum ++;
					totalSAVRSOTime += tSAVRSO;
					maxSAVRSOTime = max(maxSAVRSOTime,tSAVRSO);

					if (SAVRSOSpeeds.empty()) {
						SAVRSOFailed = true;
						SAVRSOFailedNum++;
					}
					else {
						Utility::output_one_vector(*SAVRSOSpeedsOfstreams[noK],"Result Speeds",SAVRSOSpeeds);
					}
#endif

					{
						for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
							list<double> dcList = dcListVec[noDC];
#ifdef __USING_ARBITRARY_OFFSETS__
							double pUBAOOPA =  OptimizationAlgorithm::getPerformanceDCExp(dcList,UBAOOPASpeeds, k1, k2);
							double pSAVRAO = OptimizationAlgorithm::getPerformanceDCExp(dcList,SAVRAOSpeeds, k1, k2);

							//Utility::output_one_vector(cout,"UBAOOPASpeeds", UBAOOPASpeeds);
							//Utility::output_one_vector(cout,"SAVRAOSpeeds", SAVRAOSpeeds);
							//cout << dcList.size() << " " << pUBAOOPA << " " << pSAVRAO << endl;
							//exit (EXIT_FAILURE);

							double tDAVRAO = 0.0;
							double tDAVRAOFast = 0.0;

							
							int avrTaskIndex = -1;
							timer.start();
							//UBAOSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
							vector<double> UBAOFPSpeeds = OptimizationAlgorithm::computeUBFPSpeeds(NECESSARY_ONLY1,WCETs,pTasks,engine,period,dcList,k1,k2,avrTaskIndex);
							timer.end();
							double tUBAOFP = timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
							cout << "End. time = " << tUBAOFP << endl;
#endif
							double pUBAOFP =  OptimizationAlgorithm::getPerformanceDCExp(dcList,UBAOFPSpeeds, k1, k2);

							UBAOFPNum ++;
							totalUBAOFPTime += tUBAOOPA;
							maxUBAOFPTime = max(maxUBAOFPTime,tUBAOFP);

							if (UBAOFPSpeeds.empty() || avrTaskIndex < 0) {
								UBAOFPFailed = true;
								UBAOFPFailedNum ++;
							}

							if (W_LEQ_TOL(pUBAOOPA,0.0) && !UBAOOPAFailed) {
								UBAOOPAFailed = true;
								UBAOOPAFailedNum ++;
							}

							if (W_LEQ_TOL(pUBAOFP,0.0) && !UBAOFPFailed) {
								UBAOFPFailed = true;
								UBAOFPFailedNum ++;
							}

							if (W_LEQ_TOL(pSAVRAO,0.0) && !SAVRAOFailed) {
								SAVRAOFailed = true;
								SAVRAOFailedNum ++;
							}

#ifndef __DISABLE_MOST_OUTPUT__
							Utility::output_one_vector(cout,"UBAOSpeeds",UBAOOPASpeeds);
#endif
							if (SAVRAOFailed && !UBAOOPAFailed) { // We need to recalculate 
#if 0
								pSAVRAO = INT_MIN;
								for (int noSpeed=0; noSpeed < UBAOOPASpeeds.size()-1; noSpeed++) {
									vector<double> localSpeeds;
									vector<double> localK1;
									vector<double> localK2;

									localSpeeds.push_back(UBAOOPASpeeds[noSpeed]);
									localSpeeds.push_back(UBAOOPASpeeds.back());

									localK1.push_back(k1[noSpeed]);
									localK1.push_back(k1.back());

									localK2.push_back(k2[noSpeed]);
									localK2.push_back(k2.back());

									double localPerf = OptimizationAlgorithm::getPerformanceDCExp(dcList,localSpeeds, localK1, localK2);
									pSAVRAO = max(pSAVRAO,localPerf);
								}
#else
								vector<double> localSpeeds;
								vector<double> localK1;
								vector<double> localK2;

								localSpeeds.push_back(UBAOOPASpeeds.front());
								localSpeeds.push_back(UBAOOPASpeeds.back());

								localK1.push_back(k1.front());
								localK1.push_back(k1.back());

								localK2.push_back(k2.front());
								localK2.push_back(k2.back());

								pSAVRAO = OptimizationAlgorithm::getPerformanceDCExp(dcList,localSpeeds, localK1, localK2);
#endif
								//cout << pSAVRAO << " " << pUBAOOPA << endl;
								//exit(EXIT_FAILURE);
							}

							double rSAVRAO = pSAVRAO/pUBAOOPA;
#endif


#ifdef __USING_STATIC_OFFSETS__
							double pUBSO =  OptimizationAlgorithm::getPerformanceDCExp(dcList,UBSOSpeeds, k1, k2);
							double pSAVRSO = OptimizationAlgorithm::getPerformanceDCExp(dcList,SAVRSOSpeeds, k1, k2);
							double tDAVRSO = 0.0;

							if (W_LEQ_TOL(pUBSO,0.0) && !UBSOFailed) {
								UBSOFailed = true;
								UBSOFailedNum ++;
							}

							if (W_LEQ_TOL(pSAVRSO,0.0) && !SAVRSOFailed) {
								SAVRSOFailed = true;
								SAVRSOFailedNum ++;
							}

							Utility::output_one_vector(cout,"UBSOSpeeds",UBSOSpeeds);

#ifdef __USING_ARBITRARY_OFFSETS__
							double rUBAO = pUBAOOPA/pUBSO;
							rSAVRAO = pSAVRAO/pUBSO;
#endif
							double rSAVRSO = pSAVRSO/pUBSO;
#endif

#ifndef __DISABLE_MOST_OUTPUT__
							cout << "Before DAVR->" << endl; 
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
#ifdef __USING_ARBITRARY_OFFSETS__
							cout << "=>" << rSAVRAO << " "  << tUBAOOPA << " " << tSAVRAO << endl;
#endif 

#ifdef __USING_STATIC_OFFSETS__
							cout << "=>" << rSAVRSO << " "  << tUBSO << " " << tSAVRSO << endl;
#endif
								
#endif
							list<int> dcAOResult;
							list<int> dcAOFastResult;
							list<int> dcSOResult;

#ifdef __USING_STATIC_OFFSETS__
							if (dcSOResults.find(noDC)!=dcSOResults.end()) {
								dcSOResult = dcSOResults[noDC];
							} 
							else {
								cout << "Start to calculate the DC-" << dcFiles[noDC] << " speeds with offset" << endl;
								timer.start();
#ifndef __USING_FAST_DAVR__
								dcSOResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,aTasks,engine,period,UBSOSpeeds);
#else
								dcSOResult = OptimizationAlgorithm::computeUBPerfDCFast(dcList,WCETs,aTasks,engine,period,UBSOSpeeds);
#endif
								timer.end();
								tDAVRSO = timer.getTime();
								cout << "End. time = " << tDAVRSO << endl;

								DAVRSONum ++;
								totalDAVRSOTime += tDAVRSO;
								maxDAVRSOTime = max(maxDAVRSOTime,tDAVRSO);

								dcSOResults[noDC] = dcSOResult;

								// Record dcResults
								string indexResultDC = DCSOIndexResultsDir + Utility::linkNotation()+"Factor" + Utility::int_to_string(factor) 
									+ "Run" + Utility::int_to_string(run)
									+ dcFiles[noDC] + ".result";
								ofstream foutIndexResultDC(indexResultDC, ios::out | ios::trunc);
								if (!foutIndexResultDC.is_open()) {
									cerr << "Can't open "<<indexResultDC<<" file for output" <<endl;
									exit(EXIT_FAILURE);
								}

								list<double>::iterator iter0 = dcList.begin();
								list<int>::iterator iter1 = dcSOResult.begin();
								while (iter0 != dcList.end()) {
									foutIndexResultDC << *iter0 << " " << *iter1 << endl;
									iter0++;
									iter1++;
								}

								foutIndexResultDC.close();
							}

							double pDAVRSO = OptimizationAlgorithm::getPerformanceDCExp(dcList,dcSOResult,k1,k2);	
							//cout << "pDAVR=" << pDAVR << endl; 
							double rDAVRSO = pDAVRSO/pUBSO;

							if (dcSOResult.empty() || W_LEQ_TOL(pDAVRSO,0.0)) {
								DAVRSOFailed = true;
								DAVRSOFailedNum++;
							}
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
							if (dcAOResults.find(noDC)!=dcAOResults.end()) {
								dcAOResult = dcAOResults[noDC];
							} 
							else {
#ifndef __DISABLE_MOST_OUTPUT__
								cout << "Start to calculate the DC-" << dcFiles[noDC] << " speeds without offset" << endl;
#endif

								int optNum = 0; 
								double optTime = 0;
								timer.start();
								//dcAOResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,pTasks,engine,period,UBAOOPASpeeds,optNum,optTime,maxOPTTime,granularity);
								dcAOResult = OptimizationAlgorithm::computeUBFPPerfDC(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex, optNum,optTime,maxOPTTime,granularity);
								timer.end();
								tDAVRAO = tUBAOOPA + timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
								cout << "End. time = " << tDAVRAO << endl;
#endif

								totalOPTNum += optNum;
								totalOPTTime += optTime;

								DAVRAONum ++;
								totalDAVRAOTime += tDAVRAO;
								maxDAVRAOTime = max(maxDAVRAOTime,tDAVRAO);

								dcAOResults[noDC] = dcAOResult;

#ifndef __DISABLE_MOST_OUTPUT__
								// Record dcResults
								string indexResultDC = DCAOIndexResultsDir + Utility::linkNotation()+"Factor" + Utility::int_to_string(factor) 
									+ "Run" + Utility::int_to_string(run)
									+ dcFiles[noDC] + ".result";
								ofstream foutIndexResultDC(indexResultDC, ios::out | ios::trunc);
								if (!foutIndexResultDC.is_open()) {
									cerr << "Can't open "<<indexResultDC<<" file for output" <<endl;
									exit(EXIT_FAILURE);
								}

								list<double>::iterator iter0 = dcList.begin();
								list<int>::iterator iter1 = dcAOResult.begin();
								while (iter0 != dcList.end()) {
									foutIndexResultDC << *iter0 << " " << *iter1 << endl;
									iter0++;
									iter1++;
								}

								foutIndexResultDC.close();
#endif
							}


							// fine-d2s-exact
							if (dcAOFastResults.find(noDC)!=dcAOFastResults.end()) {
								dcAOFastResult = dcAOFastResults[noDC];
							} 
							else {
#ifndef __DISABLE_MOST_OUTPUT__
								cout << "Start to fast calculate the DC-" << dcFiles[noDC] << " speeds without offset" << endl;
#endif

								
								timer.start();
								dcAOFastResult = OptimizationAlgorithm::computeUBFPPerfDCFast(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex);
								timer.end();
								tDAVRAOFast = tUBAOFP + timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
								cout << "End. time = " << tDAVRAO << endl;
#endif

								DAVRAOFastNum ++;
								totalDAVRAOFastTime += tDAVRAOFast;
								maxDAVRAOFastTime = max(maxDAVRAOFastTime,tDAVRAOFast);

								dcAOFastResults[noDC] = dcAOFastResult;
							}

							double pDAVRAO = OptimizationAlgorithm::getPerformanceDCExp(dcList,dcAOResult,k1,k2);
							double pDAVRAOFast = OptimizationAlgorithm::getPerformanceDCExp(dcList,dcAOFastResult,k1,k2);

							double rDAVRAO = pDAVRAO/pUBAOOPA;
							double rDAVRAOFast = pDAVRAOFast/pUBAOOPA;
							double rUBAOFP = pUBAOFP/pUBAOOPA;

#ifdef __USING_STATIC_OFFSETS__
							rDAVRAO = pDAVRAO/pUBSO;
#endif

							if (dcAOResult.empty() || W_LEQ_TOL(pDAVRAO,0.0)) {
								DAVRAOFailed = true;
								DAVRAOFailedNum++;
							}

							if (dcAOFastResult.empty() || W_LEQ_TOL(pDAVRAOFast,0.0)) {
								DAVRAOFastFailed = true;
								DAVRAOFastFailedNum++;
							}
#endif

#ifndef __DISABLE_MOST_OUTPUT__
							/// output the result into the corresponding result file
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
							(*dcResultOfstreams[noDC]) << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK;
							(*expFunsResultOfstreams[noK]) << "Factor: " << factor << " Run: " << run << " DC: " << dcFiles[noDC];
							foutResultCollection << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
#ifdef __USING_ARBITRARY_OFFSETS__
							cout << "=>" << rSAVRAO << " " << rUBAOFP << " " << rDAVRAO << " " << rDAVRAOFast << " " 
								<< tUBAOOPA << " " << tSAVRAO << " " << tUBAOFP << " " << tDAVRAO << " " << tDAVRAOFast << endl;
							(*dcResultOfstreams[noDC]) << "=>" << rSAVRAO << " " << rDAVRAO << " " << tUBAOOPA << " " << tSAVRAO << " " << tDAVRAO << endl;
							(*expFunsResultOfstreams[noK]) << "=>" << rSAVRAO << " " << rDAVRAO << " " << tUBAOOPA << " " << tSAVRAO << " " << tDAVRAO << endl;
							foutResultCollection << "=>" << rSAVRAO << " " << rUBAOFP << " " << rDAVRAO << " " << rDAVRAOFast << " " 
								<< tUBAOOPA << " " << tSAVRAO << " " << tUBAOFP << " " << tDAVRAO << " " << tDAVRAOFast << endl;

#endif

#ifdef __USING_STATIC_OFFSETS__
							cout << "=>";
							(*dcResultOfstreams[noDC]) << "=>";
							(*expFunsResultOfstreams[noK]) << "=>";
							foutResultCollection << "=>";
#ifdef __USING_ARBITRARY_OFFSETS__
							cout << rUBAO << " ";
							(*dcResultOfstreams[noDC]) << rUBAO << " ";
							(*expFunsResultOfstreams[noK]) << rUBAO << " ";
							foutResultCollection << rUBAO << " ";
#endif
							cout << rSAVRSO << " " << rDAVRSO << " " << tUBSO << " " << tSAVRSO << " " << tDAVRSO << endl;
							(*dcResultOfstreams[noDC]) << rSAVRSO << " " << rDAVRSO << " " << tUBSO << " " << tSAVRSO << " " << tDAVRSO << endl;
							(*expFunsResultOfstreams[noK]) << rSAVRSO << " " << rDAVRSO << " " << tUBSO << " " << tSAVRSO << " " << tDAVRSO << endl;
							foutResultCollection << rSAVRSO << " " << rDAVRSO << " " << tUBSO << " " << tSAVRSO << " " << tDAVRSO << endl;
#endif

#endif

#ifdef __USING_ARBITRARY_OFFSETS__
							if (!UBAOFPFailed) totalUBAOFPPerf += rUBAOFP;
							if (!UBAOOPAFailed) totalSAVRAOPerf += rSAVRAO;
							//if (!SAVRAOFailed) totalSAVRAOPerf += rSAVRAO;
							if (!DAVRAOFailed) totalDAVRAOPerf += rDAVRAO;
							if (!DAVRAOFastFailed) totalDAVRAOFastPerf += rDAVRAOFast;
#endif

#ifdef __USING_STATIC_OFFSETS__
#ifdef __USING_ARBITRARY_OFFSETS__
						    if (!UBAOOPAFailed)	totalUBAOOPAPerf += rUBAO;
#endif
							if (!SAVRSOFailed) totalSAVRSOPerf += rSAVRSO;
							if (!DAVRSOFailed) totalDAVRSOPerf += rDAVRSO;
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
							if (!UBAOFPFailed) maxUBAOFPPerf = max(maxUBAOFPPerf,rUBAOFP);
							if (!SAVRAOFailed) maxSAVRAOPerf = max(maxSAVRAOPerf,rSAVRAO);
							if (!DAVRAOFailed) maxDAVRAOPerf = max(maxDAVRAOPerf,rDAVRAO);
							if (!DAVRAOFastFailed) maxDAVRAOFastPerf = max(maxDAVRAOFastPerf,rDAVRAOFast);
#endif

#ifdef __USING_STATIC_OFFSETS__
#ifdef __USING_ARBITRARY_OFFSETS__
							if (!UBAOOPAFailed) maxUBAOOPAPerf = max(maxUBAOOPAPerf,rUBAO);
#endif
							if (!SAVRSOFailed) maxSAVRSOPerf = max(maxSAVRSOPerf,rSAVRSO);
							if (!DAVRSOFailed) maxDAVRSOPerf = max(maxDAVRSOPerf,rDAVRSO);
#endif

#ifdef __USING_ARBITRARY_OFFSETS__
							if (!UBAOFPFailed) minUBAOFPPerf = min(minUBAOFPPerf,rUBAOFP);
							if (!SAVRAOFailed) minSAVRAOPerf = min(minSAVRAOPerf,rSAVRAO);
							if (!DAVRAOFailed) minDAVRAOPerf = min(minDAVRAOPerf,rDAVRAO);
							if (!DAVRAOFastFailed) minDAVRAOFastPerf = min(minDAVRAOFastPerf,rDAVRAOFast);
#endif

#ifdef __USING_STATIC_OFFSETS__
#ifdef __USING_ARBITRARY_OFFSETS__
							if (!UBAOOPAFailed) minUBAOOPAPerf = min(minUBAOOPAPerf,rUBAO);
#endif
							if (!SAVRSOFailed) minSAVRSOPerf = min(minSAVRSOPerf,rSAVRSO);
							if (!DAVRSOFailed) minDAVRSOPerf = min(minDAVRSOPerf,rDAVRSO);
#endif							
						}
					}
				}

			}
		}
#ifndef __DISABLE_MOST_OUTPUT__
		cout << totalDAVRSOPerf << ", " << DAVRSONum << ", " << numK << endl;
#endif
		normalizedUBAOOPAPerfs.push_back(totalUBAOOPAPerf/((UBAOOPANum-UBAOOPAFailedNum)*numK*dcListVec.size()));
		normalizedUBAOFPPerfs.push_back(totalUBAOFPPerf/(UBAOFPNum-UBAOFPFailedNum));
		//normalizedSAVRAOPerfs.push_back(totalSAVRAOPerf/((SAVRAONum-SAVRAOFailedNum)*dcListVec.size()));
		normalizedSAVRAOPerfs.push_back(totalSAVRAOPerf/((SAVRAONum-UBAOOPAFailedNum)*dcListVec.size()));
		normalizedSAVRSOPerfs.push_back(totalSAVRSOPerf/((SAVRSONum-SAVRSOFailedNum)*dcListVec.size()));
		normalizedDAVRAOPerfs.push_back(totalDAVRAOPerf/(DAVRAONum*numK-DAVRAOFailedNum));
		normalizedDAVRAOFastPerfs.push_back(totalDAVRAOFastPerf/(DAVRAOFastNum*numK-DAVRAOFastFailedNum));
		normalizedDAVRSOPerfs.push_back(totalDAVRSOPerf/(DAVRSONum*numK-DAVRSOFailedNum));

		normalizedMaxUBAOOPAPerfs.push_back(maxUBAOOPAPerf);
		normalizedMaxUBAOFPPerfs.push_back(maxUBAOFPPerf);
		normalizedMaxSAVRAOPerfs.push_back(maxSAVRAOPerf);
		normalizedMaxSAVRSOPerfs.push_back(maxSAVRSOPerf);
		normalizedMaxDAVRAOPerfs.push_back(maxDAVRAOPerf);
		normalizedMaxDAVRAOFastPerfs.push_back(maxDAVRAOFastPerf);
		normalizedMaxDAVRSOPerfs.push_back(maxDAVRSOPerf);

		normalizedMinUBAOOPAPerfs.push_back(minUBAOOPAPerf);
		normalizedMinUBAOFPPerfs.push_back(minUBAOFPPerf);
		normalizedMinSAVRAOPerfs.push_back(minSAVRAOPerf);
		normalizedMinSAVRSOPerfs.push_back(minSAVRSOPerf);
		normalizedMinDAVRAOPerfs.push_back(minDAVRAOPerf);
		normalizedMinDAVRAOFastPerfs.push_back(minDAVRAOFastPerf);
		normalizedMinDAVRSOPerfs.push_back(minDAVRSOPerf);

		averageUBAOOPATimes.push_back(totalUBAOOPATime/UBAOOPANum);
		averageUBAOFPTimes.push_back(totalUBAOFPTime/UBAOFPNum);
		averageUBSOTimes.push_back(totalUBSOTime/UBSONum);
		averageSAVRAOTimes.push_back(totalSAVRAOTime/SAVRAONum);
		averageSAVRSOTimes.push_back(totalSAVRSOTime/SAVRSONum);
		averageDAVRAOTimes.push_back(totalDAVRAOTime/DAVRAONum);
		averageDAVRAOFastTimes.push_back(totalDAVRAOFastTime/DAVRAONum);
		averageDAVRSOTimes.push_back(totalDAVRSOTime/DAVRSONum);
		averageOPTTimes.push_back(totalOPTTime/totalOPTNum);

		maxUBAOOPATimes.push_back(maxUBAOOPATime);
		maxUBAOFPTimes.push_back(maxUBAOFPTime);
		maxUBSOTimes.push_back(maxUBSOTime);
		maxSAVRAOTimes.push_back(maxSAVRAOTime);
		maxSAVRSOTimes.push_back(maxSAVRSOTime);
		maxDAVRAOTimes.push_back(maxDAVRAOTime);
		maxDAVRAOFastTimes.push_back(maxDAVRAOFastTime);
		maxDAVRSOTimes.push_back(maxDAVRSOTime);
		maxOPTTimes.push_back(maxOPTTime);

		UBSOFailedNums.push_back(UBSOFailedNum);
		UBAOOPAFailedNums.push_back(UBAOOPAFailedNum);
		UBAOFPFailedNums.push_back(UBAOFPFailedNum);
		SAVRAOFailedNums.push_back(SAVRAOFailedNum);
		SAVRSOFailedNums.push_back(SAVRSOFailedNum);
		DAVRAOFailedNums.push_back(DAVRAOFailedNum);
		DAVRAOFastFailedNums.push_back(DAVRAOFastFailedNum);
		DAVRSOFailedNums.push_back(DAVRSOFailedNum);

		UBSONums.push_back(UBSONum);
		UBAOOPANums.push_back(UBAOOPANum);
		UBAOFPNums.push_back(UBAOFPNum);
		SAVRAONums.push_back(SAVRAONum);
		SAVRSONums.push_back(SAVRSONum);
		DAVRAONums.push_back(DAVRAONum*numK);
		DAVRAOFastNums.push_back(DAVRAOFastNum*numK);
		DAVRSONums.push_back(DAVRSONum*numK);
		OPTNums.push_back(totalOPTNum);

		Utility::output_one_vector(cout, "normalizedUBAOOPATimes", averageUBAOOPATimes);
		Utility::output_one_vector(cout, "normalizedUBAOFPTimes", averageUBAOFPTimes);
		Utility::output_one_vector(cout, "normalizedUBSOTimes", averageUBSOTimes);

		Utility::output_one_vector(cout, "normalizedUBAOOPAPerfs", normalizedUBAOOPAPerfs);

		Utility::output_one_vector(cout, "normalizedUBAOFPPerfs", normalizedUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedUBAOFPTimes", averageUBAOFPTimes);
		
		Utility::output_one_vector(cout, "normalizedSAVRAOPerfs", normalizedSAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRAOTimes", averageSAVRAOTimes);

		Utility::output_one_vector(cout, "normalizedSAVRSOPerfs", normalizedSAVRSOPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRSOTimes", averageSAVRSOTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOPerfs", normalizedDAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOTimes", averageDAVRAOTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOFastPerfs", normalizedDAVRAOFastPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOFastTimes", averageDAVRAOFastTimes);

		Utility::output_one_vector(cout, "normalizedDAVRSOPerfs", normalizedDAVRSOPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRSOTimes", averageDAVRSOTimes);

		Utility::output_one_vector(cout, "normalizedMaxUBAOOPAPerfs", normalizedMaxUBAOOPAPerfs);
		Utility::output_one_vector(cout, "normalizedMaxUBAOFPPerfs", normalizedMaxUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedMaxSAVRAOPerfs", normalizedMaxSAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedMaxSAVRSOPerfs", normalizedMaxSAVRSOPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRAOPerfs", normalizedMaxDAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRAOFastPerfs", normalizedMaxDAVRAOFastPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRSOPerfs", normalizedMaxDAVRSOPerfs);

		Utility::output_one_vector(cout, "normalizedMinUBAOPerfs", normalizedMinUBAOOPAPerfs);
		Utility::output_one_vector(cout, "normalizedMinUBAOFPPerfs", normalizedMinUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedMinSAVRAOPerfs", normalizedMinSAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedMinSAVRSOPerfs", normalizedMinSAVRSOPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRAOPerfs", normalizedMinDAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRAOFastPerfs", normalizedMinDAVRAOFastPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRSOPerfs", normalizedMinDAVRSOPerfs);

		Utility::output_one_vector(cout, "UBSOFailedNums", UBSOFailedNums);
		Utility::output_one_vector(cout, "UBAOOPAFailedNums", UBAOOPAFailedNums);
		Utility::output_one_vector(cout, "UBAOFPFailedNums", UBAOFPFailedNums);
		Utility::output_one_vector(cout, "SAVRAOFailedNums", SAVRAOFailedNums);
		Utility::output_one_vector(cout, "SAVRSOFailedNums", SAVRSOFailedNums);
		Utility::output_one_vector(cout, "DAVRAOFailedNums", DAVRAOFailedNums);
		Utility::output_one_vector(cout, "DAVRAOFastFailedNums", DAVRAOFastFailedNums);
		Utility::output_one_vector(cout, "DAVRSOFailedNums", DAVRSOFailedNums);

		Utility::output_one_vector(cout, "UBSONums", UBSONums);
		Utility::output_one_vector(cout, "UBAOOPANums", UBAOOPANums);
		Utility::output_one_vector(cout, "UBAOFPNums", UBAOFPNums);
		Utility::output_one_vector(cout, "SAVRAONums", SAVRAONums);
		Utility::output_one_vector(cout, "SAVRSONums", SAVRSONums);
		Utility::output_one_vector(cout, "DAVRAONums", DAVRAONums);
		Utility::output_one_vector(cout, "DAVRAOFastNums", DAVRAOFastNums);
		Utility::output_one_vector(cout, "DAVRSONums", DAVRSONums);

		Utility::output_one_vector(cout, "maxUBSOTimes", maxUBSOTimes);
		Utility::output_one_vector(cout, "maxUBAOOPATimes", maxUBAOOPATimes);
		Utility::output_one_vector(cout, "maxUBAOFPTimes", maxUBAOFPTimes);
		Utility::output_one_vector(cout, "maxSAVRAOTimes", maxSAVRAOTimes);
		Utility::output_one_vector(cout, "maxSAVRSOTimes", maxSAVRSOTimes);
		Utility::output_one_vector(cout, "maxDAVRAOTimes", maxDAVRAOTimes);
		Utility::output_one_vector(cout, "maxDAVRAOFastTimes", maxDAVRAOFastTimes);
		Utility::output_one_vector(cout, "maxDAVRSOTimes", maxDAVRSOTimes);

		Utility::output_one_vector(cout, "OPTNums", OPTNums);
		Utility::output_one_vector(cout, "OPTTimes", averageOPTTimes);
		Utility::output_one_vector(cout, "maxOPTTimes", maxOPTTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOOPATimes", averageUBAOOPATimes);
		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPTimes", averageUBAOFPTimes);
		Utility::output_one_vector(foutResultCollection, "normalizedUBSOTimes", averageUBSOTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOOPAPerfs", normalizedUBAOOPAPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPPerfs", normalizedUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOPerfs", normalizedSAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOTimes", averageSAVRAOTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRSOPerfs", normalizedSAVRSOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRSOTimes", averageSAVRSOTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOPerfs", normalizedDAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOTimes", averageDAVRAOTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastPerfs", normalizedDAVRAOFastPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastTimes", averageDAVRAOFastTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRSOPerfs", normalizedDAVRSOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRSOTimes", averageDAVRSOTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedMaxUBAOOPAPerfs", normalizedMaxUBAOOPAPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxUBAOFPPerfs", normalizedMaxUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxSAVRAOPerfs", normalizedMaxSAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxSAVRSOPerfs", normalizedMaxSAVRSOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRAOPerfs", normalizedMaxDAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRAOFastPerfs", normalizedMaxDAVRAOFastPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRSOPerfs", normalizedMaxDAVRSOPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedMinUBAOPerfs", normalizedMinUBAOOPAPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinUBAOFPPerfs", normalizedMinUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinSAVRAOPerfs", normalizedMinSAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinSAVRSOPerfs", normalizedMinSAVRSOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRAOPerfs", normalizedMinDAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRAOFastPerfs", normalizedMinDAVRAOFastPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRSOPerfs", normalizedMinDAVRSOPerfs);

		Utility::output_one_vector(foutResultCollection, "UBSOFailedNums", UBSOFailedNums);
		Utility::output_one_vector(foutResultCollection, "UBAOOPAFailedNums", UBAOOPAFailedNums);
		Utility::output_one_vector(foutResultCollection, "UBAOFPFailedNums", UBAOFPFailedNums);
		Utility::output_one_vector(foutResultCollection, "SAVRAOFailedNums", SAVRAOFailedNums);
		Utility::output_one_vector(foutResultCollection, "SAVRSOFailedNums", SAVRSOFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFailedNums", DAVRAOFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFastFailedNums", DAVRAOFastFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRSOFailedNums", DAVRSOFailedNums);

		Utility::output_one_vector(foutResultCollection, "UBSONums", UBSONums);
		Utility::output_one_vector(foutResultCollection, "UBAOOPANums", UBAOOPANums);
		Utility::output_one_vector(foutResultCollection, "UBAOFPNums", UBAOFPNums);
		Utility::output_one_vector(foutResultCollection, "SAVRAONums", SAVRAONums);
		Utility::output_one_vector(foutResultCollection, "SAVRSONums", SAVRSONums);
		Utility::output_one_vector(foutResultCollection, "DAVRAONums", DAVRAONums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFastNums", DAVRAOFastNums);
		Utility::output_one_vector(foutResultCollection, "DAVRSONums", DAVRSONums);

		Utility::output_one_vector(foutResultCollection, "maxUBSOTimes", maxUBSOTimes);
		Utility::output_one_vector(foutResultCollection, "maxUBAOOPATimes", maxUBAOOPATimes);
		Utility::output_one_vector(foutResultCollection, "maxUBAOFPTimes", maxUBAOFPTimes);
		Utility::output_one_vector(foutResultCollection, "maxSAVRAOTimes", maxSAVRAOTimes);
		Utility::output_one_vector(foutResultCollection, "maxSAVRSOTimes", maxSAVRSOTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRAOTimes", maxDAVRAOTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRAOFastTimes", maxDAVRAOFastTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRSOTimes", maxDAVRSOTimes);

		Utility::output_one_vector(foutResultCollection, "OPTNums", OPTNums);
		Utility::output_one_vector(foutResultCollection, "OPTTimes", averageOPTTimes);
		Utility::output_one_vector(foutResultCollection, "maxOPTTimes", maxOPTTimes);
	}
#ifndef __DISABLE_MOST_OUTPUT__
	foutUBAOSpeeds.close();
	foutUBSOSpeeds.close();
#endif
	foutResultCollection.close();

#ifndef __DISABLE_MOST_OUTPUT__
	for (auto pointer : dcResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}
#endif

	for (auto pointer : expFunsResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}
#ifndef __DISABLE_MOST_OUTPUT__
	for (auto pointer : SAVRAOSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : SAVRSOSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}
#endif
}

void RunTest::RTADAVRExp2(bool checkCaseStudy, string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, string kRatio, int tUtil, int numSystem, int minFactor, int maxFactor, int granularity)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	factorName += "kRatio" + kRatio;

	string ExpFunsResultsDir = director + Utility::linkNotation()+"ExpFunsResults"+factorName;
	Utility::makeDir(ExpFunsResultsDir);

	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	// read the exponential performance functions
	expFunsFile = director + Utility::linkNotation() + expFunsFile + ".functions";;
	int numK = 0;
	vector<vector<double>> k1s;
	vector<vector<double>> k2s;

	FileReader::ReadExponentialFunctions(expFunsFile.c_str(),numK,k1s,k2s);

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedUBAOOPAPerfs;
	vector<double> normalizedUBAOFPPerfs;
	vector<double> normalizedSAVRAOAllZeroPerfs;
	vector<double> normalizedSAVRAOAllPerfs;
	vector<double> normalizedDAVRAOAllZeroPerfs;
	vector<double> normalizedDAVRAOAllPerfs;
	vector<double> normalizedDAVRAOFastAllZeroPerfs;
	vector<double> normalizedDAVRAOFastAllPerfs;

	vector<double> normalizedMaxUBAOOPAPerfs;
	vector<double> normalizedMaxUBAOFPPerfs;
	vector<double> normalizedMaxSAVRAOAllZeroPerfs;
	vector<double> normalizedMaxSAVRAOAllPerfs;
	vector<double> normalizedMaxDAVRAOAllZeroPerfs;
	vector<double> normalizedMaxDAVRAOAllPerfs;
	vector<double> normalizedMaxDAVRAOFastAllZeroPerfs;
	vector<double> normalizedMaxDAVRAOFastAllPerfs;

	vector<double> normalizedMinUBAOOPAPerfs;
	vector<double> normalizedMinUBAOFPPerfs;
	vector<double> normalizedMinSAVRAOAllZeroPerfs;
	vector<double> normalizedMinSAVRAOAllPerfs;
	vector<double> normalizedMinDAVRAOAllZeroPerfs;
	vector<double> normalizedMinDAVRAOAllPerfs;
	vector<double> normalizedMinDAVRAOFastAllZeroPerfs;
	vector<double> normalizedMinDAVRAOFastAllPerfs;

	vector<double> averageUBAOOPATimes;
	vector<double> averageUBAOFPTimes;
	vector<double> averageSAVRAOAllZeroTimes;
	vector<double> averageSAVRAOAllTimes;
	vector<double> averageDAVRAOZeroTimes;
	vector<double> averageDAVRAOAllTimes;
	vector<double> averageDAVRAOFastAllZeroTimes;
	vector<double> averageDAVRAOFastAllTimes;
	vector<double> averageOPTTimes; 

	vector<double> maxUBAOOPATimes;
	vector<double> maxUBAOFPTimes;
	vector<double> maxSAVRAOSuccessTimes;
	vector<double> maxSAVRAOAllTimes;
	vector<double> maxDAVRAOTimes;
	vector<double> maxDAVRAOAllTimes;
	vector<double> maxDAVRAOFastTimes;
	vector<double> maxDAVRAOFastAllTimes;
	vector<double> maxOPTTimes;

	vector<int> UBAOOPAFailedNums;
	vector<int> UBAOFPFailedNums;
	vector<int> SAVRAOFailedNums;
	vector<int> DAVRAOFailedNums;
	vector<int> DAVRAOFastFailedNums;

	vector<int> UBAOOPANums;
	vector<int> UBAOFPNums;
	vector<int> SAVRAONums;
	vector<int> DAVRAONums;
	vector<int> DAVRAOFastNums;
	vector<int> OPTNums;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalUBAOOPAPerf = 0.0;
		double totalUBAOFPPerf = 0.0;
		double totalSAVRAOAllZeroPerf = 0.0;
		double totalSAVRAOAllPerf = 0.0;
		double totalDAVRAOAllZeroPerf = 0.0;
		double totalDAVRAOAllPerf = 0.0;
		double totalDAVRAOFastAllZeroPerf = 0.0;
		double totalDAVRAOFastAllPerf = 0.0;

		int totalUBAOOPANum = 0;
		int totalUBAOFPNum = 0;
		int totalSAVRAOAllZeroNum = 0;
		int totalSAVRAOAllNum = 0;
		int totalDAVRAOAllZeroNum = 0;
		int totalDAVRAOAllNum = 0;
		int totalDAVRAOFastAllZeroNum = 0;
		int totalDAVRAOFastAllNum = 0;

		double maxUBAOOPAPerf = 0.0;
		double maxUBAOFPPerf = 0.0;
		double maxSAVRAOSuccessPerf = 0.0;
		double maxSAVRAOAllPerf = 0.0;
		double maxDAVRAOPerf = 0.0;
		double maxDAVRAOAllPerf = 0.0;
		double maxDAVRAOFastPerf = 0.0;
		double maxDAVRAOFastAllPerf = 0.0;

		double minUBAOOPAPerf = INT_MAX;
		double minUBAOFPPerf = INT_MAX;
		double minSAVRAOSuccessPerf = INT_MAX;
		double minSAVRAOAllPerf = INT_MAX;
		double minDAVRAOPerf = INT_MAX;
		double minDAVRAOFastPerf = INT_MAX;

		double totalUBAOFPTime = 0.0;
		double totalSAVRAOSuccessTime = 0.0;
		double totalSAVRAOAllTime = 0.0;
		double totalDAVRAOTime = 0.0;
		double totalDAVRAOAllTime = 0.0;
		double totalDAVRAOFastTime = 0.0;
		double totalDAVRAOFastAllTime = 0.0;
		double totalOPTTime = 0.0;


		double maxUBAOFPTime = 0.0;
		double maxSAVRAOSuccessTime = 0.0;
		double maxSAVRAOAllTime = 0.0;
		double maxDAVRAOTime = 0.0;
		double maxDAVRAOAllTime = 0.0;
		double maxDAVRAOFastTime = 0.0;
		double maxDAVRAOFastAllTime = 0.0;
		double maxOPTTime = 0.0;

		int UBAOFPNum = 0;
		int SAVRAONum = 0;
		int DAVRAONum = 0;
		int DAVRAOFastNum = 0;

		int totalOPTNum = 0;

		int UBAOFPFailedNum = 0;
		int SAVRAOFailedNum = 0;
		int DAVRAOFailedNum = 0;
		int DAVRAOFastFailedNum = 0;

		//int numSucc = 0;

		bool firstRecord = true;

		//while (numSucc < numSystem) {
		for (int run=0; run < numSystem; run ++) {
#ifndef __DISABLE_MOST_OUTPUT__
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
#endif
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks, WCETs);

			bool UBAOFPFailed = false;
			bool SAVRAOFailed = false;
			bool DAVRAOFailed = false;
			bool DAVRAOFastFailed = false;

			if (checkCaseStudy) {
				for (int i=0; i<WCETs.size(); i++) {
					if (i!=WCETs.size()-1)
						WCETs[i] *= 0.1*factor;
					else WCETs[i] = 148+416+100+330+10;
				}
			}
			else {
				for (int i=0; i<WCETs.size(); i++) {
					WCETs[i] *= factor;
				}
			}

			// Determining the priority of the AVR task by deadline monotonic policy
			int avrTaskIndex = 0;
			int avrMinDeadline = mSEC_to_muSEC(engine.getMinDeadline(period,engine.SPEED_MAX));
			for (int i=0; i<pTasks.size(); i++) {
				if (pTasks[i].deadline < avrMinDeadline)
					avrTaskIndex = i+1;
			}

			vector<double> UBAOFPSpeeds;
			double tUBAOFP = 0.0;

#ifndef __DISABLE_MOST_OUTPUT__
			cout << "Start to calculate the upper bound speeds without offset" << endl;
#endif
			timer.start();
			//UBAOSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
			UBAOFPSpeeds = OptimizationAlgorithm::computeUBFPSpeeds(NECESSARY_ONLY1,WCETs,pTasks,engine,period,avrTaskIndex);
			timer.end();
			tUBAOFP = timer.getTime();

			#ifndef __DISABLE_MOST_OUTPUT__
			//Utility::output_one_vector(cout,"UBFPSPeeds",UBAOFPSpeeds);
			cout << "End. time = " << tUBAOFP << endl;
#endif

			UBAOFPNum ++;
			totalUBAOFPTime += tUBAOFP;
			maxUBAOFPTime = max(maxUBAOFPTime,tUBAOFP);

			if (UBAOFPSpeeds.empty()) {
				UBAOFPFailed = true;
				UBAOFPFailedNum ++;
			}

			{
				map<int,list<int>> dcAOResults;
				map<int,list<int>> dcAOFastResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k1 = k1s[noK];
					vector<double> k2 = k2s[noK];

					vector<double> SAVRAOSpeeds;

#ifndef __DISABLE_MOST_OUTPUT__
					cout << "Start to calculate the BS speeds without offset" << endl;
#endif
					//cout << "++++++++++++++" << endl;
					//Utility::output_one_vector(cout,"UB",UBAOSpeeds);
					//Utility::output_one_vector(cout,"WCETs",WCETs);
					//exit(EXIT_FAILURE);
					/// Perform the optimization procedures
					timer.start();
#ifndef __USING_COARSE_D2S_EXACT__
					SAVRAOSpeeds = OptimizationAlgorithm::computeBiondiBSExpFP(NECESSARY_ONLY1,k1,k2,WCETs, pTasks, engine, period,avrTaskIndex);
#else
					SAVRAOSpeeds = UBAOFPSpeeds;
#endif
					timer.end();
					double tSAVRAO = timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
					//Utility::output_one_vector(cout,"SAVRAOSPeeds",SAVRAOSpeeds);
					cout << "End. time = " << tSAVRAO << endl;
#endif

					SAVRAONum ++;
					totalSAVRAOAllTime += tSAVRAO;

					if (SAVRAOSpeeds.empty()) {
						SAVRAOFailed = true;
						SAVRAOFailedNum++;
						//exit(EXIT_FAILURE);
					}
					else {
						totalSAVRAOSuccessTime += tSAVRAO;
						maxSAVRAOSuccessTime = max(maxSAVRAOSuccessTime,tSAVRAO);
					}

					{
						for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
							list<double> dcList = dcListVec[noDC];
							double pUBAOFP =  OptimizationAlgorithm::getPerformanceDCExp(dcList,UBAOFPSpeeds, k1,k2);
							double pSAVRAO = OptimizationAlgorithm::getPerformanceDCExp(dcList,SAVRAOSpeeds, k1,k2);

							double tDAVRAO = 0.0;
							double tDAVRAOFast = 0.0;

							if (W_LEQ_TOL(pUBAOFP,0.0) && !UBAOFPFailed) {
								UBAOFPFailed = true;
								UBAOFPFailedNum ++;
							}

							/*
							if (W_LEQ_TOL(pSAVRAO,0.0) && !SAVRAOFailed) {
								SAVRAOFailed = true;
								SAVRAOFailedNum ++;
							}
							*/

							//cout << pSAVRAO << " " << SAVRAOFailed << " " << pUBAOFP << " " << UBAOFPFailed << endl;
							//exit(EXIT_FAILURE);

#ifndef __DISABLE_MOST_OUTPUT__
							Utility::output_one_vector(cout,"UBAOSpeeds",UBAOFPSpeeds);
#endif
							if (SAVRAOFailed && !UBAOFPFailed) {
								//pSAVRAO = INT_MIN;
								for (int noSpeed=0; noSpeed < UBAOFPSpeeds.size(); noSpeed++) {
									vector<double> localSpeeds;
									vector<int> localWCETs;
									vector<double> localK1;
									vector<double> localK2;

									localSpeeds.push_back(UBAOFPSpeeds.back());
									localWCETs.push_back(WCETs[noSpeed]);

									localK1.push_back(k1[noSpeed]);
									localK2.push_back(k2[noSpeed]);

									AVRTask avrTask(engine,period,localSpeeds,localWCETs);
									bool schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(NECESSARY_ONLY1,pTasks, avrTask);
									if (!schedulable) continue;
									//double localPerf = OptimizationAlgorithm::getPerformanceDCExp(dcList,localSpeeds, localK1, localK2);
									//pSAVRAO = max(pSAVRAO,localPerf);

									pSAVRAO = OptimizationAlgorithm::getPerformanceDCExp(dcList,localSpeeds, localK1, localK2);
									break;
								}

								//cerr << pSAVRAO << endl;
								//Utility::output_one_vector(cerr,"localSpeeds=",localSpeeds);
								//Utility::output_one_vector(cerr,"localWCETs=",localWCETs);
								//Utility::output_one_vector(cerr,"localK=",localK);
								//exit(EXIT_FAILURE);
							}

							double rSAVRAO = pSAVRAO/pUBAOFP;

#ifndef __DISABLE_MOST_OUTPUT__
							cout << "Before DAVR->" << endl; 
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
							cout << "=>" << rSAVRAO << " "  << tUBAOFP << " " << tSAVRAO << endl;
#endif
							list<int> dcAOResult;
							list<int> dcAOFastResult;

							if (dcAOResults.find(noDC)!=dcAOResults.end()) {
								dcAOResult = dcAOResults[noDC];
							} 
							else {
#ifndef __DISABLE_MOST_OUTPUT__
								cout << "Start to calculate the DC-" << dcFiles[noDC] << " speeds without offset" << endl;
#endif

								int optNum = 0; 
								double optTime = 0;
								timer.start();
								//dcAOResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,pTasks,engine,period,UBAOOPASpeeds,optNum,optTime,maxOPTTime,granularity);
								dcAOResult = OptimizationAlgorithm::computeUBFPPerfDC(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex, optNum,optTime,maxOPTTime,granularity);
								timer.end();
								tDAVRAO = tUBAOFP + timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
								cout << "End. time = " << tDAVRAO << endl;
#endif

								totalOPTNum += optNum;
								totalOPTTime += optTime;

								DAVRAONum ++;
								totalDAVRAOTime += tDAVRAO;
								maxDAVRAOTime = max(maxDAVRAOTime,tDAVRAO);

								dcAOResults[noDC] = dcAOResult;
							}

							// fine-d2s-exact
							if (dcAOFastResults.find(noDC)!=dcAOFastResults.end()) {
								dcAOFastResult = dcAOFastResults[noDC];
							} 
							else {
#ifndef __DISABLE_MOST_OUTPUT__
								cout << "Start to fast calculate the DC-" << dcFiles[noDC] << " speeds without offset" << endl;
#endif

								timer.start();
#ifndef __USING_COARSE_D2S_EXACT__
								dcAOFastResult = OptimizationAlgorithm::computeUBFPPerfDCFast(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex);
#else
								dcAOFastResult = OptimizationAlgorithm::computeUBFPPerfDCFastCoarse(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex);
#endif
								timer.end();
								tDAVRAOFast = tUBAOFP + timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
								cout << "End. time = " << tDAVRAO << endl;
#endif

								DAVRAOFastNum ++;
								totalDAVRAOFastTime += tDAVRAOFast;
								maxDAVRAOFastTime = max(maxDAVRAOFastTime,tDAVRAOFast);

								dcAOFastResults[noDC] = dcAOFastResult;
							}

							double pDAVRAO = OptimizationAlgorithm::getPerformanceDCExp(dcList,dcAOResult,k1,k2);
							double pDAVRAOFast = OptimizationAlgorithm::getPerformanceDCExp(dcList,dcAOFastResult,k1,k2);

							double rDAVRAO = pDAVRAO/pUBAOFP;
							double rDAVRAOFast = pDAVRAOFast/pUBAOFP;
							double rUBAOFP = pUBAOFP/pUBAOFP;

							if (dcAOResult.empty() || W_LEQ_TOL(pDAVRAO,0.0)) {
								DAVRAOFailed = true;
								DAVRAOFailedNum++;
							}

							if (dcAOFastResult.empty() || W_LEQ_TOL(pDAVRAOFast,0.0)) {
								DAVRAOFastFailed = true;
								DAVRAOFastFailedNum++;
							}

#ifndef __DISABLE_MOST_OUTPUT__
							/// output the result into the corresponding result file
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
							foutResultCollection << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 

							cout << "=>" << rSAVRAO << " " << rUBAOFP << " " << rDAVRAO << " " << rDAVRAOFast << " " 
								<< tUBAOFP << " " << tSAVRAO << " " << tUBAOFP << " " << tDAVRAO << " " << tDAVRAOFast << endl;
							foutResultCollection << "=>" << rSAVRAO << " " << rUBAOFP << " " << rDAVRAO << " " << rDAVRAOFast << " " 
								<< tUBAOFP << " " << tSAVRAO << " " << tUBAOFP << " " << tDAVRAO << " " << tDAVRAOFast << endl;
#endif

							if (!UBAOFPFailed) { totalUBAOFPPerf += rUBAOFP; totalUBAOFPNum++; }
							if (!SAVRAOFailed) { totalSAVRAOAllZeroPerf += rSAVRAO; totalSAVRAOAllZeroNum++; }
							if (!UBAOFPFailed) { totalSAVRAOAllPerf += rSAVRAO; totalSAVRAOAllNum++; }
							if (!DAVRAOFailed && !SAVRAOFailed) { totalDAVRAOAllZeroPerf += rDAVRAO; totalDAVRAOAllZeroNum++; }
							if (!DAVRAOFailed) { totalDAVRAOAllPerf += rDAVRAO; totalDAVRAOAllNum++; }
							if (!DAVRAOFastFailed && !SAVRAOFailed) { totalDAVRAOFastAllZeroPerf += rDAVRAOFast; totalDAVRAOFastAllZeroNum++; }
							if (!DAVRAOFastFailed) { totalDAVRAOFastAllPerf += rDAVRAOFast; totalDAVRAOFastAllNum++; }

							if (!UBAOFPFailed) maxUBAOFPPerf = max(maxUBAOFPPerf,rUBAOFP);
							if (!SAVRAOFailed) maxSAVRAOSuccessPerf = max(maxSAVRAOSuccessPerf,rSAVRAO);
							if (!UBAOFPFailed) maxSAVRAOAllPerf = max(maxSAVRAOAllPerf,rSAVRAO);
							if (!DAVRAOFailed && !SAVRAOFailed) maxDAVRAOPerf = max(maxDAVRAOPerf,rDAVRAO);
							if (!DAVRAOFailed) maxDAVRAOAllPerf = max(maxDAVRAOAllPerf,rDAVRAO);
							if (!DAVRAOFastFailed && !SAVRAOFailed) maxDAVRAOFastPerf = max(maxDAVRAOFastPerf,rDAVRAOFast);
							if (!DAVRAOFastFailed) maxDAVRAOFastAllPerf = max(maxDAVRAOFastAllPerf,rDAVRAOFast);

							if (!UBAOFPFailed) minUBAOFPPerf = min(minUBAOFPPerf,rUBAOFP);
							if (!SAVRAOFailed) minSAVRAOSuccessPerf = min(minSAVRAOSuccessPerf,rSAVRAO);
							if (!DAVRAOFailed) minDAVRAOPerf = min(minDAVRAOPerf,rDAVRAO);
							if (!DAVRAOFastFailed) minDAVRAOFastPerf = min(minDAVRAOFastPerf,rDAVRAOFast);					
						}
					}
				}

			}
		}

		normalizedUBAOFPPerfs.push_back(totalUBAOFPPerf/totalUBAOFPNum);
		normalizedSAVRAOAllZeroPerfs.push_back(totalSAVRAOAllZeroPerf/totalSAVRAOAllNum);
		normalizedSAVRAOAllPerfs.push_back(totalSAVRAOAllPerf/totalSAVRAOAllNum);
		normalizedDAVRAOAllZeroPerfs.push_back(totalDAVRAOAllZeroPerf/totalDAVRAOAllZeroNum);
		normalizedDAVRAOAllPerfs.push_back(totalDAVRAOAllPerf/totalDAVRAOAllNum);
		normalizedDAVRAOFastAllZeroPerfs.push_back(totalDAVRAOFastAllZeroPerf/totalDAVRAOFastAllZeroNum);
		normalizedDAVRAOFastAllPerfs.push_back(totalDAVRAOFastAllPerf/totalDAVRAOFastAllNum);

		normalizedMaxUBAOFPPerfs.push_back(maxUBAOFPPerf);
		normalizedMaxSAVRAOAllZeroPerfs.push_back(maxSAVRAOSuccessPerf);
		normalizedMaxSAVRAOAllPerfs.push_back(maxSAVRAOAllPerf);
		normalizedMaxDAVRAOAllZeroPerfs.push_back(maxDAVRAOPerf);
		normalizedMaxDAVRAOAllPerfs.push_back(maxDAVRAOAllPerf);
		normalizedMaxDAVRAOFastAllZeroPerfs.push_back(maxDAVRAOFastPerf);
		normalizedMaxDAVRAOFastAllPerfs.push_back(maxDAVRAOFastAllPerf);

		normalizedMinUBAOFPPerfs.push_back(minUBAOFPPerf);
		normalizedMinSAVRAOAllZeroPerfs.push_back(minSAVRAOSuccessPerf);
		normalizedMinSAVRAOAllPerfs.push_back(minSAVRAOAllPerf);
		normalizedMinDAVRAOAllZeroPerfs.push_back(minDAVRAOPerf);
		normalizedMinDAVRAOFastAllZeroPerfs.push_back(minDAVRAOFastPerf);

		averageUBAOFPTimes.push_back(totalUBAOFPTime/UBAOFPNum);
		averageSAVRAOAllZeroTimes.push_back(totalSAVRAOSuccessTime/SAVRAONum);
		averageSAVRAOAllTimes.push_back(totalSAVRAOAllTime/SAVRAONum);
		averageDAVRAOZeroTimes.push_back(totalDAVRAOTime/DAVRAONum);
		averageDAVRAOFastAllZeroTimes.push_back(totalDAVRAOFastTime/DAVRAONum);
		averageOPTTimes.push_back(totalOPTTime/totalOPTNum);

		maxUBAOFPTimes.push_back(maxUBAOFPTime);
		maxSAVRAOSuccessTimes.push_back(maxSAVRAOSuccessTime);
		maxSAVRAOAllTimes.push_back(maxSAVRAOAllTime);
		maxDAVRAOTimes.push_back(maxDAVRAOTime);
		maxDAVRAOFastTimes.push_back(maxDAVRAOFastTime);
		maxOPTTimes.push_back(maxOPTTime);

		UBAOOPAFailedNums.push_back(UBAOFPFailedNum);
		UBAOFPFailedNums.push_back(UBAOFPFailedNum);
		SAVRAOFailedNums.push_back(SAVRAOFailedNum);
		DAVRAOFailedNums.push_back(DAVRAOFailedNum);
		DAVRAOFastFailedNums.push_back(DAVRAOFastFailedNum);

		UBAOOPANums.push_back(UBAOFPNum);
		UBAOFPNums.push_back(UBAOFPNum);
		SAVRAONums.push_back(SAVRAONum);
		DAVRAONums.push_back(DAVRAONum*numK);
		DAVRAOFastNums.push_back(DAVRAOFastNum*numK);
		OPTNums.push_back(totalOPTNum);

		Utility::output_one_vector(cout, "normalizedUBAOOPATimes", averageUBAOOPATimes);
		Utility::output_one_vector(cout, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(cout, "normalizedUBAOOPAPerfs", normalizedUBAOOPAPerfs);

		Utility::output_one_vector(cout, "normalizedUBAOFPPerfs", normalizedUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(cout, "normalizedSAVRAOAllZeroPerfs", normalizedSAVRAOAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRAOAllZeroTimes", averageSAVRAOAllZeroTimes);

		Utility::output_one_vector(cout, "normalizedSAVRAOAllPerfs", normalizedSAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRAOAllTimes", averageSAVRAOAllTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOAllZeroPerfs", normalizedDAVRAOAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOAllZeroTimes", averageDAVRAOZeroTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOAllPerfs", normalizedDAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOAllTimes", averageDAVRAOAllTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOFastAllZeroPerfs", normalizedDAVRAOFastAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOFastAllZeroTimes", averageDAVRAOFastAllZeroTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOFastAllPerfs", normalizedDAVRAOFastAllPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOFastAllTimes", averageDAVRAOFastAllTimes);


		Utility::output_one_vector(cout, "normalizedMaxUBAOOPAPerfs", normalizedMaxUBAOOPAPerfs);
		Utility::output_one_vector(cout, "normalizedMaxUBAOFPPerfs", normalizedMaxUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedMaxSAVRAOAllZeroPerfs", normalizedMaxSAVRAOAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedMaxSAVRAOAllPerfs", normalizedMaxSAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRAOPerfs", normalizedMaxDAVRAOAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRAOFastPerfs", normalizedMaxDAVRAOFastAllZeroPerfs);

		Utility::output_one_vector(cout, "normalizedMinUBAOPerfs", normalizedMinUBAOOPAPerfs);
		Utility::output_one_vector(cout, "normalizedMinUBAOFPPerfs", normalizedMinUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedMinSAVRAOAllZeroPerfs", normalizedMinSAVRAOAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedMinSAVRAOAllPerfs", normalizedMinSAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRAOPerfs", normalizedMinDAVRAOAllZeroPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRAOFastPerfs", normalizedMinDAVRAOFastAllZeroPerfs);

		Utility::output_one_vector(cout, "UBAOOPAFailedNums", UBAOOPAFailedNums);
		Utility::output_one_vector(cout, "UBAOFPFailedNums", UBAOFPFailedNums);
		Utility::output_one_vector(cout, "SAVRAOFailedNums", SAVRAOFailedNums);
		Utility::output_one_vector(cout, "DAVRAOFailedNums", DAVRAOFailedNums);
		Utility::output_one_vector(cout, "DAVRAOFastFailedNums", DAVRAOFastFailedNums);

		Utility::output_one_vector(cout, "UBAOOPANums", UBAOOPANums);
		Utility::output_one_vector(cout, "UBAOFPNums", UBAOFPNums);
		Utility::output_one_vector(cout, "SAVRAONums", SAVRAONums);
		Utility::output_one_vector(cout, "DAVRAONums", DAVRAONums);
		Utility::output_one_vector(cout, "DAVRAOFastNums", DAVRAOFastNums);

		Utility::output_one_vector(cout, "maxUBAOOPATimes", maxUBAOOPATimes);
		Utility::output_one_vector(cout, "maxUBAOFPTimes", maxUBAOFPTimes);
		Utility::output_one_vector(cout, "maxSAVRAOTimes", maxSAVRAOSuccessTimes);
		Utility::output_one_vector(cout, "maxDAVRAOTimes", maxDAVRAOTimes);
		Utility::output_one_vector(cout, "maxDAVRAOFastTimes", maxDAVRAOFastTimes);

		Utility::output_one_vector(cout, "OPTNums", OPTNums);
		Utility::output_one_vector(cout, "OPTTimes", averageOPTTimes);
		Utility::output_one_vector(cout, "maxOPTTimes", maxOPTTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOOPATimes", averageUBAOOPATimes);
		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOOPAPerfs", normalizedUBAOOPAPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPPerfs", normalizedUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOAllZeroPerfs", normalizedSAVRAOAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOAllZeroTimes", averageSAVRAOAllZeroTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOAllPerfs", normalizedSAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOAllTimes", averageSAVRAOAllTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOAllZeroPerfs", normalizedDAVRAOAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOTimes", averageDAVRAOZeroTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOAllPerfs", normalizedDAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOAllTimes", averageDAVRAOAllTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastAllZeroPerfs", normalizedDAVRAOFastAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastTimes", averageDAVRAOFastAllZeroTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastAllPerfs", normalizedDAVRAOFastAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastAllTimes", averageDAVRAOFastAllTimes);


		Utility::output_one_vector(foutResultCollection, "normalizedMaxUBAOOPAPerfs", normalizedMaxUBAOOPAPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxUBAOFPPerfs", normalizedMaxUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxSAVRAOAllZeroPerfs", normalizedMaxSAVRAOAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxSAVRAOAllPerfs", normalizedMaxSAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRAOPerfs", normalizedMaxDAVRAOAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRAOFastPerfs", normalizedMaxDAVRAOFastAllZeroPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedMinUBAOPerfs", normalizedMinUBAOOPAPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinUBAOFPPerfs", normalizedMinUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinSAVRAOAllZeroPerfs", normalizedMinSAVRAOAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinSAVRAOAllPerfs", normalizedMinSAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRAOPerfs", normalizedMinDAVRAOAllZeroPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRAOFastPerfs", normalizedMinDAVRAOFastAllZeroPerfs);

		Utility::output_one_vector(foutResultCollection, "UBAOOPAFailedNums", UBAOOPAFailedNums);
		Utility::output_one_vector(foutResultCollection, "UBAOFPFailedNums", UBAOFPFailedNums);
		Utility::output_one_vector(foutResultCollection, "SAVRAOFailedNums", SAVRAOFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFailedNums", DAVRAOFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFastFailedNums", DAVRAOFastFailedNums);

		Utility::output_one_vector(foutResultCollection, "UBAOOPANums", UBAOOPANums);
		Utility::output_one_vector(foutResultCollection, "UBAOFPNums", UBAOFPNums);
		Utility::output_one_vector(foutResultCollection, "SAVRAONums", SAVRAONums);
		Utility::output_one_vector(foutResultCollection, "DAVRAONums", DAVRAONums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFastNums", DAVRAOFastNums);

		Utility::output_one_vector(foutResultCollection, "maxUBAOOPATimes", maxUBAOOPATimes);
		Utility::output_one_vector(foutResultCollection, "maxUBAOFPTimes", maxUBAOFPTimes);
		Utility::output_one_vector(foutResultCollection, "maxSAVRAOTimes", maxSAVRAOSuccessTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRAOTimes", maxDAVRAOTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRAOFastTimes", maxDAVRAOFastTimes);

		Utility::output_one_vector(foutResultCollection, "OPTNums", OPTNums);
		Utility::output_one_vector(foutResultCollection, "OPTTimes", averageOPTTimes);
		Utility::output_one_vector(foutResultCollection, "maxOPTTimes", maxOPTTimes);
	}

	foutResultCollection.close();
}

void RunTest::RTADAVRConst(string director, string systemDir, string constFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	// mkdir
	string SAVRSpeedsDir = director + Utility::linkNotation()+"SAVRSpeeds"+factorName;
	Utility::makeDir(SAVRSpeedsDir);

	string DCResultsDir = director + Utility::linkNotation()+"DCResults"+factorName;
	Utility::makeDir(DCResultsDir);

	string DCIndexResultsDir = director + Utility::linkNotation()+"DCIndexResults"+factorName;
	Utility::makeDir(DCIndexResultsDir);

	string ConstFunsResultsDir = director + Utility::linkNotation()+"ConstFunsResults"+factorName;
	Utility::makeDir(ConstFunsResultsDir);

	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

	string resultPUBSpeeds = director + Utility::linkNotation()+"PUBSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultAUBSpeeds = director + Utility::linkNotation()+"AUBSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";

	ofstream foutPUBSpeeds(resultPUBSpeeds, ios::out | ios::trunc);
	if (!foutPUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultPUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutAUBSpeeds(resultAUBSpeeds, ios::out | ios::trunc);
	if (!foutAUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultAUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<ofstream*> dcResultOfstreams;
	for (auto dcFile:dcFiles) {
		dcFile = DCResultsDir + Utility::linkNotation() + dcFile + ".result";
		ofstream* fout = new ofstream(dcFile,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<dcFile<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		dcResultOfstreams.push_back(fout);
	}

	// read the exponential performance functions
	constFunsFile = director + Utility::linkNotation() + constFunsFile + ".functions";
	int numK = 0;
	vector<vector<double>> ks;
	FileReader::ReadConstantFunctions(constFunsFile.c_str(),numK,ks);

	vector<ofstream*> constFunsResultOfstreams;
	for (int i=0; i<numK; i++) {
		string constFuncResult = ConstFunsResultsDir + Utility::linkNotation()+"ConstFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(constFuncResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<constFuncResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		constFunsResultOfstreams.push_back(fout);
	}

	vector<ofstream*> SAVRSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string SAVRSpeedsResult = SAVRSpeedsDir + Utility::linkNotation()+"ConstFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(SAVRSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<SAVRSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		SAVRSpeedsOfstreams.push_back(fout);
	}

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedPUBPerfs;
	vector<double> normalizedSAVRPerfs;
	vector<double> normalizedDAVRPerfs;

	vector<double> normalizedPUBTimes;
	vector<double> normalizedAUBTimes;
	vector<double> normalizedSAVRTimes;
	vector<double> normalizedDAVRTimes;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalPUBPerf = 0.0;
		double totalSAVRPerf = 0.0;
		double totalDAVRPerf = 0.0;

		double totalPUBTime = 0.0;
		double totalAUBTime = 0.0;
		double totalSAVRTime = 0.0;
		double totalDAVRTime = 0.0;

		int numAll = 0;
		int numSAVR = 0;
		int numDAVR = 0;

		int numUnFeasible = 0;

		int run = 0;
		int numSucc = 0;

		bool firstRecord = true;

		while (numSucc < numSystem) {
			//for (run=94; run <=94; run ++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks,WCETs);

			run++;
			/*
			for (int i=0; i<WCETs.size(); i++) {
				if (i!=WCETs.size()-1)
					WCETs[i] *= 0.1*factor;
				else WCETs[i] = 148+416+100+330+10;
			}
			*/

			for (int i=0; i<WCETs.size(); i++) {
				WCETs[i] *= factor;
			}

			vector<double> AUBSpeeds; 
			vector<double> PUBSpeeds;
			double tAUB = 0.0, tPUB = 0.0;
			bool found = false;

			cout << "Start to calculate the upper bound speeds without offset" << endl;
			timer.start();
			PUBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
			timer.end();
			tPUB = timer.getTime();
			cout << "End. time = " << tPUB << endl;

			if (PUBSpeeds.empty()) {
				found = false;
				goto CHECK_SUCCESS;
			}

			cout << "Start to calculate the upper bound speeds with offset" << endl;
			timer.start();
			//AUBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
			AUBSpeeds = PUBSpeeds;
			timer.end();
			tAUB = timer.getTime();
			cout << "End. time = " << tAUB << endl;

			if (AUBSpeeds.empty()) {
				found = false;
				goto CHECK_SUCCESS;
			}

			{
				map<int,list<int>> dcResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k = ks[noK];

					if (firstRecord) {
						// record factor, k1 and k2
						for (auto pointer : constFunsResultOfstreams) {
							(*pointer) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
							Utility::output_one_vector((*pointer), "k",k);
						}

						firstRecord = false;
					}

					cout << "Start to calculate the BS speeds without offset" << endl;
					/// Perform the optimization procedures
					timer.start();
					//vector<double> SAVRSpeeds = PUBSpeeds;
					vector<double> SAVRSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, pTasks, engine, period);
					timer.end();
					double tSAVR = timer.getTime();
					cout << "End. time = " << tSAVR << endl;

					if (SAVRSpeeds.empty()) {
						found = false;
						goto CHECK_SUCCESS;
					}

					{
						totalSAVRTime += tSAVR;
						numSAVR ++;

						Utility::output_one_vector(*SAVRSpeedsOfstreams[noK],"Result Speeds",SAVRSpeeds);

						for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
							list<double> dcList = dcListVec[noDC];
							
#if 0
							double pAUB =  OptimizationAlgorithm::getPerformanceDC(dcList,AUBSpeeds, k);
							double pPUB =  OptimizationAlgorithm::getPerformanceDC(dcList,PUBSpeeds, k);
							double pSAVR = OptimizationAlgorithm::getPerformanceDC(dcList,SAVRSpeeds, k);
#else
							double pAUB =  OptimizationAlgorithm::getPerformance(AUBSpeeds, k,engine);
							double pPUB =  OptimizationAlgorithm::getPerformance(PUBSpeeds, k,engine);
							double pSAVR = OptimizationAlgorithm::getPerformance(SAVRSpeeds,k, engine);
#endif

							double tDAVR = 0.0;

							double ro = pPUB/pAUB;
							double rs = pSAVR/pAUB;

							cout << "Before DAVR->" << endl; 
							cout << "Factor: " << factor << " Run: " << run << " ConstNo: " << noK << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << tAUB << " " << tPUB << " " << tSAVR << endl;

							list<int> dcResult;

#if 0
							if (dcResults.find(noDC)!=dcResults.end()) {
								dcResult = dcResults[noDC];
							} 
							else {
								cout << "Start to calculate the DC-" << dcFiles[noDC] << " speeds with offset" << endl;
								timer.start();
								dcResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,aTasks,engine,period,AUBSpeeds);
								timer.end();
								tDAVR = timer.getTime();
								cout << "End. time = " << tDAVR << endl;
								totalDAVRTime += tDAVR;
								numDAVR++;

								dcResults[noDC] = dcResult;

								// Record dcResults
								string indexResultDC = DCIndexResultsDir + Utility::linkNotation()+"Factor" + Utility::int_to_string(factor) 
																		 + "Run" + Utility::int_to_string(run)
																		 + dcFiles[noDC] + ".result";
								ofstream foutIndexResultDC(indexResultDC, ios::out | ios::trunc);
								if (!foutIndexResultDC.is_open()) {
									cerr << "Can't open "<<indexResultDC<<" file for output" <<endl;
									exit(EXIT_FAILURE);
								}

								list<double>::iterator iter0 = dcList.begin();
								list<int>::iterator iter1 = dcResult.begin();
								while (iter0 != dcList.end()) {
									foutIndexResultDC << *iter0 << " " << *iter1 << endl;
									iter0++;
									iter1++;
								}

								foutIndexResultDC.close();

							}

#endif
							double pDAVR = OptimizationAlgorithm::getPerformanceDC(dcResult,k);	
							double rd = pDAVR/pAUB;

							/// output the result into the corresponding result file
							(*dcResultOfstreams[noDC]) << "Factor: " << factor << " Run: " << run << " ConstNo: " << noK 
								<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							(*constFunsResultOfstreams[noK]) << "Factor: " << factor << " Run: " << run << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							cout << "Factor: " << factor << " Run: " << run << " ConstNo: " << noK << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							foutResultCollection << "Factor: " << factor << " Run: " << run << " ConstNo: " << noK << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							numAll++;
							totalPUBPerf += ro;
							totalSAVRPerf += rs;
							totalDAVRPerf += rd;
						}
					}
				}

				found = true;
			}

CHECK_SUCCESS:
			if (found) numSucc++;
			else {
				numUnFeasible++;
				continue;
			}

			Utility::output_one_vector(foutPUBSpeeds,"Result Speeds",PUBSpeeds);
			Utility::output_one_vector(foutAUBSpeeds,"Result Speeds",AUBSpeeds);
			totalPUBTime += tPUB;
			totalAUBTime += tAUB;
		}

		normalizedPUBPerfs.push_back(totalPUBPerf/numAll);
		normalizedSAVRPerfs.push_back(totalSAVRPerf/numAll);
		normalizedDAVRPerfs.push_back(totalDAVRPerf/numAll);

		normalizedPUBTimes.push_back(totalPUBTime/numSystem);
		normalizedAUBTimes.push_back(totalAUBTime/numSystem);
		normalizedSAVRTimes.push_back(totalSAVRTime/numSAVR);
		normalizedDAVRTimes.push_back(totalDAVRTime/numDAVR);

		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPUBTimes", normalizedPUBTimes);
		Utility::output_one_vector(cout, "normalizedAUBTimes", normalizedAUBTimes);

		Utility::output_one_vector(cout, "normalizedPUBPerfs", normalizedPUBPerfs);

		Utility::output_one_vector(cout, "normalizedSAVRPerfs", normalizedSAVRPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRTimes", normalizedSAVRTimes);

		Utility::output_one_vector(cout, "normalizedDAVRPerfs", normalizedDAVRPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRTimes", normalizedDAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPUBTimes", normalizedPUBTimes);
		Utility::output_one_vector(foutResultCollection, "normalizedAUBTimes", normalizedAUBTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPUBPerfs", normalizedPUBPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRPerfs", normalizedSAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRTimes", normalizedSAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRPerfs", normalizedDAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRTimes", normalizedDAVRTimes);
	}

	foutPUBSpeeds.close();
	foutAUBSpeeds.close();
	foutResultCollection.close();

	for (auto pointer : dcResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : constFunsResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : SAVRSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}
}

void RunTest::RTADAVRConst2(bool checkCaseStudy, string director, string systemDir, string constFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor, int granularity)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";
	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	// read the exponential performance functions
	constFunsFile = director + Utility::linkNotation() + constFunsFile + ".functions";;
	int numK = 0;
	vector<vector<double>> ks;
	FileReader::ReadConstantFunctions(constFunsFile.c_str(),numK,ks);

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedUBAOOPAPerfs;
	vector<double> normalizedUBAOFPPerfs;
	vector<double> normalizedSAVRAOSuccessPerfs;
	vector<double> normalizedSAVRAOAllPerfs;
	vector<double> normalizedDAVRAOSuccessPerfs;
	vector<double> normalizedDAVRAOAllPerfs;
	vector<double> normalizedDAVRAOFastSuccessPerfs;
	vector<double> normalizedDAVRAOFastAllPerfs;

	vector<double> normalizedMaxUBAOOPAPerfs;
	vector<double> normalizedMaxUBAOFPPerfs;
	vector<double> normalizedMaxSAVRAOSuccessPerfs;
	vector<double> normalizedMaxSAVRAOAllPerfs;
	vector<double> normalizedMaxDAVRAOPerfs;
	vector<double> normalizedMaxDAVRAOAllPerfs;
	vector<double> normalizedMaxDAVRAOFastPerfs;
	vector<double> normalizedMaxDAVRAOFastAllPerfs;

	vector<double> normalizedMinUBAOOPAPerfs;
	vector<double> normalizedMinUBAOFPPerfs;
	vector<double> normalizedMinSAVRAOSuccessPerfs;
	vector<double> normalizedMinSAVRAOAllPerfs;
	vector<double> normalizedMinDAVRAOPerfs;
	vector<double> normalizedMinDAVRAOAllPerfs;
	vector<double> normalizedMinDAVRAOFastPerfs;
	vector<double> normalizedMinDAVRAOFastAllPerfs;

	vector<double> averageUBAOOPATimes;
	vector<double> averageUBAOFPTimes;
	vector<double> averageSAVRAOSuccessTimes;
	vector<double> averageSAVRAOAllTimes;
	vector<double> averageDAVRAOTimes;
	vector<double> averageDAVRAOAllTimes;
	vector<double> averageDAVRAOFastTimes;
	vector<double> averageDAVRAOFastAllTimes;
	vector<double> averageOPTTimes; 

	vector<double> maxUBAOOPATimes;
	vector<double> maxUBAOFPTimes;
	vector<double> maxSAVRAOSuccessTimes;
	vector<double> maxSAVRAOAllTimes;
	vector<double> maxDAVRAOTimes;
	vector<double> maxDAVRAOAllTimes;
	vector<double> maxDAVRAOFastTimes;
	vector<double> maxDAVRAOFastAllTimes;
	vector<double> maxOPTTimes;

	vector<int> UBAOOPAFailedNums;
	vector<int> UBAOFPFailedNums;
	vector<int> SAVRAOFailedNums;
	vector<int> DAVRAOFailedNums;
	vector<int> DAVRAOFastFailedNums;

	vector<int> UBAOOPANums;
	vector<int> UBAOFPNums;
	vector<int> SAVRAONums;
	vector<int> DAVRAONums;
	vector<int> DAVRAOFastNums;
	vector<int> OPTNums;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalUBAOOPAPerf = 0.0;
		double totalUBAOFPPerf = 0.0;
		double totalSAVRAOSuccessPerf = 0.0;
		double totalSAVRAOAllPerf = 0.0;
		double totalDAVRAOSuccessPerf = 0.0;
		double totalDAVRAOAllPerf = 0.0;
		double totalDAVRAOFastSuccessPerf = 0.0;
		double totalDAVRAOFastAllPerf = 0.0;

		int totalUBAOOPANum = 0;
		int totalUBAOFPNum = 0;
		int totalSAVRAOSuccessNum = 0;
		int totalSAVRAOAllNum = 0;
		int totalDAVRAOSuccessNum = 0;
		int totalDAVRAOAllNum = 0;
		int totalDAVRAOFastSuccessNum = 0;
		int totalDAVRAOFastAllNum = 0;

		double maxUBAOOPAPerf = 0.0;
		double maxUBAOFPPerf = 0.0;
		double maxSAVRAOSuccessPerf = 0.0;
		double maxSAVRAOAllPerf = 0.0;
		double maxDAVRAOPerf = 0.0;
		double maxDAVRAOAllPerf = 0.0;
		double maxDAVRAOFastPerf = 0.0;
		double maxDAVRAOFastAllPerf = 0.0;

		double minUBAOOPAPerf = INT_MAX;
		double minUBAOFPPerf = INT_MAX;
		double minSAVRAOSuccessPerf = INT_MAX;
		double minSAVRAOAllPerf = INT_MAX;
		double minDAVRAOPerf = INT_MAX;
		double minDAVRAOFastPerf = INT_MAX;

		double totalUBAOFPTime = 0.0;
		double totalSAVRAOSuccessTime = 0.0;
		double totalSAVRAOAllTime = 0.0;
		double totalDAVRAOTime = 0.0;
		double totalDAVRAOAllTime = 0.0;
		double totalDAVRAOFastTime = 0.0;
		double totalDAVRAOFastAllTime = 0.0;
		double totalOPTTime = 0.0;


		double maxUBAOFPTime = 0.0;
		double maxSAVRAOSuccessTime = 0.0;
		double maxSAVRAOAllTime = 0.0;
		double maxDAVRAOTime = 0.0;
		double maxDAVRAOAllTime = 0.0;
		double maxDAVRAOFastTime = 0.0;
		double maxDAVRAOFastAllTime = 0.0;
		double maxOPTTime = 0.0;

		int UBAOFPNum = 0;
		int SAVRAONum = 0;
		int DAVRAONum = 0;
		int DAVRAOFastNum = 0;

		int totalOPTNum = 0;

		int UBAOFPFailedNum = 0;
		int SAVRAOFailedNum = 0;
		int DAVRAOFailedNum = 0;
		int DAVRAOFastFailedNum = 0;

		//int numSucc = 0;

		bool firstRecord = true;

		//while (numSucc < numSystem) {
		for (int run=0; run < numSystem; run ++) {
		//for (int run=400; run < 450; run ++) {
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks, WCETs);

			bool UBAOFPFailed = false;
			bool SAVRAOFailed = false;
			bool DAVRAOFailed = false;
			bool DAVRAOFastFailed = false;

			if (checkCaseStudy) {
				for (int i=0; i<WCETs.size(); i++) {
					if (i!=WCETs.size()-1)
						WCETs[i] *= 0.1*factor;
					else WCETs[i] = 148+416+100+330+10;
				}
			}
			else {
				for (int i=0; i<WCETs.size(); i++) {
					WCETs[i] *= factor;
				}
			}

			// Determining the priority of the AVR task by deadline monotonic policy
			int avrTaskIndex = 0;
			int avrMinDeadline = mSEC_to_muSEC(engine.getMinDeadline(period,engine.SPEED_MAX));
			for (int i=0; i<pTasks.size(); i++) {
				if (pTasks[i].deadline < avrMinDeadline)
					avrTaskIndex = i+1;
			}

			//cout << avrTaskIndex << " " << avrMinDeadline << endl;
			//exit(EXIT_FAILURE);

			vector<double> UBAOFPSpeeds;
			double tUBAOFP = 0.0;

#ifndef __DISABLE_MOST_OUTPUT__
			cout << "Start to calculate the upper bound speeds without offset" << endl;
#endif
			timer.start();
			UBAOFPSpeeds = OptimizationAlgorithm::computeUBFPSpeeds(NECESSARY_ONLY1,WCETs,pTasks,engine,period,avrTaskIndex);
			timer.end();
			tUBAOFP = timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
			//Utility::output_one_vector(cout,"UBFPSPeeds",UBAOFPSpeeds);
			cout << "End. time = " << tUBAOFP << endl;
#endif

			UBAOFPNum ++;
			totalUBAOFPTime += tUBAOFP;
			maxUBAOFPTime = max(maxUBAOFPTime,tUBAOFP);

			if (UBAOFPSpeeds.empty()) {
				UBAOFPFailed = true;
				UBAOFPFailedNum ++;
			}

			{
				map<int,list<int>> dcAOResults;
				map<int,list<int>> dcAOFastResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k = ks[noK];

					vector<double> SAVRAOSpeeds;

#ifndef __DISABLE_MOST_OUTPUT__
					cout << "Start to calculate the BS speeds without offset" << endl;
#endif
					//cout << "++++++++++++++" << endl;
					//Utility::output_one_vector(cout,"UB",UBAOSpeeds);
					//Utility::output_one_vector(cout,"WCETs",WCETs);
					//exit(EXIT_FAILURE);
					/// Perform the optimization procedures
					timer.start();
#ifndef __USING_COARSE_D2S_EXACT__
					SAVRAOSpeeds = OptimizationAlgorithm::computeBiondiBSFP(NECESSARY_ONLY1,k,WCETs, pTasks, engine, period,avrTaskIndex);
#else
					SAVRAOSpeeds = UBAOFPSpeeds;
#endif
					timer.end();
					double tSAVRAO = timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
					//Utility::output_one_vector(cout,"SAVRAOSPeeds",SAVRAOSpeeds);
					cout << "End. time = " << tSAVRAO << endl;
#endif

					SAVRAONum ++;
					totalSAVRAOAllTime += tSAVRAO;

					if (SAVRAOSpeeds.empty()) {
						SAVRAOFailed = true;
						SAVRAOFailedNum++;
						//exit(EXIT_FAILURE);
					}
					else {
						totalSAVRAOSuccessTime += tSAVRAO;
						maxSAVRAOSuccessTime = max(maxSAVRAOSuccessTime,tSAVRAO);
					}

					{
						for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
							list<double> dcList = dcListVec[noDC];
							double pUBAOFP =  OptimizationAlgorithm::getPerformanceDC(dcList,UBAOFPSpeeds, k);
							double pSAVRAO = OptimizationAlgorithm::getPerformanceDC(dcList,SAVRAOSpeeds, k);

							double tDAVRAO = 0.0;
							double tDAVRAOFast = 0.0;

							if (W_LEQ_TOL(pUBAOFP,0.0) && !UBAOFPFailed) {
								UBAOFPFailed = true;
								UBAOFPFailedNum ++;
							}

							/*
							if (W_LEQ_TOL(pSAVRAO,0.0) && !SAVRAOFailed) {
								SAVRAOFailed = true;
								SAVRAOFailedNum ++;
							}
							*/

							//cout << pSAVRAO << " " << SAVRAOFailed << " " << pUBAOFP << " " << UBAOFPFailed << endl;
							//exit(EXIT_FAILURE);

#ifndef __DISABLE_MOST_OUTPUT__
							Utility::output_one_vector(cout,"UBAOSpeeds",UBAOFPSpeeds);
#endif
							if (SAVRAOFailed && !UBAOFPFailed) {
								//pSAVRAO = INT_MIN;
								for (int noSpeed=0; noSpeed < UBAOFPSpeeds.size(); noSpeed++) {
									vector<double> localSpeeds;
									vector<int> localWCETs;
									vector<double> localK;

									localSpeeds.push_back(UBAOFPSpeeds.back());
									localWCETs.push_back(WCETs[noSpeed]);

									localK.push_back(k[noSpeed]);

									AVRTask avrTask(engine,period,localSpeeds,localWCETs);
									bool schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(NECESSARY_ONLY1,pTasks, avrTask);
									if (!schedulable) continue;
									//double localPerf = OptimizationAlgorithm::getPerformanceDCExp(dcList,localSpeeds, localK1, localK2);
									//pSAVRAO = max(pSAVRAO,localPerf);

									pSAVRAO = OptimizationAlgorithm::getPerformanceDC(dcList,localSpeeds, localK);
									break;
								}

								//cerr << pSAVRAO << endl;
								//Utility::output_one_vector(cerr,"localSpeeds=",localSpeeds);
								//Utility::output_one_vector(cerr,"localWCETs=",localWCETs);
								//Utility::output_one_vector(cerr,"localK=",localK);
								//exit(EXIT_FAILURE);
							}

							double rSAVRAO = pSAVRAO/pUBAOFP;

#ifndef __DISABLE_MOST_OUTPUT__
							cout << "Before DAVR->" << endl; 
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
							cout << "=>" << rSAVRAO << " "  << tUBAOFP << " " << tSAVRAO << endl;
#endif
							list<int> dcAOResult;
							list<int> dcAOFastResult;

							if (dcAOResults.find(noDC)!=dcAOResults.end()) {
								dcAOResult = dcAOResults[noDC];
							} 
							else {
#ifndef __DISABLE_MOST_OUTPUT__
								cout << "Start to calculate the DC-" << dcFiles[noDC] << " speeds without offset" << endl;
#endif

								int optNum = 0; 
								double optTime = 0;
								timer.start();
								//dcAOResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,pTasks,engine,period,UBAOOPASpeeds,optNum,optTime,maxOPTTime,granularity);
								dcAOResult = OptimizationAlgorithm::computeUBFPPerfDC(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex, optNum,optTime,maxOPTTime,granularity);
								timer.end();
								tDAVRAO = tUBAOFP + timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
								cout << "End. time = " << tDAVRAO << endl;
#endif

								totalOPTNum += optNum;
								totalOPTTime += optTime;

								DAVRAONum ++;
								totalDAVRAOTime += tDAVRAO;
								maxDAVRAOTime = max(maxDAVRAOTime,tDAVRAO);

								dcAOResults[noDC] = dcAOResult;
							}

							// fine-d2s-exact
							if (dcAOFastResults.find(noDC)!=dcAOFastResults.end()) {
								dcAOFastResult = dcAOFastResults[noDC];
							} 
							else {
#ifndef __DISABLE_MOST_OUTPUT__
								cout << "Start to fast calculate the DC-" << dcFiles[noDC] << " speeds without offset" << endl;
#endif

								timer.start();
#ifndef __USING_COARSE_D2S_EXACT__
								dcAOFastResult = OptimizationAlgorithm::computeUBFPPerfDCFast(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex);
#else
								dcAOFastResult = OptimizationAlgorithm::computeUBFPPerfDCFastCoarse(dcList,WCETs,pTasks,engine,period,UBAOFPSpeeds,avrTaskIndex);
#endif
								timer.end();
								tDAVRAOFast = tUBAOFP + timer.getTime();

#ifndef __DISABLE_MOST_OUTPUT__
								cout << "End. time = " << tDAVRAO << endl;
#endif

								DAVRAOFastNum ++;
								totalDAVRAOFastTime += tDAVRAOFast;
								maxDAVRAOFastTime = max(maxDAVRAOFastTime,tDAVRAOFast);

								dcAOFastResults[noDC] = dcAOFastResult;
							}

							double pDAVRAO = OptimizationAlgorithm::getPerformanceDC(dcList,dcAOResult,k);
							double pDAVRAOFast = OptimizationAlgorithm::getPerformanceDC(dcList,dcAOFastResult,k);

							double rDAVRAO = pDAVRAO/pUBAOFP;
							double rDAVRAOFast = pDAVRAOFast/pUBAOFP;
							double rUBAOFP = pUBAOFP/pUBAOFP;

							if (dcAOResult.empty() || W_LEQ_TOL(pDAVRAO,0.0)) {
								DAVRAOFailed = true;
								DAVRAOFailedNum++;
							}

							if (dcAOFastResult.empty() || W_LEQ_TOL(pDAVRAOFast,0.0)) {
								DAVRAOFastFailed = true;
								DAVRAOFastFailedNum++;
							}

#ifndef __DISABLE_MOST_OUTPUT__
							/// output the result into the corresponding result file
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 
							foutResultCollection << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC]; 

							cout << "=>" << rSAVRAO << " " << rUBAOFP << " " << rDAVRAO << " " << rDAVRAOFast << " " 
								<< tUBAOFP << " " << tSAVRAO << " " << tUBAOFP << " " << tDAVRAO << " " << tDAVRAOFast << endl;
							foutResultCollection << "=>" << rSAVRAO << " " << rUBAOFP << " " << rDAVRAO << " " << rDAVRAOFast << " " 
								<< tUBAOFP << " " << tSAVRAO << " " << tUBAOFP << " " << tDAVRAO << " " << tDAVRAOFast << endl;
#endif

							if (!UBAOFPFailed) { totalUBAOFPPerf += rUBAOFP; totalUBAOFPNum++; }
							if (!SAVRAOFailed) { totalSAVRAOSuccessPerf += rSAVRAO; totalSAVRAOSuccessNum++; }
							if (!UBAOFPFailed) { totalSAVRAOAllPerf += rSAVRAO; totalSAVRAOAllNum++; }
							if (!DAVRAOFailed && !SAVRAOFailed) { totalDAVRAOSuccessPerf += rDAVRAO; totalDAVRAOSuccessNum++; }
							if (!DAVRAOFailed) { totalDAVRAOAllPerf += rDAVRAO; totalDAVRAOAllNum++; }
							if (!DAVRAOFastFailed && !SAVRAOFailed) { totalDAVRAOFastSuccessPerf += rDAVRAOFast; totalDAVRAOFastSuccessNum++; }
							if (!DAVRAOFastFailed) { totalDAVRAOFastAllPerf += rDAVRAOFast; totalDAVRAOFastAllNum++; }

							if (!UBAOFPFailed) maxUBAOFPPerf = max(maxUBAOFPPerf,rUBAOFP);
							if (!SAVRAOFailed) maxSAVRAOSuccessPerf = max(maxSAVRAOSuccessPerf,rSAVRAO);
							if (!UBAOFPFailed) maxSAVRAOAllPerf = max(maxSAVRAOAllPerf,rSAVRAO);
							if (!DAVRAOFailed && !SAVRAOFailed) maxDAVRAOPerf = max(maxDAVRAOPerf,rDAVRAO);
							if (!DAVRAOFailed) maxDAVRAOAllPerf = max(maxDAVRAOAllPerf,rDAVRAO);
							if (!DAVRAOFastFailed && !SAVRAOFailed) maxDAVRAOFastPerf = max(maxDAVRAOFastPerf,rDAVRAOFast);
							if (!DAVRAOFastFailed) maxDAVRAOFastAllPerf = max(maxDAVRAOFastAllPerf,rDAVRAOFast);

							if (!UBAOFPFailed) minUBAOFPPerf = min(minUBAOFPPerf,rUBAOFP);
							if (!SAVRAOFailed) minSAVRAOSuccessPerf = min(minSAVRAOSuccessPerf,rSAVRAO);
							if (!DAVRAOFailed) minDAVRAOPerf = min(minDAVRAOPerf,rDAVRAO);
							if (!DAVRAOFastFailed) minDAVRAOFastPerf = min(minDAVRAOFastPerf,rDAVRAOFast);					
						}
					}
				}

			}
		}

		normalizedUBAOFPPerfs.push_back(totalUBAOFPPerf/totalUBAOFPNum);
		normalizedSAVRAOSuccessPerfs.push_back(totalSAVRAOSuccessPerf/totalSAVRAOAllNum);
		normalizedSAVRAOAllPerfs.push_back(totalSAVRAOAllPerf/totalSAVRAOAllNum);
		normalizedDAVRAOSuccessPerfs.push_back(totalDAVRAOSuccessPerf/totalDAVRAOSuccessNum);
		normalizedDAVRAOAllPerfs.push_back(totalDAVRAOAllPerf/totalDAVRAOAllNum);
		normalizedDAVRAOFastSuccessPerfs.push_back(totalDAVRAOFastSuccessPerf/totalDAVRAOFastSuccessNum);
		normalizedDAVRAOFastAllPerfs.push_back(totalDAVRAOFastAllPerf/totalDAVRAOFastAllNum);

		normalizedMaxUBAOFPPerfs.push_back(maxUBAOFPPerf);
		normalizedMaxSAVRAOSuccessPerfs.push_back(maxSAVRAOSuccessPerf);
		normalizedMaxSAVRAOAllPerfs.push_back(maxSAVRAOAllPerf);
		normalizedMaxDAVRAOPerfs.push_back(maxDAVRAOPerf);
		normalizedMaxDAVRAOAllPerfs.push_back(maxDAVRAOAllPerf);
		normalizedMaxDAVRAOFastPerfs.push_back(maxDAVRAOFastPerf);
		normalizedMaxDAVRAOFastAllPerfs.push_back(maxDAVRAOFastAllPerf);

		normalizedMinUBAOFPPerfs.push_back(minUBAOFPPerf);
		normalizedMinSAVRAOSuccessPerfs.push_back(minSAVRAOSuccessPerf);
		normalizedMinSAVRAOAllPerfs.push_back(minSAVRAOAllPerf);
		normalizedMinDAVRAOPerfs.push_back(minDAVRAOPerf);
		normalizedMinDAVRAOFastPerfs.push_back(minDAVRAOFastPerf);

		averageUBAOFPTimes.push_back(totalUBAOFPTime/UBAOFPNum);
		averageSAVRAOSuccessTimes.push_back(totalSAVRAOSuccessTime/SAVRAONum);
		averageSAVRAOAllTimes.push_back(totalSAVRAOAllTime/SAVRAONum);
		averageDAVRAOTimes.push_back(totalDAVRAOTime/DAVRAONum);
		averageDAVRAOFastTimes.push_back(totalDAVRAOFastTime/DAVRAONum);
		averageOPTTimes.push_back(totalOPTTime/totalOPTNum);

		maxUBAOFPTimes.push_back(maxUBAOFPTime);
		maxSAVRAOSuccessTimes.push_back(maxSAVRAOSuccessTime);
		maxSAVRAOAllTimes.push_back(maxSAVRAOAllTime);
		maxDAVRAOTimes.push_back(maxDAVRAOTime);
		maxDAVRAOFastTimes.push_back(maxDAVRAOFastTime);
		maxOPTTimes.push_back(maxOPTTime);

		UBAOOPAFailedNums.push_back(UBAOFPFailedNum);
		UBAOFPFailedNums.push_back(UBAOFPFailedNum);
		SAVRAOFailedNums.push_back(SAVRAOFailedNum);
		DAVRAOFailedNums.push_back(DAVRAOFailedNum);
		DAVRAOFastFailedNums.push_back(DAVRAOFastFailedNum);

		UBAOOPANums.push_back(UBAOFPNum);
		UBAOFPNums.push_back(UBAOFPNum);
		SAVRAONums.push_back(SAVRAONum);
		DAVRAONums.push_back(DAVRAONum*numK);
		DAVRAOFastNums.push_back(DAVRAOFastNum*numK);
		OPTNums.push_back(totalOPTNum);

		Utility::output_one_vector(cout, "normalizedUBAOOPATimes", averageUBAOOPATimes);
		Utility::output_one_vector(cout, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(cout, "normalizedUBAOOPAPerfs", normalizedUBAOOPAPerfs);

		Utility::output_one_vector(cout, "normalizedUBAOFPPerfs", normalizedUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(cout, "normalizedSAVRAOSuccessPerfs", normalizedSAVRAOSuccessPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRAOSuccessTimes", averageSAVRAOSuccessTimes);

		Utility::output_one_vector(cout, "normalizedSAVRAOAllPerfs", normalizedSAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRAOAllTimes", averageSAVRAOAllTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOSuccessPerfs", normalizedDAVRAOSuccessPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOSuccessTimes", averageDAVRAOTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOAllPerfs", normalizedDAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOAllTimes", averageDAVRAOAllTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOFastSuccessPerfs", normalizedDAVRAOFastSuccessPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOFastSuccessTimes", averageDAVRAOFastTimes);

		Utility::output_one_vector(cout, "normalizedDAVRAOFastAllPerfs", normalizedDAVRAOFastAllPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRAOFastAllTimes", averageDAVRAOFastAllTimes);


		Utility::output_one_vector(cout, "normalizedMaxUBAOOPAPerfs", normalizedMaxUBAOOPAPerfs);
		Utility::output_one_vector(cout, "normalizedMaxUBAOFPPerfs", normalizedMaxUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedMaxSAVRAOSuccessPerfs", normalizedMaxSAVRAOSuccessPerfs);
		Utility::output_one_vector(cout, "normalizedMaxSAVRAOAllPerfs", normalizedMaxSAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRAOPerfs", normalizedMaxDAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedMaxDAVRAOFastPerfs", normalizedMaxDAVRAOFastPerfs);

		Utility::output_one_vector(cout, "normalizedMinUBAOPerfs", normalizedMinUBAOOPAPerfs);
		Utility::output_one_vector(cout, "normalizedMinUBAOFPPerfs", normalizedMinUBAOFPPerfs);
		Utility::output_one_vector(cout, "normalizedMinSAVRAOSuccessPerfs", normalizedMinSAVRAOSuccessPerfs);
		Utility::output_one_vector(cout, "normalizedMinSAVRAOAllPerfs", normalizedMinSAVRAOAllPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRAOPerfs", normalizedMinDAVRAOPerfs);
		Utility::output_one_vector(cout, "normalizedMinDAVRAOFastPerfs", normalizedMinDAVRAOFastPerfs);

		Utility::output_one_vector(cout, "UBAOOPAFailedNums", UBAOOPAFailedNums);
		Utility::output_one_vector(cout, "UBAOFPFailedNums", UBAOFPFailedNums);
		Utility::output_one_vector(cout, "SAVRAOFailedNums", SAVRAOFailedNums);
		Utility::output_one_vector(cout, "DAVRAOFailedNums", DAVRAOFailedNums);
		Utility::output_one_vector(cout, "DAVRAOFastFailedNums", DAVRAOFastFailedNums);

		Utility::output_one_vector(cout, "UBAOOPANums", UBAOOPANums);
		Utility::output_one_vector(cout, "UBAOFPNums", UBAOFPNums);
		Utility::output_one_vector(cout, "SAVRAONums", SAVRAONums);
		Utility::output_one_vector(cout, "DAVRAONums", DAVRAONums);
		Utility::output_one_vector(cout, "DAVRAOFastNums", DAVRAOFastNums);

		Utility::output_one_vector(cout, "maxUBAOOPATimes", maxUBAOOPATimes);
		Utility::output_one_vector(cout, "maxUBAOFPTimes", maxUBAOFPTimes);
		Utility::output_one_vector(cout, "maxSAVRAOTimes", maxSAVRAOSuccessTimes);
		Utility::output_one_vector(cout, "maxDAVRAOTimes", maxDAVRAOTimes);
		Utility::output_one_vector(cout, "maxDAVRAOFastTimes", maxDAVRAOFastTimes);

		Utility::output_one_vector(cout, "OPTNums", OPTNums);
		Utility::output_one_vector(cout, "OPTTimes", averageOPTTimes);
		Utility::output_one_vector(cout, "maxOPTTimes", maxOPTTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOOPATimes", averageUBAOOPATimes);
		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOOPAPerfs", normalizedUBAOOPAPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPPerfs", normalizedUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedUBAOFPTimes", averageUBAOFPTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOSuccessPerfs", normalizedSAVRAOSuccessPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOSuccessTimes", averageSAVRAOSuccessTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOAllPerfs", normalizedSAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRAOAllTimes", averageSAVRAOAllTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOSuccessPerfs", normalizedDAVRAOSuccessPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOTimes", averageDAVRAOTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOAllPerfs", normalizedDAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOAllTimes", averageDAVRAOAllTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastSuccessPerfs", normalizedDAVRAOFastSuccessPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastTimes", averageDAVRAOFastTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastAllPerfs", normalizedDAVRAOFastAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRAOFastAllTimes", averageDAVRAOFastAllTimes);


		Utility::output_one_vector(foutResultCollection, "normalizedMaxUBAOOPAPerfs", normalizedMaxUBAOOPAPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxUBAOFPPerfs", normalizedMaxUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxSAVRAOSuccessPerfs", normalizedMaxSAVRAOSuccessPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxSAVRAOAllPerfs", normalizedMaxSAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRAOPerfs", normalizedMaxDAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMaxDAVRAOFastPerfs", normalizedMaxDAVRAOFastPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedMinUBAOPerfs", normalizedMinUBAOOPAPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinUBAOFPPerfs", normalizedMinUBAOFPPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinSAVRAOSuccessPerfs", normalizedMinSAVRAOSuccessPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinSAVRAOAllPerfs", normalizedMinSAVRAOAllPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRAOPerfs", normalizedMinDAVRAOPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedMinDAVRAOFastPerfs", normalizedMinDAVRAOFastPerfs);

		Utility::output_one_vector(foutResultCollection, "UBAOOPAFailedNums", UBAOOPAFailedNums);
		Utility::output_one_vector(foutResultCollection, "UBAOFPFailedNums", UBAOFPFailedNums);
		Utility::output_one_vector(foutResultCollection, "SAVRAOFailedNums", SAVRAOFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFailedNums", DAVRAOFailedNums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFastFailedNums", DAVRAOFastFailedNums);

		Utility::output_one_vector(foutResultCollection, "UBAOOPANums", UBAOOPANums);
		Utility::output_one_vector(foutResultCollection, "UBAOFPNums", UBAOFPNums);
		Utility::output_one_vector(foutResultCollection, "SAVRAONums", SAVRAONums);
		Utility::output_one_vector(foutResultCollection, "DAVRAONums", DAVRAONums);
		Utility::output_one_vector(foutResultCollection, "DAVRAOFastNums", DAVRAOFastNums);

		Utility::output_one_vector(foutResultCollection, "maxUBAOOPATimes", maxUBAOOPATimes);
		Utility::output_one_vector(foutResultCollection, "maxUBAOFPTimes", maxUBAOFPTimes);
		Utility::output_one_vector(foutResultCollection, "maxSAVRAOTimes", maxSAVRAOSuccessTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRAOTimes", maxDAVRAOTimes);
		Utility::output_one_vector(foutResultCollection, "maxDAVRAOFastTimes", maxDAVRAOFastTimes);

		Utility::output_one_vector(foutResultCollection, "OPTNums", OPTNums);
		Utility::output_one_vector(foutResultCollection, "OPTTimes", averageOPTTimes);
		Utility::output_one_vector(foutResultCollection, "maxOPTTimes", maxOPTTimes);
	}

	foutResultCollection.close();
}

void RunTest::RTADAVRExpMin(string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	// mkdir
	string SAVRSpeedsDir = director + Utility::linkNotation()+"SAVRSpeeds"+factorName;
	Utility::makeDir(SAVRSpeedsDir);

	string DCResultsDir = director + Utility::linkNotation()+"DCResults"+factorName;
	Utility::makeDir(DCResultsDir);

	string DCIndexResultsDir = director + Utility::linkNotation()+"DCIndexResults"+factorName;
	Utility::makeDir(DCIndexResultsDir);

	string ExpFunsResultsDir = director + Utility::linkNotation()+"ExpFunsResults"+factorName;
	Utility::makeDir(ExpFunsResultsDir);

	vector<list<double>> dcListVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}

	string resultPUBSpeeds = director + Utility::linkNotation()+"PUBSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultAUBSpeeds = director + Utility::linkNotation()+"AUBSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";

	ofstream foutPUBSpeeds(resultPUBSpeeds, ios::out | ios::trunc);
	if (!foutPUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultPUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutAUBSpeeds(resultAUBSpeeds, ios::out | ios::trunc);
	if (!foutAUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultAUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<ofstream*> dcResultOfstreams;
	for (auto dcFile:dcFiles) {
		dcFile = DCResultsDir + Utility::linkNotation() + dcFile + ".result";
		ofstream* fout = new ofstream(dcFile,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<dcFile<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		dcResultOfstreams.push_back(fout);
	}

	// read the exponential performance functions
	expFunsFile = director + Utility::linkNotation() + expFunsFile + ".functions";;
	int numK = 0;
	vector<vector<double>> k1s;
	vector<vector<double>> k2s;

	FileReader::ReadExponentialFunctions(expFunsFile.c_str(),numK,k1s,k2s);

	vector<ofstream*> expFunsResultOfstreams;
	for (int i=0; i<numK; i++) {
		string expFuncResult = ExpFunsResultsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(expFuncResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<expFuncResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		expFunsResultOfstreams.push_back(fout);
	}

	vector<ofstream*> SAVRSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string SAVRSpeedsResult = SAVRSpeedsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(SAVRSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<SAVRSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		SAVRSpeedsOfstreams.push_back(fout);
	}

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedPUBPerfs;
	vector<double> normalizedSAVRPerfs;
	vector<double> normalizedDAVRPerfs;

	vector<double> normalizedPUBTimes;
	vector<double> normalizedAUBTimes;
	vector<double> normalizedSAVRTimes;
	vector<double> normalizedDAVRTimes;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalPUBPerf = 0.0;
		double totalSAVRPerf = 0.0;
		double totalDAVRPerf = 0.0;

		double totalPUBTime = 0.0;
		double totalAUBTime = 0.0;
		double totalSAVRTime = 0.0;
		double totalDAVRTime = 0.0;

		int numAll = 0;
		int numSAVR = 0;
		int numDAVR = 0;

		int numUnFeasible = 0;

		int run = 0;
		int numSucc = 0;

		bool firstRecord = true;

		while (numSucc < numSystem) {
			//for (run=94; run <=94; run ++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks, WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++) {
				if (i!=WCETs.size()-1)
					WCETs[i] *= factor;
				else WCETs[i] = 148+416+100+330+10;
			}

			vector<double> AUBSpeeds; 
			vector<double> PUBSpeeds;
			double tAUB = 0.0, tPUB = 0.0;
			bool found = false;

			cout << "Start to calculate the upper bound speeds without offset" << endl;
			timer.start();
			PUBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
			timer.end();
			tPUB = timer.getTime();
			cout << "End. time = " << tPUB << endl;

			if (PUBSpeeds.empty()) {
				found = false;
				goto CHECK_SUCCESS;
			}

			cout << "Start to calculate the upper bound speeds with offset" << endl;
			timer.start();
			AUBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
			//AUBSpeeds = PUBSpeeds;
			timer.end();
			tAUB = timer.getTime();
			cout << "End. time = " << tAUB << endl;

			if (AUBSpeeds.empty()) {
				found = false;
				goto CHECK_SUCCESS;
			}

			{
				map<int,list<int>> dcResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k1 = k1s[noK];
					vector<double> k2 = k2s[noK];

					if (firstRecord) {
						// record factor, k1 and k2
						for (auto pointer : expFunsResultOfstreams) {
							(*pointer) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
							Utility::output_one_vector((*pointer), "k1",k1);
							Utility::output_one_vector((*pointer), "k2",k2);
						}

						firstRecord = false;
					}

					cout << "Start to calculate the BS speeds without offset" << endl;
					/// Perform the optimization procedures
					timer.start();
					//vector<double> SAVRSpeeds = PUBSpeeds;
					vector<double> SAVRSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,WCETs, pTasks, engine, period);
					timer.end();
					double tSAVR = timer.getTime();
					cout << "End. time = " << tSAVR << endl;

					if (SAVRSpeeds.empty()) {
						found = false;
						goto CHECK_SUCCESS;
					}

					{
						totalSAVRTime += tSAVR;
						numSAVR ++;

						Utility::output_one_vector(*SAVRSpeedsOfstreams[noK],"Result Speeds",SAVRSpeeds);

						for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
							list<double> dcList = dcListVec[noDC];
							double pAUB =  OptimizationAlgorithm::getPerformanceDCExp(dcList,AUBSpeeds, k1, k2);
							double pPUB =  OptimizationAlgorithm::getPerformanceDCExp(dcList,PUBSpeeds, k1, k2);
							double pSAVR = OptimizationAlgorithm::getPerformanceDCExp(dcList,SAVRSpeeds, k1, k2);
							double tDAVR = 0.0;

							double ro = pPUB/pAUB;
							double rs = pSAVR/pAUB;

							cout << "Before DAVR->" << endl; 
							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << tAUB << " " << tPUB << " " << tSAVR << endl;

							list<int> dcResult;
#if 1
							if (dcResults.find(noDC)!=dcResults.end()) {
								dcResult = dcResults[noDC];
							} 
							else {
								cout << "Start to calculate the DC-" << dcFiles[noDC] << " speeds with offset" << endl;
								timer.start();
								dcResult = OptimizationAlgorithm::computeUBPerfDC(dcList,WCETs,aTasks,engine,period,AUBSpeeds);
								timer.end();
								tDAVR = timer.getTime();
								cout << "End. time = " << tDAVR << endl;
								totalDAVRTime += tDAVR;
								numDAVR++;

								dcResults[noDC] = dcResult;

								// Record dcResults
								string indexResultDC = DCIndexResultsDir + Utility::linkNotation()+"Factor" + Utility::int_to_string(factor) 
									+ "Run" + Utility::int_to_string(run)
									+ dcFiles[noDC] + ".result";
								ofstream foutIndexResultDC(indexResultDC, ios::out | ios::trunc);
								if (!foutIndexResultDC.is_open()) {
									cerr << "Can't open "<<indexResultDC<<" file for output" <<endl;
									exit(EXIT_FAILURE);
								}

								list<double>::iterator iter0 = dcList.begin();
								list<int>::iterator iter1 = dcResult.begin();
								while (iter0 != dcList.end()) {
									foutIndexResultDC << *iter0 << " " << *iter1 << endl;
									iter0++;
									iter1++;
								}

								foutIndexResultDC.close();
							}
#endif
							double pDAVR = OptimizationAlgorithm::getPerformanceDCExp(dcList,dcResult,k1,k2);	

							double rd = pDAVR/pAUB;

							/// output the result into the corresponding result file
							(*dcResultOfstreams[noDC]) << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK 
								<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							(*expFunsResultOfstreams[noK]) << "Factor: " << factor << " Run: " << run << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							foutResultCollection << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
							<< "=>" << ro << " " << rs << " " << rd << " " << tAUB << " " << tPUB << " " << tSAVR << " " << tDAVR << endl;

							numAll++;
							totalPUBPerf += ro;
							totalSAVRPerf += rs;
							totalDAVRPerf += rd;
						}
					}
				}

				found = true;
			}

CHECK_SUCCESS:
			if (found) numSucc++;
			else {
				numUnFeasible++;
				continue;
			}

			Utility::output_one_vector(foutPUBSpeeds,"Result Speeds",PUBSpeeds);
			Utility::output_one_vector(foutAUBSpeeds,"Result Speeds",AUBSpeeds);
			totalPUBTime += tPUB;
			totalAUBTime += tAUB;
		}

		normalizedPUBPerfs.push_back(totalPUBPerf/numAll);
		normalizedSAVRPerfs.push_back(totalSAVRPerf/numAll);
		normalizedDAVRPerfs.push_back(totalDAVRPerf/numAll);

		normalizedPUBTimes.push_back(totalPUBTime/numSystem);
		normalizedAUBTimes.push_back(totalAUBTime/numSystem);
		normalizedSAVRTimes.push_back(totalSAVRTime/numSAVR);
		normalizedDAVRTimes.push_back(totalDAVRTime/numDAVR);

		cout << "numUnFeasible = " << numUnFeasible << endl;
		cout << "numSucc = " << numSucc << endl;
		cout << "run = " << run << endl;

		foutResultCollection << "numUnFeasible = " << numUnFeasible << endl;

		Utility::output_one_vector(cout, "normalizedPUBTimes", normalizedPUBTimes);
		Utility::output_one_vector(cout, "normalizedAUBTimes", normalizedAUBTimes);

		Utility::output_one_vector(cout, "normalizedPUBPerfs", normalizedPUBPerfs);

		Utility::output_one_vector(cout, "normalizedSAVRPerfs", normalizedSAVRPerfs);
		Utility::output_one_vector(cout, "normalizedSAVRTimes", normalizedSAVRTimes);

		Utility::output_one_vector(cout, "normalizedDAVRPerfs", normalizedDAVRPerfs);
		Utility::output_one_vector(cout, "normalizedDAVRTimes", normalizedDAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPUBTimes", normalizedPUBTimes);
		Utility::output_one_vector(foutResultCollection, "normalizedAUBTimes", normalizedAUBTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPUBPerfs", normalizedPUBPerfs);

		Utility::output_one_vector(foutResultCollection, "normalizedSAVRPerfs", normalizedSAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedSAVRTimes", normalizedSAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedDAVRPerfs", normalizedDAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedDAVRTimes", normalizedDAVRTimes);
	}


	foutPUBSpeeds.close();
	foutAUBSpeeds.close();
	foutResultCollection.close();

	for (auto pointer : dcResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : expFunsResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : SAVRSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}
}

void RunTest::generateRandomSystemsWithOffset(string directory, int numSystem, int numPeriodicTask, int tUtil, double d1, double d2, int numMode) {
	// mkdir directory
	Utility::makeDir(directory);

	for (int i=0; i<numSystem; i++) {
		vector<AsynchronousPeriodicTask> tasks;
		AVRTask* avrTask;

		TaskSystemGenerator::generate_random_runnables_for_engine_optimization(tasks,numPeriodicTask,1.0*tUtil/100,d1,d2, avrTask,numMode);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask);

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}
}

void RunTest::generateRandomSystemsWithOffset(string directory, int numSystem, int numPeriodicTask, int tUtil, double o1, double o2, double d1, double d2, int numMode) {
	// mkdir directory
	Utility::makeDir(directory);

	for (int i=0; i<numSystem; i++) {
		vector<AsynchronousPeriodicTask> tasks;
		AVRTask* avrTask;

		TaskSystemGenerator::generate_random_runnables_for_engine_optimization(tasks,numPeriodicTask,1.0*tUtil/100,o1,o2,d1,d2, avrTask,numMode);

		string name = directory+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(i)+".taskset";
		const char *p = name.c_str();
		FileWriter::WriteTaskSystem(p,tasks,*avrTask);

		delete avrTask;

#ifdef __DEBUG_DESIGN__
		cout << "File name = " << name << endl;
#endif
	}
}

void RunTest::RTADAVRRandomOffsetExp(string director, string systemDir, string expFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	// mkdir
	string aSAVRSpeedsDir = director + Utility::linkNotation()+"ASAVRSpeeds"+factorName;
	Utility::makeDir(aSAVRSpeedsDir);

	string pSAVRSpeedsDir = director + Utility::linkNotation()+"PSAVRSpeeds"+factorName;
	Utility::makeDir(pSAVRSpeedsDir);

	string DCResultsDir = director + Utility::linkNotation()+"DCResults"+factorName;
	Utility::makeDir(DCResultsDir);

	string ExpFunsResultsDir = director + Utility::linkNotation()+"ExpFunsResults"+factorName;
	Utility::makeDir(ExpFunsResultsDir);

	vector<list<double>> dcListVec;
	vector<map<int,int>> dcMapVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}


	string resultUBSpeeds = director + Utility::linkNotation()+"UBSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<ofstream*> dcResultOfstreams;
	for (auto dcFile:dcFiles) {
		dcFile = DCResultsDir + Utility::linkNotation() + dcFile + ".result";
		ofstream* fout = new ofstream(dcFile,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<dcFile<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		dcResultOfstreams.push_back(fout);
	}

	// read the exponential performance functions
	expFunsFile = director + Utility::linkNotation() + expFunsFile + ".functions";
	int numK = 0;
	vector<vector<double>> k1s;
	vector<vector<double>> k2s;

	FileReader::ReadExponentialFunctions(expFunsFile.c_str(),numK,k1s,k2s);

	vector<ofstream*> expFunsResultOfstreams;
	for (int i=0; i<numK; i++) {
		string expFuncResult = ExpFunsResultsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(expFuncResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<expFuncResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		expFunsResultOfstreams.push_back(fout);
	}

	vector<ofstream*> aSAVRSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string aSAVRSpeedsResult = aSAVRSpeedsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(aSAVRSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<aSAVRSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		aSAVRSpeedsOfstreams.push_back(fout);
	}

	vector<ofstream*> pSAVRSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string pSAVRSpeedsResult = pSAVRSpeedsDir + Utility::linkNotation()+"ExpFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(pSAVRSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<pSAVRSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		pSAVRSpeedsOfstreams.push_back(fout);
	}

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedPSAVRPerfs;
	vector<double> normalizedASAVRPerfs;

	vector<double> normalizedUBTimes;
	vector<double> normalizedPSAVRTimes;
	vector<double> normalizedASAVRTimes;

	vector<int> numAlls;
	vector<int> numPSAVRs;
	vector<int> numASAVRs;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalPSAVRPerf = 0.0;
		double totalASAVRPerf = 0.0;

		double totalUBTime = 0.0;
		double totalPSAVRTime = 0.0;
		double totalASAVRTime = 0.0;

		int numAll = 0;
		int numPSAVR = 0;
		int numASAVR = 0;
		
		int run = 0;

		bool firstRecord = true;

		//while (numSucc < numSystem) {
		while (run < numSystem) {
		//for (run=94; run <=94; run ++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

			timer.start();
			vector<double> aUBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
			timer.end();
			double taUB = timer.getTime();

			if (aUBSpeeds.empty()) {
				continue;
			}
			numAll++;

			{
				map<int,list<int>> dcResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k1 = k1s[noK];
					vector<double> k2 = k2s[noK];

					if (firstRecord) {
						// record factor, k1 and k2
						for (auto pointer : expFunsResultOfstreams) {
							(*pointer) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
							Utility::output_one_vector((*pointer), "k1",k1);
							Utility::output_one_vector((*pointer), "k2",k2);
						}

						firstRecord = false;
					}

					/// Perform the optimization procedures
					timer.start();
					vector<double> aSAVRSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, aTasks, engine, period);
					timer.end();
					double tASAVR = timer.getTime();

					if (aSAVRSpeeds.empty()) {
						continue;
					}
					numASAVR++;
					totalASAVRTime += tASAVR;

					timer.start();
					//vector<double> pSAVRSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs, pTasks, engine, period);
					vector<double> pSAVRSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,WCETs, pTasks, engine, period);
					timer.end();
					double tPSAVR = timer.getTime();

					if (pSAVRSpeeds.empty()) {
						continue;	
					}
					numPSAVR++;
					totalPSAVRTime += tPSAVR;
					
					Utility::output_one_vector(*aSAVRSpeedsOfstreams[noK],"Result Speeds",aSAVRSpeeds);
					Utility::output_one_vector(*pSAVRSpeedsOfstreams[noK],"Result Speeds",pSAVRSpeeds);

					for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
						list<double> dcList = dcListVec[noDC];
						double aUB =  OptimizationAlgorithm::getPerformanceDCExp(dcList,aUBSpeeds, k1, k2);
						double pSAVR = OptimizationAlgorithm::getPerformanceDCExp(dcList,pSAVRSpeeds, k1, k2);
						double aSAVR = OptimizationAlgorithm::getPerformanceDCExp(dcList,aSAVRSpeeds, k1, k2);

						double rp = pSAVR/aUB;
						double ra = aSAVR/aUB;

						/// output the result into the corresponding result file
						(*dcResultOfstreams[noDC]) << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK 
							<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						(*expFunsResultOfstreams[noK]) << "Factor: " << factor << " Run: " << run << " DC: " << dcFiles[noDC] 
						<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
						<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						foutResultCollection << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
						<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						totalPSAVRPerf += rp;
						totalASAVRPerf += ra;
					}
					
				}
			}

			Utility::output_one_vector(foutUBSpeeds,"Result Speeds",aUBSpeeds);
			totalUBTime += taUB;
		}

		numPSAVR *= dcListVec.size();
		numASAVR *= dcListVec.size();

		normalizedPSAVRPerfs.push_back(totalPSAVRPerf/numPSAVR);
		normalizedASAVRPerfs.push_back(totalASAVRPerf/numPSAVR);

		normalizedUBTimes.push_back(totalUBTime/numSystem);
		normalizedPSAVRTimes.push_back(totalPSAVRTime/numPSAVR);
		normalizedASAVRTimes.push_back(totalASAVRTime/numPSAVR);

		cout << "run = " << run << ", numAll = " << numAll << ", numPSAVR = " << numPSAVR << ", numASAVR = " << numASAVR << endl;
		foutResultCollection  << "run = " << run << ", numAll = " << numAll << ", numPSAVR = " << numPSAVR << ", numASAVR = " << numASAVR << endl;

		numAlls.push_back(numAll);
		numPSAVRs.push_back(numPSAVR);
		numASAVRs.push_back(numASAVR);

		Utility::output_one_vector(cout, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(cout, "normalizedPSAVRPerfs", normalizedPSAVRPerfs);
		Utility::output_one_vector(cout, "normalizedPSAVRTimes", normalizedPSAVRTimes);

		Utility::output_one_vector(cout, "normalizedASAVRPerfs", normalizedASAVRPerfs);
		Utility::output_one_vector(cout, "normalizedASAVRTimes", normalizedASAVRTimes);

		Utility::output_one_vector(cout, "numAll",numAlls);
		Utility::output_one_vector(cout, "numPSAVR",numPSAVRs);
		Utility::output_one_vector(cout, "numASAVR",numASAVRs);

		Utility::output_one_vector(foutResultCollection, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPSAVRPerfs", normalizedPSAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedPSAVRTimes", normalizedPSAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedASAVRPerfs", normalizedASAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedASAVRTimes", normalizedASAVRTimes);

		Utility::output_one_vector(foutResultCollection, "numAll",numAlls);
		Utility::output_one_vector(foutResultCollection, "numPSAVR",numPSAVRs);
		Utility::output_one_vector(foutResultCollection, "numASAVR",numASAVRs);
	}


	foutUBSpeeds.close();
	foutResultCollection.close();

	for (auto pointer : dcResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : expFunsResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : aSAVRSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : pSAVRSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}
}

void RunTest::RTADAVRRandomOffsetConst(string director, string systemDir, string constFunsFile, string prefix, vector<string> dcFiles, int tUtil, int numSystem, int minFactor, int maxFactor)
{
	string factorName = "factor";
	if (minFactor == maxFactor) factorName += Utility::int_to_string(minFactor);
	else factorName += Utility::int_to_string(minFactor) + "to" + Utility::int_to_string(maxFactor);

	// mkdir
	string aSAVRSpeedsDir = director + Utility::linkNotation()+"ASAVRSpeeds"+factorName;
	Utility::makeDir(aSAVRSpeedsDir);

	string pSAVRSpeedsDir = director + Utility::linkNotation()+"PSAVRSpeeds"+factorName;
	Utility::makeDir(pSAVRSpeedsDir);

	string DCResultsDir = director + Utility::linkNotation()+"DCResults"+factorName;
	Utility::makeDir(DCResultsDir);

	string ConstFunsResultsDir = director + Utility::linkNotation()+"ConstFunsResults"+factorName;
	Utility::makeDir(ConstFunsResultsDir);

	vector<list<double>> dcListVec;
	vector<map<int,int>> dcMapVec;
	for (auto dcFile : dcFiles) {
		dcFile = prefix + dcFile + ".info";
		const char *dcFilePointer = dcFile.c_str();
		list<double> dcList  = FileReader::ReadDrivingCycleToList(dcFilePointer);
		dcListVec.push_back(dcList);
	}


	string resultUBSpeeds = director + Utility::linkNotation()+"UBSpeedsUtil"+Utility::int_to_string(tUtil)+factorName+".result";
	string resultCollection = director + Utility::linkNotation()+"AllResults"+factorName+".result";

	ofstream foutUBSpeeds(resultUBSpeeds, ios::out | ios::trunc);
	if (!foutUBSpeeds.is_open()) {
		cerr << "Can't open "<<resultUBSpeeds<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	ofstream foutResultCollection(resultCollection, ios::out | ios::trunc);
	if (!foutResultCollection.is_open()) {
		cerr << "Can't open "<<resultCollection<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<ofstream*> dcResultOfstreams;
	for (auto dcFile:dcFiles) {
		dcFile = DCResultsDir + Utility::linkNotation() + dcFile + ".result";
		ofstream* fout = new ofstream(dcFile,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<dcFile<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		dcResultOfstreams.push_back(fout);
	}

	// read the exponential performance functions
	constFunsFile = director + Utility::linkNotation() + constFunsFile + ".functions";
	int numK = 0;
	vector<vector<double>> ks;
	FileReader::ReadConstantFunctions(constFunsFile.c_str(),numK,ks);

	vector<ofstream*> constFunsResultOfstreams;
	for (int i=0; i<numK; i++) {
		string constFuncResult = ConstFunsResultsDir + Utility::linkNotation()+"ConstFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(constFuncResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<constFuncResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		constFunsResultOfstreams.push_back(fout);
	}

	vector<ofstream*> aSAVRSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string aSAVRSpeedsResult = aSAVRSpeedsDir + Utility::linkNotation()+"ConstFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(aSAVRSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<aSAVRSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		aSAVRSpeedsOfstreams.push_back(fout);
	}

	vector<ofstream*> pSAVRSpeedsOfstreams;
	for (int i=0; i<numK; i++) {
		string pSAVRSpeedsResult = pSAVRSpeedsDir + Utility::linkNotation()+"ConstFuns" + Utility::int_to_string(i) + ".result";
		ofstream* fout = new ofstream(pSAVRSpeedsResult,ios::out | ios::trunc);
		if (!(*fout).is_open()) {
			cerr << "Can't open "<<pSAVRSpeedsResult<<" file for output" <<endl;
			exit(EXIT_FAILURE);
		}
		pSAVRSpeedsOfstreams.push_back(fout);
	}

	Engine engine = TaskSystemGenerator::generate_engine();
	double period = TaskSystemGenerator::ANGULAR_PERIOD;

	vector<double> normalizedPSAVRPerfs;
	vector<double> normalizedASAVRPerfs;

	vector<double> normalizedUBTimes;
	vector<double> normalizedPSAVRTimes;
	vector<double> normalizedASAVRTimes;

	vector<int> numAlls;
	vector<int> numPSAVRs;
	vector<int> numASAVRs;

	Timer timer;

	for (int factor = minFactor; factor <= maxFactor; factor++) {
		double totalPSAVRPerf = 0.0;
		double totalASAVRPerf = 0.0;

		double totalUBTime = 0.0;
		double totalPSAVRTime = 0.0;
		double totalASAVRTime = 0.0;

		int numAll = 0;
		int numPSAVR = 0;
		int numASAVR = 0;

		int run = 0;

		bool firstRecord = true;

		//while (numSucc < numSystem) {
		while (run < numSystem) {
			//for (run=94; run <=94; run ++) {
			cout << "factor = " << factor << ", run = " << run << " => ";
			foutResultCollection << "factor = " << factor << ", run = " << run << " => ";
			vector<PeriodicTask> pTasks;
			vector<AsynchronousPeriodicTask> aTasks;
			vector<int> WCETs;

			string name = director + Utility::linkNotation() + systemDir+Utility::linkNotation()+"Util"+Utility::int_to_string(tUtil)+"Run"+Utility::int_to_string(run)+".taskset";
			const char *p = name.c_str();
			FileReader::ReadTaskSystem(p,pTasks,aTasks,WCETs);

			run++;

			for (int i=0; i<WCETs.size(); i++)
				WCETs[i] *= factor;

			timer.start();
			vector<double> aUBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,aTasks,engine,period);
			timer.end();
			double taUB = timer.getTime();

			if (aUBSpeeds.empty()) {
				continue;
			}
			numAll++;

			{
				map<int,list<int>> dcResults;

				for (int noK = 0; noK < numK; noK++) {
					vector<double> k = ks[noK];

					if (firstRecord) {
						// record factor, k1 and k2
						for (auto pointer : constFunsResultOfstreams) {
							(*pointer) << "++++++++++++++ factor = " << factor << "++++++++++++++" << endl;
							Utility::output_one_vector((*pointer), "k",k);
						}

						firstRecord = false;
					}

					/// Perform the optimization procedures
					timer.start();
					//vector<double> aSAVRSpeeds = aUBSpeeds;
					vector<double> aSAVRSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, aTasks, engine, period);
					timer.end();
					double tASAVR = timer.getTime();

					if (aSAVRSpeeds.empty()) {
						continue;
					}
					numASAVR++;
					totalASAVRTime += tASAVR;

					timer.start();
					//vector<double> pSAVRSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,WCETs,pTasks,engine,period);
					vector<double> pSAVRSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,k,WCETs, pTasks, engine, period);
					timer.end();
					double tPSAVR = timer.getTime();

					if (pSAVRSpeeds.empty()) {
						continue;	
					}
					numPSAVR++;
					totalPSAVRTime += tPSAVR;

					Utility::output_one_vector(*aSAVRSpeedsOfstreams[noK],"Result Speeds",aSAVRSpeeds);
					Utility::output_one_vector(*pSAVRSpeedsOfstreams[noK],"Result Speeds",pSAVRSpeeds);

					for (int noDC = 0; noDC < dcListVec.size(); noDC ++) {
						list<double> dcList = dcListVec[noDC];
						double aUB =  OptimizationAlgorithm::getPerformanceDC(dcList,aUBSpeeds, k);
						double pSAVR = OptimizationAlgorithm::getPerformanceDC(dcList,pSAVRSpeeds, k);
						double aSAVR = OptimizationAlgorithm::getPerformanceDC(dcList,aSAVRSpeeds, k);

						double rp = pSAVR/aUB;
						double ra = aSAVR/aUB;

						/// output the result into the corresponding result file
						(*dcResultOfstreams[noDC]) << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK 
							<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						(*constFunsResultOfstreams[noK]) << "Factor: " << factor << " Run: " << run << " DC: " << dcFiles[noDC] 
						<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						cout << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
						<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						foutResultCollection << "Factor: " << factor << " Run: " << run << " ExpNo: " << noK << " DC: " << dcFiles[noDC] 
						<< "=>" << rp << " " << ra << " " << tPSAVR << " " << tASAVR << endl;

						totalPSAVRPerf += rp;
						totalASAVRPerf += ra;
					}
				}
			}

			Utility::output_one_vector(foutUBSpeeds,"Result Speeds",aUBSpeeds);
			totalUBTime += taUB;
		}

		cout << "run = " << run << ", numAll = " << numAll << ", numPSAVR = " << numPSAVR << ", numASAVR = " << numASAVR << endl;
		foutResultCollection  << "run = " << run << ", numAll = " << numAll << ", numPSAVR = " << numPSAVR << ", numASAVR = " << numASAVR << endl;

		numAlls.push_back(numAll);
		numPSAVRs.push_back(numPSAVR);
		numASAVRs.push_back(numASAVR);

		normalizedPSAVRPerfs.push_back(totalPSAVRPerf/(numPSAVR*dcListVec.size()));
		normalizedASAVRPerfs.push_back(totalASAVRPerf/(numPSAVR*dcListVec.size()));

		normalizedUBTimes.push_back(totalUBTime/numSystem);
		normalizedPSAVRTimes.push_back(totalPSAVRTime/numPSAVR);
		normalizedASAVRTimes.push_back(totalASAVRTime/numASAVR);

		Utility::output_one_vector(cout, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(cout, "normalizedPSAVRPerfs", normalizedPSAVRPerfs);
		Utility::output_one_vector(cout, "normalizedPSAVRTimes", normalizedPSAVRTimes);

		Utility::output_one_vector(cout, "normalizedASAVRPerfs", normalizedASAVRPerfs);
		Utility::output_one_vector(cout, "normalizedASAVRTimes", normalizedASAVRTimes);

		Utility::output_one_vector(cout, "numAll",numAlls);
		Utility::output_one_vector(cout, "numPSAVR",numPSAVRs);
		Utility::output_one_vector(cout, "numASAVR",numASAVRs);

		Utility::output_one_vector(foutResultCollection, "normalizedUBTimes", normalizedUBTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedPSAVRPerfs", normalizedPSAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedPSAVRTimes", normalizedPSAVRTimes);

		Utility::output_one_vector(foutResultCollection, "normalizedASAVRPerfs", normalizedASAVRPerfs);
		Utility::output_one_vector(foutResultCollection, "normalizedASAVRTimes", normalizedASAVRTimes);

		Utility::output_one_vector(foutResultCollection, "numAll",numAlls);
		Utility::output_one_vector(foutResultCollection, "numPSAVR",numPSAVRs);
		Utility::output_one_vector(foutResultCollection, "numASAVR",numASAVRs);
	}


	foutUBSpeeds.close();
	foutResultCollection.close();

	for (auto pointer : dcResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : constFunsResultOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : aSAVRSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}

	for (auto pointer : pSAVRSpeedsOfstreams) {
		(*pointer).close();
		delete pointer;
	}
}