#include "FileReader.h"
#include "TaskSystemGenerator.h"
#include "OptimizationAlgorithm.h"

void FileReader::ReadTaskSystem(const char* fname, vector<PeriodicTask> &tasks, vector<int> &WCETs) {
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	int lineNum = 0;
	int n;
	int size;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,'\t')) {
			str_list.push_back(sub_str);
		}

		if (lineNum == 0) { // the first line
			n = atof(str_list.at(0).c_str());
		}
		else {
			if (lineNum <= n) {
				if (str_list.size()==2) {
					int wcet = atof(str_list.at(0).c_str());
					int period = atof(str_list.at(1).c_str());
					tasks.push_back(PeriodicTask(wcet,period,period));
				}
				else if (str_list.size()==3) {
					int wcet = atof(str_list.at(0).c_str());
					int deadline = atof(str_list.at(1).c_str());
					int period = atof(str_list.at(2).c_str());
					tasks.push_back(PeriodicTask(wcet,deadline,period));
				}
				else {
					cerr << "Error: Input Periodic Task, (wcet,deadline,period)" << endl;
					exit(EXIT_FAILURE);
				}
				
			}
			else if (lineNum == n+1) {
				size = atof(str_list.at(0).c_str());
				for (int i=0; i<size; i++) 
					WCETs.push_back( atof(str_list.at(i+1).c_str()) );

			}
			else break;
		}
		lineNum++;
	}

	fin.close();
}

void FileReader::ReadTaskSystem(const char* fname, vector<PeriodicTask> &pTasks, vector<AsynchronousPeriodicTask> &aTasks, vector<int> &WCETs) {
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	int lineNum = 0;
	int n;
	int size;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,'\t')) {
			str_list.push_back(sub_str);
		}

		if (lineNum == 0) { // the first line
			n = atof(str_list.at(0).c_str());
		}
		else {
			if (lineNum <= n) {
				if (str_list.size()==2) {
					int wcet = atof(str_list.at(0).c_str());
					int period = atof(str_list.at(1).c_str());
					pTasks.push_back(PeriodicTask(wcet,period,period));
				}
				else if (str_list.size()==3) {
					int wcet = atof(str_list.at(0).c_str());
					int deadline = atof(str_list.at(1).c_str());
					int period = atof(str_list.at(2).c_str());
					pTasks.push_back(PeriodicTask(wcet,deadline,period));
				}
				else if (str_list.size()==4) {
					int wcet = atof(str_list.at(0).c_str());
					int deadline = atof(str_list.at(1).c_str());
					int period = atof(str_list.at(2).c_str());
					int offset = atof(str_list.at(3).c_str());
					pTasks.push_back(PeriodicTask(wcet,deadline,period));
					aTasks.push_back(AsynchronousPeriodicTask(wcet,deadline,period,offset));
				}
				else {
					cerr << "Error: Input Periodic Task, (wcet,deadline,period)" << endl;
					exit(EXIT_FAILURE);
				}

			}
			else if (lineNum == n+1) {
				size = atof(str_list.at(0).c_str());
				for (int i=0; i<size; i++) 
					WCETs.push_back( atof(str_list.at(i+1).c_str()) );

			}
			else break;
		}
		lineNum++;
	}

	fin.close();
}

void FileReader::ReadTaskSystem(const char* fname, vector<PeriodicTask> &tasks, int &avrTaskIndex, vector<int> &WCETs, vector<double> &speeds) {
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	int lineNum = 0;
	int n;
	int size;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,'\t')) {
			str_list.push_back(sub_str);
		}

		if (lineNum == 0) { // the first line
			n = atof(str_list.at(0).c_str());
		}
		else {
			if (lineNum <= n) {
				int wcet = atof(str_list.at(0).c_str());
				int deadline = atof(str_list.at(1).c_str());
				int period = atof(str_list.at(2).c_str());
				tasks.push_back(PeriodicTask(wcet,deadline,period));
			}
			else if (lineNum == n+1) {
				avrTaskIndex = atoi(str_list.at(0).c_str());
			}
			else if (lineNum == n+2) {
				size = atof(str_list.at(0).c_str());
				for (int i=0; i<size; i++) 
					WCETs.push_back( atof(str_list.at(i+1).c_str()) );
			}
			else if (lineNum == n+3) {
				size = atof(str_list.at(0).c_str());
				for (int i=0; i<size; i++) 
					speeds.push_back( atof(str_list.at(i+1).c_str()) );
			}
			else break;
		}
		lineNum++;
	}
	fin.close();
}

void FileReader::ReadTaskSystem(const char* fname, vector<PeriodicTask> &tasks, int &avrTaskIndex, Engine engine,double period, vector<AVRTask> &avrTasks) {
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	int lineNum = 0;
	int n;
	int size;
	int numAVRTasks;
	vector<int> WCETs;
	vector<double> speeds;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,'\t')) {
			str_list.push_back(sub_str);
		}

		if (lineNum == 0) { // the first line
			n = atof(str_list.at(0).c_str());
		}
		else {
			if (lineNum <= n) {
				int wcet = atof(str_list.at(0).c_str());
				int deadline = atof(str_list.at(1).c_str());
				int period = atof(str_list.at(2).c_str());
				tasks.push_back(PeriodicTask(wcet,deadline,period));
			}
			else if (lineNum == n+1) {
				avrTaskIndex = atoi(str_list.at(0).c_str());
			}
			else if (lineNum == n+2) {
				numAVRTasks = atoi(str_list.at(0).c_str());
			}
			else if ((lineNum > n+2) && ((lineNum-(n+2))%2==1)) {
				size = atof(str_list.at(0).c_str());
				for (int i=0; i<size; i++) 
					WCETs.push_back( atof(str_list.at(i+1).c_str()) );
			}
			else if ((lineNum > n+2) && ((lineNum-(n+2))%2==0)) {
				size = atof(str_list.at(0).c_str());
				for (int i=0; i<size; i++) 
					speeds.push_back( atof(str_list.at(i+1).c_str()) );

				AVRTask avrTask(engine,period,speeds,WCETs);
				avrTasks.push_back(avrTask);
				WCETs.clear();
				speeds.clear();
			}
			else break;
		}
		lineNum++;
	}
	fin.close();
}

map<int,int> FileReader::ReadDrivingCycle(const char* fname) {
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	map<int,int> ret;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,' ')) {
			str_list.push_back(sub_str);
		}

		int key = atof(str_list.at(0).c_str());
		int value = atof(str_list.at(1).c_str());

		ret[key] = value;
	}

	fin.close();

	return ret;
}

std::vector<std::map<int,int>> FileReader::ReadDrivingCycleVector(const char* fname)
{
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<map<int,int>> ret;
	bool up = true;
	bool down = false;
	bool constant = false;

	map<int,int> dc;

	vector<double> vSpeeds;
	vector<double> eSpeeds;

	vector<int> points;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,' ')) {
			str_list.push_back(sub_str);
		}

		double vSpeed = atof(str_list.at(0).c_str());
		double eSpeed = atof(str_list.at(1).c_str());

		if (fabs(vSpeed) <= Utility::EPSILON) {

#ifndef __USING_FINE_SEPARATION__
			if (!vSpeeds.empty()) {
				int position = vSpeeds.size()-1;
				vector<int>::iterator result = find(points.begin(),points.end(),position);
				if (result == points.end()) points.push_back(position);
			}
#endif
			continue;
		}
			
		vSpeeds.push_back(vSpeed);
		eSpeeds.push_back(eSpeed);
	}

#ifndef __USING_FINE_SEPARATION__
	vector<int>::iterator result = find(points.begin(),points.end(),vSpeeds.size()-1);
	if (result == points.end()) points.push_back(vSpeeds.size()-1);
#endif

	fin.close();

#ifdef __USING_FINE_SEPARATION__
	for (int i=0; i<vSpeeds.size()-1; i++) {
		double currVSpeed = vSpeeds[i];
		double nextVSpeed = vSpeeds[i+1];

		if ((down || constant) && (nextVSpeed > currVSpeed)) {
			up = true;
			down = constant = false;
			points.push_back(i);
		}
		
		if ((down || up) && (fabs(nextVSpeed-currVSpeed) <= Utility::EPSILON)) {
			constant = true;
			down = up = false;
			points.push_back(i-1);
		}

		if ((up || constant) && (nextVSpeed < currVSpeed)) {
			down = true;
			up = constant = false;
			points.push_back(i);
		}
	}
	points.push_back(vSpeeds.size()-1);
#endif	

	int start = 0;
	for (int i=0; i < points.size(); i++) {
		int end = points[i];
		map<int,int> dc;
		for (int j=start; j<= end; j++) {
			int eSpeed = Utility::round(eSpeeds[j]);
			if (dc.find(eSpeed)!=dc.end()) {
				dc[eSpeed] += 1;
			}
			else dc[eSpeed] = 1;
		}
		ret.push_back(dc);

		start = end+1;
	}

	fin.close();

	return ret;
}

std::vector<std::map<int,int>> FileReader::ReadDrivingCycleVectorWithConstantStep(const char* fname)
{
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<map<int,int>> ret;

	vector<double> vSpeeds;
	vector<double> eSpeeds;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,' ')) {
			str_list.push_back(sub_str);
		}

		double vSpeed = atof(str_list.at(0).c_str());
		double eSpeed = atof(str_list.at(1).c_str());

		if (fabs(vSpeed) <= Utility::EPSILON) {
			continue;
		}

		vSpeeds.push_back(vSpeed);
		eSpeeds.push_back(eSpeed);
	}

	map<int,int> dc;
	for (int i=0; i < eSpeeds.size(); i++) {
		if ( (i+1) % CONSTANT_DC_STEP == 0 ) {
			if (!dc.empty()) ret.push_back(dc);
			dc.clear();
		}
		int eSpeed = Utility::round(eSpeeds[i]);
		
		if (dc.find(eSpeed)!=dc.end()) {
			dc[eSpeed] += 1;
		}
		else dc[eSpeed] = 1;
	}

	if (!dc.empty()) ret.push_back(dc);

	return ret;
}

std::list<double> FileReader::ReadDrivingCycleToList(const char* fname)
{
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	list<double> ret;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		string sub_str;
		vector<string> str_list;
		while(getline(ss,sub_str,' ')) {
			str_list.push_back(sub_str);
		}

		double vSpeed = atof(str_list.at(0).c_str());
		double eSpeed = atof(str_list.at(1).c_str());

		//if (fabs(vSpeed) <= Utility::EPSILON) continue;
		if (W_LEQ_TOL(fabs(eSpeed), TaskSystemGenerator::RPM_MIN)) continue;
		ret.push_back(eSpeed);
	}

	fin.close();
	return ret;
}

map<int,vector<double>> FileReader::ComputePerformanceResult(map<int,int> dc, string fileName, string prefix) {
	const char* fname = fileName.c_str();
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	map<int,vector<double>> ret;
	vector<double> k;
	vector<double> perfs;
	int factor = 0;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		vector<string> str_list = Utility::split(line," ,=[];+");

		if (str_list.empty()) continue;

		string first = str_list.front();
		if (first.compare("factor") == 0) {
			if (!perfs.empty()) {
				ret[factor] = perfs;
				perfs.clear();
			}
			factor = atoi(str_list[1].c_str());
		}
		else if (first.compare("k") == 0) {
			k.clear();
			for (int i=1; i<str_list.size(); i++)
				k.push_back(atof(str_list[i].c_str()));
		}
		else if (first.compare(prefix) == 0) {
			vector<double> speeds;
			for (int i = 2; i<str_list.size(); i++)
				speeds.push_back(atof(str_list[i].c_str()));
			double perf = OptimizationAlgorithm::getPerformanceDC(dc,speeds,k);
			perfs.push_back(perf);
		}
		else {
			cout << "Can not arrive here!" << endl;
		}
	}

	ret[factor] = perfs;

	fin.close();

	return ret;
}

map<int,map<int,map<int, double>>> FileReader::ComputePerformanceExpResult(map<int,int> dc, string fileName, string prefix) {
	const char* fname = fileName.c_str();
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	map<int,map<int,map<int,double>>> ret;
	vector<double> k1;
	vector<double> k2;
	vector<double> perfs;
	int factor = 0;
	int kNum = -1;

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		vector<string> str_list = Utility::split(line," ,=[];+");

		if (str_list.empty()) continue;

		string first = str_list.front();
		if (first.compare("factor") == 0) {
			factor = atoi(str_list[1].c_str());
			kNum = -1;
		}
		else if (first.compare("k1") == 0) {
			k1.clear();
			for (int i=1; i<str_list.size(); i++)
				k1.push_back(atof(str_list[i].c_str()));
			kNum++;
		}
		else if (first.compare("k2") == 0) {
			k2.clear();
			for (int i=1; i<str_list.size(); i++)
				k2.push_back(atof(str_list[i].c_str()));
		}
		else if (first.compare("System") == 0) {
			int tasksetNum = atoi(str_list[1].c_str());
			vector<double> speeds;
			for (int i = 4; i<str_list.size(); i++)
				speeds.push_back(atof(str_list[i].c_str()));
			double perf = OptimizationAlgorithm::getPerformanceDCExp(dc,speeds,k1,k2);
			ret[factor][kNum][tasksetNum] = perf;
		}
		else {
			cout << "Can not arrive here!" << endl;
		}
	}

	fin.close();

	return ret;
}

vector<double> FileReader::ReadVectorFromFile(string fileName) {
	const char* fname = fileName.c_str();
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	vector<double> ret;

	string line;
	while (getline(fin,line)) {
		ret.push_back(atof(line.c_str()));
	}

	fin.close();

	return ret;
}

void FileReader::ReadExponentialFunctions(const char* fname, int& numK, vector<vector<double>>& k1s, vector<vector<double>>& k2s)
{
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		vector<string> str_list = Utility::split(line," ,=[];+");

		string first = str_list.front();
		if (first.compare("#start") == 0) {
			numK = atoi(str_list[2].c_str());
		}
		else if (first.compare("k1") == 0) {
			vector<double> k1;
			for (int i=1; i<str_list.size(); i++)
				k1.push_back(atof(str_list[i].c_str()));
			k1s.push_back(k1);
		}
		else if (first.compare("k2") == 0) {
			vector<double> k2;
			for (int i=1; i<str_list.size(); i++)
				k2.push_back(atof(str_list[i].c_str()));
			k2s.push_back(k2);
		}
		else if (first.compare("#end") == 0) {
			break;
		}
		else {
			cout << "Can not arrive here!" << endl;
		}
	}

	fin.close();
}

void FileReader::ReadConstantFunctions(const char* fname, int& numK, vector<vector<double>>& ks)
{
	ifstream fin(fname);
	if (!fin.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	string line;
	while (getline(fin,line)) {
		// parse the line
		stringstream ss(line);
		vector<string> str_list = Utility::split(line," ,=[];+");

		string first = str_list.front();
		if (first.compare("#start") == 0) {
			numK = atoi(str_list[2].c_str());
		}
		else if (first.compare("k") == 0) {
			vector<double> k;
			for (int i=1; i<str_list.size(); i++)
				k.push_back(atof(str_list[i].c_str()));
			ks.push_back(k);
		}
		else if (first.compare("#end") == 0) {
			break;
		}
		else {
			cout << "Can not arrive here!" << endl;
		}
	}

	fin.close();
}
