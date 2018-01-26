#ifndef FILEREADER_H_
#define FILEREADER_H_

//#include <vld.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "PeriodicTask.h"
#include "AsynchronousPeriodicTask.h"
#include "AVRTask.h"

class FileReader {
public:
	static void ReadTaskSystem(const char* fname, vector<PeriodicTask> &tasks, vector<int> &WCETs);
	static void ReadTaskSystem(const char* fname, vector<PeriodicTask> &pTasks, vector<AsynchronousPeriodicTask> &aTasks, vector<int> &WCETs);
	static void ReadTaskSystem(const char* fname, vector<PeriodicTask> &tasks, int &avrTaskIndex, vector<int> &WCETs, vector<double> &speeds); 
	static void ReadTaskSystem(const char* fname, vector<PeriodicTask> &tasks, int &avrTaskIndex, Engine engine, double period, vector<AVRTask> &avrTasks); 
	
	static map<int,int> ReadDrivingCycle(const char* fname);
	static vector<map<int,int>> ReadDrivingCycleVector(const char* fname);
	static vector<map<int,int>> ReadDrivingCycleVectorWithConstantStep(const char* fname);
	static list<double> ReadDrivingCycleToList(const char* fname);
	static map<int,vector<double>> ComputePerformanceResult(map<int,int> dc, string fileName, string prefix);
	static map<int,map<int,map<int,double>>> ComputePerformanceExpResult(map<int,int> dc, string fileName, string prefix);
	static vector<double> ReadVectorFromFile(string fileName);
	static void ReadExponentialFunctions(const char* fname, int& numK, vector<vector<double>>& k1s, vector<vector<double>>& k2s);
	static void ReadConstantFunctions(const char* fname, int& numK, vector<vector<double>>& ks);
};

#endif