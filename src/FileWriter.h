#ifndef FILEWRITER_H_
#define FILEWRITER_H_

#include <iostream>
#include <fstream>

#include "AVRTask.h"
#include "PeriodicTask.h"
#include "AsynchronousPeriodicTask.h"

class FileWriter {
public:
	static void WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, AVRTask avrTask);
	static void WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, vector<int> wcets);
	static void WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex);
	static void WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, vector<AVRTask*> avrTasks, int avrTaskIndex);

	static void WriteTaskSystem(const char* fname, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask);
	static void WriteTaskSystemLatex(const char* fname, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask);
};

#endif