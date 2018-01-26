#ifndef COLLECTIONRESULT_H_
#define COLLECTIONRESULT_H_

#include <vector>

#include "PeriodicTask.h"

using namespace std;

class CollectionResult {
public:
	vector<PeriodicTask> tasks;
	vector<double> speeds;

	CollectionResult() {}

	CollectionResult(vector<PeriodicTask> _tasks, vector<double> _speeds) {
		tasks = _tasks;
		speeds = _speeds;
	}
};

#endif