#ifndef PERIODICTASK_H_
#define PERIODICTASK_H_

#include <iostream>
#include <math.h>

#include "Task.h"
//#include "AsynchronousPeriodicTask.h"

class PeriodicTask : public Task {
public:
	int wcet; // wcet of the periodic task, in us
	int deadline; // deadline of the periodic task, in us
	int period; // period of the periodic task, in us

	PeriodicTask(int _wcet, int _period) {
		wcet = _wcet;
		period = _period;
		deadline = _period;
	}

	PeriodicTask(int _wcet, int _deadline, int _period) {
		wcet = _wcet;
		deadline = _deadline;
		period = _period;
	}

	/*
	PeriodicTask(AsynchronousPeriodicTask aTask) {
		wcet = aTask.wcet;
		period = aTask.period;
		deadline = aTask.deadline;
	}
	*/

	int getValue(int t) {
		int nPeriod = ceil(1.0*t/period);
		return nPeriod*wcet;
	}

	void output(std::ostream& out) {
		out << wcet << "\t" << deadline << "\t" << period << std::endl;
	}
};

#endif
