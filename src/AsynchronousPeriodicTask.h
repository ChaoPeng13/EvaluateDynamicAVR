#ifndef ASYNCHRONOUS_PERIODICTASK_H_
#define ASYNCHRONOUS_PERIODICTASK_H_

#include <iostream>
#include <algorithm>

#include "Task.h"
#include "PeriodicTask.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////
/// the definition of the asynchronous periodic tasks used in the 
/// reference paper "Response Time Analysis of Asynchronous Periodic 
/// and Sporadic Tasks Scheduled by a Fixed-Priority Preemptive Algorithm" 
//////////////////////////////////////////////////////////////////////////
class AsynchronousPeriodicTask : public Task {
public:
	int wcet; // wcet of the periodic task, in us
	int deadline; // deadline of the periodic task, in us
	int period; // period of the periodic task, in us
	int offset; // offset of the periodic task, in us
	            // activated at k*period + offset where k \in N

	AsynchronousPeriodicTask(PeriodicTask pt) {
		wcet = pt.wcet;
		period = pt.period;
		deadline = pt.deadline;
		offset = 0;
	}

	AsynchronousPeriodicTask(PeriodicTask pt, int _offset) {
		wcet = pt.wcet;
		period = pt.period;
		deadline = pt.deadline;
		offset = _offset;
	}

	AsynchronousPeriodicTask(int _wcet, int _period) {
		wcet = _wcet;
		period = _period;
		deadline = _period;
		offset = 0;
	}

	AsynchronousPeriodicTask(int _wcet, int _period, int _offset) {
		wcet = _wcet;
		period = _period;
		deadline = _period;
		offset = _offset;
	}

	AsynchronousPeriodicTask(int _wcet, int _deadline, int _period, int _offset) {
		wcet = _wcet;
		deadline = _deadline;
		period = _period;
		offset = _offset;
	}

	/// Eq. 3 in the paper
	int getWorkload(int t) {
		int ret = ceil((1.0*t-offset)/period) * wcet;
		return max(ret,0);
	}

	/// Eq. 11 in the paper
	int getWorkloadBar(int t) {
		int ret = floor(1.0+(1.0*t-offset)/period) * wcet;
		return max(ret,0);
	}

	/// Eq. 13 in the paper
	int getNextComputingInstant(int t) {
		int ret = ceil((1.0*t-offset)/period) * period;
		return max(ret,0) + offset;
	}

	void output(std::ostream& out) {
		out << wcet << "\t" << deadline << "\t" << period << "\t" << offset << std::endl;
	}
};

#endif