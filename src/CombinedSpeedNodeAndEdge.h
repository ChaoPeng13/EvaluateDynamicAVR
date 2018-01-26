#ifndef COMBINEDSPEEDNODEANDEDGE_H_
#define COMBINEDSPEEDNODEANDEDGE_H_

#include <vector>
#include <list>

class CombinedSpeedNode;
class CombinedSpeedEdge;

class CombinedSpeedNode {
public:
	string name;
	vector<double> speeds;
	int wcet; // us

	CombinedSpeedNode(string _name, vector<double> _speeds, int _wcet) {
		name = _name;
		speeds = _speeds;
		wcet = _wcet;
	}

	list<CombinedSpeedEdge*> out;
};

class CombinedSpeedEdge {
public:
	CombinedSpeedNode* src;
	CombinedSpeedNode* snk;
	int period; // us

	CombinedSpeedEdge(CombinedSpeedNode* _src, CombinedSpeedNode* _snk, int _period) {
		src = _src;
		snk = _snk;
		period = _period;
	}
};

#endif // CombinedSpeedNode