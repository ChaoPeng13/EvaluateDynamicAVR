/* \file StateGraph.h
*  this file decribes the StateNode and StateGraph class. 
*  \author Chao Peng
*  
*  Changes
*  ------
*  03-Jul-2017 : initial revision (CP)
*
*/

#ifndef STATEGRAPH_H_
#define STATEGRAPH_H_

#include <list>
#include <vector>
#include <fstream>

#include "Global.h"

using namespace std;

enum PerformanceType {
	PERF_CONSTANT,
	PERF_EXPONENTIAL_MAX,
	PERF_EXPONENTIAL_MIN
};

class StateGraphNode {
public:
	StateGraphNode(void) { }
	StateGraphNode(string _name, vector<double> _speeds, vector<int> _wcets, PerformanceType _pt, vector<vector<double>> _ks, bool _initial) {
		name = _name;
		speeds = _speeds;
		for (auto speed : speeds) 
			omegas.push_back(RPM_to_RPmSEC(speed));
		wcets = _wcets;
		pt = _pt;
		ks = _ks;

		// check performance type
		if ((pt == PERF_CONSTANT && ks.size() !=1 ) ||
			(pt == PERF_EXPONENTIAL_MAX && ks.size() != 2) ||
			(pt == PERF_EXPONENTIAL_MIN && ks.size() != 2)) {
				cerr << "Error: Performance Type!" << endl;
				exit(EXIT_FAILURE);
		}

		initial = _initial;
	}
	~StateGraphNode(void) { }

	string name;
	vector<double> speeds; // RPM
	vector<double> omegas; // RPmSEC
	vector<int> wcets; // muSec
	PerformanceType pt;
	vector<vector<double>> ks;
	bool initial;

	vector<double> transConditions;
	vector<StateGraphNode*> to;

	void addCondition(double speed) { transConditions.push_back(speed); }
	void connectTo(StateGraphNode* nextNode) { to.push_back(nextNode); }
	StateGraphNode* getNextNode(double nextSpeed);
	double getNodePerformance(double speed);
	bool equal(StateGraphNode* node);
};

class StateGraph {
public:
	StateGraph(void) { }
	StateGraph(PerformanceType _pt) { pt = _pt; }
	~StateGraph(void) { for (auto node:graph) delete node; }

	PerformanceType pt;
	vector<StateGraphNode*> graph;

	StateGraphNode* getInitialNode(double speed);
	void add(StateGraphNode* newNode);
	void add(string name, vector<double> speeds, vector<int> wcets, PerformanceType pt, vector<vector<double>> ks, bool initial);

	bool checkExisted(StateGraphNode* node);
	StateGraphNode* getExistedNode(StateGraphNode* node);

	double getTotalPerformance(list<double> speedProfile);

	/// \brief write a digraph object into an output stream in graphviz dot format
	void write_graphviz(ostream& out);
};

#endif 