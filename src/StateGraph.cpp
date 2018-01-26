#include "StateGraph.h"
#include "OptimizationAlgorithm.h"

StateGraphNode* StateGraphNode::getNextNode(double nextSpeed)
{
	if (to.empty()) {
		cerr << "Error: empty!" << endl;
		exit(EXIT_FAILURE);
	}

	if (to.size() == 1) return to.front();

	int i = 0;
	for (auto speed : transConditions) {
		if (W_LEQ_TOL(nextSpeed, speed)) break;
		i++;
	}

	if (i == to.size()) return this;

	return to[i];
}

double StateGraphNode::getNodePerformance(double speed)
{
	int index = OptimizationAlgorithm::getSpeedIndex(speeds,speed);
	double perf = 0.0;

	if (pt == PERF_CONSTANT)
		perf = ks[0][index];
	else 
		perf = ks[0][index]*exp(-ks[1][index]/speed);
	
	return perf;
}

bool StateGraphNode::equal(StateGraphNode* node)
{
	if (speeds.size() != node->speeds.size()) return false;

	for (int i=0; i<speeds.size(); i++) {
		if (wcets[i] != node->wcets[i]) return false;
		if (!W_EQ_TOL(speeds[i],node->speeds[i])) return false;
	}

	return true;
}

StateGraphNode* StateGraph::getInitialNode(double speed)
{
	StateGraphNode* result = NULL;
	double currPerf = -1.0;
	for (vector<StateGraphNode*>::iterator i = graph.begin(); i != graph.end(); i ++) {
		if (!(*i)->initial) continue;

		double perf = (*i)->getNodePerformance(speed);
		if (perf > currPerf) {
			currPerf = perf;
			result = *i;
		}
	}
	return result;
}

void StateGraph::add(StateGraphNode* newNode)
{
	graph.push_back(newNode);
}

void StateGraph::add(string name, vector<double> speeds, vector<int> wcets, PerformanceType pt, vector<vector<double>> ks, bool initial)
{
	StateGraphNode* node = new StateGraphNode(name, speeds,wcets,pt,ks,initial);
	graph.push_back(node);
}

bool StateGraph::checkExisted(StateGraphNode* node)
{
	for (vector<StateGraphNode*>::iterator it = graph.begin(); it != graph.end(); it ++)
		if ((*it)->equal(node)) return true;

	return false;
}

StateGraphNode* StateGraph::getExistedNode(StateGraphNode* node)
{
	for (vector<StateGraphNode*>::iterator it = graph.begin(); it != graph.end(); it ++)
		if ((*it)->equal(node)) return *it;

	return NULL;
}

double StateGraph::getTotalPerformance(list<double> speedProfile)
{
	double totalPerf = 0.0;

	StateGraphNode* start = NULL;
	for (auto speed : speedProfile) {
		if (start == NULL) start = getInitialNode(speed);
		else start = start->getNextNode(speed);
		totalPerf += start->getNodePerformance(speed);

		/*
		double perf = start->getNodePerformance(speed);
		cout << speed << "\t" << perf << endl;
		
		if (W_EQ_TOL(totalPerf,6606)) 
			cout << "here" << endl;
			*/
	}

	return totalPerf;
}

void StateGraph::write_graphviz(ostream& out)
{
	vector<string> sVecs;
	if (pt == PERF_CONSTANT)
		sVecs.push_back("k");
	else {
		sVecs.push_back("k1");
		sVecs.push_back("k2");
	}

	out << "digraph G {" << endl;
	for (auto node : graph) {
		out << node->name << "[label=\" " << node->name 
			<< " / " << Utility::get_one_vector("TransistionSpeeds", node->speeds)
			<< " / " << Utility::get_one_vector("WCETs", node->wcets)
			<< " / " << Utility::get_multiple_vectors(sVecs, node->ks)
		    << " \"];" << endl;
	}

	for (auto node : graph) {
		for (int i=0; i<node->to.size(); i++) {
			StateGraphNode* nextNode = node->to[i];
			out << node->name << " -> " << nextNode->name;
			if (i==0)
				cout << " [label=\" 0, "  << node->transConditions[i] << " \"]" << endl;
			else
				cout << " [label=\"  " << node->transConditions[i-1] << ", "  << node->transConditions[i] << " \"]" << endl;
		}
	}

	out << "}" <<endl;
}
