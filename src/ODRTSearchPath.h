#ifndef ODRTSEARCHPATH_H_
#define ODRTSEARCHPATH_H_

#include <vector>

#include "Digraph.h"

using namespace std;

class ODRTSearchPath {
public:
	vector<Node*> nodes;
	vector<int> tMinusVec;
	vector<int> tPlusVec;
	vector<Edge*> edges;
	//int sumMinSeparationTime;

	ODRTSearchPath() {}

	void addNode(Node* node, int tMinus, int tPlus) {
		nodes.push_back(node);
		tMinusVec.push_back(tMinus);
		tPlusVec.push_back(tPlus);
	}

	void addEdge(Edge* edge) {
		edges.push_back(edge);
		//sumMinSeparationTime += edge->separationTime;
	}

	int getSumMinSeparationTime(int t) {
		if (t < tMinusVec.front() || t > tPlusVec.front()) {
			cerr << "Error! t = " << t << ", [" << tMinusVec.front() << "," << tPlusVec.front() << ")" << endl;
			exit(EXIT_FAILURE);
		}

		for (int i=0; i<edges.size(); i++) {
			Edge* edge = edges[i];
			t = max(t+edge->separationTime,tMinusVec[i+1]);
		}
		return t;
	}

	int getSumMaxSeparationTime(int t) {
		if (t < tMinusVec.front() || t > tPlusVec.front()) {
			cerr << "Error! t = " << t << ", [" << tMinusVec.front() << "," << tPlusVec.front() << ")" << endl;
			exit(EXIT_FAILURE);
		}

		for (int i=0; i<edges.size(); i++) {
			Edge* edge = edges[i];
			t = min(t+edge->maxSeparationTime,tPlusVec[i+1]);
		}
		return t;
	}

	bool checkPathLB(Node* node, int tMinus, int tPlus, int t, int& tMinusStar) {
		tMinusStar = t;
		for (int n=1; n < nodes.size(); n++) {
			int tMinusN = tMinusVec[n];
			int tPlusN = tPlusVec[n];
			int pMin = edges[n-1]->separationTime;

			if (tMinusStar+pMin > tPlusN)
				return false;

			tMinusStar = max(tMinusStar+pMin,tMinusN);
		}

		// Check node
		Node* eNode = nodes.back();
		for (auto edge: eNode->out) {
			if (edge->snk_node == node) {
				int pMin = edge->separationTime;

				if (tMinusStar+pMin > tPlus)
					return false;

				tMinusStar = max(tMinusStar+pMin,tMinus);
			}
		}

		return true;
	}

	bool checkPathUB(Node* node, int tMinus, int tPlus, int t, int& tMinusStar, int& tPlusStar) {
		tMinusStar = t;
		tPlusStar = t;
		for (int n=1; n < nodes.size(); n++) {
			int tMinusN = tMinusVec[n];
			int tPlusN = tPlusVec[n];
			int pMin = edges[n-1]->separationTime;
			int pMax = edges[n-1]->maxSeparationTime;

			if (tPlusStar+pMax < tMinusN)
				return false;

			tMinusStar = max(tMinusStar+pMin,tMinusN);
			tPlusStar = min(tPlusStar+pMax,tPlusN);
		}

		// Check node
		Node* eNode = nodes.back();
		for (auto edge: eNode->out) {
			if (edge->snk_node == node) {
				int pMin = edge->separationTime;
				int pMax = edge->maxSeparationTime;

				if (tPlusStar+pMax < tMinus) 
					return false;

				tMinusStar = max(tMinusStar+pMin,tMinus);
				tPlusStar = min(tPlusStar+pMax,tPlus);
			}
		}

		return true;
	}

	bool update(int tMinus, int tPlus) {
//#ifdef __DEBUG_SCHED_ANALYSIS_DAVR__
#if 0
		if (tMinusVec.size()==2) {
			if (tMinusVec[0] == 13991306 && tMinusVec[1] == 14040393)
				cout << "here" << endl;
		}
#endif

		for (int n=0; n<nodes.size(); n++) {
			if (n==0) {
				tMinusVec[n] = tMinus;
				tPlusVec[n] = tPlus;
			}
			else {
				tMinusVec[n] = max(tMinusVec[n-1] + edges[n-1]->separationTime, tMinusVec[n]);
				tPlusVec[n] = min(tPlusVec[n-1] + edges[n-1]->maxSeparationTime, tPlusVec[n]);
			}

			if (tMinusVec[n] > tPlusVec[n]) return false;
		}

		return true;
	}

	void output(ostream& out) {
		for (int i=0; i<nodes.size(); i++) {
			out << nodes[i]->toString() << " (" << tMinusVec[i] << ", " << tPlusVec[i] << ") " ;
			if (i!=0)
				out << "(pmin,pmax) = (" << edges[i-1]->separationTime << ", " << edges[i-1]->maxSeparationTime << ") ";
			out << endl;
		}
	}
};

#endif