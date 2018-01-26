/* \file NodeAndEdge.h
*  this file decribes the Node and Edge class. 
*  directed graphs consist of a number of nodes and edges
*  \author Chao Peng
*  
*  Changes
*  ------
*  28-Aug-2015 : initial revision (CP)
*
*/

#ifndef NODEANDEDGE_H_
#define NODEANDEDGE_H_

//#include <vld.h>

#include <list>
#include <string>

using namespace std;

class Node;
class Edge;

// ================================================================
// \brief Class Node
// ================================================================
class Node {
public:
	//friend class Edge;
	string name;  
	int index; // identify the node

	//State* state; // used to generate the reachability digraph

	int scale;
	int wcet;
	int deadline;

	// Used for AVR task
	int nextWCET;
	int swichTime;
	double omegaL; // RPmSEC
	double omegaH; // RPmSEC

	double omega;
	bool initialDominant;

	int maxOut;

	list<Edge*> in;
	list<Edge*> out;

	list<Edge*> scc_in; // for strongly connected components
	list<Edge*> scc_out; // for strongly connected components

	// used to construct UDRT or GDRT
	Node** unitNodes;
	int unitNodeNum;
	Node** granNodes;
	int granNodeNum;

	enum Color {WHITE, GRAY, BLACK};

	// used to do deep-first-search
	Color color; // color of node
	int d; // discover time
	int f; // finish time
	Node* pi; // parent node in one tree

	bool visited; // used to detect cycle

public:
	///=============================================================================================================================
	/// Method functions
	///=============================================================================================================================
	Node(Node* pNode);
	Node(std::string _name);
	Node(std::string _name, int _index, int _scale);
	Node(std::string _name, int _scale, int _wcet, int _deadline);
	Node(std::string _name, int _index, int _scale, int _wcet, int _deadline);
	Node(std::string _name, int _index, int _scale, int _wcet, int _deadline, double _omega);
	Node(std::string _name, int _scale);
	Node(const Node& _node);
	~Node();

	void set_wcet(int _wcet);
	void set_deadline(int _deadline);

	void set_color(Color _color);
	Color get_color();

	std::string toString();
};

// ================================================================
// \brief Class Edge
// ================================================================
class Edge {
public:
	Node* src_node;
	Node* snk_node;
	int separationTime; // minimum separation time, musec
	int maxSeparationTime; // maximum separation time, musec
	int deadline;
	double weight; // edge weight, used in the general graph 

	bool visited;

	Edge(Edge* edge);
	Edge(Node* _src_node, Node* _snk_node):src_node(_src_node), snk_node(_snk_node) {}
	~Edge() {}

	void set_separation_time(int t) { separationTime = t; }
	string toString() { return src_node->name+"->"+snk_node->name; }
};

#endif