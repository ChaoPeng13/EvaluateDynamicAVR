#include "NodeAndEdge.h"

// ===========================================================================
// Method functions of Node class
// ===========================================================================
Node::Node(Node* pNode) {
	name = pNode->name;
	index = pNode->index;

	scale = pNode->scale;
	wcet = pNode->wcet;
	deadline = pNode->deadline;

	unitNodes = pNode->unitNodes;
	granNodes = pNode->granNodes;

	initialDominant = false;
}	

Node::Node(std::string _name) {
	name = _name;

	unitNodes = NULL;
	granNodes = NULL;

	initialDominant = false;
}

Node::Node(std::string _name, int _index, int _scale) {
	name = _name;
	index = _index;
	scale = _scale;

	unitNodes = NULL;
	granNodes = NULL;

	initialDominant = false;
}

Node::Node(std::string _name, int _scale, int _wcet, int _deadline) {
	name = _name;
	scale = _scale;

	set_wcet(_wcet);
	set_deadline(_deadline);

	unitNodes = NULL;
	granNodes = NULL;

	initialDominant = false;
}

Node::Node(std::string _name, int _index, int _scale, int _wcet, int _deadline) {
	name = _name;
	index = _index;
	scale = _scale;

	set_wcet(_wcet);
	set_deadline(_deadline);

	unitNodes = NULL;
	granNodes = NULL;

	initialDominant = false;
}

Node::Node(std::string _name, int _scale) {
	name = _name;
	scale = _scale;

	unitNodes = NULL;
	granNodes = NULL;

	initialDominant = false;
}

Node::Node(const Node& _node) {
	name = _node.name;
	index = _node.index;
	scale = _node.scale;

	wcet = _node.wcet;
	deadline = _node.deadline;

	in = _node.in;
	out = _node.out;

	unitNodes = _node.unitNodes;
	granNodes = _node.granNodes;

	initialDominant = false;
}

Node::Node(std::string _name, int _index, int _scale, int _wcet, int _deadline, double _omega)
{
	name = _name;
	index = _index;
	scale = _scale;

	set_wcet(_wcet);
	set_deadline(_deadline);

	omega = _omega;

	unitNodes = NULL;
	granNodes = NULL;

	initialDominant = false;
}

Node::~Node() {
	in.clear();
	out.clear();

	scc_in.clear();
	scc_out.clear();

	if (unitNodes != NULL) {
		delete[] unitNodes;
	}

	if (granNodes != NULL) delete[] granNodes;
}


void Node::set_wcet(int _wcet) {
	wcet = _wcet;
}

void Node::set_deadline(int _deadline) {
	deadline = _deadline;
}

std::string Node::toString() {
	return name;
}

void Node::set_color(Color _color) {
	this->color = _color;
}

Node::Color Node::get_color() {
	return color;
}

// ===========================================================================
// Method functions of Edge class
// ===========================================================================
Edge::Edge(Edge* edge) {
	src_node = new Node(edge->src_node);
	snk_node = new Node(edge->snk_node);

	separationTime = edge->separationTime;
	weight = edge->weight;
}