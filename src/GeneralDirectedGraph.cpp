#include "GeneralDirectedGraph.h"
#include "Utility.h"
#include "GraphAlgorithms.h"

extern double POS_INFINITY;
extern double NEG_INFINITY;

GeneralDirectedGraph::GeneralDirectedGraph() {
	strongly_connected = false;
	util = 0;
}

GeneralDirectedGraph::GeneralDirectedGraph(int n, double** matrix) {
	strongly_connected = false;
	util = 0;

	for ( int i=0; i<n; i++) {
		string name = "g_"+Utility::int_to_string(i);
		Node* node = new Node(name);
		add_node(node);
	}

	for (int i=0; i<n; i++) {
		Node* src = node_vec.at(i);

		for (int j=0; j<n; j++) {
			Node* snk = node_vec.at(j);
			if (matrix[i][j] >= 0 ) {// i to j exists an edge
				Edge* edge = new Edge(src,snk);
				edge->weight = matrix[i][j];
				add_edge(edge);
			}
		}
	}
}

GeneralDirectedGraph::~GeneralDirectedGraph() {
	//cout<<"General Directed Graph Destruction Start"<<endl;
	
	for (vector<GeneralDirectedGraph*>::iterator iter = sccs.begin(); iter != sccs.end(); iter++) {
		(*iter)->node_vec.clear();
		(*iter)->edge_vec.clear();
		delete *iter;
		*iter = NULL;
	}

	sccs.clear();
	vector<GeneralDirectedGraph*>().swap(sccs);

	// release nodes
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	node_vec.clear();
	vector<Node*>().swap(node_vec);

	// release edges
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	edge_vec.clear();
	vector<Edge*>().swap(edge_vec);

	//cout<<"General Directed Graph Destruction End"<<endl;
}

void GeneralDirectedGraph::generate_strongly_connected_components() {
	this->sccs = GraphAlgorithms::generate_strongly_connected_components(this);
	if (sccs.size() == 1) strongly_connected = true;
}

void GeneralDirectedGraph::calculate_untilization() {
	double lambda = 0.0;
	vector<GeneralDirectedGraph*>::iterator iter;
	for (iter = sccs.begin(); iter != sccs.end(); iter++) {
		GraphAlgorithms::calculate_maximum_cycle_mean(*iter);
		lambda = max(lambda,(*iter)->util);
	}
	this->util = lambda;
}

void GeneralDirectedGraph::write_graphviz(ostream& out) {
	out << "digraph G {" <<endl;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		out << node->name << " [label=\"" << node->name << "/" << node->wcet << "\"]" << endl;
	}
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		out << edge->src_node->name << " -> " << edge->snk_node->name 
			<< " [label=\"" << edge->weight << "\"]" << endl;
	}
	out << "}" <<endl;
}