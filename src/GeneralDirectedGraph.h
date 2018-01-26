/* \file GeneralDirectedGraph.h
*  This file decribes the GeneralDirectedGraph class. 
*  The general definition of directed graph.
*  \author Chao Peng
*  
*  Changes
*  ------
*  06-Sept-2015 : initial revision (CP)
*
*/

#ifndef GENERALDIRECTEDGRAPH_H_
#define GENERALDIRECTEDGRAPH_H_

//#include <vld.h>

#include<vector>

#include "NodeAndEdge.h"

/// \brief an implementation of the general directed graph
/// Edge: weight
/// Node: nothing
class GeneralDirectedGraph {
public:
	vector<Node*> node_vec;
	vector<Edge*> edge_vec;

	vector<GeneralDirectedGraph*> sccs;
	bool strongly_connected;

	double util;

	///=============================================================================================================================
	/// method functions
	///=============================================================================================================================
	GeneralDirectedGraph();
	GeneralDirectedGraph(int n, double** matrix);
	~GeneralDirectedGraph();

	void add_node(Node* p_node) { 
		this->node_vec.push_back(p_node); 
	}

	void add_edge(Edge* p_edge) { 
		this->edge_vec.push_back(p_edge); 
		p_edge->src_node->out.push_back(p_edge);
		p_edge->snk_node->in.push_back(p_edge);
	}

	void add_scc_edge(Edge* p_edge) {
		this->edge_vec.push_back(p_edge); 
		p_edge->src_node->scc_out.push_back(p_edge);
		p_edge->snk_node->scc_in.push_back(p_edge);
	}

	void generate_strongly_connected_components();
	void calculate_untilization();
	void write_graphviz(ostream& out);
};

#endif