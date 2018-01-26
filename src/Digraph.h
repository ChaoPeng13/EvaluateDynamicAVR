/* \file Digraph.h
*  this file decribes the digraph class. 
*  directed graphs consist of a number of nodes and edges
*  \author Chao Peng
*  
*  Changes
*  ------
*  21-Aug-2015 : initial revision (CP)
*
*/

#ifndef DIGRAPH_H_
#define DIGRAPH_H_

//#include <vld.h>
#include "Config.h"

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <map>
#include <set>
#include <list>

#ifdef LINUX_PLATFORM
#include <cmath>
#include <climits>
#endif

#include <cmath>
#include <climits>

#include "Utility.h"
#include "Global.h"
#include "NodeAndEdge.h"
//#include "DigraphRequestFunction.h"

#pragma once

extern double POS_INFINITY;
extern double NEG_INFINITY;

using namespace std;

class UnitDigraph;
class GranDigraph;

/// \brief an implementation of the request triple
class RequestTriple {
public:
	int e;
	int r;
	Node* node;

	RequestTriple(int _e, int _r, Node* _node) {
		e = _e;
		r = _r;
		node = _node;
	}

	~RequestTriple() {}
};

/// \brief an implementation of the demand triple
class DemandTriple {
public:
	int e;
	int d;
	Node* node;

	DemandTriple(int _e, int _d, Node* _node) {
		e = _e;
		d = _d;
		node = _node;
	}

	~DemandTriple() {}
};

/// \brief an implementation of the utilization triple
class UtilizationTriple {
public:
	int e;
	int p;
	Node* start;
	Node* end;

	UtilizationTriple(int _e, int _p, Node* _start, Node* _end) {
		e = _e;
		p = _p;
		start = _start;
		end = _end;
	}

	~UtilizationTriple() {}
};


/// \brief an implementation of the real-time task model. 
/// M. stigge et al., the digraph real-time task model, rtas2011
class Digraph {
public:
	int index;
	int scale; // scale the (small) wcet of any node to be an integer
	int switchTime;
	int minVertexC; // in musec
	int minVertexT; // in musec

	// now it is enough to decribe a directed graph by nodes and edges.
	vector<Node*> node_vec;
	vector<Edge*> edge_vec;

	vector<Edge*> cedge_vec; // critical edges

	// identify nodes and edges
	map<Node*, int> node_to_index;
	map<int, Node*> index_to_node;
	map<Edge*, int> edge_to_index;
	map<int, Edge*> index_to_edge;

	int iNode;
	int iEdge;

	int maxOut;

	// gcd
	int pGCD; // greatest common divisor of the minimum separation times
	int aGCD; // greatest common divisor of all the parameters
	vector<Digraph*> sccs; // the vector of all the strongly connected components
	vector<Digraph*> hccs; // the vector of all the highly connected components

	/// we have to explicity define two transformed digraph: UDRT and GDRT.
	UnitDigraph* unit_digraph;
	GranDigraph* gran_digraph;
	
	// linear periodicity parameters
	// i guess these parameters respectively derived from rbf and ibf might be same.
	double linear_factor;

	// linear upper bound parameters
	double c_rbf;
	double c_ibf;
	double c_dbf;
	double c_sum;

	// linear upper bounds
	double tf0; // by c_sum
	double tf1;  // zeng and di natale, using max-plus algebra to improve the analysis of non-cyclic task models, ecrts2013
	double tf2;  // new linear bound

	int tf; // the maximal instance for calculated rbf and ibf to do schedulability analysis

	// properties of the directed graph
	bool strongly_connected;
	int maximum_degree;
	double average_degree;

	map<int, int> rbf_map;
	map<int, int> rbf_map2; // used to record rbf value calculated by Guan's algorithm
	map<int, int> rbf_map2_fast; // used ot record rbf value calculated by the optimized algorithm
	map<int, int> rbf_map2_DP; // used to record rbf vlaue calculated by the dynamic programming
	map<long long int, long long int> rbf_map3_DP;

	map<int, int> ibf_map;
	map<int, int> ibf_map2; // used to record ibf value calculated by Guan's algorithm (without periodicity)
	map<int, int> ibf_map2_fast; // used to recored ibf value calculated by the optimized algorithm
	map<int, int> ibf_map2_DP; // used to record ibf vlaue calculated by the dynamic programming
	map<int, double> ibf_map3; // used to record ibf value calculated by the periodicity property (upper bound on r)
	map<int, int> dbf_map;
	map<int, int> dbf_map2_DP; // used to record dbf vlaue calculated by the dynamic programming
	map<long long int, long long int> ibf_map3_DP;

	///=============================================================================================================================
	/// method functions
	///=============================================================================================================================
	Digraph() {
		index = 0;
		scale = 0;
		unit_digraph = NULL;
		gran_digraph = NULL;

		iNode = 0;
		iEdge = 0;
	}

	Digraph(int _index) {
		index = _index;
		scale = 0;
		unit_digraph = NULL;
		gran_digraph = NULL;

		iNode = 0;
		iEdge = 0;
	}

	Digraph(int _index, int _scale) { //default constructor
		index = _index;
		scale = _scale;
		unit_digraph = NULL;
		gran_digraph = NULL;

		iNode = 0;
		iEdge = 0;
		// todo: we assert that node_vec and edge_vec should be empty here.
	}
	~Digraph();

	void update(vector<Node> _nodes, vector<Edge> _edges) {
		vector<Node*> _nodes_vec;
		vector<Edge*> _edges_vec;

		for (int i=0; i<_nodes.size(); i++) {
			Node* pNode = &_nodes[i];
			_nodes_vec.push_back(pNode);
		}

		for (int i=0; i<_edges.size(); i++) {
			Edge* pEdge = &_edges[i];
			_edges_vec.push_back(pEdge);
		}

		update(_nodes_vec,_edges_vec);
	}

	void update(vector<Node*> _nodes, vector<Edge*> _edges) {
		iNode = _nodes.size()-1;
		iEdge = _edges.size()-1;

		node_vec = _nodes;
		edge_vec = _edges;

		for (int i=0; i<=iNode; i++) {
			node_to_index[node_vec[i]] = i;
			index_to_node[i] = node_vec[i];
		}

		for (int i=0; i<=iEdge; i++) {
			edge_to_index[edge_vec[i]] = i;
			index_to_edge[i] = edge_vec[i];
		}
	}

	void add_node(Node* p_node) { 
		this->node_vec.push_back(p_node); 
		node_to_index[p_node] = iNode; 
		index_to_node[iNode] = p_node;
		iNode++;
	}

	void add_edge(Edge* p_edge) { 
		this->edge_vec.push_back(p_edge); 
		edge_to_index[p_edge] = iEdge;
		index_to_edge[iEdge] = p_edge;
		p_edge->src_node->out.push_back(p_edge);
		p_edge->snk_node->in.push_back(p_edge);
		iEdge++;
	}

	void add_scc_edge(Edge* p_edge) {
		this->edge_vec.push_back(p_edge); 
		edge_to_index[p_edge] = iEdge;
		index_to_edge[iEdge] = p_edge;
		p_edge->src_node->scc_out.push_back(p_edge);
		p_edge->snk_node->scc_in.push_back(p_edge);
		iEdge++;
	}

	void calculate_period_gcd() {
		int _gcd = 0;
		for (vector<Edge*>::iterator iter = this->edge_vec.begin(); iter != this->edge_vec.end(); iter++) {
			Edge* edge = *iter;
			_gcd = Utility::math_gcd(_gcd, edge->separationTime); 
		}
		this->pGCD = _gcd;
	}

	void calculate_all_gcd() {
		// calculate the greatest common divisor of separation times and wcets
		int _gcd = 0;
		for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
			Edge* edge = *iter;
			_gcd = Utility::math_gcd(_gcd, edge->separationTime); 
		}

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			_gcd = Utility::math_gcd(_gcd, node->wcet);
		}

		this->aGCD = _gcd;
	}

	void prepare_digraph();
	/*
	 * \brief set maxOut for the digraph and its nodes
	 */
	void set_maxOut();

	void generate_strongly_connected_components();
	void generate_highly_connected_components();
	void check_strongly_connected();
	void calculate_linear_factor();
	/*
	 * \brief An implementation of the iterative algorithm for calculating utilization
	 * in Stigge et al. The digraph real-time task model. RTAS2011.
	 * In case of the large number of nodes in the unit digraph.
	 */
	void calculate_linear_factor2();

	void calculate_csum();
	void calculate_linear_upper_bounds();

	void scale_wcet(double factor);

	void calculate_tf0(Digraph** digraphs, int i);
	void calculate_tf1(Digraph** digraphs, int i);
	void calculate_tf2(Digraph** digraphs, int i);

	void prepare_rbf_calculation(bool debug);
	/// \brief TODO: collect the runtimes for calculating rbf with or without periodicity
	void prepare_rbf_calculation2(bool debug);

	void prepare_rbf_calculation_without_periodicity(bool debug);

	void prepare_ibf_calculation(bool debug);
	/// \brief collect the runtimes for calculating ibf with or without periodicity 
	void prepare_ibf_calculation2(bool debug);

	/// \brief return the rbf(t)
	double rbf(int t);
	/// \brief return rbf2(t) by Guan's algorithm
	double rbf2(int t);
	double rbf2(int t,map<int,int> rbfs);
	double rbf2(long long int t,map<long long int, long long int> rbfs);
	/// \brief return rbf(t) by the optimized Guan's algorithm
	double rbf2_fast(int);
	/// \brief return the rbf execution request matrix at t
	double** rbf_exec_req_matrix(int t, int& n);

	bool calculate_rbf_with_periodicity(bool debug, map<int,int>& rbfs);
	bool calculate_rbf_with_periodicity(bool debug, bool& hasCalculatedLinearization, int tf, map<int,int>& rbfs, double& tCalLinearization, double& tCalNonLinearization);


	/// \brief iteratively calculate rbf within the time interval t on the digraph, i.e., the algorithm in the RTSS2014 paper
	void calculate_rbf_without_periodicity(int t, map<int,int>& rbfs);
	void calculate_rbf_without_periodicity2(int t, map<int,int>& rbfs);
	/// \brief the optimized Guan's algorithm
	void calculate_rbf_without_periodicity_fast(int t, map<int,int>& rbfs);
	void calculate_rbf_without_periodicity_DP(int t, map<int,int>& rbfs);
	/// \brief calculate rbf with initial engine speed 
	void calculate_rbf_without_periodicity_DP(double initialOmega, int t, map<int,int>& rbfs);
	void calculate_rbf_without_periodicity_DP(int t, map<double, map<int,int>>& rbfs);
	void calculate_rbf_without_periodicity_on_reachability_digraph_DP(int t, map<int,int>& rbfs);
	void calculate_rbf_with_periodicity_DP(int t, map<int,int>& rbfs);
	void calculate_rbf_with_periodicity_DP(int t, double lfac, int lper, map<int,int>& rbfs);
	void calculate_rbf_with_periodicity_DP(int t, double lfac, int lper, map<double, map<int,int>>& rbfs);
	void calculate_rbf_with_periodicity_DP(long long int t, map<long long int, long long int>& rbfs);
	//void calculate_rbf_with_periodicity_on_reachability_digraph_DP(int t, map<int,int>& rbfs);

	/// \brief return the ibf(t)
	double ibf(int t);
	/// \brief return ibf(t) by Guan's algorithm
	double ibf2(int t);
	double ibf2(int t,map<int,int> ibfs);
	double ibf2(long long int t, map<long long int, long long int> ibfs);
	/// \brief return ibf(t) by the optimized Guan's algorithm
	double ibf2_fast(int);
	/// \brief return the ibf execution request matrix at t
	double** ibf_exec_req_matrix(int t, int& n);

	bool calculate_ibf_with_periodicity(bool debug, map<int,double>& ibfs);
	bool calculate_ibf_with_periodicity_by_rbf_linear_defect(int tf_max, map<int,int>& ibfs);
	bool calculate_ibf_with_periodicity(bool debug, bool& hasCalculatedLinearization, int tf, map<int,double>& ibfs, double& tCalLinearization, double& tCalNonLinearization);

	/// \brief iteratively calculate ibf within the time interval t on the digraph, i.e., the algorithm in the RTSS2014 paper
	int calculate_ibf_without_periodicity(int t);
	int calculate_ibf_without_periodicity2(int t);
	void calculate_ibf_without_periodicity(int t, map<int,int>& ibfs);
	void calculate_ibf_without_periodicity2(int t, map<int,int>& ibfs);
	void calculate_ibf_without_periodicity(bool& hasCalculatedNonLinearization, int startTime, int endTime, int stepTime, map<int,int>& ibfs, map<int,double>& calTimes);
	void calculate_ibf_without_periodicity_fast(int t, map<int,int>& ibfs);
	void calculate_ibf_without_periodicity_DP(int t, map<int,int>& ibfs);
	void calculate_ibf_without_periodicity_on_reachability_digraph_DP(int t, map<int,int>& ibfs);
	void calculate_ibf_with_periodicity_DP(int t, map<int,int>& ibfs);
	void calculate_ibf_with_periodicity_DP(long long int t, map<long long int, long long int>& ibfs);
	//void calculate_ibf_with_periodicity_on_reachability_digraph_DP(int t, map<int,int>& ibfs);

	void calculate_ibf_without_periodicity(int t, map<int,double>& ibfs);
	double ibf_without_periodicity(int t);

	/// \brief return the dbf(t)
	double dbf(int t);
	double dbf2(int t, map<int,int> dbfs);
	/// \brief An implementation of the iterative algorithm for calculating dbf
	/// in Stigge et al. The digraph real-time task model. RTAS2011.
	double calculate_dbf(int t);

	void calculate_dbf_without_periodicity_DP(int t, map<int,int>& dbfs);

	/// \brief write a digraph object into an output stream in graphviz dot format
	void write_graphviz(ostream& out);
	void write_graphviz2(ostream& out);
	void write_graphviz_ODRT(ostream& out);
	void write_tikz_ODRT(ostream& out);
};

/// \brief an implmentation of the unit digraph task. 
/// Zeng and Di Natale, Using Max-Plus Algebra to Improve the Analysis of Non-cyclic Task Models, ECRTS2013
class UnitDigraph: public Digraph {
public:
	// point to the original digraph
	Digraph* origin;

	int n_size; // the size of nodes
	int scale;
	int gcd;  // greatest common divisor of the minimum separation times

	set<int> iSet; // set of indices for node->wcet != 0
	map<Node*, int> original_node_to_index; // the index of the original node
	map<int, Node*> index_to_original_node; // the original node at the index

	/// the execution request matrix of the transformed unit digraph task (udrt)
	/// with the greatest common divisor of the minimum separation times 
	/// (or the periods of all the edges) but not 1.
	double** matrix; // two-dimension matrix
	std::map<int, double**> matrix_map; // matrix power
	std::map<int, int> maximum_element_map; // the maximum element of the execution request matrix at the special time

	double lfac;
	int lper;
	int ldef;
	int ubldef;

	///=============================================================================================================================
	/// method functions
	///=============================================================================================================================
	UnitDigraph(Digraph* digraph) {
		origin = digraph;
		scale = digraph->scale;
		lfac = digraph->linear_factor;
		gcd = digraph->pGCD;
		tf = ceil(1.0*digraph->tf/digraph->pGCD)+10;

		unit_digraph = NULL;
		gran_digraph = NULL;

		iNode = 0;
		iEdge = 0;

		// TODO: we assert that node_vec and edge_vec should be empty here.
	}
	~UnitDigraph();

	void add_node(Node* p_node) { 
		this->node_vec.push_back(p_node); 
		node_to_index[p_node] = iNode; 
		index_to_node[iNode] = p_node;
		if (p_node->wcet != 0) iSet.insert(iNode);
		iNode++;
	}

	void add_node(Node* p_node, Node* o_node) { 
		this->node_vec.push_back(p_node); 
		node_to_index[p_node] = iNode; 
		index_to_node[iNode] = p_node;
		if (p_node->wcet != 0) {
			iSet.insert(iNode);
			original_node_to_index[o_node] = iNode;
			index_to_original_node[iNode] = o_node;
		}
		iNode++;
	}

	void add_edge(Edge* p_edge) { 
		this->edge_vec.push_back(p_edge); 
		edge_to_index[p_edge] = iEdge;
		index_to_edge[iEdge] = p_edge;
		iEdge++;
	}

	void prepare(bool debug);
	void prepare2(bool debug);
	void prepare3(bool debug);
	void prepare_without_periodicity(bool debug);
	void generate_unit_digraph();
	void scc_generate_unit_digraph(); // generate unit digraph from a strongly connected component
	void generate_exec_request_matrix();
	void calculate_linear_period(bool debug);
	void calculate_linear_defect();
	void calculate_uppper_bound_linear_defect();

	/// \brief return the rbf execution request matrix at the time t power
	void calculate_exec_request_matrix_power(int tf);
	void calculate_exec_request_matrix_power_without_periodicity(int tf);

	int get_rbf(int t);

	/// \brief calculate the dbf_{i,j}(t) = dbf(v_i,v_j,t) by the l-MAD property assumption
	/// dbf(v_i,v_j,t) = rbf(v_i,vj,t-d(v_j)-\min\limits_{v_k:(v_k,v_j)\in\mathbb{E}} p(v_k,v_j)) + e(v_j)
	int calculate_demand_bound_function(int i, int j, int t);
	int get_dbf(int t);
};

/// \brief an implmentation of the granularity digraph task. 
/// RTNS2015 version
class GranDigraph: public Digraph {
public:
	// point to the original digraph
	Digraph* origin;

	int n_size; // the size of nodes
	int scale;
	int gcd;  // greatest common divisor of the minimum separation times and wcets

	/// the execution request matrix of the transformed granularity digraph task (udrt)
	/// with the greatest common divisor of the minimum separation times 
	/// (or the periods of all the edges) and wcets but not 1.
	double** matrix; // two-dimension matrix
	std::map<int, double**> matrix_map; // matrix power
	std::map<int, int> maximum_element_map; // the maximum element of the execution request matrix at the special time

	double lfac;
	int lper;
	int ldef;
	int ubldef;

	double tCalLinearPeriod;
	double tCalLinearDefect;

	///=============================================================================================================================
	/// method functions
	///=============================================================================================================================
	GranDigraph(Digraph* digraph) {
		origin = digraph;
		scale = digraph->scale;
		lfac = digraph->linear_factor;
		gcd = digraph->aGCD;
		tf = ceil(1.0*digraph->tf/digraph->aGCD);

		unit_digraph = NULL;
		gran_digraph = NULL;

		iNode = 0;
		iEdge = 0;

		// TODO: we assert that node_vec and edge_vec should be empty here.
	}
	~GranDigraph();

	void add_node(Node* p_node) { this->node_vec.push_back(p_node); node_to_index.insert(pair<Node*,int>(p_node,iNode++));}
	void add_edge(Edge* p_edge) { this->edge_vec.push_back(p_edge); edge_to_index.insert(pair<Edge*,int>(p_edge,iEdge++));}

	void prepare(bool debug);
	/// \brief this function is used to collect the runtime for calculating ibf with or without periodicity
	void prepare2(bool debug);

	void generate_gran_digraph();
	void generate_exec_request_matrix();
	void calculate_linear_period(bool debug);
	void calculate_linear_defect();
	void calculate_uppper_bound_linear_defect();

	/// /brief return the ibf execution request matrix at the time t power
	void calculate_exec_request_matrix_power(int tf);
	int get_ibf(int t);
};

#endif