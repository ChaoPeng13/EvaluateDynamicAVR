/* \file Digraph.cpp
*  this file define method functions of digraph, unitdigraph and grandigraph class. 
*  \author Chao Peng
*  
*  changes
*  ------
*  21-aug-2015 : initial revision (CP)
*
*/

#include "Digraph.h"

#include "MaxPlusAlgebra.h"
#include "GraphAlgorithms.h"
#include "Timer.h"

extern double EPSILON;

using namespace std;

// ===========================================================================
// Method functions of Digraph class
// ===========================================================================
Digraph::~Digraph() {
	//cout<<"Digraph Destruction Start"<<endl;
	// release sccs
	for (vector<Digraph*>::iterator iter = sccs.begin(); iter != sccs.end(); iter++) {
		(*iter)->node_vec.clear();
		(*iter)->edge_vec.clear();
		delete *iter;
		*iter = NULL;
	}
	sccs.clear();
	
	// release nodes
	//cout<<node_vec.empty()<<endl;
	if (iNode != 0) {
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			delete node;
			*iter = NULL;
		}
	}
	node_vec.clear();

	// release edges
	if (iEdge!=0) {
		for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
			delete *iter;
			*iter = NULL;
		}
	}
	edge_vec.clear();

	/*
	node_to_index.clear();
	index_to_node.clear();
	edge_to_index.clear();
	index_to_edge.clear();
	*/

	if(unit_digraph != NULL) delete unit_digraph;
	if(gran_digraph != NULL) delete gran_digraph;
}

void Digraph::prepare_digraph() {
	Timer timer;

	timer.start();
	generate_strongly_connected_components();
	check_strongly_connected();
	calculate_period_gcd();
	calculate_all_gcd();
	calculate_linear_factor();
	timer.end();

	/// We close the calculation of linear upper bounds due to the time-consuming
	/// In fact, we use the maximal deadline as t_f
	/*
	timer.start();
	calculate_linear_upper_bounds();
	timer.end();
	SchedulabilityAnalysis::tDigraphCalLinearBounds += timer.getTime();
	*/
}

void Digraph::set_maxOut() {
	this->maxOut = 0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		node->maxOut = 0;

		for (list<Edge*>::iterator eIter = node->out.begin(); eIter != node->out.end(); eIter++) {
			Edge* edge = *eIter;

			node->maxOut = max(node->maxOut, edge->separationTime/this->pGCD-1);
		}
		this->maxOut = max(this->maxOut, node->maxOut);
	}
}

void Digraph::generate_strongly_connected_components() {
	this->sccs = GraphAlgorithms::generate_strongly_connected_components(this);
}

/// /brief generate highly connected components from the SCCS of this digraph
void Digraph::generate_highly_connected_components() {
	vector<Digraph*>::iterator iter;
	for (iter = sccs.begin(); iter != sccs.end(); iter++) {
		Digraph* scc = *iter;
		if (scc->linear_factor == this->linear_factor)
			this->hccs.push_back(scc);
	}
}

void Digraph::check_strongly_connected() {
	if (sccs.size() == 1) this->strongly_connected = true;
	else this->strongly_connected = false;
}

void Digraph::calculate_linear_factor() {
	//this->generate_strongly_connected_components();
	double lambda = 0.0;
	vector<Digraph*>::iterator iter;
	for (iter = sccs.begin(); iter != sccs.end(); iter++) {
		GraphAlgorithms::calculate_maximum_cycle_mean(*iter);
		lambda = max(lambda,(*iter)->linear_factor);
	}
	this->linear_factor = lambda;
}

void Digraph::calculate_linear_factor2() {
	this->linear_factor = GraphAlgorithms::calculate_maximum_cycle_mean2(this);
}

void Digraph::calculate_csum() {
	// calculate C^sum
	GraphAlgorithms::calculate_csum(this);
}

void Digraph::calculate_linear_upper_bounds() {
	// calculate C^rbf, C^ibf and C^dbf
	GraphAlgorithms::calculate_tight_linear_bounds(this);
}

void Digraph::scale_wcet(double factor) {
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int wcet = (int)(factor*node->wcet);
		if (wcet <= 0) wcet = rand()%5+1; // force the wcet not to be 0
		node->wcet = wcet;
	}
}


void Digraph::calculate_tf0(Digraph** digraphs, int i) {
	double util = 0;
	double sum = 0;
	for (int k=0; k<=i; k++) {
		Digraph* digraph = digraphs[k];
		util += digraph->linear_factor;

		sum += digraph->c_sum*2;
		if (k==i) sum -= digraph->c_sum;
	}

	if (util >= 1) tf0 = POS_INFINITY;

	tf0 = sum/(1-util);
}

void Digraph::calculate_tf1(Digraph** digraphs, int i) {
	double util = digraphs[i]->linear_factor;
	double sum = digraphs[i]->c_dbf;
	for (int k=0; k<i; k++) {
		Digraph* digraph = digraphs[k];
		util += digraph->linear_factor;

		sum += digraph->c_rbf;
	}

	if (util >= 1) tf1 = POS_INFINITY;

	tf1 = sum/(1-util);
}

void Digraph::calculate_tf2(Digraph** digraphs, int i) {
	double util = digraphs[i]->linear_factor;
	double sum = digraphs[i]->c_dbf;
	for (int k=0; k<i; k++) {
		Digraph* digraph = digraphs[k];
		util += digraph->linear_factor;

		sum += digraph->c_ibf;
	}

	if (util >= 1) tf2 = POS_INFINITY;

	tf2 = sum/(1-util);
}

void Digraph::prepare_rbf_calculation(bool debug) {
	unit_digraph = new UnitDigraph(this);
	unit_digraph->prepare(debug);
}

void Digraph::prepare_rbf_calculation2(bool debug) {
	unit_digraph = new UnitDigraph(this);
	unit_digraph->prepare2(debug);
}

void Digraph::prepare_rbf_calculation_without_periodicity(bool debug) {
	unit_digraph = new UnitDigraph(this);
	unit_digraph->prepare_without_periodicity(debug);
}

void Digraph::prepare_ibf_calculation(bool debug) {
	gran_digraph = new GranDigraph(this);
	gran_digraph->prepare(debug);
}

void Digraph::prepare_ibf_calculation2(bool debug) {
	gran_digraph = new GranDigraph(this);
	gran_digraph->prepare2(debug);
}

double Digraph::rbf(int t) {
	if (t==0) return 0;
	double rbf = unit_digraph->get_rbf(t);
	double rbf_leaf = 0; // if some nodes have no out edge
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++)
		rbf_leaf = max(rbf_leaf, (double)(*iter)->wcet);

	return max(rbf,rbf_leaf);
}

double Digraph::rbf2(int t) {
	t = ceil(1.0*t/pGCD) * pGCD;

	if (rbf_map2.find(t) == rbf_map2.end()) {
		cerr << "rbf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return rbf_map2[t];
}

double Digraph::rbf2(int t,map<int,int> rbfs) {

	if ( t<=0) return 0;

	map<int,int>::iterator iter = rbfs.lower_bound(t);

	if (iter == rbfs.end()) {
		cerr << "rbf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return iter->second;
}

double Digraph::rbf2(long long int t,map<long long int, long long int> rbfs) {

	map<long long int,long long int>::iterator iter = rbfs.lower_bound(t);

	if (iter == rbfs.end()) {
		cerr << "rbf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return iter->second;
}

double Digraph::rbf2_fast(int t) {

	map<int,int>::iterator iter = rbf_map2_fast.lower_bound(t);

	if (iter == rbf_map2_fast.end()) {
		cerr << "rbf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return iter->second;
}

double** Digraph::rbf_exec_req_matrix(int t, int& n) {
	n = unit_digraph->n_size;
	return unit_digraph->matrix_map[t];
}

bool Digraph::calculate_rbf_with_periodicity(bool debug, map<int,int>& rbfs) {
	// Two parts: 
	// one for calculating the linear parameters
	// another for calculating ibf without periodicity
	prepare_rbf_calculation2(debug);

	if (unit_digraph->ubldef == -1) {
		calculate_rbf_without_periodicity(tf, rbfs);
		return false;
	} else {
		calculate_rbf_without_periodicity(unit_digraph->ubldef+unit_digraph->lper, rbfs);
		int lper = unit_digraph->lper;
		for (int t=unit_digraph->ubldef+unit_digraph->lper+1; t <= tf; t++) {
			rbfs[t] = rbfs[t-lper] + linear_factor*lper;  
		}
		return true;
	}

}

bool Digraph::calculate_rbf_with_periodicity(bool debug, bool& hasCalculatedLinearization, int tf, map<int,int>& rbfs, double& tCalLinearization, double& tCalNonLinearization) {
	Timer timer;
	// Two parts: 
	// one for calculating the linear parameters
	// another for calculating ibf without periodicity
	if (!hasCalculatedLinearization) { // guarantee to calculate only once
		timer.start();
		prepare_rbf_calculation2(debug);
		timer.end();
		tCalLinearization = timer.getTime();
		hasCalculatedLinearization = true;
	}
	
	timer.start();
	if (unit_digraph->ubldef == -1) {
		calculate_rbf_without_periodicity(tf, rbfs);
		timer.end();
		tCalNonLinearization = timer.getTime();
		return false;
	} else {
		calculate_rbf_without_periodicity(unit_digraph->ubldef+unit_digraph->lper, rbfs);
		int lper = unit_digraph->lper;
		for (int t=unit_digraph->ubldef+unit_digraph->lper+1; t <= tf; t++) {
			rbfs[t] = rbfs[t-lper] + linear_factor*lper;  
		}
		timer.end();
		tCalNonLinearization = timer.getTime();
		return true;
	}

}

void Digraph::calculate_rbf_without_periodicity(int t, map<int,int>& rbfs) {
	rbfs[0] = 0;

	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=pGCD; k<=t; k+=pGCD) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}
			}
			/*
			if (!hasInserted) {
				RequestTriple* nrt = new RequestTriple(rt->e,rt->r,rt->node);
				RTk.push_back(nrt);
			}
			*/
		}
		if (!RTk.empty())
			RT.push_back(RTk);
	}

	for (int k=pGCD; k<=t; k+=pGCD) {
		int max_value = 0;
		for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
			vector<RequestTriple*> RTk = *iter;
			for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
				RequestTriple* rt = *it;
				if (rt->r < k) {
					int temp = rt->e;
					max_value = max(max_value,temp);
				}
			}
		}
		rbfs[k] = max_value;
	}

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();
}

void Digraph::calculate_rbf_without_periodicity2(int t, map<int,int>& rbfs) {
	rbfs[0] = 0;

	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=pGCD; k<=t; k+=pGCD) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}
			}
			/*
			if (!hasInserted) {
				RequestTriple* nrt = new RequestTriple(rt->e,rt->r,rt->node);
				RTk.push_back(nrt);
			}
			*/
		}
		if (!RTk.empty())
			RT.push_back(RTk);
		else
			break;
	}

	int max_value = 0;
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			RequestTriple* rt = *it;
			if (rt->r < t) {
				int temp = rt->e;
				max_value = max(max_value,temp);
			}
		}
	}
	rbfs[t] = max_value;

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();
}

void Digraph::calculate_rbf_without_periodicity_fast(int t, map<int,int>& rbfs) {
	rbfs.clear();
	if (!rbfs.empty()) {
		cerr << "This ibfs map is not empty!" << endl;
		exit(EXIT_FAILURE);	
	}

	map<int,int> points; // used to store the points ibf(t-\epsilon) < ibf(t) = ibf(t+\epsilon)
	//points[0] = 0;

	// initial rts
	set<RequestTriple*> rts;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		rts.insert(rt);
	}

	while(!rts.empty()) {
		set<RequestTriple*> temp_rts;

		for (set<RequestTriple*>::iterator iter = rts.begin(); iter != rts.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;

			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				if (r<=t) {
					bool dominated = false;
					for (set<RequestTriple*>::iterator it = temp_rts.begin(); it != temp_rts.end(); it++) {
						RequestTriple* rt2 = *it;
						if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
							dominated = true;
							break;
						}
						if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
							rt2->e = e;
							rt2->r = r;
							dominated = true;
							break;
						}
					}

					if (!dominated) {
						RequestTriple* nrt = new RequestTriple(e,r,snk);
						temp_rts.insert(nrt);
					}
				}
			}
		}
		
		// update points
		for (set<RequestTriple*>::iterator iter = rts.begin(); iter != rts.end(); iter++) {
			RequestTriple* rt = *iter;
			int x = rt->r;
			int y = rt->e;

			/*
			if (points.find(x) == points.end())
				points[x] = y;
			else
				points[x] = max(points[x],y);
			*/
			
			bool dominated = false;
			bool dominating = false;
			set<int> remove;

			for (map<int,int>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
				int x1 = iter1->first;
				int y1 = iter1->second;

				if (y > y1 && x <= x1) {
					dominating = true;
					remove.insert(x1);
				}

				if (y1 >= y && x1 <= x) {
					dominated = true;
				}
			}

			if(dominating) {
				// delete the elements dominated
				for (set<int>::iterator iter1 = remove.begin(); iter1 != remove.end(); iter1++) {
					points.erase(*iter1);
				}
			}

			if (!dominated)
				points[x] = y;
			
		}
		// release rts
		for (set<RequestTriple*>::iterator iter = rts.begin(); iter != rts.end(); iter++)
			delete *iter;
		rts = temp_rts;
	}

	// set rbfs
	for (map<int,int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int y = iter->second;
		map<int,int>::iterator next_Iter = iter;
		next_Iter++;
		int x;
		if (next_Iter == points.end()) {
			x = max(iter->first,t) ;
		}
		else {
			x = next_Iter->first;
		}

		rbfs[x] = y;
	}

	rbfs[0] = 0;
}

void Digraph::calculate_rbf_without_periodicity_DP(int t, map<int,int>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	/*
	double** nodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];
	*/

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
	}

	/*
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}
	*/
	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}

		/*
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		*/
	}

	double currnoderbf = -1;

	for (int i=0; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		double noderbf = 0;
		for (int j=0; j<nSize; j++) {
			noderbf = max(noderbf, preNodeReqBoundFunc[j][i]);
		}

		if (!W_EQ_TOL(currnoderbf,noderbf)) {
			rbfs[release] = noderbf;
			currnoderbf = noderbf;
		}
	}

	for (int i=0; i<nSize; i++) {
		//delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	//delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_rbf_without_periodicity_DP(double initialOmega, int t, map<int,int>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	bool foundInitialOmega = false;

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		if (W_EQ_TOL(initialOmega,node->omega)) {
			preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
			foundInitialOmega = true;
		}
		else
			preNodeReqBoundFunc[nodeIndex][0] = NEG_INFINITY;
	}

	if (!foundInitialOmega) {
		cerr << "Initial engine speed " << initialOmega << " is not the dominant one!" << endl;
		exit(EXIT_FAILURE);
	}

	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = NEG_INFINITY;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}

			if (nodeRbf < 0) 
				preNodeReqBoundFunc[nodeIndex][i] = max(preNodeReqBoundFunc[nodeIndex][i-1],NEG_INFINITY);
			else 
				preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}
	}

	double currnoderbf = -1;

	for (int i=0; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		double noderbf = 0;
		for (int j=0; j<nSize; j++) {
			noderbf = max(noderbf, preNodeReqBoundFunc[j][i]);
		}

		if (!W_EQ_TOL(currnoderbf,noderbf)) {
			rbfs[release] = noderbf;
			currnoderbf = noderbf;
		}
	}

	for (int i=0; i<nSize; i++) {
		delete[] preNodeReqBoundFunc[i];
	}

	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_rbf_without_periodicity_DP(int t, map<double, map<int,int>>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
	}

	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];

			//double nodeRbf = 0.0;
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;

				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}
	}

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];

		double currnoderbf = -1;
		for (int i=0; i<=nPeriod; i++) {
			int release = i*this->pGCD;

			/*
			double noderbf = node->wcet;
			if (i==0) noderbf = preNodeReqBoundFunc[nodeIndex][i] + node->wcet;
			else 
				noderbf = preNodeReqBoundFunc[nodeIndex][i-1] + node->wcet;
				*/

			//double noderbf = preNodeReqBoundFunc[nodeIndex][i] + node->wcet;
			
			double noderbf = preNodeReqBoundFunc[nodeIndex][i];

			if (!W_EQ_TOL(currnoderbf,noderbf)) {
				rbfs[node->omega][release] = noderbf;
				currnoderbf = noderbf;
			}
		}
	}


	for (int i=0; i<nSize; i++) {
		//delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	//delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_rbf_without_periodicity_on_reachability_digraph_DP(int t, map<int,int>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	double firstWeight = 0;
	for(vector<Edge*>::iterator eIter = edge_vec.begin(); eIter != edge_vec.end(); eIter++)
		firstWeight = max(firstWeight,(*eIter)->weight);

	double* reqBoundFunc = new double[nPeriod+1];
	for (int i=1; i<=nPeriod; i++) reqBoundFunc[i] =firstWeight;
	reqBoundFunc[0] = 0;

	for (int i=0; i<nSize; i++) {
		for (int j=0; j<=nPeriod; j++) {
			preNodeReqBoundFunc[i][j] = 0;
		}
	}
	
	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;
		/*
		for (vector<Edge*>::iterator eIter = edge_vec.begin(); eIter != edge_vec.end(); eIter++) {
			Edge* edge = *eIter;

			int srcIndex = node_to_index[edge->src_node];
			int snkIndex = node_to_index[edge->snk_node];

			int nextRelease = (release+edge->separationTime)/this->pGCD;

			if (nextRelease <= nPeriod) {
				double nodeRbf = preNodeReqBoundFunc[srcIndex][i] + edge->weight;
				preNodeReqBoundFunc[snkIndex][nextRelease] = max (preNodeReqBoundFunc[snkIndex][nextRelease], nodeRbf);
			}
		}
		*/
		
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = 0;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				int nSeparationTime = edge->separationTime/this->pGCD;
				int srcIndex = node_to_index[edge->src_node];

				if (srcRelease >= 0 && preNodeReqBoundFunc[srcIndex][srcRelease] >= 0) {
					double nextValue = edge->weight + preNodeReqBoundFunc[srcIndex][srcRelease];
					nodeRbf = max(nodeRbf, nextValue);
					for (int j=0; j<nSeparationTime; j++) {
						reqBoundFunc[i-j] = max(reqBoundFunc[i-j],nextValue);
					}
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}

		// output preNodeReqBoundFunc
		/*
		for (int j=0; j<nSize; j++)
			cout << "preNodeReqBoundFunc[" << j << "][" << i << "]=" << preNodeReqBoundFunc[j][i] << endl;
		*/
	}

	for (int i=0; i<=nPeriod; i++) {
		int release = i*this->pGCD;
		rbfs[release] = reqBoundFunc[i];
		/*
		if (i==0) {
			rbfs[release] = 0;
			continue;
		}

		double noderbf = 0;
		for (int j=0; j<nSize; j++) {
			if (preNodeReqBoundFunc[j][i] < 0) continue;
			noderbf = max(noderbf, preNodeReqBoundFunc[j][i]);
		}

		rbfs[release] = noderbf;
		*/
	}

	for (int i=0; i<nSize; i++) {
		//delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	//delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
	delete[] reqBoundFunc;
}

void Digraph::calculate_rbf_with_periodicity_DP(int t, map<int,int>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	this->set_maxOut();
	
	double** nodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
	}

	int linearPeriodRbf = this->unit_digraph->lper;
	int linearDefectRbf = nPeriod - linearPeriodRbf;
	double linearFactorRbf = this->linear_factor * this->pGCD; // we operate on the UDRT task
	bool periodic = true;
	bool found = false;
	
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}
	
	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}

		
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		
		// check linear periodicity
		periodic = true;
		if (i >= linearPeriodRbf+maxOut) {
			for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
				Node* node = *iter;
				int nodeIndx = node_to_index[node];

				for (int out=0; out<= node->maxOut; out++) {
					double expected = nodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;

					if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out])/expected >= Utility::EPSILON) {
						if(found) {
							cout << "It should be an error in the source code of the RTSJ2015 paper." << endl;
						}

						periodic = false;
						break;
					}
				}

				if (!periodic) break;
			}
			if (periodic) {
				linearDefectRbf = i-linearPeriodRbf;
				found = true;
				//cout << "Start periodic behavor at " << i << endl;
				break;
			}
		}
	}

	//cout  << found << "\tlinearFactorRbf = " << linearFactorRbf << "\tlinearPeriodRbf = " << linearPeriodRbf << "\tlinearDefectRbf = " << linearDefectRbf << endl; 

	for (int i=0; i<=nPeriod; i++) {
		int release = i*this->pGCD;
		if (i==0) {
			rbfs[release] = 0;
			continue;
		}

		double nodeRbf = 0;
		if (i > linearDefectRbf+linearPeriodRbf && periodic) {
			int preRelease = (i-linearPeriodRbf)*this->pGCD;
			nodeRbf = rbfs[preRelease] + linearFactorRbf * linearPeriodRbf; 
		}
		else {
			for (int j=0; j<nSize; j++) {
				nodeRbf = max(nodeRbf, preNodeReqBoundFunc[j][i-1]);
			}
		}

		rbfs[release] = ceil(nodeRbf);
	}

	for (int i=0; i<nSize; i++) {
		delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_rbf_with_periodicity_DP(int t, double lfac, int lper, map<int,int>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	this->set_maxOut();

	//double** nodeReqBoundFunc = new double*[nSize];
	//for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
	}

	int linearPeriodRbf = lper;
	int linearDefectRbf = nPeriod - linearPeriodRbf;
	double linearFactorRbf = lfac * this->pGCD; // we operate on the UDRT task
	bool periodic = true;
	bool found = false;

	/*
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}
	*/

	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		//cout << "release = " << release << "\t linearPeriodRbf+maxOut=" << linearPeriodRbf+maxOut << endl;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];

			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;

				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}

		/*
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		*/

		// check linear periodicity
		periodic = true;
		if (i >= linearPeriodRbf+maxOut) {
			for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
				Node* node = *iter;
				int nodeIndx = node_to_index[node];

				for (int out=0; out<= node->maxOut; out++) {
					//double expected = nodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;
					double expected = preNodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;

					//if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out])/expected >= 0.01 /* Utility::EPSILON */) {
					//if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out]) >= 0.5 /* Utility::EPSILON */) {
					if (abs(expected-preNodeReqBoundFunc[nodeIndx][i-out]) >= 0.5 /* Utility::EPSILON */) {
						//cout << "GAP = " << abs(expected-nodeReqBoundFunc[nodeIndx][i-out])  << endl;

						if(found) {
							cout << "It should be an error in the source code of the RTSJ2015 paper." << endl;
						}

						periodic = false;
						break;
					}
				}

				if (!periodic) break;
			}
			if (periodic) {
				linearDefectRbf = i-linearPeriodRbf;
				found = true;
				//cout << "Start periodic behavior at " << i << endl;
				break;
			}
		}
	}

	cout  << found << "\tlinearFactorRbf = " << linearFactorRbf << "\tlinearPeriodRbf = " << linearPeriodRbf << "\tlinearDefectRbf = " << linearDefectRbf << endl; 

	for (int i=0; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		/*
		if (i==0) {
			rbfs[release] = 0;
			continue;
		}
		*/

		double nodeRbf = 0;
		if (i > linearDefectRbf+linearPeriodRbf && periodic) {
			int preRelease = (i-linearPeriodRbf)*this->pGCD;
			nodeRbf = rbfs[preRelease] + linearFactorRbf * linearPeriodRbf; 
		}
		else {
			for (int j=0; j<nSize; j++) {
				nodeRbf = max(nodeRbf, preNodeReqBoundFunc[j][i]);
			}
		}

		rbfs[release] = ceil(nodeRbf);
	}
	
	map<int,int> temp = rbfs;
	rbfs.clear();

	double currnoderbf = -1;

	for (int i=0; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		double noderbf = temp[release];

		if (!W_EQ_TOL(currnoderbf,noderbf)) {
			rbfs[release] = noderbf;
			currnoderbf = noderbf;
		}
	}

	for (int i=0; i<nSize; i++) {
		//delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	//delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_rbf_with_periodicity_DP(int t, double lfac, int lper, map<double, map<int,int>>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	this->set_maxOut();

	//double** nodeReqBoundFunc = new double*[nSize];
	//for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
	}

	int linearPeriodRbf = lper;
	int linearDefectRbf = nPeriod - linearPeriodRbf;
	double linearFactorRbf = lfac * this->pGCD; // we operate on the UDRT task
	bool periodic = true;
	bool found = false;

	/*
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}
	*/

	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		//cout << "release = " << release << "\t linearPeriodRbf+maxOut=" << linearPeriodRbf+maxOut << endl;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];

			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;

				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}

		/*
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		*/

		// check linear periodicity
		periodic = true;
		if (i >= linearPeriodRbf+maxOut) {
			for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
				Node* node = *iter;
				int nodeIndx = node_to_index[node];

				for (int out=0; out<= node->maxOut; out++) {
					//double expected = nodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;
					double expected = preNodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;

					//if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out])/expected >= 0.01 /* Utility::EPSILON */) {
					//if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out]) >= 0.5 /* Utility::EPSILON */) {
					if (abs(expected-preNodeReqBoundFunc[nodeIndx][i-out]) >= 0.5 /* Utility::EPSILON */) {
						//cout << "GAP = " << abs(expected-nodeReqBoundFunc[nodeIndx][i-out])  << endl;

						if(found) {
							cout << "It should be an error in the source code of the RTSJ2015 paper." << endl;
						}

						periodic = false;
						break;
					}
				}

				if (!periodic) break;
			}
			if (periodic) {
				linearDefectRbf = i-linearPeriodRbf;
				found = true;
				//cout << "Start periodic behavior at " << i << endl;
				break;
			}
		}
	}

	//cout  << found << "\tlinearFactorRbf = " << linearFactorRbf << "\tlinearPeriodRbf = " << linearPeriodRbf << "\tlinearDefectRbf = " << linearDefectRbf << endl; 
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];

		double currnoderbf = -1;
		for (int i=0; i<=nPeriod; i++) {
			int release = i*this->pGCD;

			double noderbf = 0;
			if (i > linearDefectRbf+linearPeriodRbf && periodic) {
				int preRelease = (i-linearPeriodRbf)*this->pGCD;
				noderbf = preNodeReqBoundFunc[nodeIndex][preRelease] + linearFactorRbf * linearPeriodRbf;
				preNodeReqBoundFunc[nodeIndex][i] = noderbf;
			}
			else {
				noderbf = preNodeReqBoundFunc[nodeIndex][i];
			}

			if (!W_EQ_TOL(currnoderbf,noderbf)) {
				rbfs[node->omega][release] = noderbf;
				currnoderbf = noderbf;
			}
		}
	}

	for (int i=0; i<nSize; i++) {
		//delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	//delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_rbf_with_periodicity_DP(long long int t, map<long long int, long long int>& rbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	this->set_maxOut();
	
	double** nodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
	}

	int linearPeriodRbf = this->unit_digraph->lper;
	int linearDefectRbf = nPeriod - linearPeriodRbf;
	double linearFactorRbf = this->linear_factor * this->pGCD; // we operate on the UDRT task
	bool periodic = true;
	bool found = false;
	
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}
	
	for (long long int i=1; i<=nPeriod; i++) {
		long long int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
		}

		
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		
		// check linear periodicity
		periodic = true;
		if (i >= linearPeriodRbf+maxOut) {
			for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
				Node* node = *iter;
				int nodeIndx = node_to_index[node];

				for (int out=0; out<= node->maxOut; out++) {
					double expected = nodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;

					if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out])/expected >= Utility::EPSILON) {
						if(found) {
							cout << "It shoud be an error in the source code of the RTSJ2015 paper." << endl;
						}

						periodic = false;
						break;
					}
				}

				if (!periodic) break;
			}
			if (periodic) {
				linearDefectRbf = i-linearPeriodRbf;
				found = true;
				//cout << "Start periodic behavor at " << i << endl;
				break;
			}
		}
	}

	//cout  << found << "\tlinearFactorRbf = " << linearFactorRbf << "\tlinearPeriodRbf = " << linearPeriodRbf << "\tlinearDefectRbf = " << linearDefectRbf << endl; 

	for (long long int i=0; i<=nPeriod; i++) {
		long long int release = i*this->pGCD;
		if (i==0) {
			rbfs[release] = 0;
			continue;
		}

		double nodeRbf = 0;
		if (i > linearDefectRbf+linearPeriodRbf && periodic) {
			long long int preRelease = (i-linearPeriodRbf)*this->pGCD;
			nodeRbf = rbfs[preRelease] + linearFactorRbf * linearPeriodRbf; 
		}
		else {
			for (int j=0; j<nSize; j++) {
				nodeRbf = max(nodeRbf, preNodeReqBoundFunc[j][i-1]);
			}
		}

		rbfs[release] = ceil(nodeRbf);
	}

	for (int i=0; i<nSize; i++) {
		delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

double Digraph::ibf(int t) {
	if (ibf_map.find(t) != ibf_map.end()) return ibf_map[t];

	if (t==0) return 0;
	double ret = gran_digraph->get_ibf(t);
	ibf_map[t] = ret;
	return ret;
}

double Digraph::ibf2(int t) {
	t = ceil(1.0*t/aGCD) * aGCD;

	if (ibf_map2.find(t) == ibf_map2.end()) {
		cerr << "ibf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return ibf_map2[t];
}

double Digraph::ibf2(int t, map<int,int> ibfs) {

	map<int,int>::iterator iter = ibfs.lower_bound(t);

	if (iter == ibfs.end()) {
		cerr << "ibf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
		//return (--iter)->second;
	}

	return iter->second;
}

double Digraph::ibf2(long long int t,map<long long int, long long int> ibfs) {

	map<long long int,long long int>::iterator iter = ibfs.lower_bound(t);

	if (iter == ibfs.end()) {
		cerr << "ibf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return iter->second;
}

double Digraph::ibf2_fast(int t) {

	map<int,int>::iterator iter = ibf_map2_fast.lower_bound(t);

	if (iter == ibf_map2_fast.end()) {
		cerr << "rbf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
	}

	return iter->second;
}

double** Digraph::ibf_exec_req_matrix(int t, int& n) {
	n = gran_digraph->n_size;
	return gran_digraph->matrix_map[t];
}

bool Digraph::calculate_ibf_with_periodicity(bool debug, map<int,double>& ibfs) {
	// Two parts: 
	// one for calculating the linear parameters
	// another for calculating ibf without periodicity
	prepare_ibf_calculation2(debug);

	if (gran_digraph->ubldef == -1) {
		calculate_ibf_without_periodicity(tf, ibfs);
		return false;
	} else {
		calculate_ibf_without_periodicity(gran_digraph->ubldef+gran_digraph->lper, ibfs);
		int lper = gran_digraph->lper;
		for (int t=gran_digraph->ubldef+gran_digraph->lper+1; t <= tf; t++) {
			ibfs[t] = ibfs[t-lper] + linear_factor*lper;  
		}
		return true;
	}
}

bool Digraph::calculate_ibf_with_periodicity_by_rbf_linear_defect(int tf_max, map<int,int>& ibfs) {
	// Two parts: 
	// one for calculating the linear parameters
	// another for calculating ibf without periodicity
	prepare_rbf_calculation(false);

	if (unit_digraph->ldef == -1) {
		calculate_ibf_without_periodicity_fast(tf_max, ibfs);
		return false;
	} else {
		calculate_ibf_without_periodicity_fast((unit_digraph->ldef+unit_digraph->lper)*pGCD+1, ibfs);
		int lper = unit_digraph->lper*pGCD;
		for (int t=(unit_digraph->ldef+unit_digraph->lper)*pGCD+2; t <= tf_max; t++) {
			map<int,int>::const_iterator mIter = ibfs.lower_bound(t-lper);
			ibfs[t] = mIter->second + linear_factor*lper;  
		}
		return true;
	}
}

bool Digraph::calculate_ibf_with_periodicity(bool debug, bool& hasCalculatedLinearization, int tf_max, map<int,double>& ibfs, double& tCalLinearization, double& tCalNonLinearization) {
	Timer timer;
	// Two parts: 
	// one for calculating the linear parameters
	// another for calculating ibf without periodicity
	if (!hasCalculatedLinearization) { // guarantee to calculate only once
		timer.start();
		prepare_ibf_calculation2(debug);
		timer.end();
		tCalLinearization = timer.getTime();
		hasCalculatedLinearization = true;
	}

	timer.start();
	if (gran_digraph->ubldef == -1) {
		calculate_ibf_without_periodicity(tf_max, ibfs);
		timer.end();
		tCalNonLinearization = timer.getTime();
		return false;
	} else {
		calculate_ibf_without_periodicity(gran_digraph->ubldef+gran_digraph->lper, ibfs);
		int lper = gran_digraph->lper;
		for (int t=gran_digraph->ubldef+gran_digraph->lper+1; t <= tf_max; t++) {
			ibfs[t] = ibfs[t-lper] + linear_factor*lper;  
		}
		timer.end();
		tCalNonLinearization = timer.getTime();
		return true;
	}

}

double Digraph::ibf_without_periodicity(int t) {
	if (ibf_map2.find(t) != ibf_map2.end()) return ibf_map2[t];
	if (t==0) return 0;
	cerr << "The ibfs are not generated for all the time within t." << endl;
	exit(EXIT_FAILURE);
}

int Digraph::calculate_ibf_without_periodicity(int t) {
	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=1; k<=t; k++) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}

				/*
				if (r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
				}
				*/
			}
			/*
			if (!hasInserted) {
				RequestTriple* nrt = new RequestTriple(rt->e,rt->r,rt->node);
				RTk.push_back(nrt);
			}
			*/
		}
		if (!RTk.empty())
			RT.push_back(RTk);
	}

	int max_value = 0;
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			RequestTriple* rt = *it;
			int temp = rt->e - rt->node->wcet + min(rt->node->wcet, t-rt->r);
			max_value = max(max_value,temp);
		}
	}

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();

	return max_value;
}

int Digraph::calculate_ibf_without_periodicity2(int t) {
	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=1; k<=t; k++) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				if (r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}
			}
		}
		if (RTk.empty()) break;
		else RT.push_back(RTk);
	}

	int max_value = 0;
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			RequestTriple* rt = *it;
			int temp = rt->e - rt->node->wcet + min(rt->node->wcet, t-rt->r);
			max_value = max(max_value,temp);
		}
	}

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();

	return max_value;
}

void Digraph::calculate_ibf_without_periodicity(int t, map<int,int>& ibfs) {
	ibfs[0] = 0;

	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=1; k<=t; k++) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}
			}
			/*
			if (!hasInserted) {
				RequestTriple* nrt = new RequestTriple(rt->e,rt->r,rt->node);
				RTk.push_back(nrt);
			}
			*/
		}
		if (!RTk.empty())
			RT.push_back(RTk);
	}

	for (int k=1; k<=t; k++) {
		int max_value = 0;
		for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
			vector<RequestTriple*> RTk = *iter;
			for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
				RequestTriple* rt = *it;
				if (rt->r <= k) {
					int temp = rt->e - rt->node->wcet + min(rt->node->wcet, k-rt->r);
					max_value = max(max_value,temp);
				}
			}
		}
		ibfs[k] = max_value;
	}

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();
}

void Digraph::calculate_ibf_without_periodicity2(int t, map<int,int>& ibfs) {
	ibfs[0] = 0;

	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=1; k<=t; k++) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}
			}
			/*
			if (!hasInserted) {
				RequestTriple* nrt = new RequestTriple(rt->e,rt->r,rt->node);
				RTk.push_back(nrt);
			}
			*/
		}
		if (!RTk.empty())
			RT.push_back(RTk);
		else
			break;
	}

	int max_value = 0;
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			RequestTriple* rt = *it;
			if (rt->r <= t) {
				int temp = rt->e - rt->node->wcet + min(rt->node->wcet, t-rt->r);
				max_value = max(max_value,temp);
			}
		}
	}
	ibfs[t] = max_value;

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();
}

void Digraph::calculate_ibf_without_periodicity(bool& hasCalculatedNonLinearization, int startTime, int endTime, int stepTime, map<int,int>& ibfs, map<int,double>& calTimes) {
	if(hasCalculatedNonLinearization)
		return;
	
	ibfs[0] = 0;

	Timer timer;
	int curTime = startTime;
	double sumCalTime = 0;

	timer.start();
	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=1; k<=endTime; k++) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		vector<RequestTriple*> UpdateRT;

		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			//bool hasInserted = false;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=endTime) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
					//hasInserted = true;
				}
			}
			/*
			if (!hasInserted) {
				RequestTriple* nrt = new RequestTriple(rt->e,rt->r,rt->node);
				RTk.push_back(nrt);
			}
			*/
		}
		if (!RTk.empty())
			RT.push_back(RTk);

		if (k==curTime) {
			timer.end();
			sumCalTime += timer.getTime();
			calTimes[k] = sumCalTime;

			timer.start();
			int max_value = 0;
			for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
				vector<RequestTriple*> temp_RTk = *iter;
				for (vector<RequestTriple*>::iterator it = temp_RTk.begin(); it != temp_RTk.end(); it++) {
					RequestTriple* rt = *it;
					if (rt->r <= curTime) {
						int temp = rt->e - rt->node->wcet + min(rt->node->wcet, curTime-rt->r);
						max_value = max(max_value,temp);
					}
				}
			}
			ibfs[curTime] = max_value;
			timer.end();
			calTimes[k] += timer.getTime();
			

			if(k!=endTime) timer.start();
			curTime += stepTime;
		}
	}

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();

	hasCalculatedNonLinearization = true;
}

void Digraph::calculate_ibf_without_periodicity_fast(int t, map<int,int>& ibfs) {
	ibfs.clear();
	if (!ibfs.empty()) {
		cerr << "This ibfs map is not empty!" << endl;
		exit(EXIT_FAILURE);	
	}

	map<int,int> points; // used to store the points ibf(t-\epsilon) < ibf(t) = ibf(t+\epsilon)
	//points[0] = 0;

	// initial rts
	set<RequestTriple*> rts;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		rts.insert(rt);
	}

	while(!rts.empty()) {
		set<RequestTriple*> temp_rts;

		for (set<RequestTriple*>::iterator iter = rts.begin(); iter != rts.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;

			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				if (r<=t) {
					bool dominated = false;
					for (set<RequestTriple*>::iterator it = temp_rts.begin(); it != temp_rts.end(); it++) {
						RequestTriple* rt2 = *it;
						if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
							dominated = true;
							break;
						}
						if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
							rt2->e = e;
							rt2->r = r;
							dominated = true;
							break;
						}
					}

					if (!dominated) {
						RequestTriple* nrt = new RequestTriple(e,r,snk);
						temp_rts.insert(nrt);
					}
				}
			}
		}
		
		// update points
		for (set<RequestTriple*>::iterator iter = rts.begin(); iter != rts.end(); iter++) {
			RequestTriple* rt = *iter;
			int x = rt->r + rt->node->wcet;
			int y = rt->e;

			bool dominated = false;
			bool dominating = false;
			set<int> remove;

			for (map<int,int>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
				int x1 = iter1->first;
				int y1 = iter1->second;

				if (y > y1 && y-y1 >= x-x1) {
					dominating = true;
					remove.insert(x1);
				}

				if (y1 >= y && y1-y >= x1-x) {
					dominated = true;
				}
			}

			if(dominating) {
				// delete the elements dominated
				for (set<int>::iterator iter1 = remove.begin(); iter1 != remove.end(); iter1++) {
					points.erase(*iter1);
				}
			}

			if (!dominated)
				points[x] = y;
		}
		// release rts
		for (set<RequestTriple*>::iterator iter = rts.begin(); iter != rts.end(); iter++)
			delete *iter;
		rts = temp_rts;
	}

	// set ibfs
	for (map<int,int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int y = iter->second;
		map<int,int>::iterator next_Iter = iter;
		next_Iter++;
		int x;
		if (next_Iter == points.end()) {
			x = max(iter->first,t) ;
		}
		else {
			x = next_Iter->first - (next_Iter->second - iter->second);
		}

		ibfs[x] = y;
	}

	ibfs[0] = 0;
}

void Digraph::calculate_ibf_without_periodicity_DP(int t, map<int,int>& ibfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	/*
	int maxDeadlineDiff = 0;
	for (vector<Edge*>::iterator eIter = edge_vec.begin(); eIter != edge_vec.end(); eIter++) {
		Edge* edge = *eIter;
		maxDeadlineDiff = max(maxDeadlineDiff,edge->snk_node->deadline + edge->separationTime - edge->src_node->deadline);
	}
	maxDeadlineDiff = maxDeadlineDiff / this->pGCD + 1;

	dbfs[0] = 0;
	*/

	map<int,int> points; // used to store the points ibf(t-\epsilon) < ibf(t) = ibf(t+\epsilon)

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	int maxWCET = 0;

	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
		maxWCET = max(maxWCET, node->wcet);
	}

	points[maxWCET] = maxWCET;

	/*
	double** nodeBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeBoundFunc[i] = new double[maxDeadlineDiff]; 
	for (int i=0; i<nSize; i++) {
		nodeBoundFunc[i][0] = 0;
	}
	*/

	for (int i=1; i<=nPeriod; i++) {
		// prepare ibfs
		int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;

			// updates poits
			int x = release+node->wcet;
			int y = nodeRbf;

			bool dominated = false;
			bool dominating = false;
			set<int> remove;

			for (map<int,int>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
				int x1 = iter1->first;
				int y1 = iter1->second;

				//if (x1 < (i-1)*this->pGCD) continue;

				if (y > y1 && y-y1 >= x-x1) {
					dominating = true;
					remove.insert(x1);
				}

				if (y1 >= y && y1-y >= x1-x) {
					dominated = true;
				}
			}

			if(dominating) {
				// delete the elements dominated
				for (set<int>::iterator iter1 = remove.begin(); iter1 != remove.end(); iter1++) {
					points.erase(*iter1);
				}
			}

			if (!dominated)
				points[x] = y;
		}
		/*
		// prepare dbfs
		if (deadlineProperty) {
			int deadline = i*this->pGCD;
			double maxDbf = 0;

			for (vector<Node*>::iterator nIter = node_vec.begin(); nIter != node_vec.end(); nIter++) {
				Node* node = *nIter;
				int nodeIndx = node_to_index[node];

				double dbf = 0.0;
				if (deadline >= node->deadline) dbf = node->wcet;

				double nodeDbf = dbf;
				for(list<Edge*>::iterator eIter = node->in.begin(); eIter != node->in.end(); eIter++) {
					Edge* edge = *eIter;
					Node* src = edge->src_node;
					int srcIndx = node_to_index[src];
					
					int srcDeadline = (deadline - node->deadline - edge->separationTime + src->deadline) / this->pGCD;
					if (srcDeadline >= 0) {
						nodeDbf = max(nodeDbf, dbf + nodeBoundFunc[srcIndx][srcDeadline % maxDeadlineDiff]);
					}
				}
				
				nodeBoundFunc[nodeIndx][i % maxDeadlineDiff] = nodeDbf;
				
				maxDbf = max(maxDbf, nodeDbf);
			}
			
			dbfs[i] = maxDbf;
		}
		*/
	}

	// set ibfs
	for (map<int,int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int y = iter->second;
		map<int,int>::iterator next_Iter = iter;
		next_Iter++;
		int x;
		if (next_Iter == points.end()) {
			x = max(iter->first,nPeriod*this->pGCD) ;
		}
		else {
			x = next_Iter->first - (next_Iter->second - iter->second);
		}

		ibfs[x] = y;
	}

	ibfs[0] = 0;
	
	for (int i=0; i<nSize; i++) {
		delete[] preNodeReqBoundFunc[i];
		//delete[] nodeBoundFunc[i];
	}

	delete[] preNodeReqBoundFunc;
	//delete[] nodeBoundFunc;
}

void Digraph::calculate_ibf_without_periodicity_on_reachability_digraph_DP(int t, map<int,int>& ibfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	map<int,int> points; // used to store the points ibf(t-\epsilon) < ibf(t) = ibf(t+\epsilon)

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];

	for (int i=0; i<nSize; i++) {
		for (int j=0; j<=nPeriod; j++) {
			preNodeReqBoundFunc[i][j] = 0;
		}
	}

	double firstWeight = 0;
	for (vector<Edge*>::iterator eIter = edge_vec.begin(); eIter != edge_vec.end(); eIter++)
		firstWeight = max(firstWeight,(*eIter)->weight);

	points[firstWeight] = firstWeight;
	
	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;
		
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = 0;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				int nSeparationTime = edge->separationTime/this->pGCD;
				int srcIndex = node_to_index[edge->src_node];

				if (srcRelease >= 0 && preNodeReqBoundFunc[srcIndex][srcRelease] >= 0) {
					double nextValue = edge->weight + preNodeReqBoundFunc[srcIndex][srcRelease];
					nodeRbf = max(nodeRbf, nextValue);

				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;
			//if (nodeRbf <= firstWeight) continue;

			double weight = 0;
			for (list<Edge*>::iterator iter2 = node->out.begin(); iter2 != node->out.end(); iter2++) {
				Edge* edge = *iter2;
				weight = max(weight,edge->weight);
			}

			int	x = release+weight;
			int	y = nodeRbf+weight;

			bool dominated = false;
			bool dominating = false;
			set<int> remove;

			for (map<int,int>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
			//for (map<int,int>::iterator iter1 = points.upper_bound(release); iter1 != points.end(); iter1++) {
				int x1 = iter1->first;
				int y1 = iter1->second;

				//if (x1 < (i-1)*this->pGCD) continue;

				if (y > y1 && y-y1 >= x-x1) {
					dominating = true;
					remove.insert(x1);
				}

				if (y1 >= y && y1-y >= x1-x) {
					dominated = true;
				}
			}

			if(dominating) {
				// delete the elements dominated
				for (set<int>::iterator iter1 = remove.begin(); iter1 != remove.end(); iter1++) {
					points.erase(*iter1);
				}
			}

			if (!dominated)
				points[x] = y;
		}
	}

	// set ibfs
	for (map<int,int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int y = iter->second;
		map<int,int>::iterator next_Iter = iter;
		next_Iter++;
		int x;
		if (next_Iter == points.end()) {
			x = max(iter->first,nPeriod*this->pGCD) ;
		}
		else {
			x = next_Iter->first - (next_Iter->second - iter->second);
		}

		ibfs[x] = y;
	}

	ibfs[0] = 0;

	for (int i=0; i<nSize; i++) {
		delete[] preNodeReqBoundFunc[i];
	}

	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_ibf_with_periodicity_DP(int t, map<int,int>& ibfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	map<int, int> points; // used to store the points ibf(t-\epsilon) < ibf(t) = ibf(t+\epsilon)

	this->set_maxOut();
	
	double** nodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];


	int maxWCET = 0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
		maxWCET = max(maxWCET, node->wcet);
	}
	points[maxWCET] = maxWCET;
	
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}

	int linearPeriodRbf = this->unit_digraph->lper;
	int linearDefectRbf = nPeriod - linearPeriodRbf;
	double linearFactorRbf = this->linear_factor * this->pGCD; // we operate on the UDRT task
	bool periodic = true;
	bool found = false;

	int temp0 = nPeriod;
	
	for (int i=1; i<=nPeriod; i++) {
		int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;

			// updates poits
			int x = release+node->wcet;
			int y = nodeRbf;

			bool dominated = false;
			bool dominating = false;
			set<int> remove;

			for (map<int, int>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
				int x1 = iter1->first;
				int y1 = iter1->second;

				//if (x1 < (i-1)*this->pGCD) continue;

				if (y > y1 && y-y1 >= x-x1) {
					dominating = true;
					remove.insert(x1);
				}

				if (y1 >= y && y1-y >= x1-x) {
					dominated = true;
				}
			}

			if(dominating) {
				// delete the elements dominated
				for (set<int>::iterator iter1 = remove.begin(); iter1 != remove.end(); iter1++) {
					points.erase(*iter1);
				}
			}

			if (!dominated)
				points[x] = y;
		}

		
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		
		// check linear periodicity
		periodic = true;
		if (i >= linearPeriodRbf+maxOut) {
			for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
				Node* node = *iter;
				int nodeIndx = node_to_index[node];

				for (int out=0; out<= node->maxOut; out++) {
					double expected = nodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;

					if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out])/expected >= Utility::EPSILON) {
						if(found) {
							cout << "It shoud be an error in the source code of the RTSJ2015 paper." << endl;
						}

						periodic = false;
						break;
					}
				}

				if (!periodic) break;
			}
			if (periodic) {
				linearDefectRbf = i-linearPeriodRbf;
				found = true;
				temp0 = i;
				//cout << "Start periodic behavor at " << i << endl;
				break;
			}
		}
	}

	//cout  << found << "\tlinearFactorRbf = " << linearFactorRbf << "\tlinearPeriodRbf = " << linearPeriodRbf << "\tlinearDefectRbf = " << linearDefectRbf << endl; 

	// set ibf by the periodicity
	map<int, int> temp = points;
	if (found) {
		this->unit_digraph->ldef = linearDefectRbf;
		for (map<int, int>::iterator mIter = temp.begin(); mIter != temp.end(); mIter++) {
			if (mIter->first > linearDefectRbf*this->pGCD+1) {
				int n = 1;
				while (mIter->first + n*linearPeriodRbf * this->pGCD <= nPeriod * this->pGCD) {
					points[mIter->first + n*linearPeriodRbf * this->pGCD ] = mIter->second + ceil(n * linearFactorRbf * linearPeriodRbf);
					n++;
				}
			}
		}
	}

	// set ibfs
	for (map< int, int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int y = iter->second;
		map<int, int>::iterator next_Iter = iter;
		next_Iter++;
		int x;
		if (next_Iter == points.end()) {
			x = max(iter->first,nPeriod*this->pGCD);
		}
		else {
			x = next_Iter->first - (next_Iter->second - iter->second);
		}

		ibfs[x] = y;
	}

	ibfs[0] = 0;

	for (int i=0; i<nSize; i++) {
		delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_ibf_with_periodicity_DP(long long int t, map<long long int, long long int>& ibfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	map<long long int, long long int> points; // used to store the points ibf(t-\epsilon) < ibf(t) = ibf(t+\epsilon)

	this->set_maxOut();
	
	double** nodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeReqBoundFunc[i] = new double[nPeriod+1];

	double** preNodeReqBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) preNodeReqBoundFunc[i] = new double[nPeriod+1];


	int maxWCET = 0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		preNodeReqBoundFunc[nodeIndex][0] = node->wcet;
		maxWCET = max(maxWCET, node->wcet);
	}
	points[maxWCET] = maxWCET;
	
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		int nodeIndex = node_to_index[node];
		nodeReqBoundFunc[nodeIndex][0] = 0;

		for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
			Edge* edge = *iter2;
			int srcIndex = node_to_index[edge->src_node];
			nodeReqBoundFunc[nodeIndex][0] = max(nodeReqBoundFunc[nodeIndex][0], preNodeReqBoundFunc[srcIndex][0]);
		}
	}

	int linearPeriodRbf = this->unit_digraph->lper;
	int linearDefectRbf = nPeriod - linearPeriodRbf;
	double linearFactorRbf = this->linear_factor * this->pGCD; // we operate on the UDRT task
	bool periodic = true;
	bool found = false;

	int temp0 = nPeriod;
	
	for (long long int i=1; i<=nPeriod; i++) {
		long long int release = i*this->pGCD;

		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			
			double nodeRbf = node->wcet;
			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				
				long long int srcRelease = (release-edge->separationTime)/this->pGCD;
				if (srcRelease >= 0) {
					int srcIndex = node_to_index[edge->src_node];
					nodeRbf = max(nodeRbf, node->wcet + preNodeReqBoundFunc[srcIndex][srcRelease]);
				}
			}
			preNodeReqBoundFunc[nodeIndex][i] = nodeRbf;

			// updates poits
			long long int x = release+node->wcet;
			long long int y = nodeRbf;

			bool dominated = false;
			bool dominating = false;
			set<long long int> remove;

			for (map<long long int, long long int>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
				long long int x1 = iter1->first;
				long long int y1 = iter1->second;

				//if (x1 < (i-1)*this->pGCD) continue;

				if (y > y1 && y-y1 >= x-x1) {
					dominating = true;
					remove.insert(x1);
				}

				if (y1 >= y && y1-y >= x1-x) {
					dominated = true;
				}
			}

			if(dominating) {
				// delete the elements dominated
				for (set<long long int>::iterator iter1 = remove.begin(); iter1 != remove.end(); iter1++) {
					points.erase(*iter1);
				}
			}

			if (!dominated)
				points[x] = y;
		}

		
		for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
			Node* node = *iter;
			int nodeIndex = node_to_index[node];
			nodeReqBoundFunc[nodeIndex][i] = 0;

			for (list<Edge*>::iterator iter2 = node->in.begin(); iter2 != node->in.end(); iter2++) {
				Edge* edge = *iter2;
				int srcIndex = node_to_index[edge->src_node];
				nodeReqBoundFunc[nodeIndex][i] = max(nodeReqBoundFunc[nodeIndex][i], preNodeReqBoundFunc[srcIndex][i]);
			}
		}
		
		// check linear periodicity
		periodic = true;
		if (i >= linearPeriodRbf+maxOut) {
			for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
				Node* node = *iter;
				int nodeIndx = node_to_index[node];

				for (int out=0; out<= node->maxOut; out++) {
					double expected = nodeReqBoundFunc[nodeIndx][i-out-linearPeriodRbf] + linearFactorRbf * linearPeriodRbf;

					if (abs(expected-nodeReqBoundFunc[nodeIndx][i-out])/expected >= Utility::EPSILON) {
						if(found) {
							cout << "It shoud be an error in the source code of the RTSJ2015 paper." << endl;
						}

						periodic = false;
						break;
					}
				}

				if (!periodic) break;
			}
			if (periodic) {
				linearDefectRbf = i-linearPeriodRbf;
				found = true;
				temp0 = i;
				//cout << "Start periodic behavor at " << i << endl;
				break;
			}
		}
	}

	//cout  << found << "\tlinearFactorRbf = " << linearFactorRbf << "\tlinearPeriodRbf = " << linearPeriodRbf << "\tlinearDefectRbf = " << linearDefectRbf << endl; 

	// set ibf by the periodicity
	map<long long int, long long int> temp = points;
	if (found) {
		this->unit_digraph->ldef = linearDefectRbf;
		for (map<long long int, long long int>::iterator mIter = temp.begin(); mIter != temp.end(); mIter++) {
			if (mIter->first > (long long int) linearDefectRbf*this->pGCD+1) {
				long long int n = 1;
				while (mIter->first + n*linearPeriodRbf * this->pGCD <= (long long int) nPeriod * this->pGCD) {
					points[mIter->first + n*linearPeriodRbf * this->pGCD ] = mIter->second + ceil(n * linearFactorRbf * linearPeriodRbf);
					n++;
				}
			}
		}
	}

	// set ibfs
	for (map< long long int, long long int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		long long int y = iter->second;
		map<long long int, long long int>::iterator next_Iter = iter;
		next_Iter++;
		long long int x;
		if (next_Iter == points.end()) {
			x = max(iter->first,(long long int) nPeriod*this->pGCD);
		}
		else {
			x = next_Iter->first - (next_Iter->second - iter->second);
		}

		ibfs[x] = y;
	}

	ibfs[0] = 0;

	for (int i=0; i<nSize; i++) {
		delete[] nodeReqBoundFunc[i];
		delete[] preNodeReqBoundFunc[i];
	}

	delete[] nodeReqBoundFunc;
	delete[] preNodeReqBoundFunc;
}

void Digraph::calculate_ibf_without_periodicity(int t, map<int,double>& ibfs) {
	ibfs[0] = 0;

	vector<vector<RequestTriple*>> RT;
	// initial RT0
	vector<RequestTriple*> RT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		RequestTriple* rt = new RequestTriple(node->wcet, 0, node);
		RT0.push_back(rt);
	}
	RT.push_back(RT0);

	for (int k=1; k<=t; k++) {
		vector<RequestTriple*> RTk;
		vector<RequestTriple*> PrevRT = RT.back();
		for (vector<RequestTriple*>::iterator iter = PrevRT.begin(); iter != PrevRT.end(); iter++) {
			RequestTriple* rt = *iter;
			Node* src = rt->node;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				int e = rt->e + snk->wcet;
				int r = rt->r + edge->separationTime;

				bool dominated = false;
				for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
					RequestTriple* rt2 = *it;
					if (rt2->e >= e && rt2->r <= r && snk == rt2->node) {
						dominated = true;
						break;
					}
					if (rt2->e <= e && rt2->r >= r && snk == rt2->node) {
						rt2->e = e;
						rt2->r = r;
						dominated = true;
						break;
					}
				}

				if (!dominated && r<=t) {
					RequestTriple* nrt = new RequestTriple(e,r,snk);
					RTk.push_back(nrt);
				}
			}
		}
		if (!RTk.empty())
			RT.push_back(RTk);
	}

	for (int k=1; k<=t; k++) {
		int max_value = 0;
		for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
			vector<RequestTriple*> RTk = *iter;
			for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
				RequestTriple* rt = *it;
				if (rt->r <= k) {
					int temp = rt->e - rt->node->wcet + min(rt->node->wcet, k-rt->r);
					max_value = max(max_value,temp);
				}
			}
		}
		ibfs[k] = max_value;
	}

	// delete RequestTriple
	for (vector<vector<RequestTriple*>>::iterator iter = RT.begin(); iter != RT.end(); iter++) {
		vector<RequestTriple*> RTk = *iter;
		for (vector<RequestTriple*>::iterator it = RTk.begin(); it != RTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		RTk.clear();
	}
	RT.clear();
}

double Digraph::dbf(int t) {
	int large_t = 100; // sufficiently large t
	double factor = (double) t/pGCD;
	t = floor(factor);
	//write_graphviz(cout);
	if (dbf_map.find(t) != dbf_map.end())
		return dbf_map[t];
	if (t < large_t)
		return calculate_dbf(t);
	else 
		return unit_digraph->get_dbf(t);
}

double Digraph::dbf2(int t, map<int,int> dbfs) {

	map<int,int>::iterator iter = dbfs.lower_bound(t);

	if (iter == dbfs.end()) {
		cerr << "dbf(" << t << ") has not been prepared!" << endl;
		exit(EXIT_FAILURE);
		//return (--iter)->second;
	}

	return iter->second;
}

double Digraph::calculate_dbf(int t) {
	t = t/this->pGCD;
	vector<vector<DemandTriple*>> DT;
	// initial DT0
	vector<DemandTriple*> DT0;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		if (node->deadline == INT_MAX) continue;
		DemandTriple* dt = new DemandTriple(node->wcet, node->deadline, node);
		DT0.push_back(dt);
	}
	DT.push_back(DT0);

	for (int k=1; k<=t; k++) {
		vector<DemandTriple*> DTk;
		vector<DemandTriple*> PrevDT = DT.back();
		for (vector<DemandTriple*>::iterator iter = PrevDT.begin(); iter != PrevDT.end(); iter++) {
			DemandTriple* dt = *iter;
			Node* src = dt->node;
			for (list<Edge*>::iterator eIter = src->out.begin(); eIter != src->out.end(); eIter++) {
				Edge* edge = *eIter;
				Node* snk = edge->snk_node;
				if (snk->deadline == INT_MAX) continue;
				int e = dt->e + snk->wcet;
				int d = dt->d - src->deadline + edge->separationTime + snk->deadline;

				bool dominated = false;
				for (vector<DemandTriple*>::iterator it = DTk.begin(); it != DTk.end(); it++) {
					DemandTriple* dt2 = *it;
					if (dt2->e >= e && dt2->d <= d && snk == dt2->node) {
						dominated = true;
						break;
					}
					if (dt2->e <= e && dt2->d >= d && snk == dt2->node) {
						dt2->e = e;
						dt2->d = d;
						dominated = true;
						break;
					}
				}

				if (!dominated && d<=t*pGCD) {
					DemandTriple* ndt = new DemandTriple(e,d,snk);
					DTk.push_back(ndt);
				}
			}
		}
		if (!DTk.empty())
			DT.push_back(DTk);
	}

	//DT.erase(DT.begin()); // remove thd DT0
	int max_value = 0;
	for (vector<vector<DemandTriple*>>::iterator iter = DT.begin(); iter != DT.end(); iter++) {
		vector<DemandTriple*> DTk = *iter;
		for (vector<DemandTriple*>::iterator it = DTk.begin(); it != DTk.end(); it++) {
			DemandTriple* dt = *it;
			if (dt->d <= t*pGCD)
				max_value = max(max_value,dt->e);
		}
	}
	dbf_map[t] = max_value;

	// delete DemandTriple
	for (vector<vector<DemandTriple*>>::iterator iter = DT.begin(); iter != DT.end(); iter++) {
		vector<DemandTriple*> DTk = *iter;
		for (vector<DemandTriple*>::iterator it = DTk.begin(); it != DTk.end(); it++) {
			delete *it;
			*it = NULL;
		}
		DTk.clear();
	}
	DT.clear();

	return max_value;
}

void Digraph::calculate_dbf_without_periodicity_DP(int t, map<int,int>& dbfs) {
	int nSize = node_vec.size();
	int nPeriod = ceil(1.0*t/this->pGCD);

	int maxDeadlineDiff = 0;
	for (vector<Edge*>::iterator eIter = edge_vec.begin(); eIter != edge_vec.end(); eIter++) {
		Edge* edge = *eIter;
		maxDeadlineDiff = max(maxDeadlineDiff,edge->snk_node->deadline + edge->separationTime - edge->src_node->deadline);
	}
	maxDeadlineDiff = maxDeadlineDiff / this->pGCD + 1;

	dbfs[0] = 0;

	double** nodeBoundFunc = new double*[nSize];
	for (int i=0; i<nSize; i++) nodeBoundFunc[i] = new double[maxDeadlineDiff]; 
	for (int i=0; i<nSize; i++) {
		nodeBoundFunc[i][0] = 0;
	}

	for (int i=1; i<=nPeriod; i++) {
		// prepare dbfs
		int deadline = i*this->pGCD;
		double maxDbf = 0;

		for (vector<Node*>::iterator nIter = node_vec.begin(); nIter != node_vec.end(); nIter++) {
			Node* node = *nIter;
			int nodeIndx = node_to_index[node];

			double dbf = 0.0;
			if (deadline >= node->deadline) dbf = node->wcet;

			double nodeDbf = dbf;
			for(list<Edge*>::iterator eIter = node->in.begin(); eIter != node->in.end(); eIter++) {
				Edge* edge = *eIter;
				Node* src = edge->src_node;
				int srcIndx = node_to_index[src];

				int srcDeadline = (deadline - node->deadline - edge->separationTime + src->deadline) / this->pGCD;
				if (srcDeadline >= 0) {
					nodeDbf = max(nodeDbf, dbf + nodeBoundFunc[srcIndx][srcDeadline % maxDeadlineDiff]);
				}
			}

			nodeBoundFunc[nodeIndx][i % maxDeadlineDiff] = nodeDbf;

			maxDbf = max(maxDbf, nodeDbf);
		}

		dbfs[deadline] = maxDbf;
		
	}
	
	for (int i=0; i<nSize; i++) {
		delete[] nodeBoundFunc[i];
	}

	delete[] nodeBoundFunc;
}

void Digraph::write_graphviz(ostream& out) {
	out << "digraph G {" <<endl;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		out << node->name << " [label=\" " << node->name << " / " << node->wcet << " / " << node->deadline << " \"]" << endl;
	}
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		out << edge->src_node->name << " -> " << edge->snk_node->name 
			<< " [label=\" " << edge->separationTime << " \"]" << endl;
	}
	out << "}" <<endl;
}

void Digraph::write_graphviz2(ostream& out) {
	out << "digraph G {" <<endl;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		out << node->name << " [label=\" " << node->name << " \"]" << endl;
	}
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		out << edge->src_node->name << " -> " << edge->snk_node->name 
			<< " [label=\" " << edge->weight << " / " << edge->separationTime << " \"]" << endl;
	}
	out << "}" <<endl;
}

void Digraph::write_graphviz_ODRT(ostream& out) {
	out << "digraph G {" <<endl;
	out << "size=\"8,5\"" << endl;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		//out << "v" << node->index + 1 << " [label=\" (" << RPmSEC_to_RPM(node->omegaL) << "," << RPmSEC_to_RPM(node->omegaH) << "] \"]" << endl;
		out << "v" << node->index + 1 << " [label=\" " << "v" << node->index + 1 << " \"]" << endl;
	}

	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		out << "v" << edge->src_node->index + 1 << " -> " << "v" << edge->snk_node->index + 1
			<< " [label=\" <" << edge->separationTime << "," << edge->maxSeparationTime << "> \"]" << endl;
	}
	out << "}" <<endl;
}

void Digraph::write_tikz_ODRT(ostream& out) {
	out << "\\begin{tikzpicture}[->,>=stealth',shorten >=1pt,auto,node distance=2cm,semithick]" <<endl;
	out << "\t\\tikzstyle{vertex}=[circle,fill=blue!25,draw=none,text=black]" << endl;
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		Node* node = *iter;
		if (node->index == 0)
			out << "\t\\node[vertex] (v" << node->index+1 << ")\t\t{$v_" << node->index+1 << "$};" << endl;
		else
			out << "\t\\node[vertex] (v" << node->index+1 << ")\t[right of=v" << node->index << "]\t{$v_" << node->index + 1 << "$};" << endl;
	}

	// self-loop
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		Node* node = edge->src_node;
		int lIndex = node->index+1;
		if (edge->src_node != edge->snk_node) continue;
		
		out << "\t\\path \t"
			<< "(v" <<lIndex << ") edge [loop above] node {$\\langle " << Utility::round(muSEC_to_mSEC(edge->separationTime))
			<< "," << Utility::round(muSEC_to_mSEC(edge->maxSeparationTime)) << " \\rangle$}  (v" << lIndex << ");" << endl;
	}

#if 0
	// double-direction arrows
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		Node* node = edge->src_node;
		int lIndex = node->index+1;
		if (edge->src_node->index != edge->snk_node->index-1) continue;
		out << "\t\\path[<->] \t"
			<< "(v" <<lIndex << ") edge              node {$\\langle " << Utility::round(muSEC_to_mSEC(edge->separationTime))
			<< "," << Utility::round(muSEC_to_mSEC(edge->maxSeparationTime)) << " \\rangle$}  (v" << lIndex + 1 << ");" << endl;
	}
#else
	// single-direction arrows
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		Edge* edge = *iter;
		Node* src_node = edge->src_node;
		Node* snk_node = edge->snk_node;
		if (edge->src_node == edge->snk_node) continue;

		if (src_node->index < snk_node->index) {
			out << "\t\\path[->] \t"
				<< "(v" <<src_node->index + 1 << ") edge [bend left] node {$\\langle " << Utility::round(muSEC_to_mSEC(edge->separationTime))
				<< "," << Utility::round(muSEC_to_mSEC(edge->maxSeparationTime)) << " \\rangle$}  (v" << snk_node->index + 1 << ");" << endl;
		}
		else {
			out << "\t\\path[->] \t"
				<< "(v" <<src_node->index + 1 << ") edge [bend left] node {$\\langle " << Utility::round(muSEC_to_mSEC(edge->maxSeparationTime))
				<< "," << Utility::round(muSEC_to_mSEC(edge->separationTime)) << " \\rangle$}  (v" << snk_node->index + 1 << ");" << endl;
		}
		
	}
#endif

	out << "\\end{tikzpicture}" <<endl;
}

// ===========================================================================
// Method functions of UnitDigraph class
// ===========================================================================
UnitDigraph::~UnitDigraph() {
	//cout<<"UnitDigraph Destruction Start"<<endl;

	// release nodes
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	node_vec.clear();

	// release edges
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	edge_vec.clear();

	node_to_index.clear();
	index_to_node.clear();
	edge_to_index.clear();
	index_to_edge.clear();

	// release sccs
	for (vector<Digraph*>::iterator iter = sccs.begin(); iter != sccs.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	sccs.clear();

	if (unit_digraph != NULL) delete unit_digraph;
	if (gran_digraph != NULL) delete gran_digraph;

	iSet.clear();
	original_node_to_index.clear();
	index_to_original_node.clear();

	// release matrix
	/*
	for (int i=0; i<n_size; i++) {
		delete[] matrix[i];
	}
	delete[] matrix;
	*/

	// delete execution request matrix
	for (map<int, double**>::iterator iter = matrix_map.begin(); iter != matrix_map.end(); iter++) {
		double** temp = iter->second;
		for (int i=0; i<n_size; i++)
			delete[] temp[i];
		delete[] temp;
	}

	matrix_map.clear();
	maximum_element_map.clear();

	//cout<<"UnitDigraph End"<<endl;
}

/// \brief prepare function
void UnitDigraph::prepare(bool debug) {
	// calculate gcd
	//calculate_period_gcd();
	//gcd = pGCD;
	// generate UDRT
	generate_unit_digraph();
	n_size = node_vec.size();

	// generate execution request matrix
	generate_exec_request_matrix();

	// Calculate linear period
	cout << "Calculating the linear period of rbf ..." << endl;
	calculate_linear_period(debug);
	//lper = 979;

	// Calculate linear defect
	cout << "Calculating the linear defect of rbf ..." << endl;
	calculate_linear_defect();

	// Calculate the execution request matrix power
	//calculate_exec_request_matrix_power(tf);
}

/// \brief prepare function
void UnitDigraph::prepare2(bool debug) {
	// generate UDRT
	generate_unit_digraph();
	n_size = node_vec.size();

	// generate execution request matrix
	generate_exec_request_matrix();

	// Calculate linear period
	cout << "calculate the linear period of ibf ..." << endl;
	calculate_linear_period(debug);

	// Calculate linear defect
	//cout << "calculate the linear defect of ibf ..." << endl;
	//calculate_linear_defect();

	// calculate upper bound of the linear defect
	cout << "calculating the upper bound of linear defect of ibf ..." << endl;
	calculate_uppper_bound_linear_defect();
}

/// \brief prepare function
void UnitDigraph::prepare3(bool debug) {
	// generate UDRT
	generate_unit_digraph();
	n_size = node_vec.size();

	// generate execution request matrix
	generate_exec_request_matrix();

	// Calculate linear period
	//cout << "calculate the linear period of ibf ..." << endl;
	calculate_linear_period(debug);
}

void UnitDigraph::prepare_without_periodicity(bool debug) {
	generate_unit_digraph();
	n_size = node_vec.size();

	// generate execution request matrix
	generate_exec_request_matrix();

	// Calculate the execution request matrix power
	calculate_exec_request_matrix_power_without_periodicity(tf);
}

void UnitDigraph::generate_unit_digraph() {
	for (vector<Node*>::iterator iter = origin->node_vec.begin(); iter != origin->node_vec.end(); iter++) {
		Node* node = *iter;

		int maxSeparationtime = this->gcd;
		for (list<Edge*>::iterator iter = node->out.begin(); iter != node->out.end(); iter++) {
			Edge* edge = *iter;
			maxSeparationtime = max(maxSeparationtime, edge->separationTime);
		}

		int numNode = maxSeparationtime/this->gcd;
		node->unitNodes = new Node*[numNode];
		node->unitNodeNum = numNode;

		// Create new nodes
		for (int j=0; j<numNode; j++) {
			string name = "u_"+node->name+"_"+Utility::int_to_string(j);
			int deadline = 1; // not important
			int wcet = 0;
			if (j==0) {
				wcet = node->wcet;
				deadline = node->deadline; // the demand bound functions of the original digraph and UDRT are same
			}

			Node* newNode = new Node(name,node->scale,wcet,deadline);

			this->add_node(newNode, node);

			node->unitNodes[j] = newNode;
		}

		// Create new edges
		for (int j=1; j<numNode; j++) {
			Node* src = node->unitNodes[j-1];
			Node* snk = node->unitNodes[j];
			Edge* newEdge = new Edge(src, snk);
			newEdge->set_separation_time(1);
			src->out.push_back(newEdge);
			snk->in.push_back(newEdge);

			this->add_edge(newEdge);
		}
	}

	for (int i=0; i<origin->edge_vec.size(); i++) {
		Edge* edge = origin->edge_vec.at(i);
		int separation = edge->separationTime/this->gcd;
		Node* src = edge->src_node->unitNodes[separation-1];
		Node* snk = edge->snk_node->unitNodes[0];

		Edge* newEdge = new Edge(src, snk);
		newEdge->set_separation_time(1);
		src->out.push_back(newEdge);
		snk->in.push_back(newEdge);

		this->add_edge(newEdge);
	}
}

void UnitDigraph::scc_generate_unit_digraph() {
	for (vector<Node*>::iterator iter = origin->node_vec.begin(); iter != origin->node_vec.end(); iter++) {
		Node* node = *iter;

		int maxSeparationtime = this->gcd;
		for (list<Edge*>::iterator iter = node->scc_out.begin(); iter != node->scc_out.end(); iter++) {
			Edge* edge = *iter;
			maxSeparationtime = max(maxSeparationtime, edge->separationTime);
		}

		int numNode = maxSeparationtime/this->gcd;
		node->unitNodes = new Node*[numNode];
		node->unitNodeNum = numNode;

		// Create new nodes
		for (int j=0; j<numNode; j++) {
			string name = "u_"+node->name+"_"+Utility::int_to_string(j);
			int deadline = 1; // not important
			int wcet = 0;
			if (j==0) {
				wcet = node->wcet;
				deadline = node->deadline; // the demand bound functions of the original digraph and UDRT are same
			}

			Node* newNode = new Node(name,node->scale,wcet,deadline);

			this->add_node(newNode, node);

			node->unitNodes[j] = newNode;
		}

		// Create new edges
		for (int j=1; j<numNode; j++) {
			Node* src = node->unitNodes[j-1];
			Node* snk = node->unitNodes[j];
			Edge* newEdge = new Edge(src, snk);
			newEdge->set_separation_time(1);
			src->out.push_back(newEdge);
			snk->in.push_back(newEdge);

			this->add_edge(newEdge);
		}
	}

	for (int i=0; i<origin->edge_vec.size(); i++) {
		Edge* edge = origin->edge_vec.at(i);
		int separation = edge->separationTime/this->gcd;
		Node* src = edge->src_node->unitNodes[separation-1];
		Node* snk = edge->snk_node->unitNodes[0];

		Edge* newEdge = new Edge(src, snk);
		newEdge->set_separation_time(1);
		src->out.push_back(newEdge);
		snk->in.push_back(newEdge);

		this->add_edge(newEdge);
	}

	for (vector<Node*>::iterator iter = origin->node_vec.begin(); iter != origin->node_vec.end(); iter++) {
		Node* node = *iter;

		delete[] node->unitNodes;
		node->unitNodes = NULL;
	}
}

void UnitDigraph::generate_exec_request_matrix() {
	matrix = GraphAlgorithms::generate_exec_request_matrix(this);
	matrix_map[1]= matrix; // the execution request matrix of (0,gcd]
	double maximum_element = MaxPlusAlgebra::maximum_element(matrix, n_size);
	int temp = static_cast<int> (maximum_element);
	maximum_element_map[1] = temp;
	//matrix_map.insert(pair<int, double**>(1,matrix)); 
}

void UnitDigraph::calculate_linear_period(bool debug) {
	if (origin->strongly_connected)
		lper = MaxPlusAlgebra::calculate_linear_period(matrix,n_size,lfac*gcd,debug);
}

void UnitDigraph::calculate_linear_defect() {
	if (origin->strongly_connected)
		ldef = MaxPlusAlgebra::calculate_linear_defect(matrix_map, maximum_element_map, n_size, lfac, lper, tf, gcd);
}

void UnitDigraph::calculate_uppper_bound_linear_defect() {
	if (origin->strongly_connected)
		ubldef = MaxPlusAlgebra::calculate_upper_bound_linear_defect(matrix, n_size, lfac, lper, tf, gcd);
}

void UnitDigraph::calculate_exec_request_matrix_power(int tf) {
	/// Calculate all the execution request matrix from 2 to tf
	/*
	for (int t=2; t<=tf; t++) {
		if (matrix_map.find(t) != matrix_map.end())
			continue;
		double** matrix_power;
		if (t>ldef+lper && origin->strongly_connected)
			matrix_power = MaxPlusAlgebra::periodicly_calculate_maxplus_matrix_power(matrix_map[t-lper],n_size,lfac*gcd,lper);
		else
			matrix_power = MaxPlusAlgebra::multiply_maxplus_matrix(matrix_map[t-1],matrix_map[1],n_size);
		matrix_map[t] = matrix_power;
		double maximum_element = MaxPlusAlgebra::maximum_element(matrix_power, n_size);
		int temp = static_cast<int> (maximum_element);
		maximum_element_map[t] = temp;
	}
	*/

	/// Calculate all the execution request matrix from 2 to tf
	/// Use rbf(t+p)= rbf(t)+p*q to calculate rbf(t) instead of the matrix operaition
	for (int t=2; t<=tf; t++) {
		if (matrix_map.find(t) != matrix_map.end())
			continue;

		if (t>ldef+lper && origin->strongly_connected && maximum_element_map.find(t-lper)!=maximum_element_map.end()) {
			double maximum_element = maximum_element_map[t-lper]+lfac*gcd*lper;
			int temp = static_cast<int> (maximum_element);
			maximum_element_map[t] = temp;
			continue;
		}

		double** matrix_power;
		matrix_power = MaxPlusAlgebra::multiply_maxplus_matrix(matrix_map[t-1],matrix_map[1],n_size);
		matrix_map[t] = matrix_power;
		double maximum_element = MaxPlusAlgebra::maximum_element(matrix_power, n_size);
		int temp = static_cast<int> (maximum_element);
		maximum_element_map[t] = temp;
	}

}

void UnitDigraph::calculate_exec_request_matrix_power_without_periodicity(int tf) {
	for (int t=2; t<=tf; t++) {
		if (matrix_map.find(t) != matrix_map.end())
			continue;
		double** matrix_power = MaxPlusAlgebra::multiply_maxplus_matrix(matrix_map[t-1],matrix_map[1],n_size);
		matrix_map[t] = matrix_power;
		double maximum_element = MaxPlusAlgebra::maximum_element(matrix_power, n_size);
		int temp = static_cast<int> (maximum_element);
		maximum_element_map[t] = temp;
	}
}

int UnitDigraph::get_rbf(int t) {
	// we should consider the gcd
	double factor = 1.0*t/gcd;
	t = ceil(factor);
	if (maximum_element_map.find(t) != maximum_element_map.end())
		return maximum_element_map[t];
	else 
		//calculate_exec_request_matrix_power_without_periodicity(t);
		calculate_exec_request_matrix_power(t);
	return maximum_element_map[t];
}

int UnitDigraph::calculate_demand_bound_function(int i, int j, int t) {
	Node* first = index_to_original_node[i];
	Node* last = index_to_original_node[j];
	int tprim = 1000*scale;
	for (list<Edge*>::iterator iter = last->in.begin(); iter != last->in.end(); iter++) {
		Edge* edge = *iter;
		tprim = min(tprim,edge->separationTime/gcd);
	}
	t = t - last->deadline/gcd - tprim;
	if (t<0) return 0;
	if (t==0) return last->wcet;

	double** t_matrix = matrix_map[t];
	return t_matrix[i][j]+last->wcet;
}

int UnitDigraph::get_dbf(int t) {
	int max_value = 0;
	for (set<int>::iterator i_iter=iSet.begin(); i_iter!= iSet.end(); i_iter++) {
		int i = *i_iter;
		for (set<int>::iterator j_iter=iSet.begin(); j_iter!= iSet.end(); j_iter++) {
			int j = *j_iter;
			max_value = max(max_value, calculate_demand_bound_function(i,j,t));
		}
	}
	return max_value;
}

// ===========================================================================
// Method functions of GranDigraph class
// ===========================================================================
GranDigraph::~GranDigraph() {
	//cout<<"GranDigraph Destruction Start"<<endl;

	// release nodes
	for (vector<Node*>::iterator iter = node_vec.begin(); iter != node_vec.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	node_vec.clear();

	// release edges
	for (vector<Edge*>::iterator iter = edge_vec.begin(); iter != edge_vec.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	edge_vec.clear();

	node_to_index.clear();
	index_to_node.clear();
	edge_to_index.clear();
	index_to_edge.clear();

	// release sccs
	for (vector<Digraph*>::iterator iter = sccs.begin(); iter != sccs.end(); iter++) {
		delete *iter;
		*iter = NULL;
	}
	sccs.clear();

	if (unit_digraph != NULL) delete unit_digraph;
	if (gran_digraph != NULL) delete gran_digraph;

	// release matrix
	/*
	for (int i=0; i<n_size; i++) {
		delete[] matrix[i];
	}
	delete[] matrix;
	*/

	// move to the function for calculating linear defect
	// delete execution request matrix
	for (map<int, double**>::iterator iter = matrix_map.begin(); iter != matrix_map.end(); iter++) {
		double** temp = iter->second;
		for (int i=0; i<n_size; i++)
			delete[] temp[i];
		delete[] temp;
	}

	matrix_map.clear();
	maximum_element_map.clear();

	//cout<<"GranDigraph End"<<endl;
}

/// \brief prepare function
void GranDigraph::prepare(bool debug) {

	// calculate gcd
	//calculate_all_gcd();
	//gcd = aGCD;
	// generate GDRT
	generate_gran_digraph();
	n_size = node_vec.size();

	// generate execution request matrix
	generate_exec_request_matrix();

	// Calculate linear period
	cout << "calculate the linear period of ibf ..." << endl;
	calculate_linear_period(debug);

	// Calculate linear defect
	cout << "calculate the linear defect of ibf ..." << endl;
	calculate_linear_defect();

	// calculate upper bound of the linear defect
	// cout << "calculating the upper bound of linear defect of ibf ..." << endl;
	// calculate_uppper_bound_linear_defect();


	/*
	// delete execution request matrix
	for (map<int, double**>::iterator iter = matrix_map.begin(); iter != matrix_map.end(); iter++) {
		double** temp = iter->second;
		for (int i=0; i<n_size; i++)
			delete[] temp[i];
		delete[] temp;
	}
	*/

	// Calculate the execution request matrix power
	//calculate_exec_request_matrix_power(tf);
}

/// \brief prepare function
void GranDigraph::prepare2(bool debug) {
	Timer timer;
	// generate GDRT
	generate_gran_digraph();
	n_size = node_vec.size();

	// generate execution request matrix
	generate_exec_request_matrix();

	// Calculate linear period
	cout << "calculate the linear period of ibf ..." << endl;
	timer.start();
	calculate_linear_period(debug);
	timer.end();
	tCalLinearPeriod = timer.getTime();
	cout << "lper=" << lper << endl;

	// Calculate linear defect
	//cout << "calculate the linear defect of ibf ..." << endl;
	//calculate_linear_defect();

	// calculate upper bound of the linear defect
	cout << "calculating the upper bound of linear defect of ibf ..." << endl;
	timer.start();
	calculate_uppper_bound_linear_defect();
	timer.end();
	tCalLinearDefect = timer.getTime();
}

void GranDigraph::generate_gran_digraph() {
	for (vector<Node*>::iterator iter = origin->node_vec.begin(); iter != origin->node_vec.end(); iter++) {
		Node* node = *iter;

		int maxSeparationtime = this->gcd;
		for (list<Edge*>::iterator iter = node->out.begin(); iter != node->out.end(); iter++) {
			Edge* edge = *iter;
			maxSeparationtime = max(maxSeparationtime, edge->separationTime);
		}

		int numNode = maxSeparationtime/this->gcd;
		node->granNodes = new Node*[numNode];

		// Create new nodes
		for (int j=0; j<numNode; j++) {
			string name = "g_"+node->name+"_"+Utility::int_to_string(j);
			int deadline = 1; // not important
			int wcet = 0;
			if (j<node->wcet/this->gcd) wcet = this->gcd;

			Node* newNode = new Node(name,node->scale,wcet,deadline);

			this->add_node(newNode);

			node->granNodes[j] = newNode;
		}

		// Create new edges
		for (int j=1; j<numNode; j++) {
			Node* src = node->granNodes[j-1];
			Node* snk = node->granNodes[j];
			Edge* newEdge = new Edge(src, snk);
			newEdge->set_separation_time(1);
			src->out.push_back(newEdge);
			snk->in.push_back(newEdge);

			this->add_edge(newEdge);
		}
	}

	for (int i=0; i<origin->edge_vec.size(); i++) {
		Edge* edge = origin->edge_vec.at(i);
		int separation = edge->separationTime/this->gcd;
		Node* src = edge->src_node->granNodes[separation-1];
		Node* snk = edge->snk_node->granNodes[0];

		Edge* newEdge = new Edge(src, snk);
		newEdge->set_separation_time(1);
		src->out.push_back(newEdge);
		snk->in.push_back(newEdge);

		this->add_edge(newEdge);
	}
}

void GranDigraph::generate_exec_request_matrix() {
	matrix = GraphAlgorithms::generate_exec_request_matrix(this);
	matrix_map[1]= matrix; // the execution request matrix of (0,gcd]
	double maximum_element = MaxPlusAlgebra::maximum_element(matrix, n_size);
	int temp = static_cast<int> (maximum_element);
	maximum_element_map[1] = temp;
	//matrix_map.insert(pair<int, double**>(1,matrix)); 
}

void GranDigraph::calculate_linear_period(bool debug) {
	if (origin->strongly_connected) {
		lper = MaxPlusAlgebra::calculate_linear_period(matrix,n_size,lfac*gcd,debug);
		//lper = origin->unit_digraph->lper*origin->unit_digraph->gcd/gcd;
	}
}

void GranDigraph::calculate_linear_defect() {
	if (origin->strongly_connected)
		ldef = MaxPlusAlgebra::calculate_linear_defect(matrix_map, maximum_element_map, n_size, lfac, lper, tf, gcd);
}

void GranDigraph::calculate_uppper_bound_linear_defect() {
	if (origin->strongly_connected)
		ubldef = MaxPlusAlgebra::calculate_upper_bound_linear_defect(matrix, n_size, lfac, lper, tf, gcd);
}

void GranDigraph::calculate_exec_request_matrix_power(int tf) {
	/// Calculate all the execution request matrix from 2 to tf
	for (int t=2; t<=tf; t++) {
		if (matrix_map.find(t) != matrix_map.end())
			continue;
		double** matrix_power;
		if (t>ldef+lper)
			matrix_power = MaxPlusAlgebra::periodicly_calculate_maxplus_matrix_power(matrix_map[t-lper],n_size,lfac*gcd,lper);
		else
			matrix_power = MaxPlusAlgebra::multiply_maxplus_matrix(matrix_map[t-1],matrix_map[1],n_size);
		matrix_map[t] = matrix_power;
		double maximum_element = MaxPlusAlgebra::maximum_element(matrix_power, n_size);
		int temp = static_cast<int> (maximum_element);
		maximum_element_map[t] = temp;
	}
}

int GranDigraph::get_ibf(int t) {
	if (t==0) return 0;
	// we should consider the gcd
	double factor = (double) t/gcd;
	int t_gcd = ceil(factor);
	int a = maximum_element_map[t_gcd];
	int b;
	if (t_gcd-1==0) b=0; else b = maximum_element_map[t_gcd-1];
	if (abs(a-b)<EPSILON) 
		return b;
	else {
		//std::cout << "test:" <<(a == b+gcd) <<std::endl;
		return b+t-(t_gcd-1)*gcd;
	}
}


	