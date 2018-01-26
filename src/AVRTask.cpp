#include "AVRTask.h"
#include "Utility.h"
#include "OptimizationAlgorithm.h"
#include "Global.h"

#include <set>

void AVRTask::updateSpeeds(vector<double> _speeds) {
	for (int i=0; i<numMode; i++) {
		speeds[i] = _speeds[i];
		omegas[i] = RPM_to_RPmSEC(speeds[i]);
	}
}

void AVRTask::updateReducedSpeeds(vector<double> reducedSpeeds) {
	for (int i=0; i<reducedSpeeds.size(); i++) {
#if 0
		double speedLength = min(reducedSpeeds[i],speeds[i]-engine.RPM_MIN);
		speeds[i] -= speedLength;
		omegas[i] -= RPM_to_RPmSEC(speedLength);
#else
		speeds[i] -= reducedSpeeds[i];
		omegas[i] -= RPM_to_RPmSEC(reducedSpeeds[i]);
#endif
	}
}

void AVRTask::updateReducedSpeeds(map<int,int> dc, vector<double> reducedSpeeds) {
	vector<double> currSpeeds;
	for (int i=0; i<reducedSpeeds.size(); i++) {
		currSpeeds.push_back(speeds[i]-reducedSpeeds[i]);
	}
	currSpeeds.push_back(speeds[speeds.size()-1]);

	OptimizationAlgorithm::enforceDCGuards(dc,currSpeeds);
	updateSpeeds(currSpeeds);
}

void AVRTask::prepareLinearUppperBound() {
	vector<double> _speeds;
	vector<double> _omegas;
	vector<int> _wcets;

	for (auto e : speeds) _speeds.push_back(e.second);
	for (auto e : omegas) _omegas.push_back(e.second);
	for (auto e : wcets) _wcets.push_back(e.second);

	vector<int>::iterator cIter = max_element(_wcets.begin(), _wcets.end());
	_maxC = *cIter;

	//vector<double> utils = OptimizationAlgorithm::getUtilizations(_speeds,_wcets,period);
	vector<double> utils = OptimizationAlgorithm::getMaxUtilizations(_omegas,_wcets,period,engine);
	vector<double>::iterator uIter = max_element(utils.begin(), utils.end());
	_maxU = *uIter;
}

void AVRTask::scaleWCETs(int factor) {
	for (auto wcet : wcets) {
		wcet.second *= factor;
	}
}

int AVRTask::maxN(double omega) {
	if (W_LEQ_TOL(omega,engine.SPEED_MIN))
		return -1;

	int n = 0;
	while (1) {
		omega = engine.getLowerSpeed(omega,period,1);
		if (W_LEQ_TOL(omega,engine.SPEED_MIN))
			break;
		n++;
	}
	return n;
}

double AVRTask::maxM(double omega) {
	double omega_m = 0.0; 
	for (map<int,double>::iterator iter = omegas.begin(); iter != omegas.end(); iter ++) {
		double temp = iter->second;
		if (W_GEQ_TOL(temp, omega)) break;
		omega_m = temp;
	}
	return omega_m;
}


int AVRTask::getModeNumberWithNonZero()
{
	int ret = 0;
	for (auto e:speeds) {
		if (e.second > 0)
			ret ++;
	}
	return ret;
}
/*
vector<double> AVRTask::getDominants(double omega_b, double omega_a) {
	double omega = omega_a;
	vector<double> dominants;
	while (W_GEQ_TOL(omega, omega_b)) {
		dominants.push_back(omega);
		int n = maxN(omega);
		double omega_next = 0;
		for (int i=0; i <=n; i++) {
			double omega_i = engine.getLowerSpeed(omega,period,i);
			if ( W_LEQ_TOL(omega_i,engine.SPEED_MIN) ) break;
			double omega_i2 = maxM(omega_i);
			double omega_i3 = engine.getHigherSpeed(omega_i2,period,i);
			//if (omega_i3 > 0.10791)
			//	cout << "here" << endl;
			omega_next = max(omega_next,omega_i3);
		}
		//if (omega == omega_next)
		//	cout << "here" << endl;
		omega = omega_next;
	}

	bool isExisted = false;
	for (map<int,double>::iterator iter = omegas.begin(); iter != omegas.end(); iter++) {
		double temp = iter->second;
		if (W_EQ_TOL(temp,omega_b)) {
			isExisted = true;
			break;
		}
	}

	if (isExisted) dominants.push_back(omega_b);

	return dominants;
}

vector<double> AVRTask::getDominants(double omega_b, double omega_a, double timeWindowLength) {
	double omega = omega_a;
	vector<double> dominants;

	if (timeWindowLength <= 0) return dominants;

	while (W_GEQ_TOL(omega, omega_b)) {
	//while (omega > omega_b) {
		dominants.push_back(omega);
		int n = maxN(omega);
		double omega_next = 0;
		for (int i=0; i <=n; i++) {
			double omega_i = engine.getLowerSpeed(omega,period,i);
			if ( W_LEQ_TOL(omega_i,engine.SPEED_MIN) ) break;
			double period_i = engine.getLowerInterval(omega,omega_i,period,i);

			if (timeWindowLength > 0 && mSEC_to_muSEC(period_i) > timeWindowLength) break;

			double omega_i2 = maxM(omega_i);
			double omega_i3 = engine.getHigherSpeed(omega_i2,period,i);
			//if (omega_i3 > 0.10791)
			//	cout << "here" << endl;
			omega_next = max(omega_next,omega_i3);
		}
		//if (omega == omega_next)
		//	cout << "here" << endl;
		omega = omega_next;
	}

	bool isExisted = false;
	for (map<int,double>::iterator iter = omegas.begin(); iter != omegas.end(); iter++) {
		double temp = iter->second;
		if (W_EQ_TOL(temp, omega_b)) {
			isExisted = true;
			break;
		}
	}

	if (isExisted) dominants.push_back(omega_b);

	return dominants;
}
*/

vector<double> AVRTask::getDominants(double omega_b, double omega_a, double timeWindowLength) {
	vector<double> dominants;

	double w_star = omega_a;

	while((w_star > omega_b - W_TOLERANCE) && (!W_EQ_TOL(w_star,engine.SPEED_MIN)))
	{
#ifdef __DEBGU__
//#if 1
		cout << "omega_star = " << RPmSEC_to_RPM(w_star) << " omega_b = " << RPmSEC_to_RPM(omega_b) << endl;	
#endif

		dominants.push_back(w_star);
		vector<double> candidates;
		unsigned int i=0;

		// Collect candidates until the minimum speed SPEED_MIN
		while(true)
		{
			double w_i = engine.getLowerSpeed(w_star,period,i);

			// Stop condition for the while(true) based on minimum speed
			if(w_i < engine.SPEED_MIN + W_TOLERANCE)
				break;

			double T_i = engine.getLowerIntervalWithConstantAcceleration(w_star,w_i,period,i);
			T_i = mSEC_to_muSEC(T_i);

			// Stop condition for the while(true) based on maximum interference window
			if(timeWindowLength>0 && T_i>timeWindowLength) 
			{
#ifdef __DEBUG__
				cout << "Break for time limitation" << endl;
				cout << "T_i = " << T_i << ", window = " << timeWindowLength << endl;
#endif
				break;
			}

			double w_star_i = NULL;

			// Compute w_star_i = Maximum switching speed < w_i
			for(int m=omegas.size()-1; m>= 0; m--) 
			{
				if (omegas[m] + W_TOLERANCE < w_i) 
				{
					w_star_i = omegas[m];
					break;
				}
			}

			if(w_star_i!=NULL)
			{
				double w_star_candidate = engine.getHigherSpeed(w_star_i,period,i);
				candidates.push_back(w_star_candidate);
			}
			else break;

			i++;
		}

		double old_w_star = w_star;

		//---------------------------------------------------------
		// Here we have a set of candidates and we take the maximum
		//---------------------------------------------------------

		w_star = NULL;

		while (candidates.size() != 0) {
			vector<double>::iterator maxIter = max_element(begin(candidates), end(candidates));
			if(W_EQ_TOL(w_star,old_w_star)) {
				candidates.erase(maxIter);
			}
			else {
				w_star = *maxIter;
				break;
			}
		} 

		if (w_star == NULL)
			break;
	}

	return dominants;
}

std::vector<double> AVRTask::getAllDominants()
{
	// Compute all the initial dominants if not already computed
	if (initialDominants.size() == 0)
		initialDominants = getDominants(engine.SPEED_MIN,engine.SPEED_MAX);

	vector<double> allDominants = initialDominants;

	// For each initial dominants we compute the sub-dominants

	for (auto omega : initialDominants)
		collectSubDominants(omega,allDominants);

	// SORT the dominant speeds
	sort(allDominants.begin(),allDominants.end(),greater<double>());

	return allDominants;
}

void AVRTask::collectSubDominants(double omega, vector<double>& allDominants)
{
	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);
	omega_a = min(omega_a, engine.SPEED_MAX);
	omega_b = max(omega_b, engine.SPEED_MIN);

	vector<double> nextDominants = getDominants(omega_b,omega_a);

	for (auto e : nextDominants) {
		if(!alreadyStored(e,allDominants)) {
			allDominants.push_back(e);
			collectSubDominants(e,allDominants);
		}
	}
}

bool AVRTask::alreadyStored(double omega, vector<double> dominants)
{
	for (auto e : dominants) 
		if (W_EQ_TOL(omega,e)) return true;
	return false;
}

std::vector<double> AVRTask::getAllSpeedRanges()
{
	vector<double> ret;

	// Acceleration for the minimum speed
	double tempOmega = engine.SPEED_MIN;
	//while (W_LEQ_TOL(tempOmega,omegas[0])) {
	while (W_LEQ_TOL(tempOmega,engine.SPEED_MAX)) {
		Utility::insert_vector(ret,tempOmega);
		//cout << RPmSEC_to_RPM(omega) << endl;
		tempOmega = engine.getHigherSpeed(tempOmega,period,1);
	}

	//Utility::output_one_vector(cout,"500rpm",ret);

	// Acceleration for the mode speeds 
	for (auto e : omegas) {
		int index = e.first;
		double omega = e.second;

		if (index == numMode-1) {
			Utility::insert_vector(ret,omega);
			continue;
		}

		//double ubOmega = omegas[index+1];
		double ubOmega = engine.SPEED_MAX;

		while (W_LEQ_TOL(omega,ubOmega)) {
			Utility::insert_vector(ret,omega);
			//cout << RPmSEC_to_RPM(omega) << endl;
			omega = engine.getHigherSpeed(omega,period,1);
		}
	}

	// Deceleration
	for (auto e : omegas) {
		int index = e.first;
		double omega = e.second;

		//cout << "Index = " << index << ", Speed = " << RPmSEC_to_RPM(omega) << ", numMode = " << numMode  << endl;
		double lbOmega = 0.0;
		if (index == 0) lbOmega = engine.SPEED_MIN;
		else lbOmega = omegas[index-1];

		lbOmega = engine.SPEED_MIN;

		while (W_GEQ_TOL(omega,lbOmega)) {
			Utility::insert_vector(ret,omega);
			//cout << RPmSEC_to_RPM(omega) << endl;
			omega = engine.getLowerSpeed(omega,period,1);
		}
		
	}
	//exit(EXIT_FAILURE);

	Utility::insert_vector(ret,engine.SPEED_MIN);
	
	// SORT the dominant speeds
	sort(ret.begin(),ret.end(),greater<double>());
	return ret;
}

std::vector<double> AVRTask::getSpeedPartitionWithGranularity(int partitionGranularity)
{
	if (partitionGranularity <= 0) {
		cerr << "Wrong: Partition Granularity = " << partitionGranularity << endl;
		exit(EXIT_FAILURE);
	}
	
	vector<double> ret;

	vector<double> temp;
	for (auto e : omegas) {
		temp.push_back(e.second);
	}
	temp.push_back(engine.SPEED_MIN);
	sort(temp.begin(),temp.end(),greater<double>());

	for (int i=0; i<temp.size()-1; i++) {
		double rangeLength = temp[i]-temp[i+1];
		double rangeGranularity = rangeLength/partitionGranularity;
		for (int j=1; j<=partitionGranularity; j++) {
			ret.push_back(temp[i]-(j-1)*rangeGranularity);
		}
	}
	ret.push_back(engine.SPEED_MIN);

	return ret;
}

void AVRTask::buildDigraph()
{
	if (digraph != NULL)
		return;

#ifdef __DEBUG_DESIGN__
	cout << "Build digraph" << endl;
#endif

	vector<double> dominants = getAllDominants();

#ifdef __DEBUG_DESIGN__
	cout << "Computed " << dominants.size() << " total dominants" << endl;
#endif

	int index = 0;
	digraph = new Digraph(0);

	map<double, Node*> omega_to_node;

	// Build a node for each dominant speed
	for (auto omega : dominants) {
		string name = "v"+Utility::int_to_string(index) + "_" + Utility::int_to_string(RPmSEC_to_HundredRPM(omega)) + "RPM";

		Node* node = new Node(name,index++,1,getWCET(omega),0,omega);
		omega_to_node[omega] = node;

		if (find(initialDominants.begin(),initialDominants.end(),omega)!=initialDominants.end())
			node->initialDominant = true;

		digraph->add_node(node);
	}

	// Build all the edges
	for (auto omega : dominants) {
		double omega_a = engine.getHigherSpeed(omega,period,1);
		double omega_b = engine.getLowerSpeed(omega,period,1);
		omega_a = min(omega_a, engine.SPEED_MAX);
		omega_b = max(omega_b, engine.SPEED_MIN);

		if (omega_to_node.find(omega) == omega_to_node.end()) {
			cerr << "Not found omega " << omega << endl;
			exit(EXIT_FAILURE);
		}
		Node* src = omega_to_node[omega];

#if 0 // connect to the range
		for (auto next_omega : dominants) {
			if (W_GEQ_TOL(next_omega,omega_b) && W_LEQ_TOL(next_omega, omega_a)) {
				Node* snk = NULL;

				if (omega_to_node.find(next_omega) == omega_to_node.end()) {
					for (auto e : omega_to_node) {
						if (W_EQ_TOL(e.first,next_omega)) {
							snk = e.second;
							break;
						}
					}
				}
				else 
					snk = omega_to_node[next_omega];

				if (snk == NULL) {
					cerr << "Not found next omega!" << next_omega << endl;
					exit(EXIT_FAILURE);
				}

				Edge* edge = new Edge(src,snk);
				edge->separationTime = mSEC_to_muSEC(engine.getInterArrivalTime(omega,next_omega,period));

				digraph->add_edge(edge);
			}
		}
#else // connect to the dominants
		vector<double> nextDominants = getDominants(omega_b,omega_a);

		for (auto next_omega : nextDominants) {
			Node* snk = NULL;

			if (omega_to_node.find(next_omega) == omega_to_node.end()) {
				for (auto e : omega_to_node) {
					if (W_EQ_TOL(e.first,next_omega)) {
						snk = e.second;
						break;
					}
				}
			}
			else 
				snk = omega_to_node[next_omega];
			
			if (snk == NULL) {
				cerr << "Not found next omega!" << next_omega << endl;
				exit(EXIT_FAILURE);
			}

			Edge* edge = new Edge(src,snk);
#ifdef __USING_CONSTANT_ACCELERATION__
			edge->separationTime = mSEC_to_muSEC(engine.getInterArrivalTimeWithConstantAcceleration(omega,next_omega,period));
#else
			edge->separationTime = mSEC_to_muSEC(engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,next_omega,period));
#endif

			// Force the time granularity be 10 us
			edge->separationTime = Utility::round(1.0*edge->separationTime/10)*10;

			digraph->add_edge(edge);
 		}
#endif
	}

	digraph->calculate_period_gcd();
	digraph->calculate_all_gcd();
}

void AVRTask::buildSimpleODRT() {
	digraph = new Digraph(0);

	vector<double> omegaLs;
	vector<double> omegaHs;

	for(int i=0; i<numMode; i++) {
		if (i==0) {
			omegaHs.push_back(omegas[i]);
			omegaLs.push_back(engine.SPEED_MIN);
		}
		else {
			omegaHs.push_back(omegas[i]);
			omegaLs.push_back(omegas[i-1]);
		}
	}

	// Build a node for each speed interval
	for (int i=omegaLs.size()-1; i>=0; i--) {
		double omegaL = omegaLs[i];
		double omegaH = omegaHs[i];

		int nodeIndex = omegaLs.size()-1-i;

		string name = "v"+Utility::int_to_string(nodeIndex) + "_L" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaL)) + "RPM"
			+ "_H" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaH)) + "RPM";

		Node* node = new Node(name,nodeIndex,1);
		node->omegaL = omegaL;
		node->omegaH = omegaH;

		node->wcet = getWCET(omegaH);
		digraph->add_node(node);
	}

	// Build all the edges
	for (int i=0; i<digraph->node_vec.size(); i++) {
		Node* src = digraph->node_vec[i];

		for (int j=0; j<digraph->node_vec.size(); j++) {
			Node* snk = digraph->node_vec[j];

			double pmin = engine.getMinTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,period);
			double pmax = engine.getMaxTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,period);

			if (pmin > 0 && pmax > 0) {
				Edge* edge = new Edge(src,snk);
				edge->separationTime = mSEC_to_muSEC(pmin);
				edge->maxSeparationTime = mSEC_to_muSEC(pmax);
				digraph->add_edge(edge);
			}
		}
	}
}

void AVRTask::buildComplexODRT() {
	digraph = new Digraph(0);

	vector<double> speedIntervals = getAllSpeedRanges();

	vector<double> omegaLs;
	vector<double> omegaHs;

	for(int i=speedIntervals.size()-1; i > 0; i--) {
		omegaHs.push_back(speedIntervals[i-1]);
		omegaLs.push_back(speedIntervals[i]);
	}

	// Build a node for each speed interval
	for (int i=omegaLs.size()-1; i>=0; i--) {
		double omegaL = omegaLs[i];
		double omegaH = omegaHs[i];

		int nodeIndex = omegaLs.size()-1-i;

		string name = "v"+Utility::int_to_string(nodeIndex) + "_L" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaL)) + "RPM"
			+ "_H" + Utility::int_to_string(RPmSEC_to_HundredRPM(omegaH)) + "RPM";

		Node* node = new Node(name,nodeIndex,1);
		node->omegaL = omegaL;
		node->omegaH = omegaH;

		node->wcet = getWCET(omegaH);
		digraph->add_node(node);
	}

	// Build all the edges
	for (int i=0; i<digraph->node_vec.size(); i++) {
		Node* src = digraph->node_vec[i];

		for (int j=0; j<digraph->node_vec.size(); j++) {
			Node* snk = digraph->node_vec[j];

			double pmin = engine.getMinTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,period);
			double pmax = engine.getMaxTimeInterReleaseTime(src->omegaL,src->omegaH,snk->omegaL,snk->omegaH,period);

			if (pmin > 0 && pmax > 0) {
				Edge* edge = new Edge(src,snk);
				edge->separationTime = mSEC_to_muSEC(pmin);
				edge->maxSeparationTime = mSEC_to_muSEC(pmax);
				digraph->add_edge(edge);
			}
		}
	}
}

map<int,double> AVRTask::collectInfo(double omega, double t) {
	double omega_a = calHighestSpeed(omega,t);
	double omega_b = calLowestSpeed(omega,t);

	vector<double> dominants = getDominants(omega_b,omega_a);

	map<int,double> ret;
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		double point = *iter;
		int wcet = getWCET(point);
		if (ret.find(wcet) != ret.end()) {
			ret[wcet] = max(ret[wcet],point);
		}
		else {
			ret[wcet] = point;
		}
	}

	return ret;
}

map<int,double> AVRTask::collectInfo(vector<double> dominants, double t) {
	map<int,double> ret;
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		map<int,double> temp = collectInfo(*iter,t);
		for (map<int,double>::iterator mIter = temp.begin(); mIter != temp.end(); mIter++) {
			if (ret.find(mIter->first)!= ret.end()) {
				ret[mIter->first] = max(ret[mIter->first],mIter->second);
			}
			else
				ret[mIter->first] = mIter->second;
		}
	}

	return ret;
}

bool AVRTask::checkMode(double omega) {
	for (auto e: omegas) {
		if (fabs(e.second-omega) < RPM_to_RPmSEC(1.0))
			return true;
	}
	return false;
}

int AVRTask::getWCET(double omega) {
	for (map<int,double>::iterator iter = omegas.begin(); iter != omegas.end(); iter ++) {
		double temp = iter->second;
		if (W_GEQ_TOL(temp, omega)) 
			return wcets[iter->first];
	}

	cerr << "P0:getWCET, Cannot arrive here!" << "Omega = " << omega << endl;
	cerr << "omegas = ";
	for (auto e:omegas)
		cerr << e.second << " ";
	cerr << endl;
	
	cerr << "speeds = ";
	for (auto e:speeds)
		cerr << e.second << " ";
	cerr << endl;

	exit(EXIT_FAILURE);
}

int AVRTask::getWCET_RPM(double rpm) {
	for (map<int,double>::iterator iter = speeds.begin(); iter != speeds.end(); iter ++) {
		if (RPM_GEQ_TOL(iter->second,rpm)) 
			return wcets[iter->first];
	}

	cerr << "Error: rpm = " << rpm << endl;
	exit(EXIT_FAILURE);
}

double AVRTask::getOmega(double omega) {
	for (map<int,double>::iterator iter = omegas.begin(); iter != omegas.end(); iter ++) {
		double temp = iter->second;
		if (W_GEQ_TOL(temp,omega)) 
			return temp;
	}

	cerr << "getOmega, double, Cannot arrive here!" << endl;
	exit(EXIT_FAILURE);
}

int AVRTask::getOmega_RPM(double rpm) {
	for (map<int,double>::iterator iter = speeds.begin(); iter != speeds.end(); iter ++) {
		if (RPM_GEQ_TOL(iter->second,rpm)) 
			return iter->second;
	}

	cerr << "getOmega, int, Cannot arrive here!" << endl;
	exit(EXIT_FAILURE);
}

int AVRTask::getModeNum(double omega) {
	for (map<int,double>::iterator iter = omegas.begin(); iter != omegas.end(); iter ++) {
		double temp = iter->second;
		if (W_GEQ_TOL(temp, omega)) 
			return iter->first;
	}

	cerr << "P0:getWCET, Cannot arrive here!" << "Omega = " << omega << endl;
	cerr << "omegas = ";
	for (auto e:omegas)
		cerr << e.second << " ";
	cerr << endl;

	cerr << "speeds = ";
	for (auto e:speeds)
		cerr << e.second << " ";
	cerr << endl;

	exit(EXIT_FAILURE);
}

double AVRTask::calDeadline(double omega) {
	return engine.getMinDeadline(period,omega);
}

map<int,int> AVRTask::getRecord(map<int,map<int,map<int,int>>> &record, int rpm, int t) {
	map<int,int> ret;
	if (t <= 0) return ret;
	if (record.find(rpm) != record.end()) {
		map<int,map<int,int>> temp = record[rpm];
		map<int,map<int,int>>::const_iterator iter = temp.lower_bound(t);
		if (iter != temp.end()) 
			return iter->second;
	}
	return ret;
}

void AVRTask::storeRBFRecord(map<int,map<int,map<int,int>>> &record,int rpm, int t, map<int,int> points) {
	if ( t <= 0) return;

	if (record.find(rpm) != record.end()) {
		map<int,map<int,int>>& temp = record[rpm];
		map<int,map<int,int>>::iterator iter = temp.lower_bound(t);
		if (iter != temp.end()) {
			map<int,int> newPoints = combineRBFPoints(points,iter->second);
			temp[iter->first] = newPoints;
			return;
		}
	}

	record[rpm][t] = points;
}

map<int,int> AVRTask::combineRBFPoints(map<int,int> m1, map<int,int> m2) {
	map<int,int> ret = m1;
	for (map<int,int>::iterator iter = m2.begin(); iter != m2.end(); iter++) {
		if (ret.find(iter->first) == ret.end())
			ret[iter->first] = iter->second;
		else
			ret[iter->first] = max (ret[iter->first], iter->second);
	}

	vector<int> remove;
	double curY = 0;
	for (map<int,int>::iterator iter = ret.begin(); iter != ret.end(); iter++) {
		int x = iter->first;
		double y = iter->second;
		if (y <= curY) {
			remove.push_back(x);
		}
		else curY = y;
	}

	for (vector<int>::iterator iter = remove.begin(); iter != remove.end(); iter++) 
		ret.erase(*iter);

	return ret;
}

map<int,int> AVRTask::createRBFPoints(map<int,int> m1, int tNext, int wcet) {
	map<int,int> ret;
	ret[0] = wcet;

	for (map<int,int>::iterator iter = m1.begin(); iter != m1.end(); iter++) {
		int x = iter->first + tNext;
		double y = iter->second + wcet;
		ret[x] = y;
	}
	return ret;
}

void AVRTask::storeIBFRecord(map<int,map<int,map<int,int>>> &record,int rpm, int t, map<int,int> points) {
	if ( t <= 0) return;

	if (record.find(rpm) != record.end()) {
		map<int,map<int,int>>& temp = record[rpm];
		map<int,map<int,int>>::iterator iter = temp.lower_bound(t);
		if (iter != temp.end()) {
			map<int,int> newPoints = combineIBFPoints(points,iter->second);
			temp[iter->first] = newPoints;
			return;
		}
	}

	record[rpm][t] = points;
}

map<int,int> AVRTask::combineIBFPoints(map<int,int> m1, map<int,int> m2) {
	for (map<int,int>::iterator iter = m2.begin(); iter != m2.end(); iter++) {
		int x = iter->first;
		int y = iter->second;

		bool dominated = false;
		bool dominating = false;
		set<int> remove;

		for (map<int,int>::iterator iter1 = m1.begin(); iter1 != m1.end(); iter1++) {
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
				m1.erase(*iter1);
			}
		}

		if (!dominated)
			m1[x] = y;
	}
	return m1;
}

map<int,int> AVRTask::createIBFPoints(map<int,int> m1, int tNext, int wcet) {
	map<int,int> ret;
	//ret[0] = 0; 
	ret[wcet] = wcet;

	for (map<int,int>::iterator iter = m1.begin(); iter != m1.end(); iter++) {
		int x = iter->first + tNext;
		double y = iter->second + wcet;
		ret[x] = y;
	}
	return ret;
}

vector<map<int,int>> AVRTask::getRecord(map<int,map<int,vector<map<int,int>>>> &record, int rpm, int t) {
	vector<map<int,int>> ret;
	if (t <= 0) return ret;
	if (record.find(rpm) != record.end()) {
		map<int,vector<map<int,int>>> temp = record[rpm];
		map<int,vector<map<int,int>>>::const_iterator iter = temp.lower_bound(t);
		if (iter != temp.end()) 
			return iter->second;
	}
	return ret;
}

void AVRTask::storeRecord(map<int,map<int,vector<map<int,int>>>> &record,int rpm, int t, vector<vector<map<int,int>>> points) {
	if ( t <= 0) return;
	vector<map<int,int>> ret;
	for (vector<vector<map<int,int>>>::iterator iter1 = points.begin(); iter1 != points.end(); iter1++) {
		vector<map<int,int>> ret2 = *iter1;
		for (vector<map<int,int>>::iterator iter2 = ret2.begin(); iter2 != ret2.end(); iter2++)
			ret.push_back(*iter2);
	}
	record[rpm][t] = ret;
}

vector<map<int,int>> AVRTask::createPoints(vector<map<int,int>> m1, int tNext, int wcet) {
	vector<map<int,int>> ret;

	if (m1.empty()) {
		map<int,int> m2;
		m2[0] = wcet;
		ret.push_back(m2);
		return ret;
	}

	for (vector<map<int,int>>::iterator iter = m1.begin(); iter != m1.end(); iter++) {
		map<int,int> m2 = createRBFPoints(*iter,tNext,wcet);
		ret.push_back(m2);
	}
	return ret;
}

map<int,int> AVRTask::getLastRecord(map<int,map<int,map<int,int>>> record,int rpm) {
	map<int,map<int,int>>::iterator iter = record[rpm].end();
	iter--;
	return iter->second;
}

int AVRTask::getRequestBoundFunction(map<int,int> record, int t) {
	if (t <= 0) return 0;
	if (record.empty()) {
		cerr << "Empty record!" << endl;
		exit(EXIT_FAILURE);
	}
	
	map<int,int>::iterator iter = record.lower_bound(t);
	if (iter != record.end()) {
		iter--;
		return iter->second;
	}
	else {
		iter = record.end();
		iter--;
		return iter->second;
	}
	/*
	map<int,int>::iterator iter = record.upper_bound(t);
	iter --;
	return iter->second;
	*/
}

map<int,int> AVRTask::calRequestBoundFunction(double omega, int t) {
	map<int,int> ret;
	if (t <= 0) return ret;

	vector<double> dominants;
	int rpm1000 = Utility::round(RPmSEC_to_ThousandRPM(omega));

	ret = getRecord(rbfRecord,rpm1000,t);
	if (!ret.empty()) return ret;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);
	omega_a = min(omega_a, engine.SPEED_MAX);
	omega_b = max(omega_b, engine.SPEED_MIN);


	dominants = getDominants(omega_b,omega_a,t);
	vector<double> dominants2 = getDominants(omega_b,omega_a);

	//Utility::output_one_vector(cout,"dominants", dominants);

	int wcet = getWCET(omega);
	vector<map<int,int>> globalRecord;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		double omega_next = *iter;
		int rpm_next = Utility::round( RPmSEC_to_ThousandRPM(omega_next));
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period); // ms
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period); // ms
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time)); // us

		map<int,int> points = getRecord(rbfRecord,rpm_next,t-tNext);
		if (points.empty()) {
			points = calRequestBoundFunction(omega_next,t-tNext);
		} 

		map<int,int> newPoints = createRBFPoints(points,tNext,wcet);
		globalRecord.push_back(newPoints);
		
	}
	for (vector<map<int,int>>::iterator iter = globalRecord.begin(); iter != globalRecord.end(); iter++)
		storeRBFRecord(rbfRecord,rpm1000,t,*iter);

	ret = getRecord(rbfRecord,rpm1000,t);
	globalRecord.clear();
	return ret;
}

map<int,int> AVRTask::calRequestBoundFunction(double omega, int t, vector<OmegaPoint*>& points) {
	map<int,int> ret;
	if (t <= 0) return ret;

	vector<double> dominants;
	int rpm = Utility::round(RPmSEC_to_HundredRPM(omega));

	ret = getRecord(rbfRecord,rpm,t);
	if (!ret.empty()) return ret;

	if (dominantsRecord.find(rpm) != dominantsRecord.end()) {
		dominants = dominantsRecord[rpm];
	}
	else {
		double omega_a = engine.getHigherSpeed(omega,period,1);
		double omega_b = engine.getLowerSpeed(omega,period,1);
		if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
		if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

		dominants = getDominants(omega_b,omega_a);
		dominantsRecord[rpm] = dominants;
	}

	int wcet = getWCET(omega);
	vector<map<int,int>> globalRecord;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		double omega_next = *iter;
		int rpm_next = Utility::round(RPmSEC_to_HundredRPM(omega_next));
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period); // ms
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period); // ms
#endif
		int tNext = Utility::round(RPmSEC_to_HundredRPM(time)); // us

		map<int,int> points = getRecord(rbfRecord,rpm_next,t-tNext);
		if (points.empty()) {
			points = calRequestBoundFunction(omega_next,t-tNext);
		} 

		map<int,int> newPoints = createRBFPoints(points,tNext,wcet);
		globalRecord.push_back(newPoints);
		
	}
	for (vector<map<int,int>>::iterator iter = globalRecord.begin(); iter != globalRecord.end(); iter++)
		storeRBFRecord(rbfRecord,rpm,t,*iter);

	ret = getRecord(rbfRecord,rpm,t);
	globalRecord.clear();
	return ret;
}

map<int,int> AVRTask::calInterferenceBoundFunction(double omega, int t) {
	map<int,int> ret;
	if (t <= 0) return ret;

	vector<double> dominants;
	int rpm = Utility::round(RPmSEC_to_HundredRPM(omega));

	ret = getRecord(ibfRecord,rpm,t);
	if (!ret.empty()) return ret;

	if (dominantsRecord.find(rpm) != dominantsRecord.end()) {
		dominants = dominantsRecord[rpm];
	}
	else {
		double omega_a = engine.getHigherSpeed(omega,period,1);
		double omega_b = engine.getLowerSpeed(omega,period,1);
		if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
		if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

		dominants = getDominants(omega_b,omega_a);
		dominantsRecord[rpm] = dominants;
	}

	int wcet = getWCET(omega);
	vector<map<int,int>> globalRecord;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		double omega_next = *iter;
		int rpm_next = Utility::round(RPmSEC_to_HundredRPM(omega_next));
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period); // ms
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period); // ms
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time)); // us

		map<int,int> points = getRecord(ibfRecord,rpm_next,t-tNext);
		if (points.empty()) {
			points = calInterferenceBoundFunction(omega_next,t-tNext);
		} 

		map<int,int> newPoints = createIBFPoints(points,tNext,wcet);
		globalRecord.push_back(newPoints);
		
	}
	for (vector<map<int,int>>::iterator iter = globalRecord.begin(); iter != globalRecord.end(); iter++)
		storeIBFRecord(ibfRecord,rpm,t,*iter);

	ret = getRecord(ibfRecord,rpm,t);
	globalRecord.clear();
	return ret;
}

map<int,int> AVRTask::getInterferenceBoundFunction(double omega, map<int,int> points, int t) {
	int rpm = Utility::round(RPmSEC_to_HundredRPM(omega));
	if (ibfs.find(rpm) != ibfs.end()) {
		map<int,map<int,int>> temp = ibfs[rpm];
		if (temp.find(t) != temp.end())
			return temp[t];
	}

	map<int,int> ret;

	for (map<int,int>::iterator iter = points.begin(); iter != points.end(); iter++) {
		int y = iter->second;
		map<int,int>::iterator next_Iter = iter;
		next_Iter++;
		int x;
		if (next_Iter == points.end()) {
			x = max(2*t,iter->first);
		}
		else {
			x = next_Iter->first - (next_Iter->second - iter->second);
		}

		ret[x] = y;
	}

	ret[0] = 0;

	ibfs[rpm][t] = ret;
	return ret;
}

int AVRTask::getInterferenceBoundFunction(map<int,int> record, int t) {
	if (t <= 0) return 0;
	if (record.empty()) {
		cerr << "Empty record!" << endl;
		exit(EXIT_FAILURE);
	}
	
	map<int,int>::iterator iter = record.lower_bound(t);
	if (iter != record.end()) {
		return iter->second;
	}
	else {
		iter--;
		return iter->second;
	}
}

double AVRTask::getLinearUpperBound(double t) {
	// should prepare _maxC and _maxU
	if (_maxC < 0 || _maxU < 0) {
		cerr << "Have not prepared _maxC and _maxU!" << endl;
		exit(EXIT_FAILURE);
	}

	return _maxU*t + _maxC*(1.0-_maxU);
}

double AVRTask::getResponseTimeWithWCET(double wcet) {
	// should prepare _maxC and _maxU
	if (_maxC < 0 || _maxU < 0) {
		cerr << "Have not prepared _maxC and _maxU!" << endl;
		exit(EXIT_FAILURE);
	}

	return (wcet+(1.0-_maxU)*_maxC)/(1.0-_maxU);
}


std::map<int,int> AVRTask::getModePeriods()
{
	map<int,int> _modePeriods;
	for (auto e:omegas) {
#ifdef __USING_CONSTANT_ACCELERATION__
		_modePeriods[e.first] = mSEC_to_muSEC(engine.getConstantPeriodWithConstantAcceleration(e.second,period));
#else
		_modePeriods[e.first] = mSEC_to_muSEC(engine.getConstantPeriodWithArbitraryAcceleration(e.second,period));
#endif
	}

	return _modePeriods;
}

std::map<int,double> AVRTask::getModeUtilizations()
{
	map<int,double> _modeUtilizations;
	for (auto e:omegas) {
#ifdef __USING_CONSTANT_ACCELERATION__
		_modeUtilizations[e.first] = (1.0*wcets[e.first]) / mSEC_to_muSEC(engine.getConstantPeriodWithConstantAcceleration(e.second,period));
#else
		_modeUtilizations[e.first] = (1.0*wcets[e.first]) / mSEC_to_muSEC(engine.getConstantPeriodWithArbitraryAcceleration(e.second,period));
#endif
	}

	return _modeUtilizations;
}

#ifdef __USING_ILPCPLEX__
int AVRTask::calILP(int t)
{
	if (ILPs.find(t) != ILPs.end()) {
		return ILPs[t];
		#ifdef __DEBUG_DESIGN__
		cout << "Existed ILP!" << endl;
		#endif
	}

	int ret = -1;

	// CPLEX environment. Take care of everything, including memory management for CPLEX objects
	IloEnv env;

	stringstream name;
	stringstream logfile;

	// CPLEX model
	IloModel model(env);

	// Model:
	// INTEGER VARIABLE 
	//    k[i] >= 0, i = 0, ..., numMode-1 
	//
	// OBJECTIVE FUNCTION
	//    MAX sum ( k[i] * C[i] )
	//
	// CONSTRAINTS
	//	  sum ( k[i] * T[i] ) \leq t + T_max - 1

	IloIntVarArray x(env,numMode);
	IloCplex cplex(model);
	IloExpr expr(env);

	// Set options of IP solver
	cplex.setParam(IloCplex::TiLim, 100.000);
	cplex.setParam(IloCplex::Threads, 1);
	cplex.setParam(IloCplex::EpGap, 0.0);
	cplex.setParam(IloCplex::EpAGap, 0.0);
	cplex.setOut(logfile);

	// Create variables x[0], ..., x[numMode-1]
	for (int i=0; i<numMode; i++) {
		name << "mode_" << i;
		//x[i] = IloNumVar(env ,0.0, IloInfinity, ILOINT,name.str().c_str());
		x[i] = IloIntVar(env ,0, IloIntMax,name.str().c_str());
		name.str(""); // Clean name
	}

	// Create constraint
	for (int i=0; i<numMode; i++) {
		expr += x[i] * modePeriods[i];
	}

	// Add constraint
	model.add(IloRange(env,expr,t+modePeriods[0]-1,"constraint"));
	expr.clear(); // Clean expr

	// Create objective function
	for (int i=0; i<numMode; i++) {
		expr += x[i] * wcets[i];
	}
	IloObjective obj(env,expr,IloObjective::Maximize);

	// Add the objective function
	model.add(obj);

	// Free the memory used by expr
	expr.end();

	try {
#if 0
		Timer timer;
		timer.start();
#endif 
		// Solve
		cplex.solve();

#if 0
		timer.end();
#endif

#if 0
		cout << "time:" <<timer.getTime() << endl;
#endif

#if 0
		string lpname = "Models"+Utility::linkNotation() + "model" + Utility::int_to_string(t) + ".lp";
		cplex.exportModel(lpname.c_str());
#endif

		if (cplex.getStatus() == IloAlgorithm::Optimal) {
			IloNumArray vals(env);
			ret = cplex.getObjValue();
#if 0
			cplex.getValues(vals, x);
			env.out() << "Values = " << vals << endl;
			cout << "Max=" << cplex.getObjValue() << ", Time = " << cplex.getTime() << endl;
#endif
		}
		//cplex.printTime();
	} catch (IloException& e) {
		cerr << "C-Exp: " << e << endl;
		exit(EXIT_FAILURE);
	} catch (...) {
		cerr << "Unknown Exception" << endl;
		exit(EXIT_FAILURE);
	}

#if 0
	const string str = logfile.str();
	cout << str << endl;
#endif

	env.end();

	if (ret < 0) {
		cerr << "We don't find the optimal solution." << endl;
		exit(EXIT_FAILURE);
	}
	else {
		ILPs[t] = ret;
		return ret;
	}
}
#endif

int AVRTask::calAJLessThan(double omega, int objMode, int t)
{
	double tMsec = muSEC_to_mSEC(t);
	double omega_min = omegas[objMode-1];
	double omega_max = omegas[objMode];

	int n_min = 0;
	int n_max = 0;
	int n = 0;

	while (true) {
		n++;
		double omega_curr = engine.getHigherSpeed(omega,period,n);
		if (omega_curr > omega_min && W_LEQ_TOL(omega_curr,omega_max) && n_min == 0) {
			n_min = n;
		}

		if (omega_curr > omega_min && W_LEQ_TOL(omega_curr,omega_max)) {
			n_max = n;
		}

		if (W_GEQ_TOL(omega_curr,omega_max) || W_EQ_TOL(omega_curr,engine.SPEED_MAX))
			break;
	}

	double ta = 0.0, tb = 0.0, tc = 0.0;
	double rho = 0.0;

	if (n_min == 0) {
		ta = 2.0*period / (omega+omega_max);
		tb = 0.0;
		tc = 0.0;

		if (tMsec <= ta) rho = 0.0;
		else rho = omega_max * (tMsec - ta); 

	}
	else {
		double omega_a = engine.getHigherSpeed(omega,period,n_min);
		ta = (omega_a-omega)/engine.ACCELERATION;

		double omega_b = engine.getHigherSpeed(omega,period,n_max);
		if (n_min == n_max) tb = 0.0;
		else tb = (omega_b-omega_a)/engine.ACCELERATION;

		if (W_EQ_TOL(omega_b,omega_max))
			tc = 0.0;
		else
			tc = 2.0*period / (omega_b+omega_max);

		if (tMsec <= ta) 
			rho = 0.0;
		else if (tMsec>ta && tMsec <= ta + tb)
			rho = 0.5 * engine.ACCELERATION * pow(tMsec-ta,2.0) + omega_a * (tMsec-ta);
		else if (tMsec>ta + tb && tMsec <= ta + tb + tc)
			rho = 0.5 * engine.ACCELERATION * pow(tb,2.0) + omega_a * tb
			    + 0.5 * (omega_max-omega_b)/tc * pow(tMsec - ta -tb, 2.0) + omega_b * (tMsec-ta-tb);
		else if (tMsec>ta+tb+tc) {
			rho = 0.5 * engine.ACCELERATION * pow(tb,2.0) + omega_a * tb
				+ 0.5 * (omega_max+omega_b) * tc
				+ omega_max * (tMsec - ta - tb - tc);
		}
		else {
			cerr << "Error: " << endl;
			cerr << "t = " << t <<  ", tMsec = " << tMsec << endl;
			cerr << "ta = " << ta << ", tb = " << tb << ", tc = " << tc << endl;
			exit(EXIT_FAILURE);
		}
	}

	return ceil(rho/period);
}

int AVRTask::calAJEqualTo(double omega, int objMode, int t)
{
	double tMsec = muSEC_to_mSEC(t);
	double omega_max = omegas[objMode];

	int n_max = 0;
	int n = 0;

	while (true) {
		n++;
		double omega_curr = engine.getHigherSpeed(omega,period,n);

		if (W_LEQ_TOL (omega_curr,omega_max)) {
			n_max = n;
		}

		if (W_GEQ_TOL(omega_curr,omega_max) || W_EQ_TOL(omega_curr,engine.SPEED_MAX))
			break;
	}

	double ta = 0.0, tb = 0.0;
	double rho = 0.0;

	if (n_max == 0) {
		ta = 2.0*period / (omega+omega_max);
		tb = 0.0;

		if (tMsec <= ta) rho = period;
		else rho = period + omega_max * (tMsec - ta); 

	}
	else {
		double omega_a = engine.getHigherSpeed(omega,period,n_max);
		ta = (omega_a-omega)/engine.ACCELERATION;

		if (W_EQ_TOL(omega_a,omega_max))
			tb = 0.0;
		else
			tb = 2.0*period / (omega_a+omega_max);

		if (tMsec <= ta) 
			rho = 0.5 * engine.ACCELERATION * pow(tMsec,2.0) + omega_a * tMsec;
		else if (tMsec>ta && tMsec <= ta + tb)
			rho = 0.5 * engine.ACCELERATION * pow(ta,2.0) + omega_a * ta
			    + 0.5 * (omega_max-omega_a)/tb * pow(tMsec - ta, 2.0) + omega_a * (tMsec-ta);
		else if (tMsec>ta+tb) {
			rho = 0.5 * engine.ACCELERATION * pow(ta,2.0) + omega_a * ta
				+ 0.5 * (omega_max+omega_a) * tb
				+ omega_max * (tMsec - ta - tb);
		}
		else {
			cerr << "Error: " << endl;
			cerr << "t = " << t <<  ", tMsec = " << tMsec << endl;
			cerr << "ta = " << ta << ", tb = " << tb << endl;
			exit(EXIT_FAILURE);
		}
	}

	return ceil(rho/period);
}

int AVRTask::calAJGreaterThan(double omega, int objMode, int t)
{
	double tMsec = muSEC_to_mSEC(t);
	double omega_max = omegas[objMode];

	int n_max = 0;
	int n = 0;

	while (true) {
		n++;
		double omega_curr = engine.getLowerSpeed(omega,period,n);

		if (W_GEQ_TOL (omega_curr,omega_max)) {
			n_max = n;
		}

		if (W_LEQ_TOL(omega_curr,omega_max))
			break;
	}

	double ta = 0.0, tb = 0.0;
	double rho = 0.0;

	if (n_max == 0) {
		ta = 2.0*period / (omega+omega_max);
		tb = 0.0;

		if (tMsec <= ta) rho = 0.0;
		else rho = omega_max * (tMsec - ta); 

	}
	else {
		double omega_a = engine.getLowerSpeed(omega,period,n_max);
		ta = (omega-omega_a)/engine.DECELERATION;

		if (W_EQ_TOL(omega_a,omega_max))
			tb = 0.0;
		else
			tb = 2.0*period / (omega_a+omega_max);

		if (tMsec <= ta + tb) 
			rho = 0.0;
		else if (tMsec > ta + tb)
			rho = omega_max * (tMsec - ta - tb);
		else {
			cerr << "Error: " << endl;
			cerr << "t = " << t <<  ", tMsec = " << tMsec << endl;
			cerr << "ta = " << ta << ", tb = " << tb << endl;
			exit(EXIT_FAILURE);
		}
	}

	return ceil(rho/period);
}

int AVRTask::calAMax(double omega, int t)
{
	double tMsec = muSEC_to_mSEC(t);
	double ta = min(tMsec,(engine.SPEED_MAX-omega)/engine.ACCELERATION);
	double tb = tMsec-ta;
	double rhoMax = engine.ACCELERATION * pow(ta,2.0) / 2 + omega * ta + engine.SPEED_MAX * tb;
	return ceil(rhoMax/period);
}

int AVRTask::calRhoMinus(int middleMode)
{
	return floor((pow(omegas[middleMode],2) - pow(omegas[middleMode-1],2))/(2.0*engine.DECELERATION*period));
}

int AVRTask::calRhoAdd(int middleMode)
{
	return floor((pow(omegas[middleMode],2) - pow(omegas[middleMode-1],2))/(2.0*engine.ACCELERATION*period));
}

#ifdef __USING_ILPCPLEX__
int AVRTask::calILPCON(double omega, int t)
{
	int recordOmega = RPmSEC_to_ThousandRPM(omega);

	if (ILPCONs.find(recordOmega) != ILPCONs.end()) {
		map<int,int> temp = ILPCONs[recordOmega];

		if (temp.find(t) != temp.end()) {
			//cout << "found!" << endl;
			return temp[t];
		}
	}

	int ret = -1;

	// CPLEX environment. Take care of everything, including memory management for CPLEX objects
	IloEnv env;

	stringstream name;
	stringstream logfile;

	// CPLEX model
	IloModel model(env);

	// Model:
	// INTEGER VARIABLE 
	//    k[i] >= 0, i = 0, ..., numMode-1 
	//
	// OBJECTIVE FUNCTION
	//    MAX sum ( k[i] * C[i] )
	//
	// CONSTRAINTS
	//   1) k_Z >= 1
	//   2) k_j \leq A_j(\omega,t)
	//   3) \sum k_i \leq A^{max}(\omega,t)
	//   4) 1<j<M, 
	//      (k_{j-1} \leq 0) \vee 
	//      (k_{j+1} \leq 0) \vee 
	//      (k_j \geq \lfloor \frac{\rho^{-}_j}{\beta} \rfloor) \vee
	//      (k_j \geq \lfloor \frac{\rho^{+}_j}{\beta} \rfloor) \vee
	//	 5) sum ( k[i] * T[i] ) \leq t + T_Z - 1

	IloNumVarArray x(env,numMode);
	IloBoolVarArray y(env,numMode);
	IloCplex cplex(model);
	IloExpr expr(env);
	IloExprArray max_exprs(env,numMode);

	// Set options of IP solver
	cplex.setParam(IloCplex::TiLim, 100.000);
	cplex.setParam(IloCplex::Threads, 1);
	cplex.setParam(IloCplex::EpGap, 0.0);
	cplex.setParam(IloCplex::EpAGap, 0.0);
	cplex.setOut(logfile);

	// Create variables x[0], ..., x[numMode-1]
	for (int j=0; j<numMode; j++) {
		name << "mode_" << j;
		x[j] = IloNumVar(env ,0.0, IloInfinity, ILOINT,name.str().c_str());
		name.str(""); // Clean name
	}

	int Z = getModeNum(omega);
	
	// Add constraint 1: k_Z >= 1
	model.add(IloRange(env,1,x[Z],IloInfinity,"constraint_1"));

	// Add constraint 2: k_j \leq A_j(\omega,t)
	for (int j=0; j<numMode; j++) {
		int AJ = 0;
		if ( j < Z)
			AJ = calAJGreaterThan(omega, j, t);
		else if ( j > Z)
			AJ = calAJLessThan(omega, j, t);
		else if ( j==Z)
			AJ = calAJEqualTo(omega, j, t);
		else {
			cerr << "Error mode Z = " << Z << endl;
			exit(EXIT_FAILURE);
		}
		name << "constraint_2_" << j;
		model.add(IloRange(env,x[j],AJ,name.str().c_str()));
		name.str(""); // Clean name
	}

	// Add constraint 3: \sum k_i \leq A^{max}(\omega,t)
	int AMax = calAMax(omega,t);
	name << "constraint_3_max";
	model.add(IloRange(env,IloSum(x), AMax,name.str().c_str()));
	name.str(""); // Clean name

	// Add constraint 4:
	// 1<j<M, 
	//      (k_{j-1} \leq 0) \vee 
	//      (k_{j+1} \leq 0) \vee 
	//      (k_j \geq \lfloor \frac{\rho^{-}_j}{\beta} \rfloor) \vee
	//      (k_j \geq \lfloor \frac{\rho^{+}_j}{\beta} \rfloor) \vee
	for (int j=1; j<numMode-1; j++) {
		name << "constraint_4_" << j;
		IloOr or(env,name.str().c_str());
		or.add(x[j-1] <= 0);
		or.add(x[j+1] <= 0);

		int rhoMinus = calRhoMinus(j);
		int rhoAdd = calRhoAdd(j);
		or.add(x[j] >= rhoMinus);
		or.add(x[j] >= rhoAdd);

		model.add(or);
		name.str(""); // Clean name
	}

	// Get the maximum inter-arrival time for speed omega
	// double Tomega = engine.getMaxInterArrivalTime(omega,period);

	// Create binary variables: y[0], ..., y[numMode-1]
	for (int j=0; j<numMode; j++) {
		name << "binary_" << j;
		//y[j] = IloNumVar(env,0,1,ILOINT, name.str().c_str());
		y[j] = IloBoolVar(env, name.str().c_str());
		name.str(""); // Clean name
	}

	// Add constraints : \forall j, k[j] > 0 <-> y[0] = 1
	for (int j=0; j<numMode; j++) {
		name << "constraint_tmax_" << j;
		model.add(x[j] > 0 == y[j]);
		name.str("");
	}

	// Create Tmax variable: Tmax
	IloNumVar Tmax(env ,0.0, modePeriods[0], ILOINT,"Tmax");
	for (int j=0; j<numMode; j++) {
		max_exprs[j] = y[j] * modePeriods[j];
	}
	model.add(Tmax == IloMax(max_exprs));

	//model.add(IloRange(env,0,expr,0, "constraint_tmax"));
	max_exprs.end();
	expr.clear();

	// Add constraint 5: sum ( k[i] * T[i] ) \leq t + T_Z - 1
	for (int j=0; j<numMode; j++) {
		expr += x[j] * modePeriods[j];
	}
	expr -= Tmax;

	//model.add(IloRange(env,expr,t+modePeriods[Z]-1,"constraint_5"));
	//model.add(IloRange(env,expr,t+mSEC_to_muSEC(Tomega)-1,"constraint_5"));
	model.add(IloRange(env,expr,t-1,"constraint_5"));
	expr.clear(); // Clean expr

	// Create objective function
	for (int i=0; i<numMode; i++) {
		expr += x[i] * wcets[i];
	}
	IloObjective obj(env,expr,IloObjective::Maximize);

	// Add the objective function
	model.add(obj);

	// Free the memory used by expr
	expr.end();

	try {
		// Solve
		cplex.solve();

#if 0
		cplex.exportModel("Models"+Utility::linkNotation()+"mymodel.lp");
#endif

		if (cplex.getStatus() == IloAlgorithm::Optimal) {
			ret = cplex.getObjValue();
#if 0
			IloNumArray valX(env);
			IloNumArray valY(env);

			cplex.getValues(valX, x);
			cplex.getValues(valY, y);
			env.out() << "X = " << valX << endl; 
			env.out() << "Y = " << valY << endl; 
			env.out() << "Tmax = " << cplex.getValue(Tmax) << endl; 
			env.out() << "Max=" << cplex.getObjValue() << ", Time = " << cplex.getTime() << endl;
#endif
		}
	} catch (IloException& e) {
		cerr << "C-Exp: " << e << endl;
		exit(EXIT_FAILURE);
	} catch (...) {
		cerr << "Unknown Exception" << endl;
		exit(EXIT_FAILURE);
	}

#if 0
	const string str = logfile.str();
	cout << str << endl;
#endif

	env.end();

	if (ret < 0) {
		cerr << "We don't find the optimal solution." << endl;
		exit(EXIT_FAILURE);
	}
	else {
		ILPCONs[recordOmega][t]  = ret;
		return ret;
	}
}
#endif

double AVRTask::calHighestSpeed(double omega, int t) {
	double time = 0;
	double omega_curr = omega;
	while (1) {
		double omega_next = engine.getHigherSpeed(omega_curr,period,1);
		if (omega_next > engine.SPEED_MAX) omega_next = engine.SPEED_MAX;

#ifdef __USING_CONSTANT_ACCELERATION__
		time += engine.getInterArrivalTimeWithConstantAcceleration(omega_curr,omega_next,period);
#else
		time += engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega_curr,omega_next,period);
#endif

		if (mSEC_to_muSEC(time) > t) {
			break;
		}

		omega_curr = omega_next;
	}
	return omega_curr;
}

double AVRTask::calLowestSpeed(double omega, int t) {
	double time = 0;
	double omega_curr = omega;
	//double omega_min = double(MODES.begin()->first)/60000;
	while (1) {
		double omega_next = engine.getLowerSpeed(omega_curr,period,1);
		//if (omega_next <  omega_min) omega_next = omega_min;
		if (omega_next <  engine.SPEED_MIN) omega_next = engine.SPEED_MIN;

#ifdef __USING_CONSTANT_ACCELERATION__
		time += engine.getInterArrivalTimeWithConstantAcceleration(omega_curr,omega_next,period);
#else
		time += engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega_curr,omega_next,period);
#endif

		if ( mSEC_to_muSEC(time) >= t) {
			break;
		}
		omega_curr = omega_next;
	}
	return omega_curr;
}

vector<map<int,int>> AVRTask::generateRequestFunctions(double omega, int t) {
	vector<map<int,int>> ret;
	if (t <= 0) return ret;

	vector<double> dominants;
	int rpm = Utility::round(RPmSEC_to_HundredRPM(omega));

	ret = getRecord(rfRecord,rpm,t);
	if (!ret.empty()) return ret;

	if (dominantsRecord.find(rpm) != dominantsRecord.end()) {
		dominants = dominantsRecord[rpm];
	}
	else {
		double omega_a = engine.getHigherSpeed(omega,period,1);
		double omega_b = engine.getLowerSpeed(omega,period,1);
		if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
		if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

		dominants = getDominants(omega_b,omega_a);
		//dominants = getDominants(omega_b,omega_a,t);
		dominantsRecord[rpm] = dominants;
	}

	int wcet = getWCET(omega);
	vector<vector<map<int,int>>> globalRecord;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++) {
		double omega_next = *iter;
		int rpm_next = Utility::round(RPmSEC_to_HundredRPM(omega_next));

#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period); // ms
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period); // ms
#endif

		int tNext = Utility::round(mSEC_to_muSEC(time)); // us

		vector<map<int,int>> points = getRecord(rfRecord,rpm_next,t-tNext);
		if (points.empty()) {
			points = generateRequestFunctions(omega_next,t-tNext);
		} 

		vector<map<int,int>> newPoints = createPoints(points,tNext,wcet);
		if (!checkExist(globalRecord,newPoints))
			globalRecord.push_back(newPoints);
	}
	storeRecord(rfRecord,rpm,t,globalRecord);

	ret = getRecord(rfRecord,rpm,t);
	return ret;
}


int AVRTask::getJobSequencesNumber(int maxTime)
{
	int sum = 0;
	//vector<double> dominants = getDominants(engine.SPEED_MIN,engine.SPEED_MAX,maxTime);
	vector<double> dominants = getAllDominants();
	for (auto omega: dominants) {
		int num = getJobSequencesNumber(maxTime,omega,0);
		sum += num;
	}
	return sum;
}

int AVRTask::getJobSequencesNumberNO1(int maxTime)
{
	int sum = 0;
	vector<double> dominants;
	for (auto e : omegas) {
		dominants.push_back(e.second);
	}

	for (auto omega: dominants) {
		int num = getJobSequencesNumber(maxTime,omega,0);
		sum += num;
	}
	return sum;
}


int AVRTask::getJobSequencesNumberNO2(int maxTime)
{
	if (maxTime <= 50*1000) return 6;

	int sum = 0;
	vector<double> dominants;
	for (auto e : omegas) {
		dominants.push_back(e.second);
	}

	for (auto omega: dominants) {
		int num = getJobSequencesNumber(maxTime-50*1000,omega,0);
		sum += num;
	}
	return sum;
}


int AVRTask::getJobSequencesNumberNO3(int maxTime)
{
	if (maxTime <= 50*1000) return 6;

	int sum = 0;
	vector<double> dominants;
	for (auto e : omegas) {
		dominants.push_back(e.second);
	}

	for (auto omega: dominants) {
		int num = getJobSequencesNumberBinary(maxTime-50*1000,omega,0);
		sum += num;
	}
	return sum;
}

int AVRTask::getJobSequencesNumber(int maxTime, double omega, int sumPeriod)
{
	int totalNum = 0;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = getDominants(omega_b,omega_a,maxTime-sumPeriod);
	for (auto omega_next : dominants) {
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));
		if (sumPeriod+tNext >= maxTime) continue;

		int tempNum = getJobSequencesNumber(maxTime,omega_next,sumPeriod+tNext);
		totalNum += tempNum; 
	}
	if (totalNum == 0) totalNum = 1;

	return totalNum;
}

int AVRTask::getJobSequencesNumberBinary(int maxTime, double omega, int sumPeriod)
{
	int totalNum = 0;

	double omega_a = engine.getHigherSpeed(omega,period,1);
	double omega_b = engine.getLowerSpeed(omega,period,1);

	if (omega_a > engine.SPEED_MAX) omega_a = engine.SPEED_MAX;
	if (omega_b < engine.SPEED_MIN) omega_b = engine.SPEED_MIN;

	//vector<double> dominants  = avrTask.getDominants(omega_b,omega_a,maxTime);
	vector<double> dominants  = getDominants(omega_b,omega_a,maxTime-sumPeriod);
	vector<double> dominants2;

	if (dominants.size() > 3) {
		dominants2.push_back(dominants.front());
		dominants2.push_back(dominants2.back());
	}
	else dominants2 = dominants;

	for (auto omega_next : dominants2) {
#ifdef __USING_CONSTANT_ACCELERATION__
		double time = engine.getInterArrivalTimeWithConstantAcceleration(omega,omega_next,period);
#else
		double time = engine.getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,period);
#endif
		int tNext = Utility::round(mSEC_to_muSEC(time));
		if (sumPeriod+tNext >= maxTime) continue;

		int tempNum = getJobSequencesNumberBinary(maxTime,omega_next,sumPeriod+tNext);
		totalNum += tempNum; 
	}
	if (totalNum == 0) totalNum = 1;

	return totalNum;
}

bool AVRTask::checkExist(vector<vector<map<int,int>>> record, vector<map<int,int>> points) {
	for (vector<vector<map<int,int>>>::iterator iter = record.begin(); iter != record.end(); iter++) {
		vector<map<int,int>> temp = *iter;
		if (temp.size() == points.size()) {
			int sum = 0;
			for (int i=0; i<temp.size(); i++) {
				map<int,int> tempA = temp[i];
				map<int,int> tempB = points[i];
				if (tempA.size() == tempB.size()) {
					map<int,int>::iterator iterA = tempA.begin();
					map<int,int>::iterator iterB = tempB.begin();
					while (iterA->first == iterB->first && iterA->second == iterB->second) {
						iterA++;
						iterB++;
						if (iterA == tempA.end()) {
							sum++;
							break;
						}
					}
				}
			}
			if (sum == temp.size()) return true;
		}

	}
	return false;
}

void AVRTask::output(ostream &out) {
	engine.output(out);
	out << "Angular Period = " << period << endl;
	out << "Modes = " << endl;

	for (int i =0;  i< numMode; i++) {
		out << speeds[i] << "=>" << wcets[i] << endl;
	}
}

void AVRTask::outputWCETs(ostream &out) {
	out << numMode << "\t";
	for (int i=0; i<numMode; i++) {
		out << wcets[i];
		if (i!=numMode-1)
			out << "\t";
	}
	out << endl;
}

void AVRTask::outputSpeeds(ostream &out) {
	out << numMode << "\t";
	for (int i=0; i<numMode; i++) {
		out << speeds[i];
		if (i!=numMode-1)
			out << "\t";
	}
	out << endl;
}

void AVRTask::outputDomiants(ostream &out, vector<double> dominants) {
	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++)
		out << int(*iter*60000) << ",";
	out << endl;

	for (vector<double>::iterator iter = dominants.begin(); iter != dominants.end(); iter++)
		out << *iter << ",";
	out << endl;
}

void AVRTask::outputRequestFunctions(ostream &out, double omega, int t) {
	vector<map<int,int>> ret = generateRequestFunctions(omega,t);

	for (int i=0; i < ret.size(); i++) {
		map<int,int> m1 = ret.at(i);
		Utility::output_one_map(out, "rbf"+Utility::int_to_string(i), m1,t);
		//Utility::output_one_map(out, "rbf"+Utility::int_to_string(i), m1);
	}

	for (int i=0; i< ret.size(); i++) {
		string index = Utility::int_to_string(i);
		out << "ax.step(rbf"+index+"X,rbf"+index+"Y,where = 'post',color=colors_["+index+"][0],label='RF" + index+ "')" << endl;
	}
}

void AVRTask::clear(map<int,map<int,map<int,int>>> &m) {
	for (map<int,map<int,map<int,int>>>::iterator iter = m.begin(); iter != m.end(); iter++) {
		map<int,map<int,int>>& m1 = iter->second;
		for (map<int,map<int,int>>::iterator iter1 = m1.begin(); iter1 != m1.end(); iter1++) {
			map<int,int>& m2 = iter1->second;
			m2.clear();
		}
		m1.clear();
	}
	m.clear();
}


void AVRTask::clearAll() {
	dominantsRecord.clear();
	clear(rbfRecord);
	clear(ibfRecord);
	clear(ibfs);
}