#include "OptimizationAlgorithm.h"
#include "SchedAnalysis.h"
#include "specialfunctions.h"
#include "Timer.h"
#include <gsl/gsl_integration.h>

#include <algorithm>
#include <math.h>
#include <functional>
#include <queue>
#include <numeric>
#include <fstream>

using namespace alglib;

string OptimizationAlgorithm::funcNames[19] = {
	"EXACT",
	"RBF",
	"RBF_ENVELOPE",
	"DIGRAPH_RBF",
	"DIGRAPH_RBF_ENVELOPE",
	"IBF",
	"NECESSARY_ONLY1",
	"SEG_NECESSARY_ONLY1",
	"SEG_LUB_NECESSARY_ONLY1",
	"NECESSARY_ONLY2",
	"NECESSARY_ONLY3",
	"NECESSARY_ONLY1A",
	"SEG_NECESSARY_ONLY1A",
	"NECESSARY_ONLY2A",
	"LUB",
	"ILPCON",
	"ILP",
	"SEG_ILPCON_EXACT",
	"SEG_ILPCON_DIGRAPH_RBF"
};

int OptimizationAlgorithm::nonSchedNum = 0;
int OptimizationAlgorithm::schedNum = 0;

double OptimizationAlgorithm::level0_LowerSpeeds_VerificationTimes=0;
double OptimizationAlgorithm::level0_LocalSearch_VerificationTimes=0;
double OptimizationAlgorithm::level1_LowerSpeeds_VerificationTimes=0;
double OptimizationAlgorithm::level1_LocalSearch_VerificationTimes=0;
double OptimizationAlgorithm::level2_LowerSpeeds_VerificationTimes=0;
double OptimizationAlgorithm::level2_LocalSearch_VerificationTimes=0;
double OptimizationAlgorithm::level3_LowerSpeeds_VerificationTimes=0;
double OptimizationAlgorithm::level3_LocalSearch_VerificationTimes=0;
double OptimizationAlgorithm::level4_LowerSpeeds_VerificationTimes=0;
double OptimizationAlgorithm::level4_LocalSearch_VerificationTimes=0;

double OptimizationAlgorithm::level0_LowerSpeeds_FinalCheckTimes=0;
double OptimizationAlgorithm::level1_LowerSpeeds_FinalCheckTimes=0;
double OptimizationAlgorithm::level2_LowerSpeeds_FinalCheckTimes=0;
double OptimizationAlgorithm::level3_LowerSpeeds_FinalCheckTimes=0;
double OptimizationAlgorithm::level4_LowerSpeeds_FinalCheckTimes=0;

vector<double> OptimizationAlgorithm::level0_LowerSpeeds_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level0_LocalSearch_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level1_LowerSpeeds_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level1_LocalSearch_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level2_LowerSpeeds_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level2_LocalSearch_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level3_LowerSpeeds_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level3_LocalSearch_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level4_LowerSpeeds_VerificationTimes_vec;
vector<double> OptimizationAlgorithm::level4_LocalSearch_VerificationTimes_vec;

vector<double> OptimizationAlgorithm::level0_LowerSpeeds_FinalCheckTimes_vec;
vector<double> OptimizationAlgorithm::level1_LowerSpeeds_FinalCheckTimes_vec;
vector<double> OptimizationAlgorithm::level2_LowerSpeeds_FinalCheckTimes_vec;
vector<double> OptimizationAlgorithm::level3_LowerSpeeds_FinalCheckTimes_vec;
vector<double> OptimizationAlgorithm::level4_LowerSpeeds_FinalCheckTimes_vec;

double OptimizationAlgorithm::level0_LowerSpeeds_Performance=0;
double OptimizationAlgorithm::level0_LocalSearch_Performance=0;
double OptimizationAlgorithm::level1_LowerSpeeds_Performance=0;
double OptimizationAlgorithm::level1_LocalSearch_Performance=0;
double OptimizationAlgorithm::level2_LowerSpeeds_Performance=0;
double OptimizationAlgorithm::level2_LocalSearch_Performance=0;
double OptimizationAlgorithm::level3_LowerSpeeds_Performance=0;
double OptimizationAlgorithm::level3_LocalSearch_Performance=0;
double OptimizationAlgorithm::level4_LowerSpeeds_Performance=0;
double OptimizationAlgorithm::level4_LocalSearch_Performance=0;

double OptimizationAlgorithm::level0_LowerSpeeds_FinalCheckPerf=0;
double OptimizationAlgorithm::level1_LowerSpeeds_FinalCheckPerf=0;
double OptimizationAlgorithm::level2_LowerSpeeds_FinalCheckPerf=0;
double OptimizationAlgorithm::level3_LowerSpeeds_FinalCheckPerf=0;
double OptimizationAlgorithm::level4_LowerSpeeds_FinalCheckPerf=0;

vector<double> OptimizationAlgorithm::level0_LowerSpeeds_Performance_vec;
vector<double> OptimizationAlgorithm::level0_LocalSearch_Performance_vec;
vector<double> OptimizationAlgorithm::level1_LowerSpeeds_Performance_vec;
vector<double> OptimizationAlgorithm::level1_LocalSearch_Performance_vec;
vector<double> OptimizationAlgorithm::level2_LowerSpeeds_Performance_vec;
vector<double> OptimizationAlgorithm::level2_LocalSearch_Performance_vec;
vector<double> OptimizationAlgorithm::level3_LowerSpeeds_Performance_vec;
vector<double> OptimizationAlgorithm::level3_LocalSearch_Performance_vec;
vector<double> OptimizationAlgorithm::level4_LowerSpeeds_Performance_vec;
vector<double> OptimizationAlgorithm::level4_LocalSearch_Performance_vec;

vector<double> OptimizationAlgorithm::level0_LowerSpeeds_FinalCheckPerf_vec;
vector<double> OptimizationAlgorithm::level1_LowerSpeeds_FinalCheckPerf_vec;
vector<double> OptimizationAlgorithm::level2_LowerSpeeds_FinalCheckPerf_vec;
vector<double> OptimizationAlgorithm::level3_LowerSpeeds_FinalCheckPerf_vec;
vector<double> OptimizationAlgorithm::level4_LowerSpeeds_FinalCheckPerf_vec;

double OptimizationAlgorithm::level0_LowerSpeeds_Runtime=0;
double OptimizationAlgorithm::level0_LocalSearch_Runtime=0;
double OptimizationAlgorithm::level1_LowerSpeeds_Runtime=0;
double OptimizationAlgorithm::level1_LocalSearch_Runtime=0;
double OptimizationAlgorithm::level2_LowerSpeeds_Runtime=0;
double OptimizationAlgorithm::level2_LocalSearch_Runtime=0;
double OptimizationAlgorithm::level3_LowerSpeeds_Runtime=0;
double OptimizationAlgorithm::level3_LocalSearch_Runtime=0;
double OptimizationAlgorithm::level4_LowerSpeeds_Runtime=0;
double OptimizationAlgorithm::level4_LocalSearch_Runtime=0;

vector<double> OptimizationAlgorithm::level0_LowerSpeeds_Runtime_vec;
vector<double> OptimizationAlgorithm::level0_LocalSearch_Runtime_vec;
vector<double> OptimizationAlgorithm::level1_LowerSpeeds_Runtime_vec;
vector<double> OptimizationAlgorithm::level1_LocalSearch_Runtime_vec;
vector<double> OptimizationAlgorithm::level2_LowerSpeeds_Runtime_vec;
vector<double> OptimizationAlgorithm::level2_LocalSearch_Runtime_vec;
vector<double> OptimizationAlgorithm::level3_LowerSpeeds_Runtime_vec;
vector<double> OptimizationAlgorithm::level3_LocalSearch_Runtime_vec;
vector<double> OptimizationAlgorithm::level4_LowerSpeeds_Runtime_vec;
vector<double> OptimizationAlgorithm::level4_LocalSearch_Runtime_vec;

void OptimizationAlgorithm::save_results() {
	level0_LowerSpeeds_VerificationTimes_vec.push_back(level0_LowerSpeeds_VerificationTimes);
	level0_LocalSearch_VerificationTimes_vec.push_back(level0_LocalSearch_VerificationTimes);
	level1_LowerSpeeds_VerificationTimes_vec.push_back(level1_LowerSpeeds_VerificationTimes);
	level1_LocalSearch_VerificationTimes_vec.push_back(level1_LocalSearch_VerificationTimes);
	level2_LowerSpeeds_VerificationTimes_vec.push_back(level2_LowerSpeeds_VerificationTimes);
	level2_LocalSearch_VerificationTimes_vec.push_back(level2_LocalSearch_VerificationTimes);
	level3_LowerSpeeds_VerificationTimes_vec.push_back(level3_LowerSpeeds_VerificationTimes);
	level3_LocalSearch_VerificationTimes_vec.push_back(level3_LocalSearch_VerificationTimes);
	level4_LowerSpeeds_VerificationTimes_vec.push_back(level4_LowerSpeeds_VerificationTimes);
	level4_LocalSearch_VerificationTimes_vec.push_back(level4_LocalSearch_VerificationTimes);

	level0_LowerSpeeds_FinalCheckTimes_vec.push_back(level0_LowerSpeeds_FinalCheckTimes);
	level1_LowerSpeeds_FinalCheckTimes_vec.push_back(level1_LowerSpeeds_FinalCheckTimes);
	level2_LowerSpeeds_FinalCheckTimes_vec.push_back(level2_LowerSpeeds_FinalCheckTimes);
	level3_LowerSpeeds_FinalCheckTimes_vec.push_back(level3_LowerSpeeds_FinalCheckTimes);
	level4_LowerSpeeds_FinalCheckTimes_vec.push_back(level4_LowerSpeeds_FinalCheckTimes);

	level0_LowerSpeeds_Performance_vec.push_back(level0_LowerSpeeds_Performance);
	level0_LocalSearch_Performance_vec.push_back(level0_LocalSearch_Performance);
	level1_LowerSpeeds_Performance_vec.push_back(level1_LowerSpeeds_Performance);
	level1_LocalSearch_Performance_vec.push_back(level1_LocalSearch_Performance);
	level2_LowerSpeeds_Performance_vec.push_back(level2_LowerSpeeds_Performance);
	level2_LocalSearch_Performance_vec.push_back(level2_LocalSearch_Performance);
	level3_LowerSpeeds_Performance_vec.push_back(level3_LowerSpeeds_Performance);
	level3_LocalSearch_Performance_vec.push_back(level3_LocalSearch_Performance);
	level4_LowerSpeeds_Performance_vec.push_back(level4_LowerSpeeds_Performance);
	level4_LocalSearch_Performance_vec.push_back(level4_LocalSearch_Performance);

	level0_LowerSpeeds_FinalCheckPerf_vec.push_back(level0_LowerSpeeds_FinalCheckPerf);
	level1_LowerSpeeds_FinalCheckPerf_vec.push_back(level1_LowerSpeeds_FinalCheckPerf);
	level2_LowerSpeeds_FinalCheckPerf_vec.push_back(level2_LowerSpeeds_FinalCheckPerf);
	level3_LowerSpeeds_FinalCheckPerf_vec.push_back(level3_LowerSpeeds_FinalCheckPerf);
	level4_LowerSpeeds_FinalCheckPerf_vec.push_back(level4_LowerSpeeds_FinalCheckPerf);

	level0_LowerSpeeds_Runtime_vec.push_back(level0_LowerSpeeds_Runtime);
	level0_LocalSearch_Runtime_vec.push_back(level0_LocalSearch_Runtime);
	level1_LowerSpeeds_Runtime_vec.push_back(level1_LowerSpeeds_Runtime);
	level1_LocalSearch_Runtime_vec.push_back(level1_LocalSearch_Runtime);
	level2_LowerSpeeds_Runtime_vec.push_back(level2_LowerSpeeds_Runtime);
	level2_LocalSearch_Runtime_vec.push_back(level2_LocalSearch_Runtime);
	level3_LowerSpeeds_Runtime_vec.push_back(level3_LowerSpeeds_Runtime);
	level3_LocalSearch_Runtime_vec.push_back(level3_LocalSearch_Runtime);
	level4_LowerSpeeds_Runtime_vec.push_back(level4_LowerSpeeds_Runtime);
	level4_LocalSearch_Runtime_vec.push_back(level4_LocalSearch_Runtime);
}

void OptimizationAlgorithm::set_zero() {
	level0_LowerSpeeds_VerificationTimes=0;
	level0_LocalSearch_VerificationTimes=0;
	level1_LowerSpeeds_VerificationTimes=0;
	level1_LocalSearch_VerificationTimes=0;
	level2_LowerSpeeds_VerificationTimes=0;
	level2_LocalSearch_VerificationTimes=0;
	level3_LowerSpeeds_VerificationTimes=0;
	level3_LocalSearch_VerificationTimes=0;
	level4_LowerSpeeds_VerificationTimes=0;
	level4_LocalSearch_VerificationTimes=0;

	level0_LowerSpeeds_FinalCheckTimes=0;
	level1_LowerSpeeds_FinalCheckTimes=0;
	level2_LowerSpeeds_FinalCheckTimes=0;
	level3_LowerSpeeds_FinalCheckTimes=0;
	level4_LowerSpeeds_FinalCheckTimes=0;

	level0_LowerSpeeds_Performance=0;
	level0_LocalSearch_Performance=0;
	level1_LowerSpeeds_Performance=0;
	level1_LocalSearch_Performance=0;
	level2_LowerSpeeds_Performance=0;
	level2_LocalSearch_Performance=0;
	level3_LowerSpeeds_Performance=0;
	level3_LocalSearch_Performance=0;
	level4_LowerSpeeds_Performance=0;
	level4_LocalSearch_Performance=0;

	level0_LowerSpeeds_FinalCheckPerf=0;
	level1_LowerSpeeds_FinalCheckPerf=0;
	level2_LowerSpeeds_FinalCheckPerf=0;
	level3_LowerSpeeds_FinalCheckPerf=0;
	level4_LowerSpeeds_FinalCheckPerf=0;

	level0_LowerSpeeds_Runtime=0;
	level0_LocalSearch_Runtime=0;
	level1_LowerSpeeds_Runtime=0;
	level1_LocalSearch_Runtime=0;
	level2_LowerSpeeds_Runtime=0;
	level2_LocalSearch_Runtime=0;
	level3_LowerSpeeds_Runtime=0;
	level3_LocalSearch_Runtime=0;
	level4_LowerSpeeds_Runtime=0;
	level4_LocalSearch_Runtime=0;
}

void OptimizationAlgorithm::set_scale(int scale) {
	level0_LowerSpeeds_VerificationTimes/=scale;
	level0_LocalSearch_VerificationTimes/=scale;
	level1_LowerSpeeds_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level1_LocalSearch_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level2_LowerSpeeds_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level2_LocalSearch_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level3_LowerSpeeds_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level3_LocalSearch_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level4_LowerSpeeds_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);
	level4_LocalSearch_VerificationTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);

	level0_LowerSpeeds_Performance/=scale;
	level0_LocalSearch_Performance/=scale;
	level1_LowerSpeeds_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level1_LocalSearch_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level2_LowerSpeeds_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level2_LocalSearch_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level3_LowerSpeeds_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level3_LocalSearch_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level4_LowerSpeeds_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);
	level4_LocalSearch_Performance/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);

	level0_LowerSpeeds_FinalCheckPerf/=level0_LowerSpeeds_FinalCheckTimes;                                                                                                                                              
	level1_LowerSpeeds_FinalCheckPerf/=level1_LowerSpeeds_FinalCheckTimes;
	level2_LowerSpeeds_FinalCheckPerf/=level2_LowerSpeeds_FinalCheckTimes;
	level3_LowerSpeeds_FinalCheckPerf/=level3_LowerSpeeds_FinalCheckTimes;
	level4_LowerSpeeds_FinalCheckPerf/=level4_LowerSpeeds_FinalCheckTimes;

	level0_LowerSpeeds_Runtime/=scale;
	level0_LocalSearch_Runtime/=scale;
	level1_LowerSpeeds_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level1_LocalSearch_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level2_LowerSpeeds_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level2_LocalSearch_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level3_LowerSpeeds_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level3_LocalSearch_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level4_LowerSpeeds_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);
	level4_LocalSearch_Runtime/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);

	level0_LowerSpeeds_FinalCheckTimes/=scale;
	level1_LowerSpeeds_FinalCheckTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes);
	level2_LowerSpeeds_FinalCheckTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes);
	level3_LowerSpeeds_FinalCheckTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes);
	level4_LowerSpeeds_FinalCheckTimes/=(scale-level0_LowerSpeeds_FinalCheckTimes-level1_LowerSpeeds_FinalCheckTimes-level2_LowerSpeeds_FinalCheckTimes-level3_LowerSpeeds_FinalCheckTimes);

}

void OptimizationAlgorithm::reset() {
	save_results();
	set_zero();
}

void OptimizationAlgorithm::outputCollectInfo(ostream& out) {
	Utility::output_one_vector(out,"level0_LowerSpeeds_VerificationTimes_vec",level0_LowerSpeeds_VerificationTimes_vec);
	Utility::output_one_vector(out,"level0_LocalSearch_VerificationTimes_vec",level0_LocalSearch_VerificationTimes_vec);
	Utility::output_one_vector(out,"level1_LowerSpeeds_VerificationTimes_vec",level1_LowerSpeeds_VerificationTimes_vec);
	Utility::output_one_vector(out,"level1_LocalSearch_VerificationTimes_vec",level1_LocalSearch_VerificationTimes_vec);
	Utility::output_one_vector(out,"level2_LowerSpeeds_VerificationTimes_vec",level2_LowerSpeeds_VerificationTimes_vec);
	Utility::output_one_vector(out,"level2_LocalSearch_VerificationTimes_vec",level2_LocalSearch_VerificationTimes_vec);
	Utility::output_one_vector(out,"level3_LowerSpeeds_VerificationTimes_vec",level3_LowerSpeeds_VerificationTimes_vec);
	Utility::output_one_vector(out,"level3_LocalSearch_VerificationTimes_vec",level3_LocalSearch_VerificationTimes_vec);
	Utility::output_one_vector(out,"level4_LowerSpeeds_VerificationTimes_vec",level4_LowerSpeeds_VerificationTimes_vec);
	Utility::output_one_vector(out,"level4_LocalSearch_VerificationTimes_vec",level4_LocalSearch_VerificationTimes_vec);

	Utility::output_one_vector(out,"level0_LowerSpeeds_FinalCheckTimes_vec",level0_LowerSpeeds_FinalCheckTimes_vec);
	Utility::output_one_vector(out,"level1_LowerSpeeds_FinalCheckTimes_vec",level1_LowerSpeeds_FinalCheckTimes_vec);
	Utility::output_one_vector(out,"level2_LowerSpeeds_FinalCheckTimes_vec",level2_LowerSpeeds_FinalCheckTimes_vec);
	Utility::output_one_vector(out,"level3_LowerSpeeds_FinalCheckTimes_vec",level3_LowerSpeeds_FinalCheckTimes_vec);
	Utility::output_one_vector(out,"level4_LowerSpeeds_FinalCheckTimes_vec",level4_LowerSpeeds_FinalCheckTimes_vec);

	Utility::output_one_vector(out,"level0_LowerSpeeds_Performance_vec",level0_LowerSpeeds_Performance_vec);
	Utility::output_one_vector(out,"level0_LocalSearch_Performance_vec",level0_LocalSearch_Performance_vec);
	Utility::output_one_vector(out,"level1_LowerSpeeds_Performance_vec",level1_LowerSpeeds_Performance_vec);
	Utility::output_one_vector(out,"level1_LocalSearch_Performance_vec",level1_LocalSearch_Performance_vec);
	Utility::output_one_vector(out,"level2_LowerSpeeds_Performance_vec",level2_LowerSpeeds_Performance_vec);
	Utility::output_one_vector(out,"level2_LocalSearch_Performance_vec",level2_LocalSearch_Performance_vec);
	Utility::output_one_vector(out,"level3_LowerSpeeds_Performance_vec",level3_LowerSpeeds_Performance_vec);
	Utility::output_one_vector(out,"level3_LocalSearch_Performance_vec",level3_LocalSearch_Performance_vec);
	Utility::output_one_vector(out,"level4_LowerSpeeds_Performance_vec",level4_LowerSpeeds_Performance_vec);
	Utility::output_one_vector(out,"level4_LocalSearch_Performance_vec",level4_LocalSearch_Performance_vec);

	Utility::output_one_vector(out,"level0_LowerSpeeds_FinalCheckPerf_vec",level0_LowerSpeeds_FinalCheckPerf_vec);
	Utility::output_one_vector(out,"level1_LowerSpeeds_FinalCheckPerf_vec",level1_LowerSpeeds_FinalCheckPerf_vec);
	Utility::output_one_vector(out,"level2_LowerSpeeds_FinalCheckPerf_vec",level2_LowerSpeeds_FinalCheckPerf_vec);
	Utility::output_one_vector(out,"level3_LowerSpeeds_FinalCheckPerf_vec",level3_LowerSpeeds_FinalCheckPerf_vec);
	Utility::output_one_vector(out,"level4_LowerSpeeds_FinalCheckPerf_vec",level4_LowerSpeeds_FinalCheckPerf_vec);

	Utility::output_one_vector(out,"level0_LowerSpeeds_Runtime_vec",level0_LowerSpeeds_Runtime_vec);
	Utility::output_one_vector(out,"level0_LocalSearch_Runtime_vec",level0_LocalSearch_Runtime_vec);
	Utility::output_one_vector(out,"level1_LowerSpeeds_Runtime_vec",level1_LowerSpeeds_Runtime_vec);
	Utility::output_one_vector(out,"level1_LocalSearch_Runtime_vec",level1_LocalSearch_Runtime_vec);
	Utility::output_one_vector(out,"level2_LowerSpeeds_Runtime_vec",level2_LowerSpeeds_Runtime_vec);
	Utility::output_one_vector(out,"level2_LocalSearch_Runtime_vec",level2_LocalSearch_Runtime_vec);
	Utility::output_one_vector(out,"level3_LowerSpeeds_Runtime_vec",level3_LowerSpeeds_Runtime_vec);
	Utility::output_one_vector(out,"level3_LocalSearch_Runtime_vec",level3_LocalSearch_Runtime_vec);
	Utility::output_one_vector(out,"level4_LowerSpeeds_Runtime_vec",level4_LowerSpeeds_Runtime_vec);
	Utility::output_one_vector(out,"level4_LocalSearch_Runtime_vec",level4_LocalSearch_Runtime_vec);
}

bool OptimizationAlgorithm::performanceCoeffsComparator(const pair<int,double>& p1, const pair<int,double>& p2)
{
	return p1.second > p2.second;
}

bool OptimizationAlgorithm::performanceCoeffsComparatorMin(const pair<int,double>& p1, const pair<int,double>& p2)
{
	return p1.second < p2.second;
}

double OptimizationAlgorithm::getPerformance(map<int,double> speeds_map, vector<double> k, Engine engine) {
	vector<double> speeds;
	for (auto it = speeds_map.begin(); it != speeds_map.end(); it++) {
		speeds.push_back(it->second);
	}
	return getPerformance(speeds,k,engine);
}

double OptimizationAlgorithm::getPerformance(vector<double> speeds, vector<double> k, Engine engine) {
	double perf = 0.0;
	for (int i=0; i< speeds.size(); i++) {
		perf += getPerformanceIndex(speeds,k,engine,i);
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceIndex(vector<double> speeds, vector<double> k, Engine engine, int index) {
	if (index==0) {
		return k[index]*(speeds[0]-engine.RPM_MIN);
	}
	else 
		return k[index]*(speeds[index]-speeds[index-1]);
}

double OptimizationAlgorithm::getPerformanceMax(vector<double> speeds, vector<double> k, Engine engine) {
	double perf = 0.0;
	for (int i=0; i< speeds.size(); i++) {
		double perfi = getPerformanceIndex(speeds,k,engine,i);
		perf = max(perf,perfi);
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceMin(vector<double> speeds, vector<double> k, Engine engine) {
	double perf = INT_MAX;
	for (int i=0; i< speeds.size(); i++) {
		double perfi = getPerformanceIndex(speeds,k,engine,i);
		perf = min(perf,perfi);
	}
	return perf;
}

double OptimizationAlgorithm::integral(double k1, double k2, double speed) {
#ifdef __USING_BIONDI_EXP_SETTING__
	double w = RPM_to_W(speed);
#else
	double w = speed;
#endif
	return k1*k2*exponentialintegralei(-k2/w) + w*k1*exp(-k2/w);
}

double OptimizationAlgorithm::getPerformanceExp(vector<double> speeds, vector<double> k1, vector<double> k2, Engine engine) {
	double perf = 0.0;
	int size = speeds.size();
	//vector<double> omegas; // in RPmSEC
	perf += integral(k1[size-1],k2[size-1],engine.RPM_MAX) - integral(k1[0],k2[0],engine.RPM_MIN);

	double test = integral(k1[0],k2[0],engine.RPM_MAX)-integral(k1[0],k2[0],engine.RPM_MIN);

	for (int i=0; i < size-1; i++) 
		perf += integral(k1[i],k2[i],speeds[i]) - integral(k1[i+1],k2[i+1],speeds[i+1]);

	return perf;
}

double OptimizationAlgorithm::getPerformanceExp(map<int, double> speeds_map, vector<double> k1, vector<double> k2, Engine engine) {
	vector<double> speeds;
	for (auto it = speeds_map.begin(); it != speeds_map.end(); it++) {
		speeds.push_back(it->second);
	}
	return getPerformanceExp(speeds,k1,k2,engine);
}

double OptimizationAlgorithm::integralExpGsl(double x, void * params) {
	struct my_two_params * fp = (struct my_two_params *) params;
	double k1 = fp->k1;
	double k2 = fp->k2;

	double f = k1 * exp(-k2/x);
	return f;
}

double OptimizationAlgorithm::getPerformanceExpGsl(double k1, double k2, double a, double b) {
	gsl_integration_workspace * w = gsl_integration_workspace_alloc (1000);
	double result, error;
	struct my_two_params params;
	params.k1 = k1;
	params.k2 = k2;

	gsl_function F;
	F.function = &integralExpGsl;
	F.params = &params;
	gsl_integration_qags (&F, a, b, 0, 1e-7, 1000, w, &result, &error);

#ifdef ____DEBUG_DESIGN__
	cout << "result = " << result;
	cout << "estimated error = " << error;
	cout << "intervals = " << w->size;
#endif

	gsl_integration_workspace_free (w);
	return result;
}

double OptimizationAlgorithm::getPerformanceExpGsl(vector<double> speeds, vector<double> k1, vector<double> k2, Engine engine) {
	double perf = 0.0;
	for (int i=0; i<speeds.size(); i++) {
		if (i==0) {
			#ifdef __USING_BIONDI_EXP_SETTING__
				perf += getPerformanceExpGsl(k1[i],k2[i],RPM_to_W(engine.RPM_MIN),RPM_to_W(speeds[i]));
			#else
				perf += getPerformanceExpGsl(k1[i],k2[i],engine.RPM_MIN,speeds[i]);
			#endif
		}
		else {
			#ifdef __USING_BIONDI_EXP_SETTING__
				perf += getPerformanceExpGsl(k1[i],k2[i],RPM_to_W(speeds[i-1]),RPM_to_W(speeds[i]));
			#else
				perf += getPerformanceExpGsl(k1[i],k2[i],speeds[i-1],speeds[i]);
			#endif
		}
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceExpGsl(map<int, double> speeds_map, vector<double> k1, vector<double> k2, Engine engine) {
	vector<double> speeds;
	for (auto e: speeds_map)
		speeds.push_back(e.second);
	return getPerformanceExpGsl(speeds,k1,k2,engine);
}

double OptimizationAlgorithm::integralPoly3(double x, void * params) {
	struct my_three_params * fp = (struct my_three_params *) params;
	double k1 = fp->k1;
	double k2 = fp->k2;
	double k3 = fp->k3;

	double f = k1*x*x + k2*x + k3;
	return f;
}

double OptimizationAlgorithm::getPerformancePoly(double k1, double k2, double k3, double a, double b) {
	gsl_integration_workspace * w = gsl_integration_workspace_alloc (1000);
	double result, error;
	struct my_three_params params;
	params.k1 = k1;
	params.k2 = k2;
	params.k3 = k3;

	gsl_function F;
	F.function = &integralPoly3;
	F.params = &params;
	gsl_integration_qags (&F, a, b, 0, 1e-7, 1000, w, &result, &error);

#ifdef ____DEBUG_DESIGN__
	cout << "result = " << result;
	cout << "estimated error = " << error;
	cout << "intervals = " << w->size;
#endif

	gsl_integration_workspace_free (w);
	return result;
}

double OptimizationAlgorithm::getPerformancePoly(vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3, Engine engine) {
	double perf = 0.0;
	for (int i=0; i<speeds.size(); i++) {
		if (i==0) 
			perf += getPerformancePoly(k1[i],k2[i],k3[i],engine.RPM_MIN,speeds[i]);
		else
			perf += getPerformancePoly(k1[i],k2[i],k3[i],speeds[i-1],speeds[i]);
	}
	return perf;
}

double OptimizationAlgorithm::getPerformancePoly(map<int,double> speeds_map, vector<double> k1, vector<double> k2, vector<double> k3, Engine engine) {
	vector<double> speeds;
	for (auto e: speeds_map)
		speeds.push_back(e.second);
	return getPerformancePoly(speeds,k1,k2,k3,engine);
}

double OptimizationAlgorithm::integralPoly4(double x, void * params) {
	struct my_four_params * fp = (struct my_four_params *) params;
	double k1 = fp->k1;
	double k2 = fp->k2;
	double k3 = fp->k3;
	double k4 = fp->k4;

	double f = k1*x*x + k2*x + k3*TORQUE*x + k4;
	return f;
}

double OptimizationAlgorithm::getPerformancePoly(double k1, double k2, double k3, double k4, double a, double b) {
	gsl_integration_workspace * w = gsl_integration_workspace_alloc (1000);
	double result, error;
	struct my_four_params params;
	params.k1 = k1;
	params.k2 = k2;
	params.k3 = k3;
	params.k4 = k4;

	gsl_function F;
	F.function = &integralPoly4;
	F.params = &params;
	gsl_integration_qags (&F, a, b, 0, 1e-7, 1000, w, &result, &error);

#ifdef ____DEBUG_DESIGN__
	cout << "result = " << result;
	cout << "estimated error = " << error;
	cout << "intervals = " << w->size;
#endif

	gsl_integration_workspace_free (w);
	return result;
}

double OptimizationAlgorithm::getPerformancePoly(vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, Engine engine) {
	double perf = 0.0;
	for (int i=0; i<speeds.size(); i++) {
		if (i==0) 
			perf += getPerformancePoly(k1[i],k2[i],k3[i],k4[i],engine.RPM_MIN,speeds[i]);
		else
			perf += getPerformancePoly(k1[i],k2[i],k3[i],k4[i],speeds[i-1],speeds[i]);
	}
	return perf;
}

double OptimizationAlgorithm::getPerformancePoly(map<int,double> speeds_map, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, Engine engine) {
	vector<double> speeds;
	for (auto e: speeds_map)
		speeds.push_back(e.second);
	return getPerformancePoly(speeds,k1,k2,k3,k4,engine);
}

vector<double> OptimizationAlgorithm::getPerformanceDerivatives(vector<double> k) {
	vector<double> ret;
	for (unsigned int i = 0; i < k.size(); i++) {
		if (i==k.size()-1) ret.push_back(k[i]);
		else ret.push_back(k[i]-k[i+1]);
	}
	return ret;
}

double OptimizationAlgorithm::getPerformanceDerivativeIndex(vector<double> k, int index) {
	if (index==k.size()-1) {
		return k[index];
	}
	else 
		return k[index]-k[index+1];
}

double OptimizationAlgorithm::getPerformanceDerivativeMax(vector<double> k) {
	double perf = 0.0;
	for (int i=0; i< k.size(); i++) {
		double perfi = getPerformanceDerivativeIndex(k,i);
		perf = max(perf,perfi);
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDerivativeMin(vector<double> k) {
	double perf = INT_MAX;
	for (int i=0; i< k.size(); i++) {
		double perfi = getPerformanceDerivativeIndex(k,i);
		perf = min(perf,perfi);
	}
	return perf;
}

vector<double> OptimizationAlgorithm::getPerformanceDerivativesExp(vector<double> k1, vector<double> k2, vector<double> speeds) {
	vector<double> gradientCoeffs;

	for (int i=0; i <k1.size()-1; i++) {
		if (speeds[i] > 0) {
			#ifdef __USING_BIONDI_EXP_SETTING__
			gradientCoeffs.push_back(k1[i]*exp(-k2[i]/RPM_to_W(speeds[i])) - 
				k1[i+1]*exp(-k2[i+1]/RPM_to_W(speeds[i])));
			#else
			gradientCoeffs.push_back(k1[i]*exp(-k2[i]/(speeds[i])) - 
				k1[i+1]*exp(-k2[i+1]/(speeds[i])));
			#endif
		}
		else
			gradientCoeffs.push_back(0.0);
	}
	#ifdef __USING_BIONDI_EXP_SETTING__
	gradientCoeffs.push_back(k1.back()*exp(-k2.back()/RPM_to_W(speeds.back())));
	#else
	gradientCoeffs.push_back(k1.back()*exp(-k2.back()/speeds.back()));
	#endif

	return gradientCoeffs;
}

vector<double> OptimizationAlgorithm::getPerformanceDerivativesDCExp(map<int,int> dc, vector<double> k1, vector<double> k2, vector<double> speeds) {
	vector<double> gradientCoeffs;

	int sum = 0;
	for (auto e: dc) sum += e.second;

	for (int i=0; i <k1.size()-1; i++) {
		int speed = Utility::round(speeds[i]);
		int localSum = 0;
		for (auto e:dc) {
			if (speed - e.first > 1.0 * BS_DC_SEARCH_STEP_RPM) continue;
			if (e.first - speed > 0) break;
			localSum += e.second;
		}

		if (localSum==0)
			gradientCoeffs.push_back(0.0);
		else {
			double factor = 1.0*localSum/sum;
			gradientCoeffs.push_back(factor*(k1[i]*exp(-k2[i]/speeds[i]) - 
				k1[i+1]*exp(-k2[i+1]/speeds[i])));
		}
	}
	//gradientCoeffs.push_back(k1.back()*exp(-k2.back()/speeds.back()));
	// Driving cycles < 6500 rpm
	gradientCoeffs.push_back(0.0);

	return gradientCoeffs;
}

vector<double> OptimizationAlgorithm::getPerformanceDerivativesPoly(vector<double> k1, vector<double> k2, vector<double> k3, vector<double> speeds) {
	/// k3 should not be used.

	vector<double> gradientCoeffs;

	for (int i=0; i <k1.size()-1; i++) {
		gradientCoeffs.push_back(k1[i]*speeds[i] + k2[i] - 
			k1[i+1]*speeds[i+1] - k2[i+1]);
	}
	gradientCoeffs.push_back(k1.back()*speeds.back() + k2.back());

	return gradientCoeffs;
}

int OptimizationAlgorithm::getSpeedIndex(vector<double> speeds, double speed) {
	for (int i=0; i<speeds.size(); i++) {
		if (fabs(speeds[i]) <= Utility::EPSILON)
			continue;

		if (RPM_LEQ_TOL(speed,speeds[i]))
			return i;
	}

	cerr << "Error: speed = " << speed << endl;
	Utility::output_one_vector(cerr,"speeds",speeds);
	exit(EXIT_FAILURE);
}

double OptimizationAlgorithm::getPerformanceDC(map<int,int> dc, vector<double> speeds, vector<double> k) {
	double perf = 0.0;
	for (map<int,int>::iterator iter = dc.begin(); iter != dc.end(); iter++) {
		int rpm = iter->first;
		int n = iter->second;
		int index = getSpeedIndex(speeds, rpm);
		perf += k[index]*n;
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDC(list<double> dc, vector<double> speeds, vector<double> k)
{
	if (speeds.empty()) return 0;

	double perf = 0.0; 
	if (speeds.size() == 1) {
		int index = 0;
		for (list<double>::iterator iter = dc.begin(); iter != dc.end(); iter++) {
			perf += k[index];
			//cout << *iter << "\t" << k[index] << endl;
		}
	}
	else {
		for (list<double>::iterator iter = dc.begin(); iter != dc.end(); iter++) {
			int index = getSpeedIndex(speeds, *iter);
			perf += k[index];
			//cout << *iter << "\t" << k[index] << endl;
		}
	}
	
	return perf;
}

double OptimizationAlgorithm::getPerformanceDC(map<int,int> dc, map<int,double> speeds, vector<double> k) {
	vector<double> vec_speeds;
	for(auto e : speeds) {
		vec_speeds.push_back(e.second);
	}
	return getPerformanceDC(dc,vec_speeds,k);
}

double OptimizationAlgorithm::getPerformanceDC(list<int> indexs, vector<double> k)
{
	if (indexs.empty()) return 0;

	double perf = 0.0;
	for (auto index : indexs) perf += k[index];

	return perf;
}

double OptimizationAlgorithm::getPerformanceDC(list<double> dc, list<int> indexs, vector<double> k) {
	if (indexs.empty()) return 0;

	double perf = 0.0;
	list<int>::iterator iter = indexs.begin();
	for (auto speed:dc) {
		int index = *iter;
		iter++;
		perf += k[index];
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDCExp(map<int,int> dc, vector<double> speeds, vector<double> k1, vector<double> k2) {
	double perf = 0.0;
	for (map<int,int>::iterator iter = dc.begin(); iter != dc.end(); iter++) {
		int rpm = iter->first;
		int n = iter->second;
		int index = getSpeedIndex(speeds, rpm);
		//perf += integral(k1[index],k2[index],rpm);
		//perf += k1[index]*exp(-k2[index]/RPM_to_RPmSEC(rpm))*n;
		#ifdef __USING_BIONDI_EXP_SETTING__
			//perf += k1[index]*exp(-k2[index]/RPM_to_W(rpm))*n;
			perf += k1[index]*exp(-exp(k2[index])/rpm)*n;
		#else
			perf += k1[index]*exp(-k2[index]/rpm)*n;
		#endif
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDCExp(map<int,int> dc, map<int,double> speeds, vector<double> k1, vector<double> k2) {
	vector<double> vec_speeds;
	for(auto e : speeds) {
		vec_speeds.push_back(e.second);
	}
	return getPerformanceDCExp(dc,vec_speeds,k1,k2);
}

double OptimizationAlgorithm::getPerformanceDCExp(list<double> dc, vector<double> speeds, vector<double> k1, vector<double> k2) {
	if (speeds.empty()) return 0;

	double perf = 0.0;
	for (list<double>::iterator iter = dc.begin(); iter != dc.end(); iter++) {
		int index = getSpeedIndex(speeds, *iter);
		//perf += integral(k1[index],k2[index],rpm);
		//perf += k1[index]*exp(-k2[index]/RPM_to_RPmSEC(rpm))*n;
		#ifdef __USING_BIONDI_EXP_SETTING__
			perf += k1[index]*exp(-k2[index]/(RPM_to_W(*iter)));
			//perf += k1[index]*exp(-exp(k2[index])/(*iter));
			//cout << k1[index] << " " << k2[index] << " " << *iter << " " << RPM_to_W(*iter) << endl; 
			//double p0 = k1[index]*exp(-exp(k2[index])/(*iter));
			//double p1 = k1[index]*exp(-k2[index]/(RPM_to_W(*iter)));

			//cout << p0 << " " << p1 << endl;
			//perf += k1[index]*exp(-exp(k2[index]-k2[0])/(*iter));
			//cout << k1[index]*exp(-exp(k2[index])/(*iter)) << endl;
		#else
			perf += k1[index]*exp(-k2[index]/(*iter));
			//cout << k1[index]*exp(-k2[index]/(*iter)) << endl;
		#endif
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDCExp(list<double> dc, map<int,double> speeds, vector<double> k1, vector<double> k2) {
	vector<double> vec_speeds;
	for(auto e : speeds) {
		vec_speeds.push_back(e.second);
	}
	return getPerformanceDCExp(dc,vec_speeds,k1,k2);
}

double OptimizationAlgorithm::getPerformanceDCExp(list<double> dc, list<int> indexs, vector<double> k1, vector<double> k2) {
	if (indexs.empty()) return 0;
	
	double perf = 0.0;
	list<int>::iterator iter = indexs.begin();
	for (auto speed:dc) {
		int index = *iter;
		iter++;
		#ifdef __USING_BIONDI_EXP_SETTING__
			perf += k1[index]*exp(-k2[index]/(RPM_to_W(speed)));
			//perf += k1[index]*exp(-exp(k2[index])/speed);
		#else
			perf += k1[index]*exp(-k2[index]/speed);
		#endif
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDCPoly3(map<int,int> dc, vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3) {
	double perf = 0.0;
	for (map<int,int>::iterator iter = dc.begin(); iter != dc.end(); iter++) {
		int rpm = iter->first;
		int n = iter->second;
		int index = getSpeedIndex(speeds, rpm);
		perf += ( k1[index]*rpm*rpm + k2[index]*rpm + k3[index] )*n;
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDCPoly3(map<int,int> dc, map<int,double> speeds, vector<double> k1, vector<double> k2, vector<double> k3) {
	vector<double> vec_speeds;
	for(auto e : speeds) {
		vec_speeds.push_back(e.second);
	}
	return getPerformanceDCPoly3(dc,vec_speeds,k1,k2,k3);
}

double OptimizationAlgorithm::getPerformanceDCPoly(double speed, double torque, double k1, double k2, double k3, double k4) {
#ifdef __CONSTANT_TORQUE__
	torque = TORQUE;
	//cout << "torque = " << torque << endl;
#endif

	return k1*speed*speed + k2*speed + k3*speed*torque + k4;
}

double OptimizationAlgorithm::getPerformanceDCPoly(vector<double> dc_speeds, vector<double> dc_torques, vector<double> speeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4) {
	double perf = 0.0;
	for (int i=0; i<dc_speeds.size(); i++) {
		double speed = dc_speeds[i];
		double torque = dc_torques[i];
		int index = getSpeedIndex(speeds,speed);
		perf += getPerformanceDCPoly(speed,torque,k1[index],k2[index],k3[index],k4[index]);
	}
	return perf;
}

double OptimizationAlgorithm::getPerformanceDCPoly(vector<double> dc_speeds, vector<double> dc_torques, map<int,double> speeds_map, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4) {
	vector<double> speeds;
	for (auto e : speeds_map)
		speeds.push_back(e.second);

	return getPerformanceDCPoly(dc_speeds,dc_torques,speeds,k1,k2,k3,k4);
}

vector<double> OptimizationAlgorithm::getMaxUtilizations(vector<double> omegas, vector<int> wcets, double angular_period, Engine engine) {
	vector<double> ret;
	for (unsigned int i = 0; i < omegas.size(); i++) {
		double minPeriod = engine.getMinDeadline(angular_period,omegas[i]);
		ret.push_back(1.0*wcets[i]/(mSEC_to_muSEC(minPeriod)));
	}
	return ret;
}

vector<double> OptimizationAlgorithm::getUtilizations(vector<double> speeds, vector<int> wcets, double angular_period) {
	vector<double> ret;
	for (unsigned int i = 0; i < speeds.size(); i++) {
		if (speeds[i] > 0)
			ret.push_back(1.0*wcets[i]/(angular_period/RPM_to_RPmuSEC(speeds[i])));
		else
			ret.push_back(0.0);
	}
	return ret;
}

double OptimizationAlgorithm::getUtilizationIndex(vector<double> speeds, vector<int> wcets, int index, double angular_period) {
	return 1.0*wcets[index]/(angular_period/RPM_to_RPmuSEC(speeds[index]));
}

double OptimizationAlgorithm::getUtilizationMax(vector<double> speeds, vector<int> wcets, double angular_period) {
	double util = 0.0;
	for (int i=0; i<speeds.size(); i++) {
		double utilIndex = getUtilizationIndex(speeds,wcets,i,angular_period);
		util = max(util,utilIndex);
	}
	return util;
}

double OptimizationAlgorithm::getUtilizationMin(vector<double> speeds, vector<int> wcets, double angular_period) {
	double util = INT_MAX;
	for (int i=0; i<speeds.size(); i++) {
		double utilIndex = getUtilizationIndex(speeds,wcets,i,angular_period);
		util = min(util,utilIndex);
	}
	return util;
}

double OptimizationAlgorithm::calStepIndex(vector<double> speeds, vector<int> wcets, vector<double> k, int index, double angular_period) {
	double utilIndex = getUtilizationIndex(speeds,wcets,index,angular_period);
	double utilMin = getUtilizationMin(speeds,wcets,angular_period);
	double utilMax = getUtilizationMax(speeds,wcets,angular_period);
	double utilBar = (utilIndex-utilMin)/(utilMax-utilMin);

	/*
	double perfIndex = getPerformanceIndex(speeds,k,index);
	double perfMax = getPerformanceMax(speeds,k,size);
	double perfMin = getPerformanceMin(speeds,k,size);
	*/

	double perfIndex = getPerformanceDerivativeIndex(k,index);
	double perfMax = getPerformanceDerivativeMax(speeds);
	double perfMin = getPerformanceDerivativeMin(speeds);

	double perfBar = (perfMax-perfIndex)/(perfMax-perfMin);

	return BS_SEARCH_STEP_RPM*((1.0-STARV_WEIGHT)*utilBar + STARV_WEIGHT + perfBar);
}

vector<double> OptimizationAlgorithm::calSteps(vector<double> speeds, vector<int> wcets, vector<double> k, double angular_period) {
	vector<double> ret;

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivatives(k);

	double minU = *min_element(utils.begin(), utils.end());
	double maxU = *max_element(utils.begin(), utils.end());
	double normalizedU = maxU - minU;

	double minGradientCoeff = *min_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double maxGradientCoeff = *max_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = BS_SEARCH_STEP_RPM
			* ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
			+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);
		ret.push_back(step);
	}
	return ret;
}

vector<double> OptimizationAlgorithm::calSteps(vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period) {
	vector<double> ret;

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);

	double minU = *min_element(utils.begin(), utils.end());
	double maxU = *max_element(utils.begin(), utils.end());
	double normalizedU = maxU - minU;

	double minGradientCoeff = *min_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double maxGradientCoeff = *max_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = BS_SEARCH_STEP_RPM
			* ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
			+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);

#ifdef __USING_MINIMUM_SEARCH_STEP_RPM__
		step = max(step,BS_SEARCH_STEP_RPM);
#endif
		ret.push_back(step);
	}
	return ret;
}

vector<double> OptimizationAlgorithm::calSteps(vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, vector<double> k3, double angular_period) {
	vector<double> ret;

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesPoly(k1,k2,k3,speeds);

	double minU = *min_element(utils.begin(), utils.end());
	double maxU = *max_element(utils.begin(), utils.end());
	double normalizedU = maxU - minU;

	double minGradientCoeff = *min_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double maxGradientCoeff = *max_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = BS_SEARCH_STEP_RPM
			* ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
			+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);
		ret.push_back(step);
	}
	return ret;
}

vector<double> OptimizationAlgorithm::calStepsMin(vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period) {
	vector<double> ret;

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);

	double minU = *min_element(utils.begin(), utils.end());
	double maxU = *max_element(utils.begin(), utils.end());
	double normalizedU = maxU - minU;

	double minGradientCoeff = *min_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double maxGradientCoeff = *max_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = BS_SEARCH_STEP_RPM
			* ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
			+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);
		ret.push_back(max(MINIMUM_STEP,step));
	}
	return ret;
}

vector<double> OptimizationAlgorithm::computeUBSpeeds(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask) {
	vector<double> UBSpeeds;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	int minWCET = wcets.back();
	// For each mode other than the first one (i=size()-1);
	for (unsigned int i=0; i<wcets.size()-1; i++) {
		int wcet = wcets[i];
		double lbSpeed = engine.RPM_MIN;
		double ubSpeed = engine.RPM_MAX;

		double speed;

		bool schedulable;

		// Binary search
		while(ubSpeed-lbSpeed > BIN_SEARCH_PRECISION_RPM) {
			speed = (ubSpeed + lbSpeed)/2.0;

#ifdef __DEBUG_DESIGN__
			cout << "mode = " << i << "=>" << ubSpeed << ", " << speed << ", " << lbSpeed << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			if (RPM_LEQ_TOL(speed,engine.RPM_MIN)) {
#ifdef __DEBUG_DESIGN__
				cout << "ERROR: DESIGN IMPOSSIBLE!" << endl;
#endif
				throw excDesignImpossible();
			}

			schedulable = false;

			try {
				// Prepare AVR task with two modes
				vector<double> avr_speeds;
				vector<int> avr_wcets;

				avr_speeds.push_back(speed);
				avr_speeds.push_back(engine.RPM_MAX);

				avr_wcets.push_back(wcet);
				avr_wcets.push_back(minWCET);

				AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
				// Check schedulability
#ifdef __DEBUG_DESIGN__
				cout << "Check schedulable ..." << endl;
#endif
				if (prevAvrTask != NULL) {
					//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
					//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask,prevAvrTask);
					schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, avrTask);
					//schedulable = SchedAnalysis::checkSchedulabilityAudsleyOPA(mc,tasks,avrTask,prevAvrTask);
				}
				else {
					//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,avrTask);
					schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, avrTask);
					//schedulable = SchedAnalysis::checkSchedulabilityAudsleyOPA(mc,tasks,avrTask);
				}

#ifdef __DEBUG_DESIGN__
				cout << "End schedulable ..." << endl;
#endif

				if (schedulable)
					lbSpeed = speed;
				else ubSpeed = speed;
			}
			catch(exception& e) {
				cout << e.what() << endl;
				throw e;
			}
		}

		UBSpeeds.push_back(speed);
	}

	UBSpeeds.push_back(engine.RPM_MAX);

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Computing upper bound speeds, times = " << times << endl;
#endif

	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeUBSpeedsFast(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask) {
	vector<double> UBSpeeds;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	int minWCET = wcets.back();
	// For each mode other than the first one (i=size()-1);
	for (unsigned int i=0; i<wcets.size()-1; i++) {
		int wcet = wcets[i];
		double lbSpeed = engine.RPM_MIN;
		double ubSpeed = engine.RPM_MAX;

		double speed;

		bool schedulable;

		bool requriedNO2 = true;
		bool requriedNO1A = true;
		bool requiredNO1 = true;

		// Binary search
		while(ubSpeed-lbSpeed > BIN_SEARCH_PRECISION_RPM) {
			speed = (ubSpeed + lbSpeed)/2.0;

#ifdef __DEBUG_DESIGN__
			cout << "mode = " << i << "=>" << ubSpeed << ", " << speed << ", " << lbSpeed << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			if (RPM_LEQ_TOL(speed,engine.RPM_MIN)) {
#ifdef __DEBUG_DESIGN__
				cout << "ERROR: DESIGN IMPOSSIBLE!" << endl;
#endif
				throw excDesignImpossible();
			}

			schedulable = false;

			try {
				// Prepare AVR task with two modes
				vector<double> avr_speeds;
				vector<int> avr_wcets;

				avr_speeds.push_back(speed);
				avr_speeds.push_back(engine.RPM_MAX);

				avr_wcets.push_back(wcet);
				avr_wcets.push_back(minWCET);

				AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
				// Check schedulability
#ifdef __DEBUG_DESIGN__
				cout << "Check schedulable ..." << endl;
#endif
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTask,prevAvrTask);
				if (schedulable) {
					//cout << "+++++0" << endl;
					goto BINARY_SETTING;
				}

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(SEG_NECESSARY_ONLY1A,tasks,avrTask,prevAvrTask);
				if (!schedulable) {
					//cout << "+++++1" << endl;
					goto BINARY_SETTING;
				}

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(SEG_NECESSARY_ONLY1,tasks,avrTask,prevAvrTask);
				if (!schedulable) {
					//cout << "+++++2" << endl;
					goto BINARY_SETTING;
				}
				
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,avrTask,prevAvrTask);
				

#ifdef __DEBUG_DESIGN__
				cout << "End schedulable ..." << endl;
#endif

BINARY_SETTING:
				if (schedulable)
					lbSpeed = speed;
				else ubSpeed = speed;
			}
			catch(exception& e) {
				cout << e.what() << endl;
				throw e;
			}
		}

		UBSpeeds.push_back(speed);
	}

	UBSpeeds.push_back(engine.RPM_MAX);

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Computing upper bound speeds, times = " << times << endl;
#endif

	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeUBFPSpeeds(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period,int avrTaskIndex) {
#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	vector<double> UBSpeeds;
	int minWCET = wcets.back();
	// For each mode other than the first one (i=size()-1);
	for (unsigned int i=0; i<wcets.size()-1; i++) {
		int wcet = wcets[i];
		double lbSpeed = engine.RPM_MIN;
		double ubSpeed = engine.RPM_MAX;

		double speed;

		bool schedulable;

		// Binary search
		while(ubSpeed-lbSpeed > BIN_SEARCH_PRECISION_RPM) {
			speed = (ubSpeed + lbSpeed)/2.0;

#ifdef __DEBUG_DESIGN__
			cout << "mode = " << i << "=>" << ubSpeed << ", " << speed << ", " << lbSpeed << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			if (RPM_LEQ_TOL(speed,engine.RPM_MIN)) {
#ifdef __DEBUG_DESIGN__
				cout << "ERROR: DESIGN IMPOSSIBLE!" << endl;
#endif
				throw excDesignImpossible();
			}

			schedulable = false;

			try {
				// Prepare AVR task with two modes
				vector<double> avr_speeds;
				vector<int> avr_wcets;

				avr_speeds.push_back(speed);
				avr_speeds.push_back(engine.RPM_MAX);

				avr_wcets.push_back(wcet);
				avr_wcets.push_back(minWCET);

				AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
				// Check schedulability
#ifdef __DEBUG_DESIGN__
				cout << "Check schedulable ..." << endl;
#endif
				schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, avrTask,avrTaskIndex);

#ifdef __DEBUG_DESIGN__
				cout << "End schedulable ..." << endl;
#endif

				if (schedulable)
					lbSpeed = speed;
				else ubSpeed = speed;
			}
			catch(exception& e) {
				cout << e.what() << endl;
				throw e;
			}
		}

		UBSpeeds.push_back(speed);
	}

	UBSpeeds.push_back(engine.RPM_MAX);


#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Computing upper bound speeds, times = " << times << endl;
#endif

	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeUBFPSpeeds(MethodChoice mc, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, list<double> dcList, vector<double> k1, vector<double> k2, int& avrTaskIndex) {
	vector<double> UBSpeeds;
	double currPerf = INT_MIN;

	int minWCET = wcets.back();
	for (int index = 0; index < tasks.size()+1; index++) {
#ifdef __DEBUG_VERIFICATION_TIMES__
		int times = 0;
#endif
		vector<double> localUBSpeeds;
		// For each mode other than the first one (i=size()-1);
		for (unsigned int i=0; i<wcets.size()-1; i++) {
			int wcet = wcets[i];
			double lbSpeed = engine.RPM_MIN;
			double ubSpeed = engine.RPM_MAX;

			double speed;

			bool schedulable;

			// Binary search
			while(ubSpeed-lbSpeed > BIN_SEARCH_PRECISION_RPM) {
				speed = (ubSpeed + lbSpeed)/2.0;

#ifdef __DEBUG_DESIGN__
				cout << "mode = " << i << "=>" << ubSpeed << ", " << speed << ", " << lbSpeed << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				if (RPM_LEQ_TOL(speed,engine.RPM_MIN)) {
#ifdef __DEBUG_DESIGN__
					cout << "ERROR: DESIGN IMPOSSIBLE!" << endl;
#endif
					throw excDesignImpossible();
				}

				schedulable = false;

				try {
					// Prepare AVR task with two modes
					vector<double> avr_speeds;
					vector<int> avr_wcets;

					avr_speeds.push_back(speed);
					avr_speeds.push_back(engine.RPM_MAX);

					avr_wcets.push_back(wcet);
					avr_wcets.push_back(minWCET);

					AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
					// Check schedulability
#ifdef __DEBUG_DESIGN__
					cout << "Check schedulable ..." << endl;
#endif
					schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, avrTask,index);

#ifdef __DEBUG_DESIGN__
					cout << "End schedulable ..." << endl;
#endif

					if (schedulable)
						lbSpeed = speed;
					else ubSpeed = speed;
				}
				catch(exception& e) {
					cout << e.what() << endl;
					throw e;
				}
			}

			localUBSpeeds.push_back(speed);
		}

		localUBSpeeds.push_back(engine.RPM_MAX);

		double localPerf = OptimizationAlgorithm::getPerformanceDCExp(dcList,localUBSpeeds, k1, k2);
		if (localPerf > currPerf) {
			currPerf = localPerf;
			UBSpeeds = localUBSpeeds;
			avrTaskIndex = index;
		}

#ifdef __DEBUG_VERIFICATION_TIMES__
		cout << "Computing upper bound speeds, times = " << times << endl;
#endif
	}

	return UBSpeeds;
}

double OptimizationAlgorithm::computeUBPerfDC(ostream& out, list<double> dc, MethodChoice mc, vector<int> wcets, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, Engine engine, double period)
{
	double totalPerf = 0.0;

	vector<double> maxSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	for (auto speed : dc) {
		int index = getSpeedIndex(maxSpeeds,speed);
		if (index == wcets.size()-1) goto MAXMODE;

		if (prevSpeeds.empty()) { // First transition speed configuration
			if (index != wcets.size()-1) {
				prevSpeeds.push_back(speed);
				prevWCETs.push_back(wcets[index]);
			}
		}
		else {
			int minWCET = wcets.back();
			// For each mode other than the first one (i=size()-1);
			while (index < wcets.size()-1) {
				int wcet = wcets[index];
				
				bool schedulable = false;

				try {
					// Prepare AVR task with two modes
					vector<double> avr_speeds;
					vector<int> avr_wcets;

					avr_speeds.push_back(speed);
					avr_speeds.push_back(engine.RPM_MAX);

					avr_wcets.push_back(wcet);
					avr_wcets.push_back(minWCET);

					AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
					AVRTask prevAvrTask(engine,period,prevSpeeds,prevWCETs);
						
					//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask,&prevAvrTask);
						
				}
				catch(exception& e) {
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) break;
				index ++;
			}

			// reset the previous avr task
			prevSpeeds.clear();
			prevWCETs.clear();

			if (index != wcets.size()-1) {
				prevSpeeds.push_back(speed);
				prevWCETs.push_back(wcets[index]);
			}	
		}

	MAXMODE:
		prevSpeeds.push_back(maxSpeeds.back());
		prevWCETs.push_back(wcets.back());

		out << speed << " " << index << endl;

		totalPerf += k1[index]*exp(-k2[index]/speed);
	}

	return totalPerf;
}

list<int> OptimizationAlgorithm::computeUBPerfDC(list<double> dc,vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int& optNum, double& optTime,double& maxOptTime, int granularity)
{
	Timer timer;
	list<int> dcResult;

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;
#ifdef __OUTPUT_OPTIMIAZATION_DC__
	int checkSANum = 0;
#endif

	for (auto speed : dc) {
		int index = getSpeedIndex(UBSpeeds,speed);
		int tmpIndex = index;
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;
		
		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) >= int(prevSpeed) && int(recordPair.speed) >= int(speed)
							&& recordPair.prevIndex == prevIndex && recordPair.index == index ) {
					found = true;
				}
			}
		}

		if (!found) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();

				bool needOpt = false;
				if (index < wcets.size()-1 && index != dcResult.back()) {
					needOpt = true;
					optNum++;
					timer.start();
				}

				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					int wcet = wcets[index];

					bool schedulable = false;

					try {
						// Prepare AVR task with two modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						avr_speeds.push_back(speed);
						avr_speeds.push_back(engine.RPM_MAX);

						avr_wcets.push_back(wcet);
						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
						AVRTask prevAvrTask(engine,period,prevSpeeds,prevWCETs);

						// Prepare the mixed AVR task with two/three modes
						vector<double> mixed_avr_speeds;
						vector<int> mixed_avr_wcets;

						if (prevIndex == index) {
							mixed_avr_speeds.push_back(max(speed,prevSpeeds.front()));

							mixed_avr_wcets.push_back(wcets[index]);
						}
						else if (prevIndex < index) {
							mixed_avr_speeds.push_back(prevSpeeds.front());
							mixed_avr_speeds.push_back(speed);

							mixed_avr_wcets.push_back(wcets.front());
							mixed_avr_wcets.push_back(wcets[index]);
						}
						else {
							mixed_avr_speeds.push_back(speed);
							mixed_avr_speeds.push_back(prevSpeeds.front());

							mixed_avr_wcets.push_back(wcets[index]);
							mixed_avr_wcets.push_back(wcets.front());
						}


						mixed_avr_speeds.push_back(engine.RPM_MAX);

						mixed_avr_wcets.push_back(minWCET);

						AVRTask mixedAvrTask(engine,period,mixed_avr_speeds,mixed_avr_wcets);
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						//Timer timer;
						timer.start();
#endif
						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, prevAvrTask, avrTask,dcResult.size()*CONSTANT_SWITCH_TIME);
						schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, mixedAvrTask, prevAvrTask, avrTask, CONSTANT_SWITCH_TIME,granularity);
						
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						timer.end();
						checkSANum++;
						//cout << checkSANum << "\t" << timer.getTime() << endl;
						cout << dcResult.size() << " => " << prevSpeed << ", " << prevIndex << ", " << speed << ", " << index << ", " << tmpIndex << endl;
#endif
						//exit(EXIT_FAILURE);
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}
					//cout << optTime << "\t" << timer.getTime() << endl;
					
					if (schedulable) break;
					index ++;
				}

				if (needOpt) {
					timer.end();
					double currOptTime = timer.getTime();
					maxOptTime = max(maxOptTime,currOptTime);
					optTime += currOptTime;
				}
				//cout << optTime << endl;

#ifdef __OUTPUT_OPTIMIAZATION_DC__
//#if 0
				//cout << dcResult.size() << "=>" << speed << "," << index << endl;

				/*
				if (dcResult.size() == 116) {
					exit(EXIT_FAILURE);
				}
				*/

				if (index != tmpIndex)
					cout << "Diff: " << dcResult.size() << " => " << speed << ", " << index << ", " << tmpIndex << endl;
#endif

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(UBSpeeds.back());
		prevWCETs.push_back(wcets.back());

		dcResult.push_back(index);

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	//cout << foundNum << endl;
	//exit(EXIT_FAILURE);

	return dcResult;
}

list<int> OptimizationAlgorithm::computeUBFPPerfDC(list<double> dc,vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int avrTaskIndex, int& optNum, double& optTime,double& maxOptTime, int granularity)
{
	Timer timer;
	list<int> dcResult;

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;
#ifdef __OUTPUT_OPTIMIAZATION_DC__
	int checkSANum = 0;
#endif

	for (auto speed : dc) {
		int index = getSpeedIndex(UBSpeeds,speed);
		int tmpIndex = index;
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;
		
		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) >= int(prevSpeed) && int(recordPair.speed) >= int(speed)
							&& recordPair.prevIndex == prevIndex && recordPair.index == index ) {
					found = true;
				}
			}
		}

		if (!found) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();

				bool needOpt = false;
				if (index < wcets.size()-1 && index != dcResult.back()) {
					needOpt = true;
					optNum++;
					timer.start();
				}

				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					int wcet = wcets[index];

					bool schedulable = false;

					try {
						// Prepare AVR task with two modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						avr_speeds.push_back(speed);
						avr_speeds.push_back(engine.RPM_MAX);

						avr_wcets.push_back(wcet);
						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
						AVRTask prevAvrTask(engine,period,prevSpeeds,prevWCETs);

						// Prepare the mixed AVR task with two/three modes
						vector<double> mixed_avr_speeds;
						vector<int> mixed_avr_wcets;

						if (prevIndex == index) {
							mixed_avr_speeds.push_back(max(speed,prevSpeeds.front()));

							mixed_avr_wcets.push_back(wcets[index]);
						}
						else if (prevIndex < index) {
							mixed_avr_speeds.push_back(prevSpeeds.front());
							mixed_avr_speeds.push_back(speed);

							mixed_avr_wcets.push_back(wcets.front());
							mixed_avr_wcets.push_back(wcets[index]);
						}
						else {
							mixed_avr_speeds.push_back(speed);
							mixed_avr_speeds.push_back(prevSpeeds.front());

							mixed_avr_wcets.push_back(wcets[index]);
							mixed_avr_wcets.push_back(wcets.front());
						}


						mixed_avr_speeds.push_back(engine.RPM_MAX);

						mixed_avr_wcets.push_back(minWCET);

						AVRTask mixedAvrTask(engine,period,mixed_avr_speeds,mixed_avr_wcets);
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						//Timer timer;
						timer.start();
#endif
						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, prevAvrTask, avrTask,dcResult.size()*CONSTANT_SWITCH_TIME);
						schedulable = SchedAnalysis::checkSchedulabilityAllPrioritiesFP(EXACT,tasks, mixedAvrTask, prevAvrTask, avrTask, avrTaskIndex, CONSTANT_SWITCH_TIME,granularity);
						
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						timer.end();
						checkSANum++;
						//cout << checkSANum << "\t" << timer.getTime() << endl;
						cout << dcResult.size() << " => " << prevSpeed << ", " << prevIndex << ", " << speed << ", " << index << ", " << tmpIndex << endl;
#endif
						//exit(EXIT_FAILURE);
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}
					//cout << optTime << "\t" << timer.getTime() << endl;
					
					if (schedulable) break;
					index ++;
				}

				if (needOpt) {
					timer.end();
					double currOptTime = timer.getTime();
					maxOptTime = max(maxOptTime,currOptTime);
					optTime += currOptTime;
				}
				//cout << optTime << endl;

#ifdef __OUTPUT_OPTIMIAZATION_DC__
//#if 0
				//cout << dcResult.size() << "=>" << speed << "," << index << endl;

				/*
				if (dcResult.size() == 116) {
					exit(EXIT_FAILURE);
				}
				*/

				if (index != tmpIndex)
					cout << "Diff: " << dcResult.size() << " => " << speed << ", " << index << ", " << tmpIndex << endl;
#endif

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(UBSpeeds.back());
		prevWCETs.push_back(wcets.back());

		dcResult.push_back(index);

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	//cout << foundNum << endl;
	//exit(EXIT_FAILURE);

	return dcResult;
}

list<int> OptimizationAlgorithm::computeUBPerfDC(list<double> dc,vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds)
{
	list<int> dcResult;
	vector<double> maxSpeeds = UBSpeeds;
	//vector<double> maxSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	//Utility::output_one_vector(cout,"maxSpeeds",maxSpeeds);

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;

	for (auto speed : dc) {
		int index = getSpeedIndex(maxSpeeds,speed);
		int tmpIndex = index;
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;

		
		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) == int(prevSpeed) && recordPair.prevIndex == prevIndex && int(recordPair.speed) == int(speed)) {
					found = true;
					index = recordPair.index;
					//cout << dcResult.size() << "=>" << speed << "," << index << endl;
					//cout << "Found One!" << endl;
					//foundNum++;
				}
			}
		}
		
		if (!found) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();
				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					int wcet = wcets[index];

					bool schedulable = false;

					try {
						// Prepare AVR task with two modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						avr_speeds.push_back(speed);
						avr_speeds.push_back(engine.RPM_MAX);

						avr_wcets.push_back(wcet);
						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
						AVRTask prevAvrTask(engine,period,prevSpeeds,prevWCETs);

						// Prepare the mixed AVR task with two/three modes
						vector<double> mixed_avr_speeds;
						vector<int> mixed_avr_wcets;

						if (prevIndex == index) {
							mixed_avr_speeds.push_back(max(speed,prevSpeeds.front()));

							mixed_avr_wcets.push_back(wcets[index]);
						}
						else if (prevIndex < index) {
							mixed_avr_speeds.push_back(prevSpeeds.front());
							mixed_avr_speeds.push_back(speed);

							mixed_avr_wcets.push_back(wcets.front());
							mixed_avr_wcets.push_back(wcets[index]);
						}
						else {
							mixed_avr_speeds.push_back(speed);
							mixed_avr_speeds.push_back(prevSpeeds.front());

							mixed_avr_wcets.push_back(wcets[index]);
							mixed_avr_wcets.push_back(wcets.front());
						}


						mixed_avr_speeds.push_back(engine.RPM_MAX);

						mixed_avr_wcets.push_back(minWCET);

						AVRTask mixedAvrTask(engine,period,mixed_avr_speeds,mixed_avr_wcets);

						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						Timer timer;
						timer.start();
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, prevAvrTask, avrTask,dcResult.size()*CONSTANT_SWITCH_TIME);
						schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,mixedAvrTask, prevAvrTask, avrTask,CONSTANT_SWITCH_TIME);
						timer.end();

#ifdef __OUTPUT_OPTIMIAZATION_DC__
						cout << timer.getTime() << endl;
#endif
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}

					if (schedulable) break;
					index ++;
				}
#ifdef __OUTPUT_OPTIMIAZATION_DC__
				cout << dcResult.size() << "=>" << speed << "," << index << endl;
#endif
				
#ifdef __OUTPUT_OPTIMIAZATION_DC__
				if (tmpIndex != index)
					cout << dcResult.size() << "=>" << speed << "," << index << "," << tmpIndex << endl;
#endif

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(maxSpeeds.back());
		prevWCETs.push_back(wcets.back());

		dcResult.push_back(index);
		//cout << dcResult.size() << "=>" << speed << "," << index << endl;

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	//cout << foundNum << endl;
	//exit(EXIT_FAILURE);

	return dcResult;
}

list<int> OptimizationAlgorithm::computeUBPerfDCFast(list<double> dc,vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds)
{
	list<int> dcResult;
	vector<double> maxSpeeds = UBSpeeds;

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;
	int checkSANum = 0;

	for (auto speed : dc) {
		int index = getSpeedIndex(maxSpeeds,speed);
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;

		
		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) == int(prevSpeed) && recordPair.prevIndex == prevIndex && int(recordPair.speed) == int(speed)) {
					found = true;
					index = recordPair.index;
				}
			}
		}

		if (!found && prevIndex != wcets.size()-1) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();
				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					bool schedulable = false;

					try {
						// Prepare AVR task with two/three modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						if (prevIndex == index) {
							avr_speeds.push_back(max(speed,prevSpeeds.front()));

							avr_wcets.push_back(wcets[index]);
						}
						else if (prevIndex < index) {
							avr_speeds.push_back(prevSpeeds.front());
							avr_speeds.push_back(speed);

							avr_wcets.push_back(wcets.front());
							avr_wcets.push_back(wcets[index]);
						}
						else {
							avr_speeds.push_back(speed);
							avr_speeds.push_back(prevSpeeds.front());
							
							avr_wcets.push_back(wcets[index]);
							avr_wcets.push_back(wcets.front());
						}

						
						avr_speeds.push_back(engine.RPM_MAX);

						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);

						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						Timer timer;
						timer.start();
						schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, avrTask);
						timer.end();
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						checkSANum++;
						cout << checkSANum << "\t" << timer.getTime() << endl;
#endif
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}

					if (schedulable) break;
					index ++;
				}

				//cout << dcResult.size() << "=>" << speed << "," << index << endl;

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(maxSpeeds.back());
		prevWCETs.push_back(wcets.back());

		dcResult.push_back(index);
		//cout << dcResult.size() << "=>" << speed << "," << index << endl;

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	//cout << foundNum << endl;
	//exit(EXIT_FAILURE);

	return dcResult;
}

list<int> OptimizationAlgorithm::computeUBFPPerfDCFast(list<double> dc,vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int avrTaskIndex)
{
	list<int> dcResult;
	vector<double> maxSpeeds = UBSpeeds;

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;
	int checkSANum = 0;

	for (auto speed : dc) {
		int index = getSpeedIndex(maxSpeeds,speed);
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;


		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) == int(prevSpeed) && recordPair.prevIndex == prevIndex && int(recordPair.speed) == int(speed)) {
					found = true;
					index = recordPair.index;
				}
			}
		}

		if (!found && prevIndex != wcets.size()-1) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();
				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					bool schedulable = false;

					try {
						// Prepare AVR task with two/three modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						if (prevIndex == index) {
							avr_speeds.push_back(max(speed,prevSpeeds.front()));

							avr_wcets.push_back(wcets[index]);
						}
						else if (prevIndex < index) {
							avr_speeds.push_back(prevSpeeds.front());
							avr_speeds.push_back(speed);

							avr_wcets.push_back(wcets.front());
							avr_wcets.push_back(wcets[index]);
						}
						else {
							avr_speeds.push_back(speed);
							avr_speeds.push_back(prevSpeeds.front());

							avr_wcets.push_back(wcets[index]);
							avr_wcets.push_back(wcets.front());
						}


						avr_speeds.push_back(engine.RPM_MAX);

						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);

						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						Timer timer;
						timer.start();
						schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(NECESSARY_ONLY1,tasks, avrTask,avrTaskIndex);
						timer.end();
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						checkSANum++;
						cout << checkSANum << "\t" << timer.getTime() << endl;
#endif
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}

					if (schedulable) break;
					index ++;
				}

				//cout << dcResult.size() << "=>" << speed << "," << index << endl;

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(maxSpeeds.back());
		prevWCETs.push_back(wcets.back());

		dcResult.push_back(index);
		//cout << dcResult.size() << "=>" << speed << "," << index << endl;

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	//cout << foundNum << endl;
	//exit(EXIT_FAILURE);

	return dcResult;
}

list<int> OptimizationAlgorithm::computeUBPerfDCFast(list<double> dc,vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds)
{
	list<int> dcResult;
	vector<double> maxSpeeds = UBSpeeds;

	vector<double> prevSpeeds;
	vector<int> prevWCETs;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;

	for (auto speed : dc) {
		int index = getSpeedIndex(maxSpeeds,speed);
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;

		
		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) == int(prevSpeed) && recordPair.prevIndex == prevIndex && int(recordPair.speed) == int(speed)) {
					found = true;
					index = recordPair.index;
				}
			}
		}

		if (!found && prevIndex != wcets.size()-1) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();
				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					bool schedulable = false;

					try {
						// Prepare AVR task with two/three modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						if (prevIndex == index) {
							avr_speeds.push_back(max(speed,prevSpeeds.front()));

							avr_wcets.push_back(wcets[index]);
						}
						else if (prevIndex < index) {
							avr_speeds.push_back(prevSpeeds.front());
							avr_speeds.push_back(speed);

							avr_wcets.push_back(wcets.front());
							avr_wcets.push_back(wcets[index]);
						}
						else {
							avr_speeds.push_back(speed);
							avr_speeds.push_back(prevSpeeds.front());

							avr_wcets.push_back(wcets[index]);
							avr_wcets.push_back(wcets.front());
						}


						avr_speeds.push_back(engine.RPM_MAX);

						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);

						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						Timer timer;
						timer.start();
						schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, avrTask);
						timer.end();
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						cout << timer.getTime() << endl;
#endif
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}

					if (schedulable) break;
					index ++;
				}
#ifdef __OUTPUT_OPTIMIAZATION_DC__
				cout << dcResult.size() << "=>" << speed << "," << index << endl;
#endif

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(maxSpeeds.back());
		prevWCETs.push_back(wcets.back());

		dcResult.push_back(index);
		//cout << dcResult.size() << "=>" << speed << "," << index << endl;

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	//cout << foundNum << endl;
	//exit(EXIT_FAILURE);

	return dcResult;
}

list<int> OptimizationAlgorithm::computeUBFPPerfDCFastCoarse(list<double> dc,vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, vector<double> UBSpeeds, int avrTaskIndex)
{
	list<int> dcResult;
	vector<double> maxSpeeds = UBSpeeds;

	vector<double> prevSpeeds;
	vector<int> prevWCETs;
	AVRTask* prevAvrTask = NULL;

	vector<RecordPair> recordPairs;

	double prevSpeed = -1;
	int prevIndex = -1;

	//int foundNum = 0;
	int checkSANum = 0;

	AVRTask *pTask;

	for (auto speed : dc) {
		int index = getSpeedIndex(maxSpeeds,speed);
		bool found = false;
		if (index == wcets.size()-1) goto MAXMODE;


		/// check whether the pair of the speed and its previous speed has been assigned
		if (!recordPairs.empty()) {
			for (auto recordPair : recordPairs) {
				if (int(recordPair.prevSpeed) == int(prevSpeed) && recordPair.prevIndex == prevIndex && int(recordPair.speed) == int(speed)) {
					found = true;
					index = recordPair.index;
				}
			}
		}

		if (!found && prevIndex != wcets.size()-1) {
			if (prevSpeeds.empty()) { // First transition speed configuration
				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}
			}
			else {
				int minWCET = wcets.back();
				// For each mode other than the first one (i=size()-1);
				while (index < wcets.size()-1 && index != dcResult.back()) {
					bool schedulable = false;

					try {

						// Prepare AVR task with two/three modes
						vector<double> avr_speeds;
						vector<int> avr_wcets;

						avr_speeds.push_back(speed);
						avr_wcets.push_back(wcets[index]);

						avr_speeds.push_back(engine.RPM_MAX);
						avr_wcets.push_back(minWCET);

						AVRTask avrTask(engine,period,avr_speeds,avr_wcets);

						vector<AVRTask> two;
						two.push_back(*prevAvrTask);
						two.push_back(avrTask);
			
						AVRTask mixedAvrTask(engine,period,two);

						/*
						Utility::output_one_map(cout,"prevSpeeds",prevAvrTask->speeds);
						Utility::output_one_map(cout,"prevWCETs",prevAvrTask->wcets);

						Utility::output_one_map(cout,"currSpeeds",avrTask.speeds);
						Utility::output_one_map(cout,"currWCETs",avrTask.wcets);

						Utility::output_one_map(cout,"mixedSpeeds", mixedAvrTask.speeds);
						Utility::output_one_map(cout,"mixedWCETs",mixedAvrTask.wcets);

						exit(EXIT_FAILURE);
						*/

						//schedulable = true;
						//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,*prevAvrTask,&avrTask);
						Timer timer;
						timer.start();
						//schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(NECESSARY_ONLY1,tasks, avrTask,avrTaskIndex);
						schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(NECESSARY_ONLY1,tasks, mixedAvrTask,avrTaskIndex);
						timer.end();
#ifdef __OUTPUT_OPTIMIAZATION_DC__
						checkSANum++;
						cout << checkSANum << "\t" << timer.getTime() << endl;
#endif
					}
					catch(exception& e) {
						cout << e.what() << endl;
						throw e;
					}

					if (schedulable) break;
					index ++;
				}

				//cout << dcResult.size() << "=>" << speed << "," << index << endl;

				// reset the previous avr task
				prevSpeeds.clear();
				prevWCETs.clear();

				if (index != wcets.size()-1) {
					prevSpeeds.push_back(speed);
					prevWCETs.push_back(wcets[index]);
				}	
			}
		}

MAXMODE:
		prevSpeeds.push_back(maxSpeeds.back());
		prevWCETs.push_back(wcets.back());

		if (prevAvrTask == NULL) {
			prevAvrTask = new AVRTask(engine,period,prevSpeeds,prevWCETs);
		} else {
			// Combine the two consecutive configurations
			AVRTask currAvrTask(engine,period,prevSpeeds,prevWCETs);
			vector<AVRTask> two;
			two.push_back(*prevAvrTask);
			two.push_back(currAvrTask);

			/*
			Utility::output_one_map(cout,"prevSpeeds",prevAvrTask->speeds);
			Utility::output_one_map(cout,"prevWCETs",prevAvrTask->wcets);

			Utility::output_one_map(cout,"currSpeeds",currAvrTask.speeds);
			Utility::output_one_map(cout,"currWCETs",currAvrTask.wcets);
			*/
			
			delete prevAvrTask;
			prevAvrTask = new AVRTask(engine,period,two);

			/*
			Utility::output_one_map(cout,"mixedSpeeds", prevAvrTask->speeds);
			Utility::output_one_map(cout,"mixedWCETs",prevAvrTask->wcets);

			exit(EXIT_FAILURE);
			*/
		}

		dcResult.push_back(index);
		//cout << dcResult.size() << "=>" << speed << "," << index << endl;

		if (!found && prevSpeed > 0) {
			RecordPair rp = {prevSpeed,prevIndex,speed,index};
			recordPairs.push_back(rp);
		}

		prevSpeed = speed;
		prevIndex = index;
	}

	if (dcResult.size() != dc.size()) {
		cerr << "Not equal!" << endl;
		exit(EXIT_FAILURE);
	}

	return dcResult;
}

vector<double> OptimizationAlgorithm::computeUBSpeeds(MethodChoice mc, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	int minWCET = wcets.back();
	// For each mode other than the first one (i=size()-1);
	for (unsigned int i=0; i<wcets.size()-1; i++) {
		int wcet = wcets[i];
		double lbSpeed = engine.RPM_MIN;
		double ubSpeed = engine.RPM_MAX;

		double speed;

		bool schedulable;

		// Binary search
		while(ubSpeed-lbSpeed > BIN_SEARCH_PRECISION_RPM) {
			speed = (ubSpeed + lbSpeed)/2.0;

#ifdef __DEBUG_DESIGN__
			cout << "mode = " << i << "=>" << ubSpeed << ", " << speed << ", " << lbSpeed << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			if (RPM_LEQ_TOL(speed,engine.RPM_MIN)) {
#ifdef __DEBUG_DESIGN__
				cout << "ERROR: DESIGN IMPOSSIBLE!" << endl;
#endif
				throw excDesignImpossible();
			}

			schedulable = false;

			try {
				// Prepare AVR task with two modes
				vector<double> avr_speeds;
				vector<int> avr_wcets;

				avr_speeds.push_back(speed);
				avr_speeds.push_back(engine.RPM_MAX);

				avr_wcets.push_back(wcet);
				avr_wcets.push_back(minWCET);

				AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
				// Check schedulability
#ifdef __DEBUG_DESIGN__
				cout << "Check schedulable ..." << endl;
#endif
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,avrTask);

#ifdef __DEBUG_DESIGN__
				cout << "End schedulable ..." << endl;
#endif

				if (schedulable)
					lbSpeed = speed;
				else ubSpeed = speed;
			}
			catch(exception& e) {
				cout << e.what() << endl;
				throw e;
			}
		}

		UBSpeeds.push_back(speed);
	}

	UBSpeeds.push_back(engine.RPM_MAX);

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Computing upper bound speeds, times = " << times << endl;
#endif

	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeUBSpeeds(MethodChoice mc, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period, AVRTask prevAvrTask, int switchTime) {
	vector<double> UBSpeeds;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	int minWCET = wcets.back();
	// For each mode other than the first one (i=size()-1);
	for (unsigned int i=0; i<wcets.size()-1; i++) {
		int wcet = wcets[i];
		double lbSpeed = engine.RPM_MIN;
		double ubSpeed = engine.RPM_MAX;

		double speed;

		bool schedulable;

		// Binary search
		while(ubSpeed-lbSpeed > BIN_SEARCH_PRECISION_RPM) {
			speed = (ubSpeed + lbSpeed)/2.0;

#ifdef __DEBUG_DESIGN__
			cout << "mode = " << i << "=>" << ubSpeed << ", " << speed << ", " << lbSpeed << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			if (RPM_LEQ_TOL(speed,engine.RPM_MIN)) {
#ifdef __DEBUG_DESIGN__
				cout << "ERROR: DESIGN IMPOSSIBLE!" << endl;
#endif
				throw excDesignImpossible();
			}

			schedulable = false;

			try {
				// Prepare AVR task with two modes
				vector<double> avr_speeds;
				vector<int> avr_wcets;

				avr_speeds.push_back(speed);
				avr_speeds.push_back(engine.RPM_MAX);

				avr_wcets.push_back(wcet);
				avr_wcets.push_back(minWCET);

				AVRTask avrTask(engine,period,avr_speeds,avr_wcets);
				// Check schedulability
#ifdef __DEBUG_DESIGN__
				cout << "Check schedulable ..." << endl;
#endif
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,avrTask);
				if (schedulable) schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,avrTask,prevAvrTask,avrTask,switchTime);

#ifdef __DEBUG_DESIGN__
				cout << "End schedulable ..." << endl;
#endif

				if (schedulable)
					lbSpeed = speed;
				else ubSpeed = speed;
			}
			catch(exception& e) {
				cout << e.what() << endl;
				throw e;
			}
		}

		UBSpeeds.push_back(speed);
	}

	UBSpeeds.push_back(engine.RPM_MAX);

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Computing upper bound speeds, times = " << times << endl;
#endif

	return UBSpeeds;
}

StateGraph* OptimizationAlgorithm::generateStateGraph(MethodChoice mc, PerformanceType pt,  vector<vector<double>> ks, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	StateGraph* stateGraph = new StateGraph(pt);
	vector<double> ubSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	vector<double> lbSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,ks[0],wcets, tasks, engine, period);
	vector<double> maxSpeeds;
	for (int i=0; i<ubSpeeds.size(); i++)
		maxSpeeds.push_back((ubSpeeds[i]+lbSpeeds[i])/2.0);

	/*
	Utility::output_one_vector(cout,"ubSpeed",ubSpeeds);
	Utility::output_one_vector(cout,"lbSpeed",lbSpeeds);
	Utility::output_one_vector(cout,"maxSpeed",maxSpeeds);
	*/

	// Create the initial state nodes
	for (int i=0; i<maxSpeeds.size()-1; i++) {
		vector<double> avr_speeds;
		vector<vector<double>> avr_ks;
		vector<int> avr_wcets;

		avr_speeds.push_back(maxSpeeds[i]);
		avr_speeds.push_back(maxSpeeds.back());

		if (pt == PERF_CONSTANT) {
			vector<double> avr_k;
			avr_k.push_back(ks[0][i]);
			avr_k.push_back(ks[0].back());
			avr_ks.push_back(avr_k);
		}
		else {
			vector<double> avr_k0;
			vector<double> avr_k1;

			avr_k0.push_back(ks[0][i]);
			avr_k0.push_back(ks[0].back());

			avr_k1.push_back(ks[1][i]);
			avr_k1.push_back(ks[1].back());

			avr_ks.push_back(avr_k0);
			avr_ks.push_back(avr_k1);
		}
		
		avr_wcets.push_back(wcets[i]);
		avr_wcets.push_back(wcets.back());

		StateGraphNode* node = new StateGraphNode("TSC"+Utility::int_to_string(i),avr_speeds,avr_wcets,pt,avr_ks,true);
		stateGraph->add(node);
	}

	// Create the non-initial state nodes and connect them
	vector<StateGraphNode*> initialNodes = stateGraph->graph;
	for (int i = 0; i<initialNodes.size(); i++) {
		StateGraphNode* initialNode = initialNodes[i];
		AVRTask nextAvrTask(engine,period,initialNode->speeds,initialNode->wcets);
		vector<double> nextMaxSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period,&nextAvrTask);
		nextMaxSpeeds[i] = maxSpeeds[i];
		for (int j=0; j<nextMaxSpeeds.size()-1; j++) {
			vector<double> avr_speeds;
			vector<vector<double>> avr_ks;
			vector<int> avr_wcets;

			avr_speeds.push_back(nextMaxSpeeds[j]);
			avr_speeds.push_back(nextMaxSpeeds.back());

			if (pt == PERF_CONSTANT) {
				vector<double> avr_k;
				avr_k.push_back(ks[0][j]);
				avr_k.push_back(ks[0].back());
				avr_ks.push_back(avr_k);
			}
			else {
				vector<double> avr_k0;
				vector<double> avr_k1;

				avr_k0.push_back(ks[0][j]);
				avr_k0.push_back(ks[0].back());

				avr_k1.push_back(ks[1][j]);
				avr_k1.push_back(ks[1].back());

				avr_ks.push_back(avr_k0);
				avr_ks.push_back(avr_k1);
			}

			avr_wcets.push_back(wcets[j]);
			avr_wcets.push_back(wcets.back());

			initialNode->addCondition(nextMaxSpeeds[j]);

			StateGraphNode* nextNode = new StateGraphNode("TSC"+Utility::int_to_string(stateGraph->graph.size()),avr_speeds,avr_wcets,pt,avr_ks,false);

			StateGraphNode* foundNextNode = stateGraph->getExistedNode(nextNode);
			if (foundNextNode == NULL) {
				stateGraph->add(nextNode);
				initialNode->connectTo(nextNode);

				nextNode->addCondition(nextMaxSpeeds.back());
				nextNode->connectTo(initialNodes[j]);
			}
			else {
				initialNode->connectTo(foundNextNode);
				delete nextNode;
			}
		}
	}

	return stateGraph;
}

StateGraph* OptimizationAlgorithm::generateStateGraphFast(MethodChoice mc, PerformanceType pt,  vector<vector<double>> ks, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	StateGraph* stateGraph = new StateGraph(pt);
	vector<double> maxSpeeds = computeUBSpeedsFast(mc,wcets,tasks,engine,period);
	// Create the initial state nodes
	for (int i=0; i<maxSpeeds.size()-1; i++) {
		vector<double> avr_speeds;
		vector<vector<double>> avr_ks;
		vector<int> avr_wcets;

		avr_speeds.push_back(maxSpeeds[i]);
		avr_speeds.push_back(maxSpeeds.back());

		if (pt == PERF_CONSTANT) {
			vector<double> avr_k;
			avr_k.push_back(ks[0][i]);
			avr_k.push_back(ks[0].back());
			avr_ks.push_back(avr_k);
		}
		else {
			vector<double> avr_k0;
			vector<double> avr_k1;

			avr_k0.push_back(ks[0][i]);
			avr_k0.push_back(ks[0].back());

			avr_k1.push_back(ks[1][i]);
			avr_k1.push_back(ks[1].back());

			avr_ks.push_back(avr_k0);
			avr_ks.push_back(avr_k1);
		}

		avr_wcets.push_back(wcets[i]);
		avr_wcets.push_back(wcets.back());

		StateGraphNode* node = new StateGraphNode("TSC"+Utility::int_to_string(i),avr_speeds,avr_wcets,pt,avr_ks,true);
		stateGraph->add(node);
	}

	// Create the non-initial state nodes and connect them
	vector<StateGraphNode*> initialNodes = stateGraph->graph;
	for (int i = 0; i<initialNodes.size(); i++) {
		StateGraphNode* initialNode = initialNodes[i];
		AVRTask prevAvrTask(engine,period,initialNode->speeds,initialNode->wcets);
		vector<double> nextMaxSpeeds = computeUBSpeedsFast(mc,wcets,tasks,engine,period,&prevAvrTask);
		nextMaxSpeeds[i] = maxSpeeds[i];
		for (int j=0; j<nextMaxSpeeds.size()-1; j++) {
			vector<double> avr_speeds;
			vector<vector<double>> avr_ks;
			vector<int> avr_wcets;

			avr_speeds.push_back(nextMaxSpeeds[j]);
			avr_speeds.push_back(nextMaxSpeeds.back());

			if (pt == PERF_CONSTANT) {
				vector<double> avr_k;
				avr_k.push_back(ks[0][j]);
				avr_k.push_back(ks[0].back());
				avr_ks.push_back(avr_k);
			}
			else {
				vector<double> avr_k0;
				vector<double> avr_k1;

				avr_k0.push_back(ks[0][j]);
				avr_k0.push_back(ks[0].back());

				avr_k1.push_back(ks[1][j]);
				avr_k1.push_back(ks[1].back());

				avr_ks.push_back(avr_k0);
				avr_ks.push_back(avr_k1);
			}

			avr_wcets.push_back(wcets[j]);
			avr_wcets.push_back(wcets.back());

			initialNode->addCondition(nextMaxSpeeds[j]);

			StateGraphNode* nextNode = new StateGraphNode("TSC"+Utility::int_to_string(stateGraph->graph.size()),avr_speeds,avr_wcets,pt,avr_ks,false);

			StateGraphNode* foundNextNode = stateGraph->getExistedNode(nextNode);
			if (foundNextNode == NULL) {
				stateGraph->add(nextNode);
				initialNode->connectTo(nextNode);

				nextNode->addCondition(nextMaxSpeeds.back());
				nextNode->connectTo(initialNodes[j]);
			}
			else {
				initialNode->connectTo(foundNextNode);
				delete nextNode;
			}
		}
	}

	return stateGraph;
}

StateGraph* OptimizationAlgorithm::generateStateGraph(MethodChoice mc, PerformanceType pt,  vector<vector<double>> ks, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period)
{
	StateGraph* stateGraph = new StateGraph(pt);
	vector<double> ubSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	vector<double> lbSpeeds = OptimizationAlgorithm::computeBiondiBS(EXACT,ks[0],wcets, tasks, engine, period);
	vector<double> maxSpeeds;
	for (int i=0; i<ubSpeeds.size(); i++)
		maxSpeeds.push_back((ubSpeeds[i]+lbSpeeds[i])/2.0);

	/*
	Utility::output_one_vector(cout,"ubSpeed",ubSpeeds);
	Utility::output_one_vector(cout,"lbSpeed",lbSpeeds);
	Utility::output_one_vector(cout,"maxSpeed",maxSpeeds);
	*/

	// Create the initial state nodes
	for (int i=0; i<maxSpeeds.size()-1; i++) {
		vector<double> avr_speeds;
		vector<vector<double>> avr_ks;
		vector<int> avr_wcets;

		avr_speeds.push_back(maxSpeeds[i]);
		avr_speeds.push_back(maxSpeeds.back());

		if (pt == PERF_CONSTANT) {
			vector<double> avr_k;
			avr_k.push_back(ks[0][i]);
			avr_k.push_back(ks[0].back());
			avr_ks.push_back(avr_k);
		}
		else {
			vector<double> avr_k0;
			vector<double> avr_k1;

			avr_k0.push_back(ks[0][i]);
			avr_k0.push_back(ks[0].back());

			avr_k1.push_back(ks[1][i]);
			avr_k1.push_back(ks[1].back());

			avr_ks.push_back(avr_k0);
			avr_ks.push_back(avr_k1);
		}
		
		avr_wcets.push_back(wcets[i]);
		avr_wcets.push_back(wcets.back());

		StateGraphNode* node = new StateGraphNode("TSC"+Utility::int_to_string(i),avr_speeds,avr_wcets,pt,avr_ks,true);
		stateGraph->add(node);
	}

	// Create the non-initial state nodes and connect them
	vector<StateGraphNode*> initialNodes = stateGraph->graph;
	for (int i = 0; i<initialNodes.size(); i++) {
		StateGraphNode* initialNode = initialNodes[i];
		AVRTask prevAvrTask(engine,period,initialNode->speeds,initialNode->wcets);
		vector<double> nextMaxSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period,prevAvrTask,500000);
		nextMaxSpeeds[i] = maxSpeeds[i];
		for (int j=0; j<nextMaxSpeeds.size()-1; j++) {
			vector<double> avr_speeds;
			vector<vector<double>> avr_ks;
			vector<int> avr_wcets;

			avr_speeds.push_back(nextMaxSpeeds[j]);
			avr_speeds.push_back(nextMaxSpeeds.back());

			if (pt == PERF_CONSTANT) {
				vector<double> avr_k;
				avr_k.push_back(ks[0][j]);
				avr_k.push_back(ks[0].back());
				avr_ks.push_back(avr_k);
			}
			else {
				vector<double> avr_k0;
				vector<double> avr_k1;

				avr_k0.push_back(ks[0][j]);
				avr_k0.push_back(ks[0].back());

				avr_k1.push_back(ks[1][j]);
				avr_k1.push_back(ks[1].back());

				avr_ks.push_back(avr_k0);
				avr_ks.push_back(avr_k1);
			}

			avr_wcets.push_back(wcets[j]);
			avr_wcets.push_back(wcets.back());

			initialNode->addCondition(nextMaxSpeeds[j]);

			StateGraphNode* nextNode = new StateGraphNode("TSC"+Utility::int_to_string(stateGraph->graph.size()),avr_speeds,avr_wcets,pt,avr_ks,false);

			StateGraphNode* foundNextNode = stateGraph->getExistedNode(nextNode);
			if (foundNextNode == NULL) {
				stateGraph->add(nextNode);
				initialNode->connectTo(nextNode);

				nextNode->addCondition(nextMaxSpeeds.back());
				nextNode->connectTo(initialNodes[j]);
			}
			else {
				initialNode->connectTo(foundNextNode);
				delete nextNode;
			}
		}
	}

	return stateGraph;
}

void OptimizationAlgorithm::enforceModeGuards(vector<double>& maxSpeeds)
{
	for(int i=maxSpeeds.size()-2; i>=0; i--)
	{
		if(maxSpeeds[i]>maxSpeeds[i+1]-MODE_GUARD_RPM)
			maxSpeeds[i] = maxSpeeds[i+1]-MODE_GUARD_RPM;
	}
}

void OptimizationAlgorithm::enforceModeReduced(vector<double>& speeds, double cStep)
{
	for(int i=0; i<speeds.size()-1; i++)
	{
		if (speeds[i] > 0) {
			speeds[i] = 0;
			break;
		}
	}
}

void OptimizationAlgorithm::doLocalSearch(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformance(avrTask->speeds,k,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

	// Sort modes depending on their performance coefficients
	vector<double> gradientCoeffs = getPerformanceDerivatives(k);
	vector<pair<int,double>> performanceCoeffs;

	for(unsigned int i = 0; i < k.size()-1; i++)
	{
		pair<int,double> newPair(i,gradientCoeffs[i]);
		performanceCoeffs.push_back(newPair);
	}

	sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//------------------------------------------------

	// Perform local search as a binary search
	//------------------------------------------------
	for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
	{
		unsigned int mode = performanceCoeffs[index].first;
		double lb = avrTask->speeds[mode];
		double leftSpeed = lb;
		double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
		cout << "Mode = " << mode << endl;
		cout << "Left = " << leftSpeed << endl;
		cout << "Right = " << rightSpeed << endl;
#endif

		while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
		{
#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			double speed = (rightSpeed+leftSpeed)/2.0;

			avrTask->speeds[mode] = speed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

			if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
			{ 
#ifdef __DEBUG_DESIGN__
				cout << "No solution" << endl; 
#endif
				break;  
			}

#ifdef __DEBUG_DESIGN__
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

				if(schedulable) 
					leftSpeed = speed;
				else rightSpeed = speed;
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}
		}

		avrTask->speeds[mode] = leftSpeed;
		avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
	}
#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Local Search Times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchFP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine, int avrTaskIndex) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformance(avrTask->speeds,k,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

	// Sort modes depending on their performance coefficients
	vector<double> gradientCoeffs = getPerformanceDerivatives(k);
	vector<pair<int,double>> performanceCoeffs;

	for(unsigned int i = 0; i < k.size()-1; i++)
	{
		pair<int,double> newPair(i,gradientCoeffs[i]);
		performanceCoeffs.push_back(newPair);
	}

	sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//------------------------------------------------

	// Perform local search as a binary search
	//------------------------------------------------
	for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
	{
		unsigned int mode = performanceCoeffs[index].first;
		double lb = avrTask->speeds[mode];
		double leftSpeed = lb;
		double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
		cout << "Mode = " << mode << endl;
		cout << "Left = " << leftSpeed << endl;
		cout << "Right = " << rightSpeed << endl;
#endif

		while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
		{
#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			double speed = (rightSpeed+leftSpeed)/2.0;

			avrTask->speeds[mode] = speed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

			if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
			{ 
#ifdef __DEBUG_DESIGN__
				cout << "No solution" << endl; 
#endif
				break;  
			}

#ifdef __DEBUG_DESIGN__
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
				schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks,*avrTask,avrTaskIndex);

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

				if(schedulable) 
					leftSpeed = speed;
				else rightSpeed = speed;
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}
		}

		avrTask->speeds[mode] = leftSpeed;
		avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
	}
#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Local Search Times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = avrTask->speeds[mode];
			double leftSpeed = lb;
			double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftSpeed << endl;
			cout << "Right = " << rightSpeed << endl;
#endif

			while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double speed = (rightSpeed+leftSpeed)/2.0;

				avrTask->speeds[mode] = speed;
				avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

				if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif

					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftSpeed = speed;
					else rightSpeed = speed;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			avrTask->speeds[mode] = leftSpeed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchExp2(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			double speed = avrTask->speeds[mode] + BS_SEARCH_STEP_RPM;
			if (speed >= avrTask->speeds[mode+1] || speed >= UBSpeeds[mode]) continue;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif
				//Timer timer;
				//timer.start();
				//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
				schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, *avrTask);
				//timer.end();
				//cout << timer.getTime() << endl;

				//cout << avrTask->digraph_rbf.size() << endl;
#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
				times++;
				cout << times << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move anymore the current mode

			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchExp2FP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine, int avrTaskIndex) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			double speed = avrTask->speeds[mode] + BS_SEARCH_STEP_RPM;
			if (speed >= avrTask->speeds[mode+1] || speed >= UBSpeeds[mode]) continue;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif
				//Timer timer;
				//timer.start();
				//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
				schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, *avrTask, avrTaskIndex);
				//timer.end();
				//cout << timer.getTime() << endl;

				//cout << avrTask->digraph_rbf.size() << endl;
#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
				times++;
				cout << times << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move anymore the current mode

			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchPoly(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExp(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesPoly(k1,k2,k3,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = avrTask->speeds[mode];
			double leftSpeed = lb;
			double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftSpeed << endl;
			cout << "Right = " << rightSpeed << endl;
#endif

			while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
			{

				double speed = (rightSpeed+leftSpeed)/2.0;

				avrTask->speeds[mode] = speed;
				avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

				if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif

					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftSpeed = speed;
					else rightSpeed = speed;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			avrTask->speeds[mode] = leftSpeed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
		}
	}
}

void OptimizationAlgorithm::doLocalSearch(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<AsynchronousPeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformance(avrTask->speeds,k,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

	// Sort modes depending on their performance coefficients
	vector<double> gradientCoeffs = getPerformanceDerivatives(k);
	vector<pair<int,double>> performanceCoeffs;

	for(unsigned int i = 0; i < k.size()-1; i++)
	{
		pair<int,double> newPair(i,gradientCoeffs[i]);
		performanceCoeffs.push_back(newPair);
	}

	sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//------------------------------------------------

	// Perform local search as a binary search
	//------------------------------------------------
	for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
	{
		unsigned int mode = performanceCoeffs[index].first;
		double lb = avrTask->speeds[mode];
		double leftSpeed = lb;
		double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
		cout << "Mode = " << mode << endl;
		cout << "Left = " << leftSpeed << endl;
		cout << "Right = " << rightSpeed << endl;
#endif

		while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
		{
#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			double speed = (rightSpeed+leftSpeed)/2.0;

			avrTask->speeds[mode] = speed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

			if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
			{ 
#ifdef __DEBUG_DESIGN__
				cout << "No solution" << endl; 
#endif
				break;  
			}

#ifdef __DEBUG_DESIGN__
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

				if(schedulable) 
					leftSpeed = speed;
				else rightSpeed = speed;
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}
		}

		avrTask->speeds[mode] = leftSpeed;
		avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
	}
#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Local Search Times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchExp2(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<AsynchronousPeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			double speed = avrTask->speeds[mode] + BS_SEARCH_STEP_RPM;
			if (speed >= avrTask->speeds[mode+1] || speed >= UBSpeeds[mode]) continue;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif
				//Timer timer;
				//timer.start();
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
				//timer.end();
				//cout << timer.getTime() << endl;

				//cout << avrTask->digraph_rbf.size() << endl;
#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
				times++;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move anymoe the current mode

			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = avrTask->speeds[mode];
			double leftSpeed = lb;
			double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftSpeed << endl;
			cout << "Right = " << rightSpeed << endl;
#endif

			while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double speed = (rightSpeed+leftSpeed)/2.0;

				avrTask->speeds[mode] = speed;
				avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

				if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif

					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftSpeed = speed;
					else rightSpeed = speed;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			avrTask->speeds[mode] = leftSpeed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchExpMin2(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			double speed = avrTask->speeds[mode] + BS_SEARCH_STEP_RPM;
			if (speed >= avrTask->speeds[mode+1] || speed >= UBSpeeds[mode]) continue;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move anymoe the current mode

			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesDCExp(dc,k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double performanceCoeff = performanceCoeffs[index].second;

			if (fabs(performanceCoeff) < Utility::EPSILON) continue;

			double lb = avrTask->speeds[mode];
			double leftSpeed = lb;
			double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftSpeed << endl;
			cout << "Right = " << rightSpeed << endl;
#endif

			while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double speed = (rightSpeed+leftSpeed)/2.0;

				avrTask->speeds[mode] = speed;
				avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

				if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif

					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftSpeed = speed;
					else rightSpeed = speed;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			avrTask->speeds[mode] = leftSpeed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

void OptimizationAlgorithm::doLocalSearchDCExpMin2(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<PeriodicTask> tasks, AVRTask* avrTask, Engine engine) {
	double p;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move anymoe the current mode

			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif
}

vector<double> OptimizationAlgorithm::doLowerSpeeds(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k,period);
	/*
	int factor = 2;
	for (auto& e: reducedSpeeds)
	e *= factor;
	*/
#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks, avrTask);
	if (!schedulable) schedulable =	SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);

	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

		pTask->updateReducedSpeeds(reducedSpeeds);

#ifdef __DEBUG_VERIFICATION_TIMES__
		times ++;
#endif

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);

			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks, avrTask);
			if (!schedulable) schedulable =	SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);

#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Lower Speeds Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLocalSearch(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformance(avrTask->speeds,k,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

	// Sort modes depending on their performance coefficients
	vector<double> gradientCoeffs = getPerformanceDerivatives(k);
	vector<pair<int,double>> performanceCoeffs;

	for(unsigned int i = 0; i < k.size()-1; i++)
	{
		pair<int,double> newPair(i,gradientCoeffs[i]);
		performanceCoeffs.push_back(newPair);
	}

	sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

	//------------------------------------------------
#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	// Perform local search as a binary search
	//------------------------------------------------
	for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
	{
		unsigned int mode = performanceCoeffs[index].first;
		double lb = avrTask->speeds[mode];
		double leftSpeed = lb;
		double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
		cout << "Mode = " << mode << endl;
		cout << "Left = " << leftSpeed << endl;
		cout << "Right = " << rightSpeed << endl;
#endif

		while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
		{
#ifdef __DEBUG_VERIFICATION_TIMES__
			times ++;
#endif

			double speed = (rightSpeed+leftSpeed)/2.0;

			avrTask->speeds[mode] = speed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

			if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
			{ 
#ifdef __DEBUG_DESIGN__
				cout << "No solution" << endl; 
#endif
				break;  
			}

#ifdef __DEBUG_DESIGN__
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

				if(schedulable) 
					leftSpeed = speed;
				else rightSpeed = speed;
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}
		}

		avrTask->speeds[mode] = leftSpeed;
		avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Local Search Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLowerSpeeds_CollectInfo(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, double &times) {
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k,period);

	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks, avrTask);
	if (!schedulable) schedulable =	SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
	times ++;

	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);

			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks, avrTask);
			if (!schedulable) schedulable =	SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
			times ++;

#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Lower Speeds Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLocalSearch_CollectInfo(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, double &times) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformance(avrTask->speeds,k,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

	// Sort modes depending on their performance coefficients
	vector<double> gradientCoeffs = getPerformanceDerivatives(k);
	vector<pair<int,double>> performanceCoeffs;

	for(unsigned int i = 0; i < k.size()-1; i++)
	{
		pair<int,double> newPair(i,gradientCoeffs[i]);
		performanceCoeffs.push_back(newPair);
	}

	sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

	//------------------------------------------------
	// Perform local search as a binary search
	//------------------------------------------------
	for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
	{
		unsigned int mode = performanceCoeffs[index].first;
		double lb = avrTask->speeds[mode];
		double leftSpeed = lb;
		double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
		cout << "Mode = " << mode << endl;
		cout << "Left = " << leftSpeed << endl;
		cout << "Right = " << rightSpeed << endl;
#endif

		while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
		{

			double speed = (rightSpeed+leftSpeed)/2.0;

			avrTask->speeds[mode] = speed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

			if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
			{ 
#ifdef __DEBUG_DESIGN__
				cout << "No solution" << endl; 
#endif
				break;  
			}

#ifdef __DEBUG_DESIGN__
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB, tasks, *avrTask);
				if (!schedulable) schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
				times ++;

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif

				if(schedulable) 
					leftSpeed = speed;
				else rightSpeed = speed;
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}
		}

		avrTask->speeds[mode] = leftSpeed;
		avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Local Search Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBS(MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
#ifdef __UB_USING_EXACT_ANALYSIS__
	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
#else
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
#endif
	enforceModeGuards(UBSpeeds);
	return computeBiondiBS(mc,UBSpeeds,k,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBS(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformance(UBSpeeds,k,engine);
	cout << "pUB = " << pUB << endl;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
	vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
	vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k,period);
#endif


	AVRTask* pTask = &avrTask;
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);

	int times = 0;
	while (!schedulable) {
		AVRTask lastAVR = *pTask;
		times ++;
		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

#if 1
		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}
#endif

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);

#ifdef __DEBUG_STATISTICS__
			if (schedulable) schedNum++;
			else nonSchedNum++;
#endif

#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}
	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------
#ifdef __LS_USING_EXACT_ANALYSIS__
	doLocalSearch(EXACT,UBSpeeds,k,tasks,pTask,engine);
#else
	doLocalSearch(mc,UBSpeeds,k,tasks,pTask,engine);
#endif



#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformance(pTask->speeds,k,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBSFP(MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex) {
#ifdef __UB_USING_EXACT_ANALYSIS__
	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
#else
	vector<double> UBSpeeds = computeUBFPSpeeds(mc,wcets,tasks,engine,period,avrTaskIndex);
#endif
	enforceModeGuards(UBSpeeds);
	return computeBiondiBSFP(mc,UBSpeeds,k,wcets,tasks,engine,period,avrTaskIndex);
}

vector<double> OptimizationAlgorithm::computeBiondiBSFP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformance(UBSpeeds,k,engine);
	cout << "pUB = " << pUB << endl;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
	vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
	vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k,period);
#endif


	AVRTask* pTask = &avrTask;
	bool schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, *pTask,avrTaskIndex); 

	int times = 0;
	while (!schedulable) {
		AVRTask lastAVR = *pTask;
		times ++;
		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

#if 1
		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}
#endif

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

			//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);
			schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, *pTask,avrTaskIndex);

#ifdef __DEBUG_STATISTICS__
			if (schedulable) schedNum++;
			else nonSchedNum++;
#endif

#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}
	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------
#ifdef __LS_USING_EXACT_ANALYSIS__
	doLocalSearch(EXACT,UBSpeeds,k,tasks,pTask,engine);
#else
	doLocalSearch(mc,UBSpeeds,k,tasks,pTask,engine);
#endif

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformance(pTask->speeds,k,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBS(MethodChoice mc, vector<double> k, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period) {
#ifdef __UB_USING_EXACT_ANALYSIS__
	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
#else
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
#endif
	enforceModeGuards(UBSpeeds);
	return computeBiondiBS(mc,UBSpeeds,k,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBS(MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformance(UBSpeeds,k,engine);
	cout << "pUB = " << pUB << endl;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
	vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
	vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k,period);
#endif


	AVRTask* pTask = &avrTask;
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);

	int times = 0;
	while (!schedulable) {
		AVRTask lastAVR = *pTask;
		times ++;
		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);

#ifdef __DEBUG_STATISTICS__
			if (schedulable) schedNum++;
			else nonSchedNum++;
#endif

#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}
	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------
#ifdef __LS_USING_EXACT_ANALYSIS__
	doLocalSearch(EXACT,UBSpeeds,k,tasks,pTask,engine);
#else
	doLocalSearch(mc,UBSpeeds,k,tasks,pTask,engine);
#endif



#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformance(pTask->speeds,k,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLowerSpeedsExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//bool schedulable = false;
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times ++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLocalSearchExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExp(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = avrTask->speeds[mode];
			double leftSpeed = lb;
			double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftSpeed << endl;
			cout << "Right = " << rightSpeed << endl;
#endif

			while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double speed = (rightSpeed+leftSpeed)/2.0;

				avrTask->speeds[mode] = speed;
				avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

				if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif

					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftSpeed = speed;
					else rightSpeed = speed;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			avrTask->speeds[mode] = leftSpeed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLowerSpeedsExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//bool schedulable = false;
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times ++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> reducedSpeeds = calStepsMin(UBSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLocalSearchExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExp(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = avrTask->speeds[mode];
			double leftSpeed = lb;
			double rightSpeed = UBSpeeds[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftSpeed << endl;
			cout << "Right = " << rightSpeed << endl;
#endif

			while(rightSpeed-leftSpeed>BS_BIN_SEARCH_PRECISION_RPM)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double speed = (rightSpeed+leftSpeed)/2.0;

				avrTask->speeds[mode] = speed;
				avrTask->omegas[mode] = RPM_to_RPmSEC(speed);

				if(fabs(speed-lb)<BS_BIN_SEARCH_PRECISION_RPM) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif

					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);

#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftSpeed = speed;
					else rightSpeed = speed;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			avrTask->speeds[mode] = leftSpeed;
			avrTask->omegas[mode] = RPM_to_RPmSEC(leftSpeed);
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLocalSearchExpMin2(MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;

			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);
#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move anymoe the current mode

			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLowerSpeedsDCExp(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//bool schedulable = false;
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times ++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> reducedSpeeds = calStepsDC(dc,UBSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}


vector<double> OptimizationAlgorithm::doLocalSearchDCExp(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesDCExp(dc,k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			double speed = avrTask->speeds[mode] + BS_SEARCH_STEP_RPM;
			if (speed >= avrTask->speeds[mode+1] || speed >= UBSpeeds[mode]) continue;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;
			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);

				#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
				#endif

				//cout << "+++++++++++++++" << endl;

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move any step for the current mode
			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::doLowerSpeedsDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif
	//bool schedulable = false;
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times ++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> reducedSpeeds = calStepsDCMin(dc,UBSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds Times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}


vector<double> OptimizationAlgorithm::doLocalSearchDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> LBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	double p;

	AVRTask avr(engine,period,LBSpeeds,wcets);
	AVRTask* avrTask = &avr;

#ifdef __DEBUG_DESIGN__
	p = getPerformanceExpGsl(avrTask->speeds,k1,k2,engine);

	cout << "END (first phase)" << endl;

	cout << "Performance = " << p << endl;
	Utility::output_one_map(cout,"Current Speed", avrTask->speeds);

	cout << "[##] Local Search" << endl;
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> speeds;
		for (auto e: avrTask->speeds)
			speeds.push_back(e.second);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesDCExp(dc,k1,k2,speeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;

			double speed = avrTask->speeds[mode] + BS_SEARCH_STEP_RPM;
			if (speed >= avrTask->speeds[mode+1] || speed >= UBSpeeds[mode]) continue;

			avrTask->speeds[mode] += BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] += BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			Utility::output_one_map(cout,"Current Speed", avrTask->speeds);
#endif

			bool schedulable = false;
			try 
			{
#ifdef __DEBUG_DESIGN__
				cout << "Checking schedulability...";
#endif

				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *avrTask);

#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				//cout << "+++++++++++++++" << endl;

#ifdef __DEBUG_DESIGN__
				cout << "DONE!" << endl;
#endif
			}
			catch(exception& e) 
			{
				cout << e.what() << endl;
				throw e;
			}


			if (schedulable) {
				atLeastOneUpdate = true;
				break;
			}

			// OTHERWISE, we cannot move any step for the current mode
			avrTask->speeds[mode] -= BS_SEARCH_STEP_RPM;
			avrTask->omegas[mode] -= BS_SEARCH_STEP;

#ifdef __DEBUG_DESIGN__
			cout << "End while" << endl;
#endif
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search times = " << times << endl;
#endif

	vector<double> BSSpeeds;
	for (auto it = avrTask->speeds.begin(); it != avrTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBSExp(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	return computeBiondiBSExp(mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBSExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformanceExp(UBSpeeds,k1,k2,engine);
	cout << "pUB = " << pUB << endl;
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	int uIndex = 0;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
		cout << times << endl;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> currSpeeds;
		for (auto e : pTask->speeds)
			currSpeeds.push_back(e.second);

		vector<double> reducedSpeeds = calSteps(currSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

#if 1
		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}
#endif

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			Timer timer;
			timer.start();
#endif

			//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
			schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, *pTask);

#ifdef __DEBUG_VERIFICATION_TIMES__
			timer.end();
			cout << timer.getTime() << endl;
#endif

			/*
			int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
			int AVRIndex = 0;
			for (int i=0; i<tasks.size(); i++) {
				if (tasks[i].deadline < AVRDeadline)
					AVRIndex = i+1;
			}

			Timer timer;
			timer.start();
			schedulable = SchedAnalysis::exact_analysis(tasks,avrTask,AVRIndex,uIndex);
			timer.end();
			cout << "Time = " << timer.getTime() << endl;
			*/
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

#ifdef __DEBUG_DESIGN__
	cout << "Before the local search!" << endl;
	vector<double> temp_speeds;
	for (auto e:pTask->speeds)
		temp_speeds.push_back(e.second);
	Utility::output_one_vector(cout,"current speeds",temp_speeds);
#endif

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearchExp2(mc,UBSpeeds,k1,k2,tasks,pTask,engine);

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceExp(pTask->speeds,k1,k2,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBSExpFP(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex) {
	vector<double> UBSpeeds = computeUBFPSpeeds(mc,wcets,tasks,engine,period,avrTaskIndex);
	enforceModeGuards(UBSpeeds);
	return computeBiondiBSExpFP(mc,UBSpeeds,k1,k2,wcets,tasks,engine,period,avrTaskIndex);
}

vector<double> OptimizationAlgorithm::computeBiondiBSExpFP(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int avrTaskIndex) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformanceExp(UBSpeeds,k1,k2,engine);
	cout << "pUB = " << pUB << endl;
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	int uIndex = 0;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
		cout << times << endl;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> currSpeeds;
		for (auto e : pTask->speeds)
			currSpeeds.push_back(e.second);

		vector<double> reducedSpeeds = calSteps(currSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

#if 1
		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}
#endif

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

#ifdef __DEBUG_VERIFICATION_TIMES__
			Timer timer;
			timer.start();
#endif

			//schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
			schedulable = SchedAnalysis::checkSchedulabilityBiondiFP(mc,tasks, *pTask, avrTaskIndex);

#ifdef __DEBUG_VERIFICATION_TIMES__
			timer.end();
			cout << timer.getTime() << endl;
#endif

			/*
			int AVRDeadline = mSEC_to_muSEC(avrTask.calDeadline(avrTask.engine.SPEED_MAX));
			int AVRIndex = 0;
			for (int i=0; i<tasks.size(); i++) {
				if (tasks[i].deadline < AVRDeadline)
					AVRIndex = i+1;
			}

			Timer timer;
			timer.start();
			schedulable = SchedAnalysis::exact_analysis(tasks,avrTask,AVRIndex,uIndex);
			timer.end();
			cout << "Time = " << timer.getTime() << endl;
			*/
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

#ifdef __DEBUG_DESIGN__
	cout << "Before the local search!" << endl;
	vector<double> temp_speeds;
	for (auto e:pTask->speeds)
		temp_speeds.push_back(e.second);
	Utility::output_one_vector(cout,"current speeds",temp_speeds);
#endif

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearchExp2FP(mc,UBSpeeds,k1,k2,tasks,pTask,engine,avrTaskIndex);

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceExp(pTask->speeds,k1,k2,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBSExp(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	return computeBiondiBSExp(mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBSExp(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformanceExp(UBSpeeds,k1,k2,engine);
	cout << "pUB = " << pUB << endl;
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> currSpeeds;
		for (auto e : pTask->speeds)
			currSpeeds.push_back(e.second);

		vector<double> reducedSpeeds = calSteps(currSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			//Timer timer;
			//timer.start();
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
			//timer.end();
			//cout << timer.getTime() << endl;
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

#ifdef __DEBUG_DESIGN__
	cout << "Before the local search!" << endl;
	vector<double> temp_speeds;
	for (auto e:pTask->speeds)
		temp_speeds.push_back(e.second);
	Utility::output_one_vector(cout,"current speeds",temp_speeds);
#endif

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearchExp2(mc,UBSpeeds,k1,k2,tasks,pTask,engine);

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceExp(pTask->speeds,k1,k2,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBSExpMin(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	return computeBiondiBSExpMin(mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBSExpMin(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformanceExp(UBSpeeds,k1,k2,engine);
	cout << "pUB = " << pUB << endl;
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> currSpeeds;
		for (auto e : pTask->speeds)
			currSpeeds.push_back(e.second);

		vector<double> reducedSpeeds = calStepsMin(currSpeeds,wcets,k1,k2,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

#ifdef __DEBUG_DESIGN__
	cout << "Before the local search!" << endl;
	Utility::output_one_vector(cout,"max speeds", UBSpeeds);

	vector<double> temp_speeds;
	for (auto e:pTask->speeds)
		temp_speeds.push_back(e.second);
	Utility::output_one_vector(cout,"current speeds",temp_speeds);
#endif

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearchExpMin2(mc,UBSpeeds,k1,k2,tasks,pTask,engine);

#ifdef __DEBUG_DESIGN__
	cout << "After the local search!" << endl;

	vector<double> temp_speeds2;
	for (auto e:pTask->speeds)
		temp_speeds2.push_back(e.second);
	Utility::output_one_vector(cout,"final speeds",temp_speeds2);
#endif

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceExp(pTask->speeds,k1,k2,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBiondiBSPoly3(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	return computeBiondiBSPoly3(mc,UBSpeeds,k1,k2,k3,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBSPoly3(MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformancePoly(UBSpeeds,k1,k2,k3,engine);
	cout << "pUB = " << pUB << endl;
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k1,k2,k3,period);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_DESIGN__
	cout << "Before the local search!" << endl;
	vector<double> temp_speeds;
	for (auto e:pTask->speeds)
		temp_speeds.push_back(e.second);
	Utility::output_one_vector(cout,"current speeds",temp_speeds);
#endif

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearchPoly(mc,UBSpeeds,k1,k2,k3,tasks,pTask,engine);

	// Retrieve the final performance
	double p = getPerformancePoly(pTask->speeds,k1,k2,k3,engine);

#ifdef __DEBUG_DESIGN__
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

double OptimizationAlgorithm::findMaxSpeed(MethodChoice mc, vector<PeriodicTask> tasks, AVRTask avrTask, double lb, double ub, unsigned int mode) {
	AVRTask* pTask = &avrTask;

	double leftSpeed = lb;
	double rightSpeed = ub;

	double speed;

	bool schedulable;

	while(rightSpeed-leftSpeed>BIN_SEARCH_PRECISION_RPM)
	{

		speed = (rightSpeed+leftSpeed)/2.0;

		pTask->speeds[mode] = speed;
		pTask->omegas[mode] = RPM_to_RPmSEC(speed);
		for(unsigned int i=mode+1; i<pTask->speeds.size()-1; i++) {
			pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
			pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]);
		}

		if(fabs(speed-lb)<BIN_SEARCH_PRECISION_RPM)
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "No solution" << endl; 
#endif
			return lb-BIN_SEARCH_PRECISION_RPM;  
		}

#ifdef __DEBUG_DESIGN__
		for(auto mode : pTask->speeds)
			cout << "Current speeds: " << mode.second << ", ";
		cout << endl;
#endif
		schedulable = false;

		try 
		{

#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

#ifndef __BB_USE_NECESSARY_ONLY__
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#else
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(NECESSARY_ONLY2,tasks,*pTask);

			if (schedulable) schedNum ++;
			else nonSchedNum ++;

			if (schedulable) schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);

			//cout << schedulable << endl;
#endif
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif

			if(schedulable) 
			{
				leftSpeed = speed;
			}
			else rightSpeed = speed;
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			while(1);
		}
	}
	return leftSpeed;
}

double OptimizationAlgorithm::findMaxSpeed(MethodChoice mc, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask, double lb, double ub, unsigned int mode) {
	AVRTask* pTask = &avrTask;

	double leftSpeed = lb;
	double rightSpeed = ub;

	double speed;

	bool schedulable;

	while(rightSpeed-leftSpeed>BIN_SEARCH_PRECISION_RPM)
	{

		speed = (rightSpeed+leftSpeed)/2.0;

		pTask->speeds[mode] = speed;
		pTask->omegas[mode] = RPM_to_RPmSEC(speed);
		for(unsigned int i=mode+1; i<pTask->speeds.size()-1; i++) {
			pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
			pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]);
		}

		if(fabs(speed-lb)<BIN_SEARCH_PRECISION_RPM)
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "No solution" << endl; 
#endif
			return lb-BIN_SEARCH_PRECISION_RPM;  
		}

#ifdef __DEBUG_DESIGN__
		for(auto mode : pTask->speeds)
			cout << "Current speeds: " << mode.second << ", ";
		cout << endl;
#endif
		schedulable = false;

		try 
		{

#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif

#ifndef __BB_USE_NECESSARY_ONLY__
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#else
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(NECESSARY_ONLY2,tasks,*pTask);

			if (schedulable) schedNum ++;
			else nonSchedNum ++;

			if (schedulable) schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks, *pTask);

			//cout << schedulable << endl;
#endif
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif

			if(schedulable) 
			{
				leftSpeed = speed;
			}
			else rightSpeed = speed;
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			while(1);
		}
	}
	return leftSpeed;
}

double OptimizationAlgorithm::getUBPerformance(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformance(speeds,k,avrTask.engine);
}

void OptimizationAlgorithm::doBiondiBBSearch(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformance(pTask->speeds,k,pTask->engine);

		if(curPerformance>maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformance(*pTask,mode,maxSpeeds,k);

		// PRUNING
		if(nextMaxPerformance<=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearch(mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBB(MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check whether there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBS(mc,UBSpeeds,k,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	double perfLB = getPerformance(BSSpeeds,k,engine);

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearch(mc,perfLB,UBSpeeds,tasks,avrTask,0,k,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

void OptimizationAlgorithm::doBiondiBBSearch(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<AsynchronousPeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformance(pTask->speeds,k,pTask->engine);

		if(curPerformance>maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformance(*pTask,mode,maxSpeeds,k);

		// PRUNING
		if(nextMaxPerformance<=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearch(mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBB(MethodChoice mc, vector<double> k, vector<int> wcets, vector<AsynchronousPeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check whether there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBS(mc,UBSpeeds,k,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	double perfLB = getPerformance(BSSpeeds,k,engine);

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearch(mc,perfLB,UBSpeeds,tasks,avrTask,0,k,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

double OptimizationAlgorithm::getUBPerformanceExp(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	//return getPerformanceExp(speeds,k1,k2,avrTask.engine);
	return getPerformanceExpGsl(speeds,k1,k2,avrTask.engine);
}

void OptimizationAlgorithm::doBiondiBBSearchExp(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) {
		if (mode != 0) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
		else lowerBound = pTask->engine.RPM_MIN + SPEED_PRECISION_RPM;
	}
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceExpGsl(pTask->speeds,k1,k2,pTask->engine);

		if(curPerformance>maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformanceExp(*pTask,mode,maxSpeeds,k1,k2);

		// PRUNING
		if(nextMaxPerformance<=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchExp(mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBExp(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

#ifdef __BB_USING_PREPROCESS__
	vector<double> BSSpeeds = computeBiondiBSExp(mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}
	double perfLB = getPerformanceExp(BSSpeeds,k1,k2,engine);
#else
	vector<double> BSSpeeds;
	double perfLB = -1.0;
#endif

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchExp(mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

double OptimizationAlgorithm::getUBPerformanceExpMin(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformanceExpGsl(speeds,k1,k2,avrTask.engine);
}

void OptimizationAlgorithm::doBiondiBBSearchExpMin(MethodChoice mc, double& minPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) {
		if (mode != 0) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
		else lowerBound = pTask->engine.RPM_MIN + SPEED_PRECISION_RPM;
	}
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceExpGsl(pTask->speeds,k1,k2,pTask->engine);

		if(curPerformance<minPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			minPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMinPerformance = getUBPerformanceExpMin(*pTask,mode,maxSpeeds,k1,k2);

		// PRUNING
		if(nextMinPerformance>=minPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif
			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchExpMin(mc,minPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBExpMin(MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

#ifdef __BB_USING_PREPROCESS__
	vector<double> BSSpeeds = computeBiondiBSExpMin(mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}
	double perfLB = getPerformanceExpGsl(BSSpeeds,k1,k2,engine);
#else
	vector<double> BSSpeeds;
	double perfLB = INT_MAX;
#endif

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchExpMin(mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else {
		return BSSpeeds;
	}
}

double OptimizationAlgorithm::getUBPerformancePoly3(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformancePoly(speeds,k1,k2,k3,avrTask.engine);
}

void OptimizationAlgorithm::doBiondiBBSearchPoly3(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformancePoly(pTask->speeds,k1,k2,k3,pTask->engine);

		if(curPerformance < maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformancePoly3(*pTask,mode,maxSpeeds,k1,k2,k3);

		// PRUNING
		if(nextMaxPerformance>=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchPoly3(mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,k3,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBPoly3(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBSPoly3(mc,UBSpeeds,k1,k2,k3,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	//double perfLB = getPerformancePoly(BSSpeeds,k1,k2,k3,engine);
	double perfLB = INT_MAX;

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchPoly3(mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,k3,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

double OptimizationAlgorithm::getUBPerformancePoly4(AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformancePoly(speeds,k1,k2,k3,k4,avrTask.engine);
}

void OptimizationAlgorithm::doBiondiBBSearchPoly4(MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformancePoly(pTask->speeds,k1,k2,k3,k4,pTask->engine);

		if(curPerformance < maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformancePoly4(*pTask,mode,maxSpeeds,k1,k2,k3,k4);

		// PRUNING
		if(nextMaxPerformance>=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchPoly4(mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,k3,k4,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBPoly4(MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBS(mc,UBSpeeds,k1,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	//double perfLB = getPerformancePoly(BSSpeeds,k1,k2,k3,k4,engine);
	double perfLB = INT_MAX;

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchPoly4(mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,k3,k4,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

double OptimizationAlgorithm::getDrivingCycleUBPerformance(map<int,int> dc, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformanceDC(dc,speeds,k);
}

void OptimizationAlgorithm::doDrivingCycleBBSearch(map<int,int> dc, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceDC(dc, pTask->speeds,k);

		if(curPerformance>maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getDrivingCycleUBPerformance(dc,*pTask,mode,maxSpeeds,k);

		// PRUNING
		if(nextMaxPerformance<=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doDrivingCycleBBSearch(dc,mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k,result);
	}
}

vector<double> OptimizationAlgorithm::computeDrivingCycleBB(map<int,int> dc, MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBS(mc,UBSpeeds,k,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	double perfLB = getPerformanceDC(dc,BSSpeeds,k);

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doDrivingCycleBBSearch(dc,mc,perfLB,UBSpeeds,tasks,avrTask,0,k,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

double OptimizationAlgorithm::getUBPerformanceDCExp(map<int,int> dc, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformanceDCExp(dc,speeds,k1,k2);
}

void OptimizationAlgorithm::doBiondiBBSearchDCExp(map<int,int> dc, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) {
		if (mode != 0) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
		else lowerBound = pTask->engine.RPM_MIN + SPEED_PRECISION_RPM;
	}
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceDCExp(dc,pTask->speeds,k1,k2);

		if(curPerformance>maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformanceDCExp(dc,*pTask,mode,maxSpeeds,k1,k2);

		// PRUNING
		if(nextMaxPerformance<=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchDCExp(dc,mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBDCExp(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

#ifdef __BB_USING_PREPROCESS__
	vector<double> BSSpeeds = computeBiondiBS(mc,UBSpeeds,k1,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}
	double perfLB = getPerformanceDCExp(dc,BSSpeeds,k1,k2);
#else	
	vector<double> BSSpeeds;
	double perfLB = -1.0;
#endif

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchDCExp(dc,mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

void OptimizationAlgorithm::doBiondiBBSearchDCExpMin(map<int,int> dc, MethodChoice mc, double& minPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) {
		if (mode != 0) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
		else lowerBound = pTask->engine.RPM_MIN + SPEED_PRECISION_RPM;
	}
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceDCExp(dc,pTask->speeds,k1,k2);

		if(curPerformance<minPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			minPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMinPerformance = getUBPerformanceDCExp(dc,*pTask,mode,maxSpeeds,k1,k2);

		// PRUNING
		if(nextMinPerformance>=minPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif
			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchDCExpMin(dc,mc,minPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

#ifdef __BB_USING_PREPROCESS__
	//vector<double> BSSpeeds = computeBiondiBSDCExpMin(dc,mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	vector<double> BSSpeeds = computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}
	double perfLB = getPerformanceDCExp(dc,BSSpeeds,k1,k2);
#else
	vector<double> BSSpeeds;
	double perfLB = INT_MAX;
#endif

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchDCExpMin(dc,mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else {
		return BSSpeeds;
	}
}

double OptimizationAlgorithm::getUBPerformanceDCPoly3(map<int,int> dc, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformanceDCPoly3(dc,speeds,k1,k2,k3);
}

void OptimizationAlgorithm::doBiondiBBSearchDCPoly3(map<int,int> dc, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceDCPoly3(dc,pTask->speeds,k1,k2,k3);

		if(curPerformance<maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);
		double nextMaxPerformance = getUBPerformanceDCPoly3(dc,*pTask,mode,maxSpeeds,k1,k2,k3);

		// PRUNING
		if(nextMaxPerformance>=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchDCPoly3(dc,mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,k3,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBDCPoly3(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBS(mc,UBSpeeds,k1,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	//double perfLB = getPerformanceDCExp(dc,BSSpeeds,k1,k2);
	double perfLB = INT_MAX;

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchDCPoly3(dc,mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,k3,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

double OptimizationAlgorithm::getUBPerformanceDCPoly4(vector<double> dc_speeds, vector<double> dc_torques, AVRTask avrTask, unsigned int mode, vector<double> maxSpeeds, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4) {
	vector<double> speeds;
	for (int i=0; i<=mode; i++) speeds.push_back(avrTask.speeds[i]);
	for (int i=mode+1; i<maxSpeeds.size(); i++) speeds.push_back(maxSpeeds[i]);

	return getPerformanceDCPoly(dc_speeds,dc_torques,speeds,k1,k2,k3,k4);
}

void OptimizationAlgorithm::doBiondiBBSearchDCPoly4(vector<double> dc_speeds, vector<double> dc_torques, MethodChoice mc, double& maxPerformance, vector<double> &maxSpeeds, vector<PeriodicTask> &tasks, AVRTask &avrTask, unsigned int mode, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<double>& result) {
#ifdef __DEBUG_DESIGN__
	cout << "new branch, mode = " << mode << endl;
#endif

	AVRTask* pTask = &avrTask;

	double lowerBound;

	// In the exploration of this sub-tree, the speed of (mode+1)-th mode is fixed
	// (it's the root of this sub-tree), hence it is used as a lower-bound
	if(mode!=0)
		lowerBound = pTask->speeds[mode-1] + SEARCH_PRECISION_RPM; 
	else  
		//Except for the last mode that has W_MIN as (implicit) lower-bound
		lowerBound = pTask->engine.RPM_MIN + SEARCH_PRECISION_RPM;

	if (lowerBound >= maxSpeeds[mode]) lowerBound = pTask->speeds[mode-1] + SPEED_PRECISION_RPM;
	double maxSpeed = findMaxSpeed(mc,tasks,avrTask,lowerBound,maxSpeeds[mode],mode);

	if(maxSpeed<lowerBound) 
	{
#ifdef __DEBUG_DESIGN__
		cout << "Unfeasible branch" << endl;
#endif

		return;
	}

	if (mode==k1.size()-2)
	{
#ifdef __DEBUG_DESIGN__
		cout << "Last mode reached" << endl;
#endif

		pTask->speeds[mode] = maxSpeed;
		pTask->omegas[mode] = RPM_to_RPmSEC(maxSpeed);

		double curPerformance = getPerformanceDCPoly(dc_speeds,dc_torques,pTask->speeds,k1,k2,k3,k4);

		if(curPerformance < maxPerformance) 
		{
#ifdef __DEBUG_DESIGN__
			cout << "[###] Performance updated to: " << curPerformance << endl;
#endif

			// Update the best solution found so far
			maxPerformance = curPerformance;
			result.clear();
			for(auto mode : pTask->speeds)
				result.push_back(mode.second);
		}

		return;
	}

	//cout << "start = " << W_to_RPM(maxSpeed) << endl;
	//cout << "stop  = " << W_to_RPM(lowerBound) << endl;

	for(double w=maxSpeed; w>= lowerBound ; w-=SEARCH_PRECISION_RPM)
	{
		pTask->speeds[mode] = w;
		pTask->omegas[mode] = RPM_to_RPmSEC(w);


		double nextMaxPerformance = getUBPerformanceDCPoly4(dc_speeds,dc_torques,*pTask,mode,maxSpeeds,k1,k2,k3,k4);

		// PRUNING
		if(nextMaxPerformance>=maxPerformance) 
		{ 
#ifdef __DEBUG_DESIGN__
			cout << "Branch killed" << endl; 
#endif

			break;
		}

#ifdef __DEBUG_DESIGN__
		cout << "[###] Opening a new branch..." << endl;
#endif

		// NOT PRUNED, open a new branch
		doBiondiBBSearchDCPoly4(dc_speeds,dc_torques,mc,maxPerformance, maxSpeeds, tasks, avrTask, mode+1,k1,k2,k3,k4,result);
	}
}

vector<double> OptimizationAlgorithm::computeBiondiBBDCPoly4(vector<double> dc_speeds, vector<double> dc_torques, MethodChoice mc, vector<double> k1, vector<double> k2, vector<double> k3, vector<double> k4, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> result;

	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	// Check wheter there exists a feasible solution
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	vector<double> BSSpeeds = computeBiondiBSPoly3(mc,UBSpeeds,k1,k2,k4,wcets,tasks,engine,period);
	// Check whether there exists a feasbile solution for BS
	if (BSSpeeds.empty()) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	//double perfLB = getPerformanceDCPoly(dc_speeds,dc_torques,BSSpeeds,k1,k2,k3,k4);
	double perfLB = INT_MAX;

	AVRTask avrTask(engine,period,UBSpeeds,wcets);

	doBiondiBBSearchDCPoly4(dc_speeds,dc_torques,mc,perfLB,UBSpeeds,tasks,avrTask,0,k1,k2,k3,k4,result);

	// If no better solution has been found the result vector is empty!
	if(result.size()>0) 
	{

#ifdef __DEBUG_DESIGN__
		cout << "END" << endl;
		for(auto w : result)
			cout << w << ", ";
		cout << endl;
#endif

		return result;
	}
	else return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeChaoBS(map<int,int> dc, MethodChoice mc, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	// Check wheter there exists a feasible solution
	vector<double> result;
	if (UBSpeeds.front() < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
		cout << "ERROR: DESIGN IMPOSSIBLE!" << endl; 
#endif
		return result;
	}

	return computeChaoBS(dc,mc,UBSpeeds,k,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeChaoBS(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformanceDC(dc,UBSpeeds,k);
	cout << "pUB = " << pUB << endl;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
	vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
	vector<double> reducedSpeeds = calSteps(UBSpeeds,wcets,k,period);
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearch(mc,UBSpeeds,k,tasks,pTask,engine);

	// Retrieve the final performance
	double p = getPerformanceDC(dc,pTask->speeds,k);

#ifdef __DEBUG_DESIGN__
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

void OptimizationAlgorithm::enforceDCGuards(map<int,int> dc, vector<double>& speeds) {
	map<int,int>::iterator iter = dc.begin();
	for (int i=0; i<speeds.size()-1; i++) {
		int temp = -1;
		if (iter==dc.end()) break;
		while (iter->first <= speeds[i]) {
			temp = iter->first;
			iter++;
			if (iter == dc.end()) {
				if (temp != -1) speeds[i] = temp;
				goto ENFORCEDCGUARDS_END;
			}
		}
		if (temp != -1) speeds[i] = temp;
	}

ENFORCEDCGUARDS_END:
	iter = dc.end();
	iter--;
	int maxSpeed = iter->first+1;
	for (int i=0; i<speeds.size()-1; i++) {
		if (speeds[i] >= maxSpeed) {
			speeds[i] = maxSpeed;
			maxSpeed++;
		}
	}
}

vector<double> OptimizationAlgorithm::calStepsDC(map<int,int> dc, vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period) {
	vector<double> ret;

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesDCExp(dc,k1,k2,speeds);

	double minU = *min_element(utils.begin(), utils.end());
	double maxU = *max_element(utils.begin(), utils.end());
	double normalizedU = maxU - minU;

	/*
	double minGradientCoeff = *min_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double maxGradientCoeff = *max_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;
	*/

	double minGradientCoeff = INT_MAX;
	double maxGradientCoeff = INT_MIN;
	double sumGradientCoeff = 0.0;

	double sumU = accumulate(utils.begin(),utils.end(),0.0);

	for (auto e: gradientCoeffs) {
		if (fabs(e) <= Utility::EPSILON) 
			continue;
		minGradientCoeff = min(minGradientCoeff,e);
		maxGradientCoeff = max(maxGradientCoeff,e);
		sumGradientCoeff += e;
	}
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif
	/*
	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(gradientCoeffs[i]) <= Utility::EPSILON) 
			step = BS_DC_SEARCH_STEP_RPM;
		else {
			step = BS_DC_SEARCH_STEP_RPM
				* ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
				+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);

		}
		ret.push_back(max(step,MINIMUM_STEP));
	}
	*/
	
	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(gradientCoeffs[i]) <= Utility::EPSILON) 
			step = BS_DC_SEARCH_STEP_RPM;
		else {
			step = BS_DC_SEARCH_STEP_RPM
				* ((1.0-STARV_WEIGHT)*utils[i]/sumU + STARV_WEIGHT 
				+ (1.0-STARV_WEIGHT2)*gradientCoeffs[i]/sumGradientCoeff + STARV_WEIGHT2);

		}
		ret.push_back(max(step,MINIMUM_STEP));
	}
	

	return ret;
}

vector<double> OptimizationAlgorithm::calStepsDCMin(map<int,int> dc, vector<double> speeds, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period) {
	vector<double> ret;

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesDCExp(dc,k1,k2,speeds);

	double minU = *min_element(utils.begin(), utils.end());
	double maxU = *max_element(utils.begin(), utils.end());
	double normalizedU = maxU - minU;

	/*
	double minGradientCoeff = *min_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double maxGradientCoeff = *max_element(gradientCoeffs.begin(), gradientCoeffs.end());
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;
	*/

	double minGradientCoeff = INT_MAX;
	double maxGradientCoeff = INT_MIN;
	double sumGradientCoeff = 0.0;

	double sumU = accumulate(utils.begin(),utils.end(),0.0);

	for (auto e: gradientCoeffs) {
		if (fabs(e) <= Utility::EPSILON) 
			continue;
		minGradientCoeff = min(minGradientCoeff,e);
		maxGradientCoeff = max(maxGradientCoeff,e);
		sumGradientCoeff += e;
	}
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif
	/*
	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(gradientCoeffs[i]) <= Utility::EPSILON) 
			step = BS_DC_SEARCH_STEP_RPM;
		else {
			step = BS_DC_SEARCH_STEP_RPM
				* ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
				+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);

		}
		ret.push_back(max(step,MINIMUM_STEP));
	}
	*/

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(gradientCoeffs[i]) <= Utility::EPSILON) 
			step = BS_DC_SEARCH_STEP_RPM;
		else {
			step = BS_DC_SEARCH_STEP_RPM
				* ((1.0-STARV_WEIGHT)*utils[i]/sumU + STARV_WEIGHT 
				+ (1.0-STARV_WEIGHT2)*gradientCoeffs[i]/sumGradientCoeff + STARV_WEIGHT2);

		}
		ret.push_back(max(step,MINIMUM_STEP));
	}

	return ret;
}

vector<double> OptimizationAlgorithm::computeBiondiBSDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	//enforceDCGuards(dc,UBSpeeds);
	return computeBiondiBSDCExpMin(dc,mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBiondiBSDCExpMin(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	AVRTask avrTask(engine,period,UBSpeeds,wcets);

#ifdef __DEBUG_DESIGN__
	double pUB = getPerformanceExp(UBSpeeds,k1,k2,engine);
	cout << "pUB = " << pUB << endl;
#endif

	bool schedulable = false;
	AVRTask* pTask = &avrTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	while (!schedulable) {
		AVRTask lastAVR = *pTask;

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
#endif

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		vector<double> currSpeeds;
		for (auto e : pTask->speeds)
			currSpeeds.push_back(e.second);

		vector<double> reducedSpeeds = calStepsDCMin(dc,currSpeeds,wcets,k1,k2,period);
#endif
#ifdef __DEBUG_DESIGN__
		Utility::output_one_vector(cout,"Reduced Speeds = ", reducedSpeeds);
#endif

		pTask->updateReducedSpeeds(reducedSpeeds);

		// Enforce constraint
		for (int i =  wcets.size()-2; i > 0; i--) {
			if(pTask->speeds[i] - MODE_GUARD_RPM < pTask->speeds[i-1]) {
				pTask->speeds[i] = pTask->speeds[i-1] + MODE_GUARD_RPM;
				pTask->omegas[i] = RPM_to_RPmSEC(pTask->speeds[i]); 
			}
		}

		for (auto e: pTask->speeds) {
			if (e.second < engine.RPM_MIN) {
#ifdef __DEBUG_DESIGN__
				cout << "Unfeasible solution!" << endl;
				Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif
				vector<double> ret;
				return ret;
			}
		}

		bool atLeastOneUpdate = false;
		for(unsigned int i=0; i < wcets.size(); i++) 
			if(!W_EQ_TOL(pTask->omegas[i],lastAVR.omegas[i]))
				atLeastOneUpdate = true;

		if(!atLeastOneUpdate) break;

#ifdef __DEBUG_DESIGN__
		Utility::output_one_map(cout,"Current Speeds = ", pTask->speeds);
#endif

		try 
		{		
#ifdef __DEBUG_DESIGN__
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *pTask);
#ifdef __DEBUG_DESIGN__
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

#ifdef __DEBUG_DESIGN__
	cout << "Before the local search!" << endl;
	vector<double> temp_speeds;
	for (auto e:pTask->speeds)
		temp_speeds.push_back(e.second);
	Utility::output_one_vector(cout,"current speeds",temp_speeds);
#endif

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------

	doLocalSearchExpMin2(mc,UBSpeeds,k1,k2,tasks,pTask,engine);

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceExp(pTask->speeds,k1,k2,engine);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
	cout << "Lower-Bound on Optimality Ratio = " << (p/pUB)*100.0 << "%" << endl;
	//for(unsigned int i=0; i < pTask->modes.size(); i++)
	//cout << "Mode " << i << " ==> w = " << W_to_RPM(pTask->modes[i].second) << " RPM" << endl;
	pTask->output(cout);
#endif

	vector<double> BSSpeeds;
	for (auto it = pTask->speeds.begin(); it != pTask->speeds.end(); it++)
		BSSpeeds.push_back(it->second);
	return BSSpeeds;
}

double OptimizationAlgorithm::getPerformanceDCExpWithESVSF(map<int,int>dc, EngineSpeedVirtualStepFunction esvsf, vector<double> virtualSteps, vector<double> k1, vector<double> k2)
{
	vector<double> speeds = esvsf.getEngineSpeeds(virtualSteps);
	return getPerformanceDCExp(dc,speeds,k1,k2);
}

vector<double> OptimizationAlgorithm::calVirtualStepsMax(EngineSpeedVirtualStepFunction esvsf, vector<double> virtualSteps, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period) {
	vector<double> ret;

	vector<double> speeds = esvsf.getEngineSpeeds(virtualSteps);
	int nonZeroNum = 0;
	for (auto e:speeds) {
		if (e > 0) nonZeroNum++;
	}
	if (nonZeroNum == 0) {
		cerr << "Error Speeds." << endl;
		Utility::output_one_vector(cerr,"Speeds",speeds);
		exit(EXIT_FAILURE);
	}
	if (nonZeroNum == 1) return ret;
	if (nonZeroNum == 2) {
		bool used = false;
		for (auto e:speeds) {
			if (e > 0 && !used) {
				ret.push_back(0.01);
				used = true;
			}
			else
				ret.push_back(0.0);
		}
		return ret;
	}

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);

	double minU = INT_MAX;
	double maxU = INT_MIN;

	for (int i=0; i<utils.size()-1; i++) {
		double e = utils[i];
		if (fabs(e) <= Utility::EPSILON)
			continue;
		minU = min(minU,e);
		maxU = max(maxU,e);
	}

	double normalizedU = maxU - minU;

	double minGradientCoeff = INT_MAX;
	double maxGradientCoeff = INT_MIN;

	for (int i=0; i<gradientCoeffs.size()-1; i++) {
		double e = gradientCoeffs[i];
		if (fabs(e) <= Utility::EPSILON) 
			continue;
		minGradientCoeff = min(minGradientCoeff,e);
		maxGradientCoeff = max(maxGradientCoeff,e);
	}
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(utils[i]) > Utility::EPSILON) {
			step = ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
				+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);
			step /= 100.0;
		}
		ret.push_back(step);
	}

	// max mode
	ret.push_back(0.0);
	return ret;
}

std::vector<double> OptimizationAlgorithm::doLowerSpeedsDCExpMaxWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask)
{
	vector<double> currentVirtualSteps = UBVirtualSteps;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool schedulable = false;
	while (!schedulable) {
		AVRTask avrTask(esvsf,engine,period,currentVirtualSteps,wcets);
		//avrTask.outputSpeeds(cout);

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
#endif

		try 
		{		
//#ifdef __DEBUG_DESIGN__
#if 0
			cout << "Checking schedulability...";
#endif
			if (prevAvrTask != NULL)
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, *prevAvrTask, &avrTask);
			else
				schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks,avrTask);
//#ifdef __DEBUG_DESIGN__
#if 0
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}

		if (schedulable) break;

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		/*
		if (avrTask.getModeNumberWithNonZero() == 1) { // cannot reduce the speed
			vector<double> empty;
			return empty;
		}
		*/

		vector<double> reducedVirtualSteps = calVirtualStepsMax(esvsf,currentVirtualSteps,wcets,k1,k2,period);
		if (reducedVirtualSteps.empty()) return reducedVirtualSteps;
		//cout << "Cal reduced virtual steps..." << endl;
#endif
#ifdef __DEBUG_DESIGN__
		Utility::output_one_vector(cout,"Reduced Virtual Steps", reducedVirtualSteps);
#endif

		currentVirtualSteps = esvsf.getNextVirtualSteps(currentVirtualSteps,reducedVirtualSteps);
		//cout << "Cal next virtual steps..." << endl;
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

	return currentVirtualSteps;
}

std::vector<double> OptimizationAlgorithm::doLocalSearchDCExpMaxWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> LBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask)
{
	vector<double> currentVirtualSteps = LBVirtualSteps;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> currentEngineSpeeds = esvsf.getEngineSpeeds(currentVirtualSteps);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,currentEngineSpeeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			if (fabs(gradientCoeffs[i]) <= Utility::EPSILON)
				continue;
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparator);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = currentVirtualSteps[mode];
			double leftVirtualStep = lb;
			double rightVirtualStep = UBVirtualSteps[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftVirtualStep << endl;
			cout << "Right = " << rightVirtualStep << endl;
#endif

			while(rightVirtualStep-leftVirtualStep>BS_BIN_SEARCH_PRECISION_VIRTUAL_STEP)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double virtualStep = (rightVirtualStep+leftVirtualStep)/2.0;

				currentVirtualSteps[mode] = virtualStep; 

				if(fabs(virtualStep-lb)<BS_BIN_SEARCH_PRECISION_VIRTUAL_STEP) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_vector(cout,"Current Virtual Steps", currentVirtualSteps);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif
					AVRTask avrTask(esvsf,engine,period,currentVirtualSteps,wcets);
					if (prevAvrTask != NULL)
						schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, *prevAvrTask, &avrTask);
					else
						schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftVirtualStep = virtualStep;
					else rightVirtualStep = virtualStep;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			currentVirtualSteps[mode] = leftVirtualStep;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search Times = " << times << endl;
#endif

	return currentVirtualSteps;
}

vector<double> OptimizationAlgorithm::computeBSDCExpMaxWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	//enforceDCGuards(dc,UBSpeeds);
	return computeBSDCExpMaxWithESVSF(dc,mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBSDCExpMaxWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	EngineSpeedVirtualStepFunction esvsf(dc,engine);
	vector<double> initialVirtualSteps = esvsf.getInitialVirtualSteps(UBSpeeds);

	//------------------------------------------------
	//----------------[ LOWER SPEEDS ]----------------
	//------------------------------------------------
	vector<double> currentVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(mc,esvsf,initialVirtualSteps,k1,k2,wcets,tasks,engine,period);

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------
	currentVirtualSteps = doLocalSearchDCExpMaxWithESVSF(mc,esvsf,initialVirtualSteps,currentVirtualSteps,k1,k2,wcets,tasks,engine,period);

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceDCExpWithESVSF(dc,esvsf,currentVirtualSteps,k1,k2);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
#endif

	return esvsf.getEngineSpeeds(currentVirtualSteps);
}

std::vector<double> OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(map<int,int> dc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask)
{
	vector<double> MAXVirtualSteps = UBVirtualSteps;
	vector<double> BSVirtualSteps;

	AVRTask avrTaskLevel0(esvsf,engine,period,UBVirtualSteps,wcets);

#if 0 // With optimization
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0,prevAvrTask);
	if (schedulable) goto UBVIRTUALSTEPS_END;

	{
		// NECESSARY ONLY 2A
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(NECESSARY_ONLY2A,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period,prevAvrTask);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMaxWithESVSF(NECESSARY_ONLY2A,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period, prevAvrTask);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel1(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1,prevAvrTask);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// NECESSARY ONLY 1A
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(NECESSARY_ONLY1A,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period,prevAvrTask);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMaxWithESVSF(NECESSARY_ONLY1A,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel2(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2,prevAvrTask);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// NECESSARY ONLY 1
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(NECESSARY_ONLY1,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period,prevAvrTask);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMaxWithESVSF(NECESSARY_ONLY1,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period,prevAvrTask);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel3(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3,prevAvrTask);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// EXACT ANALYSIS
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(EXACT,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period,prevAvrTask);
	}

#else // Without optimization

	// EXACT Analysis
	BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(EXACT,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period,prevAvrTask);
	if (BSVirtualSteps.empty()) return BSVirtualSteps;

	BSVirtualSteps = doLocalSearchDCExpMaxWithESVSF(EXACT,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period, prevAvrTask);
	if (BSVirtualSteps.empty()) return BSVirtualSteps;

#endif

BSVIRTUALSTEPS_END:
	return BSVirtualSteps;

UBVIRTUALSTEPS_END:
	return UBVirtualSteps;
}

std::vector<double> OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, AVRTaskPointer prevAvrTask)
{
	EngineSpeedVirtualStepFunction esvsf(dc,engine);

	vector<double> UBSpeeds;
	//UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period,prevAvrTask);
	//UBSpeeds = computeUBSpeeds(SEG_NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	//Utility::output_one_vector(cout,"UBSpeeds",UBSpeeds);

	double LBPerf = INT_MIN;
	
	//vector<double> UBVirtualSteps = esvsf.getInitialVirtualSteps(UBSpeeds);
	vector<double> retVirtualSteps;

	vector<vector<double>> virtualStepsVector = esvsf.getVirtualStepsVector(UBSpeeds);

	for (auto e : virtualStepsVector) {
		double currentPerf = getPerformanceDCExpWithESVSF(dc,esvsf,e,k1,k2);
		if (currentPerf < LBPerf) continue;

		vector<double> BSVirtualSteps = computeBSDCExpMaxWithESVSF_NECESSARYONLY(dc,esvsf,e,k1,k2,wcets,tasks,engine,period,prevAvrTask);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		double BSPerf = getPerformanceDCExpWithESVSF(dc,esvsf,BSVirtualSteps,k1,k2);

		if (BSPerf > LBPerf) {
			LBPerf = BSPerf;
			retVirtualSteps = BSVirtualSteps;
		}
	}
	return esvsf.getEngineSpeeds(retVirtualSteps);
}

std::vector<double> OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(map<int,int> dc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	vector<double> MAXVirtualSteps = UBVirtualSteps;
	vector<double> BSVirtualSteps;

	AVRTask avrTaskLevel0(esvsf,engine,period,UBVirtualSteps,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBVIRTUALSTEPS_END;

	{
		// SEGMENTAL NECESSARY ONLY 1A
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(SEG_NECESSARY_ONLY1A,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMaxWithESVSF(SEG_NECESSARY_ONLY1A,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel2(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// SEGMENTAL NECESSARY ONLY 1
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(SEG_NECESSARY_ONLY1,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMaxWithESVSF(SEG_NECESSARY_ONLY1,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel3(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// RBF ANALYSIS
		UBVirtualSteps = BSVirtualSteps;
		//BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(DIGRAPH_RBF,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
		BSVirtualSteps = doLowerSpeedsDCExpMaxWithESVSF(SEG_LUB_NECESSARY_ONLY1,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
	}

BSVIRTUALSTEPS_END:
	return BSVirtualSteps;

UBVIRTUALSTEPS_END:
	return UBVirtualSteps;
}

std::vector<double> OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	EngineSpeedVirtualStepFunction esvsf(dc,engine);

	vector<double> UBSpeeds;
	//UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	//UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	//UBSpeeds = computeUBSpeeds(SEG_NECESSARY_ONLY1A,wcets,tasks,engine,period);
	UBSpeeds = computeUBSpeedsFast(SEG_LUB_NECESSARY_ONLY1,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	double LBPerf = INT_MIN;

	//vector<double> UBVirtualSteps = esvsf.getInitialVirtualSteps(UBSpeeds);
	vector<double> retVirtualSteps;

	vector<vector<double>> virtualStepsVector = esvsf.getVirtualStepsVector(UBSpeeds);

	for (auto e : virtualStepsVector) {
		double currentPerf = getPerformanceDCExpWithESVSF(dc,esvsf,e,k1,k2);
		if (currentPerf < LBPerf) continue;

		vector<double> BSVirtualSteps = computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(dc,esvsf,e,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		double BSPerf = getPerformanceDCExpWithESVSF(dc,esvsf,BSVirtualSteps,k1,k2);

		if (BSPerf > LBPerf) {
			LBPerf = BSPerf;
			retVirtualSteps = BSVirtualSteps;
		}
	}
	return esvsf.getEngineSpeeds(retVirtualSteps);
}

vector<double> OptimizationAlgorithm::calVirtualStepsMin(EngineSpeedVirtualStepFunction esvsf, vector<double> virtualSteps, vector<int> wcets, vector<double> k1, vector<double> k2, double angular_period) {
	vector<double> ret;

	vector<double> speeds = esvsf.getEngineSpeeds(virtualSteps);
	int nonZeroNum = 0;
	for (auto e:speeds) {
		if (e > 0) nonZeroNum++;
	}
	if (nonZeroNum == 0) {
		cerr << "Error Speeds." << endl;
		Utility::output_one_vector(cerr,"Speeds",speeds);
		exit(EXIT_FAILURE);
	}
	if (nonZeroNum == 1) return ret;
	if (nonZeroNum == 2) {
		bool used = false;
		for (auto e:speeds) {
			if (e > 0 && !used) {
				ret.push_back(0.01);
				used = true;
			}
			else
				ret.push_back(0.0);
		}
		return ret;
	}

	vector<double> utils = getUtilizations(speeds,wcets,angular_period);
	vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,speeds);

	double minU = INT_MAX;
	double maxU = INT_MIN;
	double sumU = 0.0;

	for (int i=0; i<utils.size()-1; i++) {
		double e = utils[i];
		if (fabs(e) <= Utility::EPSILON)
			continue;
		minU = min(minU,e);
		maxU = max(maxU,e);
		sumU += e;
	}

	double normalizedU = maxU - minU;

	double minGradientCoeff = INT_MAX;
	double maxGradientCoeff = INT_MIN;
	double sumGradientCoeff = 0.0;

	for (int i=0; i<gradientCoeffs.size()-1; i++) {
		double e = gradientCoeffs[i];
		if (fabs(e) <= Utility::EPSILON) 
			continue;
		minGradientCoeff = min(minGradientCoeff,e);
		maxGradientCoeff = max(maxGradientCoeff,e);
		sumGradientCoeff += e;
	}
	double normalizedGradientCoeff = maxGradientCoeff - minGradientCoeff;

#ifdef __DEBUG_DESIGN__
	cout << "minU = " << minU << endl;
	cout << "maxU = " << maxU << endl;
	cout << "minGradientCoeff = " << minGradientCoeff << endl;
	cout << "maxGradientCoeff = " << maxGradientCoeff << endl;
	cout << "normalizedU = " << normalizedU << endl;
	cout << "normalizedGradientCoeff = " << normalizedGradientCoeff << endl;
#endif

	/*
	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(utils[i]) > Utility::EPSILON) {
			step = ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
				+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);
			step /= 100.0;
		}
		ret.push_back(step);
	}
	*/

	for (unsigned int i = 0; i < speeds.size()-1; i++) {
		double step = 0.0;
		if (fabs(utils[i]) > Utility::EPSILON) {
			step = ((1.0-STARV_WEIGHT)*(utils[i]-minU)/normalizedU + STARV_WEIGHT 
				+ (maxGradientCoeff-gradientCoeffs[i])/normalizedGradientCoeff);
			step /= 100.0;
		}
		ret.push_back(step);
	}

	// max mode
	ret.push_back(0.0);
	return ret;
}

std::vector<double> OptimizationAlgorithm::doLowerSpeedsDCExpMinWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	vector<double> currentVirtualSteps = UBVirtualSteps;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool schedulable = false;
	while (!schedulable) {
		AVRTask avrTask(esvsf,engine,period,currentVirtualSteps,wcets);
		//avrTask.outputSpeeds(cout);

#ifdef __DEBUG_VERIFICATION_TIMES__
		times++;
#endif

		try 
		{		
//#ifdef __DEBUG_DESIGN__
#if 0
			cout << "Checking schedulability...";
#endif
			schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc,tasks, avrTask);
//#ifdef __DEBUG_DESIGN__
#if 0
			cout << "DONE!" << endl;
#endif
		}
		catch(exception& e) 
		{
			cout << e.what() << endl;
			throw e;
		}

		if (schedulable) break;

#ifdef __DEBUG_CONSTANT_STEP__
		vector<double> reducedSpeeds(wcets.size()-1,CONSTANT_REDUCED_STEP);
#else
		/*
		if (avrTask.getModeNumberWithNonZero() == 1) { // cannot reduce the speed
			vector<double> empty;
			return empty;
		}
		*/

		vector<double> reducedVirtualSteps = calVirtualStepsMin(esvsf,currentVirtualSteps,wcets,k1,k2,period);
		if (reducedVirtualSteps.empty()) return reducedVirtualSteps;
		//cout << "Cal reduced virtual steps..." << endl;
#endif
#ifdef __DEBUG_DESIGN__
		Utility::output_one_vector(cout,"Reduced Virtual Steps", reducedVirtualSteps);
#endif

		currentVirtualSteps = esvsf.getNextVirtualSteps(currentVirtualSteps,reducedVirtualSteps);
		//cout << "Cal next virtual steps..." << endl;
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Lower Speeds times = " << times << endl;
#endif

	return currentVirtualSteps;
}

std::vector<double> OptimizationAlgorithm::doLocalSearchDCExpMinWithESVSF(MethodChoice mc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> LBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	vector<double> currentVirtualSteps = LBVirtualSteps;

#ifdef __DEBUG_VERIFICATION_TIMES__
	int times = 0;
#endif

	bool atLeastOneUpdate = true;

	while (atLeastOneUpdate) {

#ifdef __DEBUG_DESIGN__
		cout << "Begin external while" << endl;
#endif

		atLeastOneUpdate = false;

		vector<double> currentEngineSpeeds = esvsf.getEngineSpeeds(currentVirtualSteps);

		// Sort modes depending on their performance coefficients
		vector<double> gradientCoeffs = getPerformanceDerivativesExp(k1,k2,currentEngineSpeeds);
		vector<pair<int,double>> performanceCoeffs;

		for(unsigned int i = 0; i < k1.size()-1; i++)
		{
			if (fabs(gradientCoeffs[i]) <= Utility::EPSILON)
				continue;
			pair<int,double> newPair(i,gradientCoeffs[i]);
			performanceCoeffs.push_back(newPair);
		}

		//------------------------------------------------
		sort(performanceCoeffs.begin(),performanceCoeffs.end(),performanceCoeffsComparatorMin);

		// Perform local search as a binary search
		//------------------------------------------------
		for(unsigned int index=0; index<performanceCoeffs.size() ; index++)
		{
			unsigned int mode = performanceCoeffs[index].first;
			double lb = currentVirtualSteps[mode];
			double leftVirtualStep = lb;
			double rightVirtualStep = UBVirtualSteps[mode];

#ifdef __DEBUG_DESIGN__
			cout << "Mode = " << mode << endl;
			cout << "Left = " << leftVirtualStep << endl;
			cout << "Right = " << rightVirtualStep << endl;
#endif

			while(rightVirtualStep-leftVirtualStep>BS_BIN_SEARCH_PRECISION_VIRTUAL_STEP)
			{
#ifdef __DEBUG_VERIFICATION_TIMES__
				times ++;
#endif

				double virtualStep = (rightVirtualStep+leftVirtualStep)/2.0;

				currentVirtualSteps[mode] = virtualStep; 

				if(fabs(virtualStep-lb)<BS_BIN_SEARCH_PRECISION_VIRTUAL_STEP) 
				{ 
#ifdef __DEBUG_DESIGN__
					cout << "No solution" << endl; 
#endif
					break;  
				}

#ifdef __DEBUG_DESIGN__
				Utility::output_one_vector(cout,"Current Virtual Steps", currentVirtualSteps);
#endif

				bool schedulable = false;

				try 
				{
#ifdef __DEBUG_DESIGN__
					cout << "Checking schedulability...";
#endif
					AVRTask avrTask(esvsf,engine,period,currentVirtualSteps,wcets);
					schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(mc, tasks, avrTask);
#ifdef __DEBUG_DESIGN__
					cout << "DONE!" << endl;
#endif

					if(schedulable) 
						leftVirtualStep = virtualStep;
					else rightVirtualStep = virtualStep;
				}
				catch(exception& e) 
				{
					cout << e.what() << endl;
					throw e;
				}

				if (schedulable) atLeastOneUpdate = true;
			}

			currentVirtualSteps[mode] = leftVirtualStep;
		}
	}

#ifdef __DEBUG_VERIFICATION_TIMES__
	cout << "Exp Local Search Times = " << times << endl;
#endif

	return currentVirtualSteps;
}

vector<double> OptimizationAlgorithm::computeBSDCExpMinWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds = computeUBSpeeds(mc,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	//enforceDCGuards(dc,UBSpeeds);
	return computeBSDCExpMinWithESVSF(dc,mc,UBSpeeds,k1,k2,wcets,tasks,engine,period);
}

vector<double> OptimizationAlgorithm::computeBSDCExpMinWithESVSF(map<int,int> dc, MethodChoice mc, vector<double> UBSpeeds, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {	
	EngineSpeedVirtualStepFunction esvsf(dc,engine);
	vector<double> initialVirtualSteps = esvsf.getInitialVirtualSteps(UBSpeeds);

	//------------------------------------------------
	//----------------[ LOWER SPEEDS ]----------------
	//------------------------------------------------
	vector<double> currentVirtualSteps = doLowerSpeedsDCExpMinWithESVSF(mc,esvsf,initialVirtualSteps,k1,k2,wcets,tasks,engine,period);

	//------------------------------------------------
	//----------------[ LOCAL SEARCH ]----------------
	//------------------------------------------------
	currentVirtualSteps = doLocalSearchDCExpMinWithESVSF(mc,esvsf,initialVirtualSteps,currentVirtualSteps,k1,k2,wcets,tasks,engine,period);

#ifdef __DEBUG_DESIGN__
	// Retrieve the final performance
	double p = getPerformanceDCExpWithESVSF(dc,esvsf,currentVirtualSteps,k1,k2);
	cout << "END" << endl;
	cout << "Performance = " << p << endl;
#endif

	return esvsf.getEngineSpeeds(currentVirtualSteps);
}

std::vector<double> OptimizationAlgorithm::computeBSDCExpMinWithESVSF_NECESSARYONLY(map<int,int> dc, EngineSpeedVirtualStepFunction esvsf, vector<double> UBVirtualSteps, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	vector<double> MAXVirtualSteps = UBVirtualSteps;
	vector<double> BSVirtualSteps;

	AVRTask avrTaskLevel0(esvsf,engine,period,UBVirtualSteps,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBVIRTUALSTEPS_END;

	{
		// NECESSARY ONLY 2A
		BSVirtualSteps = doLowerSpeedsDCExpMinWithESVSF(NECESSARY_ONLY2A,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMinWithESVSF(NECESSARY_ONLY2A,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel1(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// NECESSARY ONLY 1A
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMinWithESVSF(NECESSARY_ONLY1A,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMinWithESVSF(NECESSARY_ONLY1A,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel2(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// NECESSARY ONLY 1
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMinWithESVSF(NECESSARY_ONLY1,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		BSVirtualSteps = doLocalSearchDCExpMinWithESVSF(NECESSARY_ONLY1,esvsf,UBVirtualSteps,BSVirtualSteps,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		AVRTask avrTaskLevel3(esvsf,engine,period,BSVirtualSteps,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSVIRTUALSTEPS_END;

	{
		// EXACT ANALYSIS
		UBVirtualSteps = BSVirtualSteps;
		BSVirtualSteps = doLowerSpeedsDCExpMinWithESVSF(EXACT,esvsf,UBVirtualSteps,k1,k2,wcets,tasks,engine,period);
	}

BSVIRTUALSTEPS_END:
	return BSVirtualSteps;

UBVIRTUALSTEPS_END:
	return UBVirtualSteps;
}

std::vector<double> OptimizationAlgorithm::computeBSDCExpMinWithESVSF_NECESSARYONLY(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	EngineSpeedVirtualStepFunction esvsf(dc,engine);

	vector<double> UBSpeeds;
	//UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	double UBPerf = INT_MAX;

	//vector<double> UBVirtualSteps = esvsf.getInitialVirtualSteps(UBSpeeds);
	vector<double> retVirtualSteps;

	vector<vector<double>> virtualStepsVector = esvsf.getVirtualStepsVector(UBSpeeds);

	for (auto e : virtualStepsVector) {
		double currentPerf = getPerformanceDCExpWithESVSF(dc,esvsf,e,k1,k2);
		if (currentPerf > UBPerf) continue;

		vector<double> BSVirtualSteps = computeBSDCExpMinWithESVSF_NECESSARYONLY(dc,esvsf,e,k1,k2,wcets,tasks,engine,period);
		if (BSVirtualSteps.empty()) return BSVirtualSteps;

		double BSPerf = getPerformanceDCExpWithESVSF(dc,esvsf,BSVirtualSteps,k1,k2);

		if (BSPerf < UBPerf) {
			UBPerf = BSPerf;
			retVirtualSteps = BSVirtualSteps;
		}
	}
	return esvsf.getEngineSpeeds(retVirtualSteps);
}

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//--------------------[[[ HEURISTIC WITH NECESSARY ONLY CONDITIONS ]]]--------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
vector<double> OptimizationAlgorithm::computeBS_NECESSARYONLY(vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {


	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	double tSum = 0;
	cout << "Computing UB Speeds by using NECESSARY ONLY 1A ..." << endl;
	Timer timer;
	timer.start();
	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	timer.end();
	double tUB = timer.getTime();

	double pUB = getPerformance(UBSpeeds, k, engine);
	cout << "tUB = " << tUB << ", pUB = " << pUB << endl;

#if 0
	cout << "Computing BS Speeds by using NECESSARY ONLY 3 ..." << endl;
	timer.start();
	BSSpeeds = doLowerSpeeds(NECESSARY_ONLY3,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds3 = timer.getTime();
	tSum += tDoLowerSpeeds3;

	double pDoLowerSpeeds3 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds3 = " << tDoLowerSpeeds3 << ", pDoLowerSpeeds3 = " << pDoLowerSpeeds3 << endl;

	timer.start();
	BSSpeeds = doLocalSearch(NECESSARY_ONLY3,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch3 = timer.getTime();
	tSum += tDoLocalSearch3;

	double pDoLocalSearch3 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch3 = " << tDoLocalSearch3 << ", pDoLocalSearch2 = " << pDoLocalSearch3 << endl;
#endif

#if 0
	cout << "Computing BS Speeds by using NECESSARY ONLY 2A ..." << endl;
	timer.start();
	//UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeeds(NECESSARY_ONLY2A,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds2A = timer.getTime();
	tSum += tDoLowerSpeeds2A;

	double pDoLowerSpeeds2A = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds2A = " << tDoLowerSpeeds2A << ", pDoLowerSpeeds2A = " << pDoLowerSpeeds2A << endl;

	timer.start();
	BSSpeeds = doLocalSearch(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch2A = timer.getTime();
	tSum += tDoLocalSearch2A;

	double pDoLocalSearch2A = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch2A = " << tDoLocalSearch2A << ", pDoLocalSearch2A = " << pDoLocalSearch2A << endl;
#endif

#if 0
	cout << "Computing BS Speeds by using NECESSARY ONLY 2 ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeeds(NECESSARY_ONLY2,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds2 = timer.getTime();
	tSum += tDoLowerSpeeds2;

	double pDoLowerSpeeds2 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds2 = " << tDoLowerSpeeds2 << ", pDoLowerSpeeds2 = " << pDoLowerSpeeds2 << endl;

	timer.start();
	BSSpeeds = doLocalSearch(NECESSARY_ONLY2,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch2 = timer.getTime();
	tSum += tDoLocalSearch2;

	double pDoLocalSearch2 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch2 = " << tDoLocalSearch2 << ", pDoLocalSearch2 = " << pDoLocalSearch2 << endl;
#endif

#if 1
	cout << "Computing BS Speeds by using NECESSARY ONLY 1A ..." << endl;
	timer.start();
	//UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeeds(NECESSARY_ONLY1A,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds1A = timer.getTime();
	tSum += tDoLowerSpeeds1A;

	double pDoLowerSpeeds1A = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds1A = " << tDoLowerSpeeds1A << ", pDoLowerSpeeds1A = " << pDoLowerSpeeds1A << endl;

	timer.start();
	BSSpeeds = doLocalSearch(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch1A = timer.getTime();
	tSum += tDoLocalSearch1A;

	double pDoLocalSearch1A = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch1A = " << tDoLocalSearch1A << ", pDoLocalSearch1A = " << pDoLocalSearch1A << endl;
#endif

#if 0
	cout << "Computing BS Speeds by using NECESSARY ONLY 1 ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeeds(NECESSARY_ONLY1,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds1 = timer.getTime();
	tSum += tDoLowerSpeeds1;

	double pDoLowerSpeeds1 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds1 = " << tDoLowerSpeeds1 << ", pDoLowerSpeeds1 = " << pDoLowerSpeeds1 << endl;
#endif

#if 0
	timer.start();
	BSSpeeds = doLocalSearch(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch1 = timer.getTime();
	tSum += tDoLocalSearch1;

	double pDoLocalSearch1 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch1 = " << tDoLocalSearch1 << ", pDoLocalSearch1 = " << pDoLocalSearch1 << endl;
#endif
#if 1
	cout << "Computing BS Speeds by using EXACT ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeeds(EXACT,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds0 = timer.getTime();
	tSum += tDoLowerSpeeds0;

	double pDoLowerSpeeds0 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds0 = " << tDoLowerSpeeds0 << ", pDoLowerSpeeds0 = " << pDoLowerSpeeds0 << endl;
#endif

#if 0
	timer.start();
	BSSpeeds = doLocalSearch(EXACT,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch0 = timer.getTime();
	tSum += tDoLocalSearch0;

	double pDoLocalSearch0 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch0 = " << tDoLocalSearch0 << ", pDoLocalSearch0 = " << pDoLocalSearch0 << endl;
#endif

#if 1
	//double total = tUB + tDoLowerSpeeds3 + tDoLowerSpeeds2 + tDoLowerSpeeds1 + tDoLowerSpeeds0 + tDoLocalSearch3 + tDoLocalSearch2 + tDoLocalSearch1;
	//double total = tUB + tDoLowerSpeeds2 + tDoLowerSpeeds1 + tDoLowerSpeeds0 + tDoLocalSearch2 + tDoLocalSearch1;
	//double total = tUB + tDoLowerSpeeds1 + tDoLowerSpeeds0 + tDoLocalSearch1;
	cout << "Total time = " << tSum << endl;
#endif
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBS_NECESSARYONLY_CollectInfo(vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;
	vector<double> prevUBSpeeds;

	Timer timer;

	timer.start();
	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	timer.end();
	level0_LowerSpeeds_Runtime += timer.getTime();

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	double perfUB = getPerformance(UBSpeeds,k,engine);
	level0_LowerSpeeds_Performance += 1.0;

	//bool schedulableLevel0 = SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,avrTaskLevel0);

	if (SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,avrTaskLevel0)) {
		level0_LowerSpeeds_FinalCheckTimes ++;
		level0_LowerSpeeds_FinalCheckPerf += 1.0;
	}

	if (schedulable) goto UBSPEEDS_END;

	{
		prevUBSpeeds = UBSpeeds;

		// NECESSARY ONLY 2A
		timer.start();
		BSSpeeds = doLowerSpeeds_CollectInfo(NECESSARY_ONLY2A,UBSpeeds,k,wcets,tasks,engine,period,level1_LowerSpeeds_VerificationTimes);
		timer.end();
		level1_LowerSpeeds_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf10 = getPerformance(BSSpeeds,k,engine);
		level1_LowerSpeeds_Performance += perf10/perfUB;

		timer.start();
		BSSpeeds = doLocalSearch_CollectInfo(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period,level1_LocalSearch_VerificationTimes);
		timer.end();
		level1_LocalSearch_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf11 = getPerformance(BSSpeeds,k,engine);
		level1_LocalSearch_Performance += perf11/perfUB;

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);

		if (SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,avrTaskLevel1)) {
			level1_LowerSpeeds_FinalCheckTimes ++;
			level1_LowerSpeeds_FinalCheckPerf +=perf11/perfUB;
		}
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		UBSpeeds = BSSpeeds;
		timer.start();
		BSSpeeds = doLowerSpeeds_CollectInfo(NECESSARY_ONLY1A,UBSpeeds,k,wcets,tasks,engine,period,level2_LowerSpeeds_VerificationTimes);
		timer.end();
		level2_LowerSpeeds_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf20 = getPerformance(BSSpeeds,k,engine);
		level2_LowerSpeeds_Performance += perf20/perfUB;

		timer.start();
		BSSpeeds = doLocalSearch_CollectInfo(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period,level2_LocalSearch_VerificationTimes);
		timer.end();
		level2_LocalSearch_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf21 = getPerformance(BSSpeeds,k,engine);
		level2_LocalSearch_Performance += perf21/perfUB;

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);

		if (SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,avrTaskLevel2)) {
			level2_LowerSpeeds_FinalCheckTimes ++;
			level2_LowerSpeeds_FinalCheckPerf +=perf21/perfUB;
		}
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		prevUBSpeeds = UBSpeeds;

		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		timer.start();
		BSSpeeds = doLowerSpeeds_CollectInfo(NECESSARY_ONLY1,UBSpeeds,k,wcets,tasks,engine,period,level3_LowerSpeeds_VerificationTimes);
		timer.end();
		level3_LowerSpeeds_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf30 = getPerformance(BSSpeeds,k,engine);
		level3_LowerSpeeds_Performance += perf30/perfUB;

		timer.start();
		BSSpeeds = doLocalSearch_CollectInfo(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period,level3_LocalSearch_VerificationTimes);
		timer.end();
		level3_LocalSearch_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf31 = getPerformance(BSSpeeds,k,engine);
		level3_LocalSearch_Performance += perf31/perfUB;

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);

		if (SchedAnalysis::checkSchedulabilityAllPriorities(EXACT,tasks,avrTaskLevel3)) {
			level3_LowerSpeeds_FinalCheckTimes ++;
			level3_LowerSpeeds_FinalCheckPerf +=perf31/perfUB;
		}
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		timer.start();
		BSSpeeds = doLowerSpeeds_CollectInfo(EXACT,UBSpeeds,k,wcets,tasks,engine,period,level4_LowerSpeeds_VerificationTimes);
		timer.end();
		level4_LowerSpeeds_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf40 = getPerformance(BSSpeeds,k,engine);
		level4_LowerSpeeds_Performance += perf40/perfUB;

		timer.start();
		BSSpeeds = doLocalSearch_CollectInfo(EXACT,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period,level4_LocalSearch_VerificationTimes);
		timer.end();
		level4_LocalSearch_Runtime += timer.getTime();

		if (BSSpeeds.empty()) return BSSpeeds;

		double perf41 = getPerformance(BSSpeeds,k,engine);
		level4_LocalSearch_Performance += perf41/perfUB;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeBS_NECESSARYONLY_WithoutDebug(vector<double> k, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBSPEEDS_END;

	{
		// NECESSARY ONLY 2A
		BSSpeeds = doLowerSpeeds(NECESSARY_ONLY2A,UBSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearch(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeeds(NECESSARY_ONLY1A,UBSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearch(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeeds(NECESSARY_ONLY1,UBSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearch(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeeds(EXACT,UBSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearch(EXACT,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeBSExp_NECESSARYONLY(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	double tSum = 0;
	cout << "Computing UB Speeds by using NECESSARY ONLY 2A ..." << endl;
	Timer timer;
	timer.start();
	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	timer.end();
	double tUB = timer.getTime();

	double pUB = getPerformanceExp(UBSpeeds, k1,k2, engine);
	cout << "tUB = " << tUB << ", pUB = " << pUB << endl;

#if 1
	cout << "Computing BS Speeds by using NECESSARY ONLY 2A ..." << endl;
	timer.start();
	//UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeedsExp(NECESSARY_ONLY2A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds2A = timer.getTime();
	tSum += tDoLowerSpeeds2A;

	double pDoLowerSpeeds2A = getPerformanceExp(BSSpeeds, k1,k2, engine);
	cout << "tDoLowerSpeeds2A = " << tDoLowerSpeeds2A << ", pDoLowerSpeeds2A = " << pDoLowerSpeeds2A << endl;

	timer.start();
	BSSpeeds = doLocalSearchExp(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch2A = timer.getTime();
	tSum += tDoLocalSearch2A;

	double pDoLocalSearch2A = getPerformanceExp(BSSpeeds, k1,k2, engine);
	cout << "tDoLocalSearch2A = " << tDoLocalSearch2A << ", pDoLocalSearch2A = " << pDoLocalSearch2A << endl;
#endif

#if 0
	cout << "Computing BS Speeds by using NECESSARY ONLY 2 ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeeds(NECESSARY_ONLY2,UBSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds2 = timer.getTime();
	tSum += tDoLowerSpeeds2;

	double pDoLowerSpeeds2 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLowerSpeeds2 = " << tDoLowerSpeeds2 << ", pDoLowerSpeeds2 = " << pDoLowerSpeeds2 << endl;

	timer.start();
	BSSpeeds = doLocalSearch(NECESSARY_ONLY2,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch2 = timer.getTime();
	tSum += tDoLocalSearch2;

	double pDoLocalSearch2 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch2 = " << tDoLocalSearch2 << ", pDoLocalSearch2 = " << pDoLocalSearch2 << endl;
#endif

#if 1
	cout << "Computing BS Speeds by using NECESSARY ONLY 1A ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeedsExp(NECESSARY_ONLY1A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds1A = timer.getTime();
	tSum += tDoLowerSpeeds1A;

	double pDoLowerSpeeds1A = getPerformanceExp(BSSpeeds, k1,k2, engine);
	cout << "tDoLowerSpeeds1A = " << tDoLowerSpeeds1A << ", pDoLowerSpeeds1A = " << pDoLowerSpeeds1A << endl;

	timer.start();
	BSSpeeds = doLocalSearchExp(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch1A = timer.getTime();
	tSum += tDoLocalSearch1A;

	double pDoLocalSearch1A = getPerformanceExp(BSSpeeds, k1, k2, engine);
	cout << "tDoLocalSearch1A = " << tDoLocalSearch1A << ", pDoLocalSearch1A = " << pDoLocalSearch1A << endl;
#endif

#if 1
	cout << "Computing BS Speeds by using NECESSARY ONLY 1 ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeedsExp(NECESSARY_ONLY1,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds1 = timer.getTime();
	tSum += tDoLowerSpeeds1;

	double pDoLowerSpeeds1 = getPerformanceExp(BSSpeeds, k1,k2, engine);
	cout << "tDoLowerSpeeds1 = " << tDoLowerSpeeds1 << ", pDoLowerSpeeds1 = " << pDoLowerSpeeds1 << endl;
#endif

#if 1
	timer.start();
	BSSpeeds = doLocalSearchExp(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1, k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch1 = timer.getTime();
	tSum += tDoLocalSearch1;

	double pDoLocalSearch1 = getPerformanceExp(BSSpeeds, k1,k2, engine);
	cout << "tDoLocalSearch1 = " << tDoLocalSearch1 << ", pDoLocalSearch1 = " << pDoLocalSearch1 << endl;
#endif

#if 1
	cout << "Computing BS Speeds by using EXACT ..." << endl;
	timer.start();
	UBSpeeds = BSSpeeds;
	BSSpeeds = doLowerSpeedsExp(EXACT,UBSpeeds,k1,k2,wcets,tasks,engine,period);
	timer.end();
	double tDoLowerSpeeds0 = timer.getTime();
	tSum += tDoLowerSpeeds0;

	double pDoLowerSpeeds0 = getPerformanceExp(BSSpeeds, k1,k2, engine);
	cout << "tDoLowerSpeeds0 = " << tDoLowerSpeeds0 << ", pDoLowerSpeeds0 = " << pDoLowerSpeeds0 << endl;
#endif

#if 0
	timer.start();
	BSSpeeds = doLocalSearch(EXACT,UBSpeeds,BSSpeeds,k,wcets,tasks,engine,period);
	timer.end();
	double tDoLocalSearch0 = timer.getTime();
	tSum += tDoLocalSearch0;

	double pDoLocalSearch0 = getPerformance(BSSpeeds, k, engine);
	cout << "tDoLocalSearch0 = " << tDoLocalSearch0 << ", pDoLocalSearch0 = " << pDoLocalSearch0 << endl;
#endif

#if 1
	//double total = tUB + tDoLowerSpeeds3 + tDoLowerSpeeds2 + tDoLowerSpeeds1 + tDoLowerSpeeds0 + tDoLocalSearch3 + tDoLocalSearch2 + tDoLocalSearch1;
	//double total = tUB + tDoLowerSpeeds2 + tDoLowerSpeeds1 + tDoLowerSpeeds0 + tDoLocalSearch2 + tDoLocalSearch1;
	//double total = tUB + tDoLowerSpeeds1 + tDoLowerSpeeds0 + tDoLocalSearch1;
	cout << "Total time = " << tSum << endl;
#endif
	return BSSpeeds;
}

vector<double> OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBSPEEDS_END;

	{
		// NECESSARY ONLY 2A
		BSSpeeds = doLowerSpeedsExp(NECESSARY_ONLY2A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExp(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsExp(NECESSARY_ONLY1A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExp(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsExp(NECESSARY_ONLY1,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExp(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsExp(EXACT,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExp(EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeBSExp_NECESSARYONLY_WithoutDebug_Min(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBSPEEDS_END;

	{
		// NECESSARY ONLY 2A
		BSSpeeds = doLowerSpeedsExpMin(NECESSARY_ONLY2A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsExpMin(NECESSARY_ONLY1A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsExpMin(NECESSARY_ONLY1,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsExpMin(EXACT,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

		BSSpeeds = doLocalSearchExpMin(EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> MAXSpeeds;
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	MAXSpeeds = UBSpeeds;

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBSPEEDS_END;

	{
		// NECESSARY ONLY 2A
		BSSpeeds = doLowerSpeedsDCExp(dc,NECESSARY_ONLY2A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		BSSpeeds = doLocalSearchExp(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		BSSpeeds = doLocalSearchDCExp(dc,NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExp(dc,NECESSARY_ONLY1A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		BSSpeeds = doLocalSearchExp(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		BSSpeeds = doLocalSearchDCExp(dc,NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExp(dc,NECESSARY_ONLY1,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		//BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		BSSpeeds = doLocalSearchExp(NECESSARY_ONLY1,MAXSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		//BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		BSSpeeds = doLocalSearchDCExp(dc,NECESSARY_ONLY1,MAXSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExp(dc,EXACT,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		//BSSpeeds = doLocalSearchExpMin(EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		//BSSpeeds = doLocalSearchDCExpMin(dc,EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_Min(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> maxSpeeds;
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	double tSum = 0;
	cout << "Computing UB Speeds by using NECESSARY ONLY 2A ..." << endl;
	Timer timer;
	timer.start();
	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	maxSpeeds = UBSpeeds;
	timer.end();
	double tUB = timer.getTime();
	tSum += tUB;

	double pUB = getPerformanceDCExp(dc,UBSpeeds, k1,k2);
	cout << "tUB = " << tUB << ", pUB = " << pUB << endl;

	Utility::output_one_vector(cout,"NO1A UB: UB Speeds", UBSpeeds);

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBSPEEDS_END;

	{
		// NECESSARY ONLY 2A
		timer.start();
		BSSpeeds = doLowerSpeedsDCExpMin(dc,NECESSARY_ONLY2A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		timer.end();
		if (BSSpeeds.empty()) return BSSpeeds;

		Utility::output_one_vector(cout,"NO2A Lower: BS Speeds", BSSpeeds);
		
		double tDoLowerSpeeds2A = timer.getTime();
		tSum += tDoLowerSpeeds2A;

		double pDoLowerSpeeds2A = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLowerSpeeds2A = " << tDoLowerSpeeds2A << ", pDoLowerSpeeds2A = " << pDoLowerSpeeds2A << ", " << pUB / pDoLowerSpeeds2A << endl;

		timer.start();
#if 0
#ifndef __USING_LOCAL_SEARCH_DC__
		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
#endif
		if (BSSpeeds.empty()) return BSSpeeds;
		timer.end();

		double tDoLocalSearch2A = timer.getTime();
		tSum += tDoLocalSearch2A;

		double pDoLocalSearch2A = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLocalSearch2A = " << tDoLocalSearch2A << ", pDoLocalSearch2A = " << pDoLocalSearch2A << ", " << pUB / pDoLocalSearch2A << endl;

		Utility::output_one_vector(cout,"NO2A Local: BS Speeds", BSSpeeds);

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		timer.start();
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExpMin(dc,NECESSARY_ONLY1A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;
		timer.end();
		double tDoLowerSpeeds1A = timer.getTime();
		tSum += tDoLowerSpeeds1A;

		double pDoLowerSpeeds1A = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLowerSpeeds1A = " << tDoLowerSpeeds1A << ", pDoLowerSpeeds1A = " << pDoLowerSpeeds1A << ", " << pUB / pDoLowerSpeeds1A << endl;

		Utility::output_one_vector(cout,"NO1A Lower: BS Speeds", BSSpeeds);

		timer.start();
#if 0
#ifndef __USING_LOCAL_SEARCH_DC__
		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
#endif
		if (BSSpeeds.empty()) return BSSpeeds;
		timer.end();
		double tDoLocalSearch1A = timer.getTime();
		tSum += tDoLocalSearch1A;

		double pDoLocalSearch1A = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLocalSearch1A = " << tDoLocalSearch1A << ", pDoLocalSearch1A = " << pDoLocalSearch1A << ", " << pUB / pDoLocalSearch1A << endl;

		Utility::output_one_vector(cout,"NO1A Local: BS Speeds", BSSpeeds);

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		cout << "Computing BS Speeds by using NECESSARY ONLY 1 ..." << endl;
		timer.start();
		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExpMin(dc,NECESSARY_ONLY1,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;
		timer.end();
		double tDoLowerSpeeds1 = timer.getTime();
		tSum += tDoLowerSpeeds1;

		double pDoLowerSpeeds1 = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLowerSpeeds1 = " << tDoLowerSpeeds1 << ", pDoLowerSpeeds1 = " << pDoLowerSpeeds1 << ", " << pUB / pDoLowerSpeeds1 << endl;

		Utility::output_one_vector(cout,"NO1 Lower: BS Speeds", BSSpeeds);

		timer.start();
#ifndef __USING_LOCAL_SEARCH_DC__
		//BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1,maxSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		//BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1,maxSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;
		timer.end();
		double tDoLocalSearch1 = timer.getTime();
		tSum += tDoLocalSearch1;

		double pDoLocalSearch1 = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLocalSearch1 = " << tDoLocalSearch1 << ", pDoLocalSearch1 = " << pDoLocalSearch1 << ", " << pUB / pDoLocalSearch1 << endl;

		Utility::output_one_vector(cout,"NO1 Local: BS Speeds", BSSpeeds);

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		timer.start();
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExpMin(dc,EXACT,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;
		timer.end();
		double tDoLowerSpeeds0 = timer.getTime();
		tSum += tDoLowerSpeeds0;

		double pDoLowerSpeeds0 = getPerformanceDCExp(dc,BSSpeeds, k1,k2);
		cout << "tDoLowerSpeeds0 = " << tDoLowerSpeeds0 << ", pDoLowerSpeeds0 = " << pDoLowerSpeeds0 << ", " << pUB / pDoLowerSpeeds0 << endl;

		Utility::output_one_vector(cout,"Exact Lower: BS Speeds", BSSpeeds);

		cout << "Total time = " << tSum << endl;

#ifndef __USING_LOCAL_SEARCH_DC__
		//BSSpeeds = doLocalSearchExpMin(EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		//BSSpeeds = doLocalSearchDCExpMin(dc,EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}

vector<double> OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period) {
	vector<double> MAXSpeeds;
	vector<double> UBSpeeds;
	vector<double> BSSpeeds;

	UBSpeeds = computeUBSpeeds(NECESSARY_ONLY1A,wcets,tasks,engine,period);
	enforceModeGuards(UBSpeeds);
	MAXSpeeds = UBSpeeds;

	AVRTask avrTaskLevel0(engine,period,UBSpeeds,wcets);
	bool schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel0);

	if (schedulable) goto UBSPEEDS_END;

	{
		// NECESSARY ONLY 2A
		BSSpeeds = doLowerSpeedsDCExpMin(dc,NECESSARY_ONLY2A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY2A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel1(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel1);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1A
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExpMin(dc,NECESSARY_ONLY1A,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1A,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel2(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel2);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// NECESSARY ONLY 1
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExpMin(dc,NECESSARY_ONLY1,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		//BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		BSSpeeds = doLocalSearchExpMin(NECESSARY_ONLY1,MAXSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		//BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
		BSSpeeds = doLocalSearchDCExpMin(dc,NECESSARY_ONLY1,MAXSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;

		AVRTask avrTaskLevel3(engine,period,BSSpeeds,wcets);
		schedulable = SchedAnalysis::checkSchedulabilityAllPriorities(LUB,tasks,avrTaskLevel3);
	}

	if (schedulable) goto BSSPEEDS_END;

	{
		// EXACT ANALYSIS
		UBSpeeds = BSSpeeds;
		BSSpeeds = doLowerSpeedsDCExpMin(dc,EXACT,UBSpeeds,k1,k2,wcets,tasks,engine,period);
		if (BSSpeeds.empty()) return BSSpeeds;

#ifndef __USING_LOCAL_SEARCH_DC__
		//BSSpeeds = doLocalSearchExpMin(EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#else
		//BSSpeeds = doLocalSearchDCExpMin(dc,EXACT,UBSpeeds,BSSpeeds,k1,k2,wcets,tasks,engine,period);
#endif
		if (BSSpeeds.empty()) return BSSpeeds;
	}

BSSPEEDS_END:
	return BSSpeeds;

UBSPEEDS_END:
	return UBSpeeds;
}


bool OptimizationAlgorithm::doAllOptimizationsExpMin(vector<double>& UBSpeeds, double &pUB, double &tUB,
													 bool bOPT, vector<double>& OPTSpeeds, double &pOPT, double &tOPT,
													 bool bBS, vector<double>& BSSpeeds, double &pBS, double &tBS,
													 bool bOPTDC, vector<double>& OPTDCSpeeds, double &pOPTDC, double &tOPTDC,
													 bool bBSDC, vector<double>& BSDCSpeeds, double &pBSDC, double &tBSDC,
													 bool bBSDCSec, vector<vector<double>>& BSDCSecSpeedsVector, double &pBSDCSec, double &tBSDCSec,
													 map<int,int> dc, vector<map<int,int>> dcVector, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{
	Timer timer;

	timer.start();
	UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	OptimizationAlgorithm::enforceModeGuards(UBSpeeds);
	timer.end();

	if (UBSpeeds.empty()) return false;
	pUB = OptimizationAlgorithm::getPerformanceDCExp(dc,UBSpeeds, k1, k2);
	tUB = timer.getTime();

	if (bOPT) {
		timer.start();
		OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExpMin(EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();
		

		if (OPTSpeeds.empty()) return false;
		pOPT = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTSpeeds, k1, k2);
		tOPT = timer.getTime();
	}

	if (bBS) {
		timer.start();
		BSSpeeds = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (BSSpeeds.empty()) return false;
		pBS = OptimizationAlgorithm::getPerformanceDCExp(dc,BSSpeeds, k1, k2);
		tBS = timer.getTime();
	}

	if (bOPTDC) {
		timer.start();
		OPTDCSpeeds = OptimizationAlgorithm::computeBiondiBBDCExpMin(dc,EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (OPTDCSpeeds.empty()) return false;
		pOPTDC = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTDCSpeeds, k1, k2);
		tOPTDC = timer.getTime();
	}
	
	if (bBSDC) {
		timer.start();
		//BSDCSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,wcets, tasks, engine, period);
		BSDCSpeeds = OptimizationAlgorithm::computeBSDCExpMinWithESVSF_NECESSARYONLY(dc,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (BSDCSpeeds.empty()) return false;
		pBSDC = OptimizationAlgorithm::getPerformanceDCExp(dc,BSDCSpeeds, k1, k2);
		tBSDC = timer.getTime();
	}

	if (bBSDCSec) {
		//cout << Do BSDCSec..." << endl;
#if 0
		int time = 0;
#endif

		for (auto e:dcVector) {
			timer.start();
			vector<double> BSDCSecSpeeds = OptimizationAlgorithm::computeBSDCExpMinWithESVSF_NECESSARYONLY(e,k1,k2,wcets, tasks, engine, period);
			timer.end();

			if (BSDCSecSpeeds.empty()) return false;

			BSDCSecSpeedsVector.push_back(BSDCSecSpeeds);
			pBSDCSec += OptimizationAlgorithm::getPerformanceDCExp(e,BSDCSecSpeeds, k1, k2);
			tBSDCSec += timer.getTime();
#if 0
			cout << "Time = " << time << ", " << timer.getTime() << endl; 
			time ++;
#endif
		}
		tBSDCSec /= dcVector.size();

	}

	return true;
}


bool OptimizationAlgorithm::doAllOptimizationsExpMax(vector<double>& UBSpeeds, double &pUB, double &tUB,
													 bool bOPT, vector<double>& OPTSpeeds, double &pOPT, double &tOPT,
													 bool bBS, vector<double>& BSSpeeds, double &pBS, double &tBS,
													 bool bOPTDC, vector<double>& OPTDCSpeeds, double &pOPTDC, double &tOPTDC,
													 bool bBSDC, vector<double>& BSDCSpeeds, double &pBSDC, double &tBSDC,
													 bool bBSDCSec, vector<vector<double>>& BSDCSecSpeedsVector, double &pBSDCSec, double &tBSDCSec,
													 map<int,int> dc, vector<map<int,int>> dcVector, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{

	MethodChoice mc = SEG_LUB_NECESSARY_ONLY1;
	//MethodChoice mc = DIGRAPH_RBF;
	//MethodChoice mc = SEG_NECESSARY_ONLY1;
	Timer timer;

	timer.start();
	//cout << "Do UB ..." << endl;
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_ILPCON,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(DIGRAPH_RBF,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_ILPCON_DIGRAPH_RBF,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(LUB,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_LUB_NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeedsFast(SEG_LUB_NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(mc,wcets,tasks,engine,period);
	UBSpeeds = OptimizationAlgorithm::computeUBSpeedsFast(mc,wcets,tasks,engine,period);
	OptimizationAlgorithm::enforceModeGuards(UBSpeeds);
	timer.end();

	if (UBSpeeds.empty()) return false;
	pUB = OptimizationAlgorithm::getPerformanceDCExp(dc,UBSpeeds, k1, k2);
	tUB = timer.getTime();

	//cout << funcNames[mc] << "=> tUB = " << tUB << ", pUB = " << pUB << endl;

	if (bOPT) {
		//cout << "Do OPT..." << endl;
		timer.start();
		OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExp(EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (OPTSpeeds.empty()) return false;
		pOPT = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTSpeeds, k1, k2);
		tOPT = timer.getTime();
	}

	if (bBS) {
		//cout << "Do BS..." << endl;
		timer.start();
		BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(DIGRAPH_RBF,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(ILPCON,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(ILP,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(SEG_ILPCON_EXACT,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(SEG_ILPCON_DIGRAPH_RBF,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(SEG_NECESSARY_ONLY1,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (BSSpeeds.empty()) return false;
		pBS = OptimizationAlgorithm::getPerformanceDCExp(dc,BSSpeeds, k1, k2);
		tBS = timer.getTime();

		//cout << "tBS = " << tBS << ", pBS = " << pBS << endl;
	}

	if (bOPTDC) {
		//cout << "Do OPTDC..." << endl;
		timer.start();
		OPTDCSpeeds = OptimizationAlgorithm::computeBiondiBBDCExp(dc,EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (OPTDCSpeeds.empty()) return false;
		pOPTDC = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTDCSpeeds, k1, k2);
		tOPTDC = timer.getTime();
	}

	if (bBSDC) {
		//cout << "Do BSDC..." << endl;
		timer.start();
		//BSDCSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug(dc,k1,k2,wcets, tasks, engine, period);
		BSDCSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(dc,k1,k2,wcets, tasks, engine, period);
		//BSDCSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(dc,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (BSDCSpeeds.empty()) return false;
		pBSDC = OptimizationAlgorithm::getPerformanceDCExp(dc,BSDCSpeeds, k1, k2);
		tBSDC = timer.getTime();

		//cout << "tBSDC = " << tBSDC << ", pBSDC = " << pBSDC << endl;
	}

	if (bBSDCSec) {
		//cout << Do BSDCSec..." << endl;
#if 0
		int time = 0;
#endif

		for (auto e:dcVector) {
			timer.start();
			vector<double> BSDCSecSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(e,k1,k2,wcets, tasks, engine, period);
			timer.end();

			if (BSDCSecSpeeds.empty()) return false;

			BSDCSecSpeedsVector.push_back(BSDCSecSpeeds);
			pBSDCSec += OptimizationAlgorithm::getPerformanceDCExp(e,BSDCSecSpeeds, k1, k2);
			tBSDCSec += timer.getTime();
#if 0
			cout << "Time = " << time << ", " << timer.getTime() << endl; 
			time ++;
#endif
		}

		tBSDCSec /= dcVector.size();
		
	}

	return true;
}

bool OptimizationAlgorithm::doAllOptimizationsExpMax(vector<double>& UBSpeeds, double &pUB, double &tUB, double &pUBd, double &tUBd,
													 bool bOPT, vector<double>& OPTSpeeds, double &pOPT, double &tOPT,
													 bool bBS, vector<double>& BSSpeeds, double &pBS, double &tBS,
													 bool bOPTDC, vector<double>& OPTDCSpeeds, double &pOPTDC, double &tOPTDC,
													 bool bBSDC, vector<double>& BSDCSpeeds, double &pBSDC, double &tBSDC,
													 bool bBSDCSec, vector<vector<double>>& BSDCSecSpeedsVector, double &pBSDCSec, double &tBSDCSec,
													 list<double> dc, vector<map<int,int>> dcVector, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period)
{

	MethodChoice mc = SEG_LUB_NECESSARY_ONLY1;
	//MethodChoice mc = DIGRAPH_RBF;
	//MethodChoice mc = SEG_NECESSARY_ONLY1;
	Timer timer;

	timer.start();
	//cout << "Do UB ..." << endl;
	UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_ILPCON,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(DIGRAPH_RBF,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_ILPCON_DIGRAPH_RBF,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(LUB,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(SEG_LUB_NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeedsFast(SEG_LUB_NECESSARY_ONLY1,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeeds(mc,wcets,tasks,engine,period);
	//UBSpeeds = OptimizationAlgorithm::computeUBSpeedsFast(mc,wcets,tasks,engine,period);
	OptimizationAlgorithm::enforceModeGuards(UBSpeeds);
	timer.end();

	if (UBSpeeds.empty()) return false;
	pUB = OptimizationAlgorithm::getPerformanceDCExp(dc,UBSpeeds, k1, k2);
	tUB = timer.getTime();

	//double pUBDC = OptimizationAlgorithm::computeUBPerfDC(dc,EXACT,wcets,k1,k2,tasks,engine,period);
	//pOPT = pUBDC;

#if 0
	timer.start();
	vector<vector<double>> ks;
	ks.push_back(k1);
	ks.push_back(k2);

	StateGraph* sg = OptimizationAlgorithm::generateStateGraph(EXACT,PERF_EXPONENTIAL_MAX,ks,wcets,tasks,engine,period);
	timer.end();

	//sg->write_graphviz(cout);

	pUBd = sg->getTotalPerformance(dc);
	tUBd = timer.getTime();
	//cout << "=======================" << endl;
	//cout << pUB << "\t" << pUBDC2 << endl;
	delete sg;
#endif 

	//cout << funcNames[mc] << "=> tUB = " << tUB << ", pUB = " << pUB << endl;

	if (bOPT) {
		//cout << "Do OPT..." << endl;
		timer.start();
		OPTSpeeds = OptimizationAlgorithm::computeBiondiBBExp(EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (OPTSpeeds.empty()) return false;
		pOPT = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTSpeeds, k1, k2);
		tOPT = timer.getTime();
	}

	if (bBS) {
		//cout << "Do BS..." << endl;
		timer.start();
		BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(EXACT,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(DIGRAPH_RBF,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(ILPCON,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(ILP,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(SEG_ILPCON_EXACT,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(SEG_ILPCON_DIGRAPH_RBF,k1,k2,wcets, tasks, engine, period);
		//BSSpeeds = OptimizationAlgorithm::computeBiondiBSExp(SEG_NECESSARY_ONLY1,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (BSSpeeds.empty()) return false;
		pBS = OptimizationAlgorithm::getPerformanceDCExp(dc,BSSpeeds, k1, k2);
		tBS = timer.getTime();

		//cout << "tBS = " << tBS << ", pBS = " << pBS << endl;
	}
#if 0
	if (bOPTDC) {
		//cout << "Do OPTDC..." << endl;
		timer.start();
		OPTDCSpeeds = OptimizationAlgorithm::computeBiondiBBDCExp(dc,EXACT,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (OPTDCSpeeds.empty()) return false;
		pOPTDC = OptimizationAlgorithm::getPerformanceDCExp(dc,OPTDCSpeeds, k1, k2);
		tOPTDC = timer.getTime();
	}

	if (bBSDC) {
		//cout << "Do BSDC..." << endl;
		timer.start();
		//BSDCSpeeds = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug(dc,k1,k2,wcets, tasks, engine, period);
		BSDCSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(dc,k1,k2,wcets, tasks, engine, period);
		//BSDCSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY_BIGSYSTEM(dc,k1,k2,wcets, tasks, engine, period);
		timer.end();

		if (BSDCSpeeds.empty()) return false;
		pBSDC = OptimizationAlgorithm::getPerformanceDCExp(dc,BSDCSpeeds, k1, k2);
		tBSDC = timer.getTime();

		//cout << "tBSDC = " << tBSDC << ", pBSDC = " << pBSDC << endl;
	}
#endif

#if 1
	if (bBSDCSec) {
		//cout << Do BSDCSec..." << endl;
#if 0
		int time = 0;
#endif
		vector<double> BSDCSecSpeeds;
		for (auto e:dcVector) {
			timer.start();
			if (BSDCSecSpeeds.empty()) {
				BSDCSecSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(e,k1,k2,wcets, tasks, engine, period);
				//Utility::output_one_vector(cout,"BSDCSecSpeeds",BSDCSecSpeeds);
			}
			else {
				AVRTask prevAvrTask(engine,period,BSDCSecSpeeds,wcets);
				vector<double> temp_BSDCSecSpeeds = OptimizationAlgorithm::computeBSDCExpMaxWithESVSF_NECESSARYONLY(e,k1,k2,wcets, tasks, engine, period,&prevAvrTask);
				
				if (temp_BSDCSecSpeeds.empty()) enforceModeReduced(BSDCSecSpeeds,MODE_GUARD_RPM);
				else BSDCSecSpeeds = temp_BSDCSecSpeeds;

				//Utility::output_one_vector(cout,"BSDCSecSpeeds",BSDCSecSpeeds);
			}

			timer.end();

			if (BSDCSecSpeeds.empty()) return false;

			BSDCSecSpeedsVector.push_back(BSDCSecSpeeds);
			pBSDCSec += OptimizationAlgorithm::getPerformanceDCExp(e,BSDCSecSpeeds, k1, k2);
			tBSDCSec += timer.getTime();
#if 0
			cout << "Time = " << time << ", " << timer.getTime() << endl; 
			time ++;
#endif
		}

		tBSDCSec /= dcVector.size();

	}
#endif

	return true;
}


std::vector<double> OptimizationAlgorithm::generatePerformanceFunctionsByWCET(vector<int> WCETs)
{
	vector<double> ret;
	for (auto e:WCETs) {
		ret.push_back(PERFORMANCE_FACTOR*e*e);
	}
	return ret;
}

std::vector<double> OptimizationAlgorithm::generatePerformanceFunctionsByWCETExponential(vector<int> WCETs)
{
	vector<double> ret;
	for (auto e:WCETs) {
		double perfFunc = exp(PERFORMANCE_FACTOR*e);
		ret.push_back(perfFunc);
	}
	return ret;
}

CollectionResult OptimizationAlgorithm::doOptimalAlgorithmDFSMappingAndBnBMin(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	double pOpt = INT_MAX;
	vector<PeriodicTask> sOpt;
	vector<double> wOpt;
	doDepthFirstSearchMin(pOpt,sOpt,wOpt,k1,k2,wcets,tasks,engine,period,numConstraint);
	return CollectionResult(sOpt,wOpt);
}

void OptimizationAlgorithm::doDepthFirstSearchMin(double &pOpt, vector<PeriodicTask> &sOpt, vector<double> &wOpt, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	if (tasks.size() <= numConstraint) {
		vector<double> wCur = computeBiondiBBExpMin(EXACT,k1,k2,wcets,tasks,engine,period);
		double pCur = getPerformanceExpGsl(wCur,k1,k2,engine);

		if (pCur < pOpt) {
			sOpt = tasks;
			wOpt = wCur;
			pOpt = pCur;
		}
		return;
	}

	for (int i=tasks.size()-1; i >= 0; i--) {
		for (int j=i-1; j>=0; j--) {
			PeriodicTask taski = tasks[i];
			PeriodicTask taskj = tasks[j];
			if (taski.period == taskj.period) {
				int wcet = taski.wcet + taskj.wcet;
				int deadline = min(taski.deadline,taskj.deadline);
				PeriodicTask taskij(wcet,deadline,taski.period);

				vector<PeriodicTask> newTasks;
				for (int k=0;k<tasks.size(); k++) {
					if (k==i || k==j) continue;
					newTasks.push_back(tasks[k]);
				}
				newTasks.push_back(taskij);

				// if {\Call{Schedulable}{$\mathcal{S}'$}}
				if (SchedAnalysis::checkSchedulabilityAudsleyOPA(newTasks)) {
					vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
					if (!UBSpeeds.empty()) {
						double pUB = getPerformanceExpGsl(UBSpeeds,k1,k2,engine);

						if (pUB < pOpt) {
							doDepthFirstSearchMin(pOpt,sOpt,wOpt,k1,k2,wcets,newTasks,engine,period,numConstraint);
						}
					}
				}
			}
		}
	}
}


bool OptimizationAlgorithm::SortByDeadline(const PeriodicTask task0, const PeriodicTask task1)
{
	return task0.deadline < task1.deadline;
}

double OptimizationAlgorithm::calClusteringHeuristicFactor(vector<PeriodicTask> tasks)
{
	// Sort the periodic tasks based on the deadline, using the deadline monotonic priority
	sort(tasks.begin(),tasks.end(),SortByDeadline);

	// Calculate the heurisitic factor, h(S) = \sum\limits_{k=0}{|S|-1} \frac{R_k}{D_k}
	double hFactor = 0.0;
	for (int i=0; i<tasks.size(); i++) {
		PeriodicTask taski = tasks[i];
		double rt = SchedAnalysis::calculate_response_time(tasks,i-1,taski.wcet);
		double deadline = taski.deadline;

		if (rt > deadline) return INT_MAX;
		hFactor += rt/deadline;
	}
	return hFactor;
}

CollectionResult OptimizationAlgorithm::doOptimalAlgorithmDFSMappingAndBnBMin(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	double pOpt = INT_MAX;
	vector<PeriodicTask> sOpt;
	vector<double> wOpt;
	doDepthFirstSearchMin(pOpt,sOpt,wOpt,dc,k1,k2,wcets,tasks,engine,period,numConstraint);
	return CollectionResult(sOpt,wOpt);
}

void OptimizationAlgorithm::doDepthFirstSearchMin(double &pOpt, vector<PeriodicTask> &sOpt, vector<double> &wOpt,map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	//cout << tasks.size() << endl;
	if (tasks.size() <= numConstraint) {
		//cout << "One Child: start" << endl;
		vector<double> wCur = computeBiondiBBDCExpMin(dc,EXACT,k1,k2,wcets,tasks,engine,period);
		if (wCur.empty()) return;
		double pCur = getPerformanceDCExp(dc,wCur,k1,k2);

		if (pCur < pOpt) {
			sOpt = tasks;
			wOpt = wCur;
			pOpt = pCur;
		}

		//cout << "One Child: end" << endl;
		return;
	}

	for (int i=tasks.size()-1; i >= 0; i--) {
		for (int j=i-1; j>=0; j--) {
			PeriodicTask taski = tasks[i];
			PeriodicTask taskj = tasks[j];
			if (taski.period == taskj.period) {
				int wcet = taski.wcet + taskj.wcet;
				int deadline = min(taski.deadline,taskj.deadline);
				PeriodicTask taskij(wcet,deadline,taski.period);

				vector<PeriodicTask> newTasks;
				for (int k=0;k<tasks.size(); k++) {
					if (k==i || k==j) continue;
					newTasks.push_back(tasks[k]);
				}
				newTasks.push_back(taskij);

				// if {\Call{Schedulable}{$\mathcal{S}'$}}
				if (SchedAnalysis::checkSchedulabilityAudsleyOPA(newTasks)) {
					vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
					if (!UBSpeeds.empty()) {
						double pUB = getPerformanceDCExp(dc,UBSpeeds,k1,k2);

						if (pUB < pOpt) {
							doDepthFirstSearchMin(pOpt,sOpt,wOpt,dc,k1,k2,wcets,newTasks,engine,period,numConstraint);
						}
					}
				}
			}
		}
	}
}

CollectionResult OptimizationAlgorithm::doHeuristicAlgorithmBFSMappingAndBSMin(vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	if (UBSpeeds.empty()) return CollectionResult(); 
	
	double pUBMax = getPerformanceExpGsl(UBSpeeds,k1,k2,engine);
	vector<PeriodicTask> sH = doBestFirstSearchMin(pUBMax,k1,k2,wcets,tasks,engine,period,numConstraint);
	
	vector<double> wH = OptimizationAlgorithm::computeBiondiBSExpMin(EXACT,k1,k2,wcets, sH, engine, period);

	return CollectionResult(sH,wH);
}

vector<PeriodicTask> OptimizationAlgorithm::doBestFirstSearchMin(double pUBMax, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {	
	if (tasks.size() <= numConstraint)
		return tasks;
	
	double pMin = INT_MAX;
	vector<PeriodicTask> sMin;

	for (int i=tasks.size()-1; i >= 0; i--) {
		for (int j=i-1; j>=0; j--) {
			PeriodicTask taski = tasks[i];
			PeriodicTask taskj = tasks[j];
			if (taski.period == taskj.period) {
				int wcet = taski.wcet + taskj.wcet;
				int deadline = min(taski.deadline,taskj.deadline);
				PeriodicTask taskij(wcet,deadline,taski.period);

				vector<PeriodicTask> newTasks;
				for (int k=0;k<tasks.size(); k++) {
					if (k==i || k==j) continue;
					newTasks.push_back(tasks[k]);
				}
				newTasks.push_back(taskij);

				// if {\Call{Schedulable}{$\mathcal{S}'$}}
				if (SchedAnalysis::checkSchedulabilityAudsleyOPA(newTasks)) {
					vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
					if (!UBSpeeds.empty()) {
						double pUB = getPerformanceExp(UBSpeeds,k1,k2,engine);

						if (pUB < pMin) {
							pMin = pUB;
							sMin = newTasks;
						}
					}
				}
			}
		}
	}

	if (sMin.size() == numConstraint) return sMin;
	else return doBestFirstSearchMin(pUBMax,k1,k2,wcets,sMin,engine,period,numConstraint);
}

CollectionResult OptimizationAlgorithm::doHeuristicAlgorithmBFSMappingAndBSMin(map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
	if (UBSpeeds.empty()) return CollectionResult(); 

#ifdef __DEBUG_DESIGN__
	Timer timer;
	timer.start();
#endif
	
	double pUBMax = getPerformanceDCExp(dc,UBSpeeds,k1,k2);
	vector<PeriodicTask> sH = doBestFirstSearchMin(pUBMax,dc,k1,k2,wcets,tasks,engine,period,numConstraint);
	
#ifdef __DEBUG_DESIGN__
	timer.end();
	cout << "Mapping time = " << timer.getTime() << endl;
#endif

	if (sH.empty()) return CollectionResult();

#ifdef __DEBUG_DESIGN__
	timer.start();
#endif

	vector<double> wH = OptimizationAlgorithm::computeDCBSExp_NECESSARYONLY_WithoutDebug_Min(dc,k1,k2,wcets, sH, engine, period);
	
#ifdef __DEBUG_DESIGN__
	timer.end();
	cout << "BS time = " << timer.getTime() << endl;
#endif

	return CollectionResult(sH,wH);
}

vector<PeriodicTask> OptimizationAlgorithm::doBestFirstSearchMin(double pUBMax, map<int,int> dc, vector<double> k1, vector<double> k2, vector<int> wcets, vector<PeriodicTask> tasks, Engine engine, double period, int numConstraint) {
	if (tasks.size() <= numConstraint) return tasks;
	
	double pMin = INT_MAX;
	vector<PeriodicTask> sMin;

	for (int i=tasks.size()-1; i >= 0; i--) {
		for (int j=i-1; j>=0; j--) {
			PeriodicTask taski = tasks[i];
			PeriodicTask taskj = tasks[j];
			if (taski.period == taskj.period) {
				int wcet = taski.wcet + taskj.wcet;
				int deadline = min(taski.deadline,taskj.deadline);
				PeriodicTask taskij(wcet,deadline,taski.period);

				vector<PeriodicTask> newTasks;
				for (int k=0;k<tasks.size(); k++) {
					if (k==i || k==j) continue;
					newTasks.push_back(tasks[k]);
				}
				newTasks.push_back(taskij);

				// if {\Call{Schedulable}{$\mathcal{S}'$}}
				if (SchedAnalysis::checkSchedulabilityAudsleyOPA(newTasks)) {
#if 0
					vector<double> UBSpeeds = computeUBSpeeds(EXACT,wcets,tasks,engine,period);
					if (!UBSpeeds.empty()) {
						double pUB = getPerformanceDCExp(dc,UBSpeeds,k1,k2);

						if (pUB < pMin) {
							pMin = pUB;
							sMin = newTasks;
						}
					}
#endif

#if 1
					double pCur = 0.0;
					for (auto task:newTasks) {
						pCur += (double)task.wcet/task.deadline;
					}
					if (pCur < pMin) {
						pMin = pCur;
						sMin = newTasks;
					}
#endif

#if 0
					double pCur = calClusteringHeuristicFactor(newTasks);
					if (pCur < pMin) {
						pMin = pCur;
						sMin = newTasks;
					}
#endif

				}
			}
		}
	}

	if (sMin.size() == numConstraint) return sMin;
	else return doBestFirstSearchMin(pUBMax,dc,k1,k2,wcets,sMin,engine,period,numConstraint);
}
