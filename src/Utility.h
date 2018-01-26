/* \file Utility.h
 *  This file implements some math-related functions.
 *  \author Chao Peng
 *  
 *  Changes
 *  ------
 *  22-Aug-2015 : Initial revision (CP)
 *
 */

#ifndef UTILITY_H
#define UTILITY_H

#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <set>
#include <time.h>
#include <algorithm>
#include <functional>
#include "Config.h"
#include "Definitions.h"

#ifdef WINDOWS_PLATFORM
#include <direct.h>
#endif

#ifdef LINUX_PLATFORM
#include <sys/types.h>
#include <sys/stat.h>
#endif



using namespace std;

/// Helper functions
double RPM_to_W(double rpm); // rpm to radian/musec
double W_to_RPM(double w); // radian/musec to rpm
double RPM_to_RPmSEC(double w); // rpm to rev/msec
double RPmSEC_to_RPM(double w); // rev/msec to rpm
double RPM_to_RPmuSEC(double w); // rpm to rev/musec
double RPmuSEC_to_RPM(double w); // rev/musec to rpm
double RPmSEC_to_HundredRPM(double w); // rev/msec to 100*rpm
double HundredRPM_to_RPmSEC(double w); // 100*rpm to rev/msec
double RPmSEC_to_ThousandRPM(double w); // rev/msec to 1000*rpm
double ThousandRPM_to_RPmSEC(double w); // 1000*rpm to rev/msec
double mSEC_to_muSEC(double T);
double muSEC_to_mSEC(double T);

class Utility {
public:
	static double EPSILON;

	// return the greatest common divisor for two integers
	static int math_gcd(int a, int b);
	static int math_lcm(int a, int b);
	static int round(double r);
	static std::string int_to_string(int a);
	static double** creat_matrix(int nrow, int ncol);
	static void output_matrix(double** A, int nrow, int ncol);
	static void output_latex_matrix(double** A, int nrow, int ncol);
	static vector<double> uniformly_distributed(int n, double tUtil);
	static vector<double> uniformly_distributed(int n, double tUtil, double minUtil);
	static double* uniformly_distributed2(int n, double tUtil);
	static double max_uniformly_distributed(int n, double tUtil);
	static bool compare_two_matrices(double** A, double** B, int n);
	
	static void output_one_vector(ostream& out, string sVec, vector<double> vec);
	static string get_one_vector(string sVec, vector<double> vec);
	static void output_one_vector(ostream& out, string sVec, vector<int> vec);
	static string get_one_vector(string sVec, vector<int> vec);

	static void output_multiple_vectors(ostream& out, vector<string> sVecs, vector<vector<double>> vecs);
	static string get_multiple_vectors(vector<string> sVecs, vector<vector<double>> vecs);

	static void output_one_map(ostream& out, string sVec, map<int,double> mm);
	static void output_one_map(ostream& out, string sVec, map<double,int> mm);
	static void output_one_map(ostream& out, string sVec, map<int,int> mm);
	static void output_one_map(ostream& out, string sVec, map<int,int> mm, int t);
	static void output_map_map(ostream& out, string sVec, map<double, map<int,int>> mm);
	
	static int getWCET(double omega, double omegas[], int wcets[], int numMode);
	static bool testSeperation(set<int> testSet, int cSep, int value);
	static bool testSeperation(vector<int> testSet, int cSep, int value);
	static vector<int>  integerUniforms(bool order, int cMin, int cMax, int cSep, int n);
	static vector<int>  integerUniforms(bool order, int cMin, int cMax, int n);
	static vector<double> uniforms(bool order, int cMin, int cMax, int cSep, int n);
	static vector<double> uniforms(bool order, double cMin, double cMax, int n);
	static vector<string> split(const string &s, const string &separator);
	static double random(double cMin, double cMax);
	static map<int,int> merge(map<int,int> map_a, map<int,int> map_b);
	static vector<double> merge(vector<double> vec0, vector<double> vec1);

	static void insert_vector(vector<double>& vec, double e);

	// mkdir
	static void makeDir(string dirName);

	// Directory Link String
	// +Windows: "\\"
	// +Linux: "/"
	static string linkNotation();
};

#endif