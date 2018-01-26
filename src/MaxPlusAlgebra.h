/* \file MaxPlusAlgebra.h
*  this file implements some algorithms for linear matrix. 
*  \author Chao Peng
*  
*  Changes
*  ------
*  25-aug-2015 : initial revision (CP)
*
*/

#ifndef MAXPLUSALGEBRA_H
#define MAXPLUSALGEBRA_H

//#include <vld.h>

#include <algorithm>
#include <vector>
#include <map>
#include <set>

class MaxPlusAlgebra {
public:
	static double** calculate_metric_matrix(double** B, int n);
	static int calculate_linear_period(double** A, int n, double lfac, bool debug);
	static int calculate_gcd_cycle_length_hcc(double** A, int n, std::set<int> hcc, bool debug);
	static int calculate_linear_defect(std::map<int, double**>& m_map, std::map<int, int> &me_map, int n, double lfac, int lper, int tf, int gcd);
	static int calculate_linear_defect(std::map<int, double**>& m_map, int n, double lfac, int lper, int tf);
	// TODO: upper bounds on the linear defect
	static int calculate_base_matrix(std::map<int,double**>& base_matrices, double** matrix, int n, int lper);
	static double** calculate_period_matrix(std::map<int,double**>& base_matrices, int n, int lper);
	static int calculate_upper_bound_linear_defect(double** matrix, int n, double lfac, int lper, int tf, int gcd);
	static double maximum_element(double** A, int n);
	static double** periodicly_calculate_maxplus_matrix_power(double** A, int n, double lfac, int lper);
	static double** multiply_maxplus_matrix(double** A, double** B, int n);
	static double** power_maxplus_matrix(double** A, int n, int power);
	static double** multiply_maxplus_matrix(std::map<int,double**> & matrices, int t, int n);
};

#endif