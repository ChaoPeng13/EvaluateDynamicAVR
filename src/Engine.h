#ifndef ENGINE_H_
#define ENGINE_H_

#include <math.h>
#include <iostream>

#include "Global.h"

using namespace std;

class Engine {
public:
	double RPM_MIN; // the minimum engine speed, rpm (revolutions per minute)
	double RPM_MAX; // the maximum engine speed, rpm (revolutions per minute)
	double SPEED_MIN; // the minimum engine speed, revolutions per msec
	double SPEED_MAX; // the maximum engine speed, revolutions per msec
	//double ANGULAR_PERIOD; // the angular period of the engine 
	double ACCELERATION; // the acceleration factor of the engine, rev/msec^2
	double DECELERATION; // the deceleration factor of the engine, rev/msec^2

	Engine() {}

	Engine(double _rpm_min, double _rpm_max, double _acceleration, double _deceleration) {
		RPM_MIN = _rpm_min;
		RPM_MAX = _rpm_max;
		ACCELERATION = _acceleration;
		DECELERATION = _deceleration;
		SPEED_MIN = RPM_to_RPmSEC(RPM_MIN);
		SPEED_MAX = RPM_to_RPmSEC(RPM_MAX);
	}

	Engine(const Engine &engine) {
		RPM_MIN = engine.RPM_MIN;
		RPM_MAX = engine.RPM_MAX;
		SPEED_MIN = engine.SPEED_MIN;
		SPEED_MAX = engine.SPEED_MAX;
		ACCELERATION = engine.ACCELERATION;
		DECELERATION = engine.DECELERATION;
	}

	/// in rev/msec
	double getHigherSpeed(double omega, double angular_period, int n) {
		if (n==0) return omega;

		double temp = omega*omega+2.0*angular_period*ACCELERATION;
		//if (W_GEQ_TOL(temp, SPEED_MAX*SPEED_MAX)) return -1; 

		double omega_next = sqrt(temp);
		return getHigherSpeed(omega_next,angular_period,n-1);
	}

	/// in rev/msec
	double getLowerSpeed(double omega, double angular_period, int n) {
		if (n==0) return omega;

		double temp = omega*omega-2.0*angular_period*DECELERATION;
		if (W_LEQ_TOL(temp, SPEED_MIN*SPEED_MIN)) return -1; 

		double omega_next = sqrt(temp);
		return getLowerSpeed(omega_next,angular_period,n-1);
	}

	/// in msec
	double getInterArrivalTimeWithConstantAcceleration(double omega_a, double omega_b, double angular_period) {
		return 2.0*angular_period/(omega_a+omega_b);
	}

	/// in msec
	double getMinInterArrivalTimeWithArbitraryAcceleration(double omega_a, double omega_b, double angular_period) {
		// force omega_a <= omega_b
		if (W_GEQ_TOL(omega_a,omega_b)) {
			double temp = omega_a;
			omega_a = omega_b;
			omega_b = temp;
		}

		double omega_m = sqrt((ACCELERATION * pow(omega_b,2.0) 
								+ DECELERATION * pow(omega_a,2.0) 
								+ 2.0 * ACCELERATION * DECELERATION * angular_period)
							 /(ACCELERATION + DECELERATION));
		if (W_LEQ_TOL(omega_m,SPEED_MAX)) {
			return (omega_m-omega_a) / ACCELERATION + (omega_m-omega_b) / DECELERATION;
		}
		else {
			double t1 = (SPEED_MAX-omega_a) / ACCELERATION;
			double t2 = (angular_period 
				- (pow(SPEED_MAX,2.0)-pow(omega_a,2.0)) / (2.0 * ACCELERATION) 
				- (pow(SPEED_MAX,2.0)-pow(omega_b,2.0)) / (2.0 * DECELERATION) ) / SPEED_MAX ;
			double t3 = (SPEED_MAX-omega_b) / DECELERATION;
			return t1 + t2 + t3;
		}
	}

	/// in msec
	double getMaxInterArrivalTimeWithArbitraryAcceleration(double omega_a, double omega_b, double angular_period) {
		// force omega_a <= omega_b
		if (W_GEQ_TOL(omega_a,omega_b)) {
			double temp = omega_a;
			omega_a = omega_b;
			omega_b = temp;
		}

		double omega_m = getMaxTimeOmegaDot(omega_a, omega_b, angular_period);

		if (W_GEQ_TOL(omega_m,SPEED_MIN)) {
			return (omega_a-omega_m) / DECELERATION + (omega_b - omega_m) / ACCELERATION;
		}
		else {
			double t1 = (omega_a - SPEED_MIN) / DECELERATION;
			double t2 = (angular_period 
				- (pow(omega_a,2.0)-pow(SPEED_MIN,2.0)) / (2.0 * DECELERATION) 
				- (pow(omega_b,2.0)-pow(SPEED_MIN,2.0)) / (2.0 * ACCELERATION) ) / SPEED_MIN ;
			double t3 = (omega_b - SPEED_MIN) / ACCELERATION;
			return t1 + t2 + t3;
		}
	}

	/// in msec
	double getConstantPeriodWithConstantAcceleration(double omega, double angular_period) {
		return getInterArrivalTimeWithConstantAcceleration(omega,omega,angular_period);
	}

	/// in msec
	double getConstantPeriodWithArbitraryAcceleration(double omega, double angular_period) {
		return getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega,angular_period);
	}

	/// in msec
	double getLowerIntervalWithConstantAcceleration(double omega_a, double omega_b, double angular_period, int n) {
		return 2.0*n*angular_period/(omega_a+omega_b);
	}

	// in msec
	double getMinDeadline(double angular_period, double omega) {
		if ( W_EQ_TOL(omega, SPEED_MAX)) return ANGULAR_DEADLINE_FACTOR * angular_period / SPEED_MAX;
		return ANGULAR_DEADLINE_FACTOR * (sqrt(omega*omega+2.0*ACCELERATION*angular_period)-omega) / ACCELERATION;
	}

	// in msec
	double getMaxInterArrivalTime(double omega, double angular_period) {
		double omega_next = getLowerSpeed(omega,angular_period,1);
		if (omega_next < 0) omega_next = SPEED_MIN;

#ifdef __USING_CONSTANT_ACCELERATION__
		return getInterArrivalTimeWithConstantAcceleration(omega,omega_next,angular_period);
#else
		return getMinInterArrivalTimeWithArbitraryAcceleration(omega,omega_next,angular_period);
#endif
	}

	double getMinTimeOmegaDot(double omega_a, double omega_b, double angular_period) {
		return sqrt((ACCELERATION * pow(omega_b,2.0) 
					+ DECELERATION * pow(omega_a,2.0) 
					+ 2.0 * ACCELERATION * DECELERATION * angular_period)
					/(ACCELERATION + DECELERATION));
	}

	// in msec, Algorithm 2 in the paper "Refinement of Workload Models for Engine Controllers by State Space Partitioning"
	double getMinTimeInterReleaseTime(double omega_i_low, double omega_i_high, double omega_f_low, double omega_f_high, double angular_period) {
		double Omega_m = sqrt(omega_i_low*omega_i_low-2.0*angular_period*DECELERATION);
		double Omega_M_Minus = sqrt(omega_i_high*omega_i_high-2.0*angular_period*DECELERATION);
		double Omega_M_Plus = getHigherSpeed(omega_i_high,angular_period,1);
		double rho = getMinTimeOmegaDot(omega_i_high, omega_f_high,angular_period);

		if (W_GEQ_TOL(omega_f_low,Omega_M_Plus) || W_LEQ_TOL(omega_f_high,Omega_m)) 
			return -1.0; // +infty
		else if ( W_LEQ_TOL (omega_f_low,Omega_M_Plus) && W_LEQ_TOL (Omega_M_Plus,omega_f_high) )
			return 2.0*angular_period / (omega_i_high + Omega_M_Plus);
		else if ( W_LEQ_TOL (Omega_m, omega_f_high) && W_LEQ_TOL (omega_f_high,Omega_M_Minus) )
			return (sqrt(omega_f_high*omega_f_high + 2.0*angular_period*DECELERATION)-omega_f_high) / DECELERATION;
		else if ( W_LEQ_TOL (rho,SPEED_MAX)) {
			return (rho-omega_i_high) / ACCELERATION + (rho-omega_f_high) / DECELERATION;
		}
		else {
			double t1 = (SPEED_MAX-omega_i_high) / ACCELERATION;
			double t2 = (angular_period 
				- (pow(SPEED_MAX,2.0)-pow(omega_i_high,2.0)) / (2.0 * ACCELERATION) 
				- (pow(SPEED_MAX,2.0)-pow(omega_f_high,2.0)) / (2.0 * DECELERATION) ) / SPEED_MAX ;
			double t3 = (SPEED_MAX-omega_f_high) / DECELERATION;
			return t1 + t2 + t3;
		}
	}

	double getMaxTimeOmegaDot(double omega_a, double omega_b, double angular_period) {
		return sqrt((ACCELERATION * pow(omega_a,2.0) 
			+ DECELERATION * pow(omega_b,2.0) 
			- 2.0 * ACCELERATION * DECELERATION * angular_period)
			/(ACCELERATION + DECELERATION));
	}

	// in msec, following Algorithm 2 in the paper "Refinement of Workload Models for Engine Controllers by State Space Partitioning"
	double getMaxTimeInterReleaseTime(double omega_i_low, double omega_i_high, double omega_f_low, double omega_f_high, double angular_period) {
		double Omega_m = sqrt(omega_i_low*omega_i_low-2.0*angular_period*DECELERATION);
		double Omega_M_Minus = sqrt(omega_i_low*omega_i_low+2.0*angular_period*ACCELERATION);
		double Omega_M_Plus = getHigherSpeed(omega_i_high,angular_period,1);
		//double Omega_M_Minus2 = sqrt(omega_f_low*omega_f_low+2.0*angular_period*DECELERATION);
		double rho = getMaxTimeOmegaDot(omega_i_low, omega_f_low,angular_period);
		//double omega_f_low_Minus =  sqrt(omega_f_low*omega_f_low - 2.0*angular_period*ACCELERATION);

		if (W_GEQ_TOL(omega_f_low,Omega_M_Plus) || W_LEQ_TOL(omega_f_high,Omega_m)) 
			return -1.0; // +infty
		else if ( W_LEQ_TOL (omega_f_low,Omega_m) && W_LEQ_TOL (Omega_m,omega_f_high) )
			return 2.0*angular_period / (omega_i_low + Omega_m);
		else if ( W_LEQ_TOL (Omega_M_Minus, omega_f_low) && W_LEQ_TOL (omega_f_low,Omega_M_Plus) )
			return (omega_f_low - sqrt(omega_f_low*omega_f_low - 2.0*angular_period*ACCELERATION)) / ACCELERATION;
		//else if ( W_LEQ_TOL (omega_i_low,omega_f_low_Minus) && W_LEQ_TOL (omega_f_low_Minus,omega_i_high) )
		//	return  (omega_f_low - omega_f_low_Minus) / ACCELERATION;
		else if ( W_GEQ_TOL (rho,SPEED_MIN)) {
			return - (rho-omega_i_low) / DECELERATION - (rho-omega_f_low) / ACCELERATION;
		}
		else {
			double t1 = (omega_i_low - SPEED_MIN) / DECELERATION;
			double t2 = (angular_period 
				- (pow(omega_i_low,2.0) - pow(SPEED_MIN,2.0)) / (2.0 * DECELERATION) 
				- (pow(omega_f_low,2.0) - pow(SPEED_MIN,2.0)) / (2.0 * ACCELERATION) ) / SPEED_MIN ;
			double t3 = (omega_f_low-SPEED_MIN) / ACCELERATION;
			return t1 + t2 + t3;
		}
	}

	void output(ostream &out) {
		out << "========Engine Features===========" << endl;
		out << "Minimum Engine Speed = " << RPM_MIN << " rpm" << endl;
		out << "Maximum Engine Speed = " << RPM_MAX << " rpm" << endl;
		out << "Minimum Engine Speed = " << SPEED_MIN << " rev/msec" << endl;
		out << "Maximum Engine Speed = " << SPEED_MAX << " rev/msec" << endl;
		out << "Acceleration Factor = " << ACCELERATION << " rev/msec^2" << endl;
		out << "Deceleration Factor = " << DECELERATION << " rev/msec^2" << endl;
	}
};

#endif // Engine.h