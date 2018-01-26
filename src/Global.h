#ifndef GLOBAL_H_
#define GLOBAL_H_

#include "Utility.h"
#include <exception>

#define ANGULAR_DEADLINE_FACTOR 1.0
#define PERFORMANCE_FACTOR 0.001

#define M_PI 3.14159265358979323846
#define STARV_WEIGHT 0.2
#define STARV_WEIGHT2 0.5
#define DEADLINE_FACTOR 1.0
#define TORQUE 200.0

#define CONSTANT_SWITCH_TIME 500000

#define MINIMUM_STEP 1.0

#define CONSTANT_REDUCED_STEP          5.0
#define CONSTANT_DC_STEP               10

#define SPEED_PRECISION                RPM_to_RPmSEC(1)
#define SPEED_PRECISION_RPM            1.0
#define SEARCH_PRECISION               RPM_to_RPmSEC(20)
//#define SEARCH_PRECISION_RPM           5.0
#define SEARCH_PRECISION_RPM           20.0
#define BIN_SEARCH_PRECISION           RPM_to_RPmSEC(20)
#define BIN_SEARCH_PRECISION_RPM       20.0
//#define BIN_SEARCH_PRECISION_RPM       1.0
#define MODE_OFFSET                    RPM_to_RPmSEC(5)

#define MODE_GUARD                     PRM_to_RPmSEC(5)
#define MODE_GUARD_RPM                 5.0
#define LOWER_INTERVAL_RPM             5.0

#define BS_BIN_SEARCH_PRECISION        RPM_to_RPmSEC(5)
#define BS_BIN_SEARCH_PRECISION_RPM    5.0
#define BS_BIN_SEARCH_PRECISION_VIRTUAL_STEP    0.001
#define BS_SEARCH_STEP			       RPM_to_RPmSEC(5)
#define BS_SEARCH_STEP_RPM	           5.0
#define BS_DC_SEARCH_STEP_RPM	       5.0

#define TBL_SIZE(a) ( (sizeof(a)) / (sizeof(a[0])) )

#define W_EQ_TOL(X,Y) (fabs(X-Y)<=RPM_to_RPmSEC(1))
#define W_GEQ_TOL(X,Y) (X >= Y - RPM_to_RPmSEC(1))
#define W_LEQ_TOL(X,Y) (X <= Y + RPM_to_RPmSEC(1))
#define W_TOLERANCE RPM_to_RPmSEC(1)

#define RPM_EQ_TOL(X,Y) (fabs(X-Y)<=1.0)
#define RPM_GEQ_TOL(X,Y) (X >= Y - 1.0)
#define RPM_LEQ_TOL(X,Y) (X <= Y + 1.0)

#define T_EQ_TOL(X,Y) (fabs(X-Y)<=muSEC_to_mSEC(1.0))
#define T_GEQ_TOL(X,Y) (X >= Y - muSEC_to_mSEC(1.0))
#define T_LEQ_TOL(X,Y) (X <= Y + muSEC_to_mSEC(1.0))

#define ANALYSIS_SEGMENT 100000 // 100 ms
#define MAX_ERROR 10 // 10 musec

//-------------------------------------------------
//----------[ VERY SIMPLE EXCEPTIONS ]-------------
//-------------------------------------------------

class excUndefinedWCET: public exception
{
public:
  excUndefinedWCET(double _w) { w = _w; }

  double w;
	
  virtual const char* what() const throw()
  { 
	cout << "w = " << RPmSEC_to_RPM(w) << endl;
	return "Undefined speed in the WCET function for speed";	  
  }
};

class excNoAVRTask: public exception
{
public:
	
  virtual const char* what() const throw()
  { 
	return "No AVRtask in the taskset";  
  }
};

class excDesignImpossible: public exception
{
public:
	
  virtual const char* what() const throw()
  { 
	return "Design impossible for this taskset";  
  }
};

#endif