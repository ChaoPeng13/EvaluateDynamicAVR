#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#pragma once

//#define GOOGLETEST
//#define VLDTEST
//#define EXACTANALYSIS

//#define __DEBUG__
//#define __DEBUG_DESIGN__
//#define __DEBUG_CONSTANT_STEP__
//#define __DEBUG_SCHED_ANALYSIS__
//#define __DEBUG_STATISTICS__
//#define __CONSTANT_TORQUE__
//#define __DEBUG_NECESSARY_ONLY__
//#define __USING_NECESSARY_ONLY__
//#define __BB_USE_NECESSARY_ONLY__
//#define __USING_PRE_PROCESSING_RBF__
//#define __UB_USING_EXACT_ANALYSIS__
//#define __LS_USING_EXACT_ANALYSIS__
//#define __DEBUG_VERIFICATION_TIMES__
//#define __DEBUG_COLLECT_LEVEL_INFO__
//#define __DEBUG_SCHED_ANALYSIS_DAVR__

//#define __USING_LOCAL_SEARCH_DC__
//#define __USING_AUDSLEY_OPA__
#define __USING_ALL_TASKS_DMPO__
//#define __USING_PERIODIC_TASKS_DMPO__
//#define __USING_CONSTANT_PRIORITIES__

#define __TEST_LOCAL_BB__
#define __USING_FIXED_PERIODS__

#define __BB_USING_PREPROCESS__
//#define __USING_PERFORMANCE_FACTOR__

// Note: only one of the following three setting can exist.
#define __USING_BIONDI_PERIOD_SETTING__ // {5,10,20,50,80,100} ms
//#define __USING_FUEL_INJECTION_PERIOD_SETTING__ // {4,12,50,100,1000} ms
//#define __USING_REAL_WORLD_PERIOD_SETTTING__ // {1,2,5,10,20,50,100,200,1000} ms

#define __IMPLICIT_DEADLINE__
//#define __TEST_RBF_ANALYSES__

#define __USING_FINE_SEPARATION__
#define __USING_CONSTANT_SEPARATION__ 

#define __USING_DIGRAPH_RBF__
//#define __USING_ILPCPLEX__

//#define __USING_CONSTANT_ACCELERATION__
#define __USING_BIONDI_EXP_SETTING__ // call function RPM_to_W
#define __USING_MINIMUM_SEARCH_STEP_RPM__ 

//#define __OUTPUT_OPTIMIAZATION_DC__
//#define __USING_FAST_DAVR__
#define __USING_MIXED_AVRTASK__
//#define __USING_MAX_DIGRAPH__
//#define __USING_STATIC_OFFSETS__
#define __USING_ARBITRARY_OFFSETS__

//#define __PARTITIONING_WITH_CONSTANT_SCALE__
#define __PARTITIONING_WITH_CONSTANT_LENGTH__

#define __GENERATING_NEW_TASKSETS_4_SCHED_ANALYSIS__
#define __USING_COARSE_D2S_EXACT__

//#define __DISABLE_MOST_OUTPUT__

#endif