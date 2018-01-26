#include "Timer.h"
//#include <Windows.h>

Timer::Timer() {
#ifdef WINDOWS_PLATFORM
	QueryPerformanceFrequency(&m_i64CPUFreq);
#endif
}

Timer::~Timer() {}

void Timer::start() {
	reset();
#ifdef WINDOWS_PLATFORM
	QueryPerformanceCounter(&m_i64Begin);
#endif

#ifdef LINUX_PLATFORM
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start_time);
#endif
}

void Timer::end() {
#ifdef WINDOWS_PLATFORM
	QueryPerformanceCounter(&m_i64End);
#endif

#ifdef LINUX_PLATFORM
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &end_time);
#endif
}

double Timer::getTime() const {
#ifdef WINDOWS_PLATFORM
	return (double)(m_i64End.QuadPart-m_i64Begin.QuadPart)/(double)m_i64CPUFreq.QuadPart;
#endif

#ifdef LINUX_PLATFORM
	timespec temp;
	if ((end_time.tv_nsec-start_time.tv_nsec)<0) {
		temp.tv_sec = end_time.tv_sec-start_time.tv_sec-1;
		temp.tv_nsec = 1000000000+end_time.tv_nsec-start_time.tv_nsec;
	} else {
		temp.tv_sec = end_time.tv_sec-start_time.tv_sec;
		temp.tv_nsec = end_time.tv_nsec-start_time.tv_nsec;
	}
	return (double)temp.tv_nsec/1000000000;
#endif
}

void Timer::reset() {
#ifdef WINDOWS_PLATFORM
	m_i64Begin = m_i64End;
#endif

#ifdef LINUX_PLATFORM
	start_time = end_time;
#endif
}