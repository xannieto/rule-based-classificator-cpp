#ifndef RULE_BASED_CLASSIFIER_CPP_ACCUM_TIME_H
#define RULE_BASED_CLASSIFIER_CPP_ACCUM_TIME_H

#include "TimeWatcher.h"

#include <map>
#include <string>
#include <vector>

struct Interval
{
	std::string measure_info;
	long seconds;
	long milliseconds;
	long nanoseconds;

	Interval() : measure_info(""), seconds(0), milliseconds(0), nanoseconds(0) {}
	Interval(std::string& sDetails, long s, long m, long n)
		: measure_info(sDetails), seconds(s), milliseconds(m), nanoseconds(n) {}
};

class AccumTime
{
private:
	TimeWatcher m_timewatcher;

	long m_seconds;
	long m_milliseconds;
	long m_nanoseconds;

	long m_overhead_seconds;
	long m_overhead_milliseconds;
	long m_overhead_nanoseconds;

	std::vector<Interval> m_records;
	
	AccumTime();

	void addNewTime(const std::string&);

public:
	static AccumTime& instance();
	AccumTime(AccumTime const&) = delete;
	void operator=(AccumTime const&) = delete;

	void start();
	void stop(const std::string& info);

	void computeOverhead();

	long overheadSeconds() { return m_overhead_seconds; }
	long overheadMilliseconds() { return m_overhead_milliseconds; }
	long overheadNanoseconds() { return m_overhead_nanoseconds; }

	std::vector<Interval>& getRecords() { return m_records; }

	void report(ostream & os);
};

#endif