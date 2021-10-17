#include "AccumTime.h"
#include "TimeWatcher.h"

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

AccumTime::AccumTime()
: m_timewatcher(), m_seconds{}, m_milliseconds{}, m_nanoseconds{},
	m_overhead_seconds{}, m_overhead_milliseconds{}, m_overhead_nanoseconds{}
{
	computeOverhead(); // compute the overhead between calls
}

AccumTime& AccumTime::instance()
{
	static AccumTime instance;

	return instance;
}

void AccumTime::start()
{
	m_timewatcher.start();
}

void AccumTime::stop(const std::string& details)
{	
	m_timewatcher.stop();

	Interval interval{};
	interval.measure_info = details;
	interval.seconds = m_timewatcher.getElapsedSeconds() - m_overhead_seconds;
	interval.milliseconds = m_timewatcher.getElapsedMillis() - m_overhead_milliseconds;
	interval.nanoseconds = m_timewatcher.getElapsedNanos() - m_overhead_nanoseconds;

	m_seconds += interval.seconds;
	m_milliseconds += interval.milliseconds;
	m_nanoseconds += interval.nanoseconds;
	m_records.emplace_back(interval);
}

void AccumTime::report(ostream & os)
{
	std::stringstream ss;

	ss << "--- REPORTED TIMES ---\n";
	for (auto& interval : m_records)
	{
		double decimalSeconds(interval.milliseconds / 1000.0);
		ss << interval.measure_info << ": " << decimalSeconds << "s\n"; 
	}
	ss << "TOTAL TIME: " << m_milliseconds / 1000.0 << "s\n";

	os << ss.str();
}

void AccumTime::computeOverhead()
{
	m_timewatcher.start();
	m_timewatcher.stop();

	m_overhead_seconds = m_timewatcher.getElapsedSeconds();
	m_overhead_milliseconds = m_timewatcher.getElapsedMillis();
	m_overhead_nanoseconds = m_timewatcher.getElapsedNanos();
}