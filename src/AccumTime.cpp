#include "AccumTime.h"
#include "TimeWatcher.h"

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

AccumTime::AccumTime()
: TimeWatcher(), m_seconds{}, m_milliseconds{}, m_nanoseconds{},
	m_overhead_seconds{}, m_overhead_milliseconds{}, m_overhead_nanoseconds{}
{
}

void AccumTime::stop()
{	
	tEnd = std::make_unique<std::chrono::high_resolution_clock::time_point>(
	    std::chrono::high_resolution_clock::now()
	);
	addNewTime();
}

void AccumTime::addNewTime()
{
	auto duration(std::chrono::high_resolution_clock::duration(*tEnd - *tStart));

	m_seconds += std::chrono::duration_cast<std::chrono::seconds>(duration).count();
	m_milliseconds += std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
	m_nanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

double AccumTime::getElapsedDecimalSeconds()
{
	return static_cast<double>(m_milliseconds) / 1000.0;
}

long AccumTime::getElapsedSeconds()
{
	return m_seconds;
}

long AccumTime::getElapsedMillis()
{
	return m_milliseconds;
}

long AccumTime::getElapsedNanos()
{
	return m_nanoseconds;
}

std::string AccumTime::getElapsedFormat()
{
	long seconds = m_seconds;
	long minutes = seconds / 60;
	long hours   = minutes / 60;
	minutes      = minutes % 60;
	seconds      = seconds % 60;
	std::stringstream strm;
	strm << hours << ":";
	strm << std::setfill('0') << std::setw(2);
	strm << minutes << ":" << seconds;
	return strm.str();
}

void AccumTime::reportSeconds(ostream & os, const string & msg)
{
	std::stringstream ss;
	ss << msg << m_seconds;
	os << ss.str();
}

void AccumTime::reportMillis(ostream & os, const string & msg)
{
	std::stringstream ss;
	ss << msg << m_milliseconds << '\n';
	os << ss.str();
}

void AccumTime::reportFormat(ostream & os, const std::string & msg)
{
	std::stringstream ss;
	ss << msg << getElapsedFormat() << '\n';
	os << ss.str();
}

void AccumTime::computeOverhead()
{
	start();
	stop();

	m_overhead_seconds = m_seconds;
	m_overhead_milliseconds = m_milliseconds;
	m_overhead_nanoseconds = m_nanoseconds;

	m_seconds = 0;
	m_milliseconds = 0;
	m_nanoseconds = 0;
}