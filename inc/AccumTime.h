#ifndef RULE_BASED_CLASSIFIER_CPP_ACCUM_TIME_H
#define RULE_BASED_CLASSIFIER_CPP_ACCUM_TIME_H

#include "TimeWatcher.h"

class AccumTime : public TimeWatcher
{
private:
	long m_seconds;
	long m_milliseconds;
	long m_nanoseconds;

	long m_overhead_seconds;
	long m_overhead_milliseconds;
	long m_overhead_nanoseconds;

	void addNewTime();

public:
	AccumTime();
	~AccumTime() = default;

	/**
   * @brief Stop the time watcher, which sets the ending point for a time
   *  measure
   */
	virtual void stop();

	/**
   * @brief Obtain the elapsed time as the real number of seconds
   * @return Elapsed time in real seconds
   */
	virtual double getElapsedDecimalSeconds();
	/**
   * @brief Obtain the elapsed time as the integer number of seconds
   * @return Elapsed time in integer seconds
   */
	virtual long getElapsedSeconds();
	/**
   * @brief Obtain the elapsed time as the integer number of milliseconds
   * @return Elapsed time in integer milliseconds
   */
	virtual long getElapsedMillis();
	/**
   * @brief Obtain the elapsed time as the integer number of nanoseconds
   * @return Elapsed time in integer nanoseconds
   */
	virtual long getElapsedNanos();
	/**
   * @brief Obtain the elapsed time as a string with format "HH:MM:SS"
   * @return Elapsed time as "HH:MM:SS" string
   */
	virtual string getElapsedFormat();

	/**
   * @brief Report elapsed seconds through specified output stream
   * @param os Output stream for the report
   * @param msg Message to be shown by the report. By default
   *  "Total elapsed seconds: "
   */
	virtual void reportSeconds(ostream & os, const string& msg = "Total elapsed seconds: ");
	/**
   * @brief Report elapsed milliseconds through specified output stream
   * @param os Output stream for the report
   * @param msg Message to be shown by the report. By default
   *  "Total elapsed milliseconds: "
   */
	virtual void reportMillis(ostream & os, const string& msg = "Total elapsed milliseconds: ");
	/**
   	* @brief Report elapsed time through specified output stream using
   	*  "HH:MM:SS" format
   	* @param os Output stream for the report
   	* @param msg Message to be shown by the report. By default
   	*  "Total elapsed time: "
   	*/
	virtual void reportFormat(ostream & os, const string& msg = "Total elapsed time: ");

	/**
	* @brief Compute the overhead between two calls
	*/
	void computeOverhead();

	long overheadSeconds() { return m_overhead_seconds; }
	long overheadMilliseconds() { return m_overhead_milliseconds; }
	long overheadNanoseconds() { return m_overhead_nanoseconds; }
};

#endif