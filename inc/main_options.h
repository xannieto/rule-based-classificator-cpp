//
// Created by miguelyermo on 11/3/20.
//

/*
* FILENAME :  main_options.h  
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION :
*  
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 18:50 11/3/20
*
*/

#ifndef CPP_MAIN_OPTIONS_H
#define CPP_MAIN_OPTIONS_H

#include <getopt.h>
#include <iostream>

enum class Algorithm
{
	EMPTY,
	CSF,
	SMRF
};

class main_options
{
	public:
	// Files & paths
	std::string   inputFile{};
	unsigned char userOutputDirName{};
	std::string   outputDirName{};
	Algorithm	  algorithm{Algorithm::EMPTY};

	// Numerical parameters
	bool         userNumPoints{};
	unsigned int numPoints{};
	unsigned int numCols{};

};

extern main_options mainOptions;

// Define short options
const char * const short_opts = "a:h:i:o:n";

// Define long options
const option long_opts[] = {
	{ "help", no_argument, nullptr, 'h' },
	{ "num-points", required_argument, nullptr, 1 }
};

void printHelp();
void processArgs(int argc, char ** argv);
void setDefaults();

#endif //CPP_MAIN_OPTIONS_H
