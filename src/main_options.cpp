//
// Created by miguelyermo on 11/3/20.
//

#include "main_options.h"
#include <bits/getopt_core.h>
#include <iostream>
#include <memory>
#include <omp.h>
#include <string>

main_options mainOptions{};

void printHelp()
{
	std::cout
	    << "-h: Show this message\n"
		   "-a: Choose the algorithm (SMRF, CSF)"
	       "-i: Path to input file\n"
	       "-o: Path to output file\n"
		   "-t: Set the number of threads\n"
	       "-n <--num-points>     : Number of points to be read\n"
	       "--dtm-file: Path to the dtm file, if existing\n";
	exit(1);
}

void setDefaults()
{
	if (!mainOptions.userOutputDirName) mainOptions.outputDirName = "out";
}


void processArgs(int argc, char ** argv)
{
	while (true)
	{
		const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);
		if (-1 == opt)
		{
			break;
		}
		switch (opt)
		{
			// Short Options
			case 'a':
			{
				std::string algorithm(optarg);
				if (algorithm == "CSF")
					mainOptions.algorithm = Algorithm::CSF;
				else if (algorithm == "SMRF")
					mainOptions.algorithm = Algorithm::SMRF;
				break;
			}
			case 'h':
			{
				printHelp();
				break;
			}
			case 'i':
			{
				mainOptions.inputFile = std::string(optarg);
				std::cout << "Read file set to: " << mainOptions.inputFile << "\n";
				break;
			}
			case 'n':
			{
				mainOptions.numPoints     = std::stoi(optarg);
				mainOptions.userNumPoints = true;
				std::cout << "Number of points: " << mainOptions.numPoints << "\n";
				break;
			}
			case 'o':
			{
				mainOptions.outputDirName     = std::string(optarg);
				mainOptions.userOutputDirName = 1;
				std::cout << "Output path set to: " << mainOptions.outputDirName << "\n";
				break;
			}
			case 't':
			{
				int num_threads( std::stoi(optarg) );
				if (num_threads > 0)
				{
					omp_set_dynamic(0);
					omp_set_num_threads(num_threads);
				}
				break;
			}
			// Long Options
			case 0: // Help
			{
				printHelp();
				break;
			}
			case 1: // Number of points
			{
				mainOptions.numPoints     = std::stoi(optarg);
				mainOptions.userNumPoints = true;
				std::cout << "Number of points: " << mainOptions.numPoints << "\n";
				break;
			}

			case '?': // Unrecognized option
			default:
				printHelp();
				break;
		}
	}
}