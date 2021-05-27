#include "Cfg.h"
#include "CSF.h"
#include "SMRF.h"
#include "handlers.h"
#include "main_options.h"
#include "octree.h"
#include "point.h"
#include "TimeWatcher.h"

#include <iomanip>
#include <iostream>

int main(int argc, char * argv[])
{
	setDefaults();
	processArgs(argc, argv);

	// Handling Directories
	createDirectory(mainOptions.outputDirName);

	// Time measurements class
	TimeWatcher tw{};

	// Print decimals
	std::cout << std::fixed;
	std::cout << std::setprecision(3);

	if (mainOptions.algorithm == Algorithm::EMPTY)
	{
		std::cout << "Setting SMRF as algorithm." << '\n';
		mainOptions.algorithm = Algorithm::SMRF;
	}
	if (mainOptions.algorithm == Algorithm::SMRF)
	{
		Cfg cfg("smrf.cfg");

		if (!cfg.isGoodRead())
		{
			std::istringstream cell_size(cfg.getValue("cell_size"));
			std::istringstream max_slope(cfg.getValue("max_slope"));
			std::istringstream max_window_size(cfg.getValue("max_window_size"));

			cell_size >> SMRF::CELL_SIZE;
			max_slope >> SMRF::MAX_SLOPE;
			max_window_size >> SMRF::MAX_WINDOW_SIZE;
		}

		std::vector<Lpoint> points = readPointCloud(mainOptions.inputFile);
		// Handle number of points
		handleNumberOfPoints(points);

		// Global Octree Creation
		float  maxRadius = 0.0;
		Vector center    = mbb(points, maxRadius);
		std::cout << "Building global octree..." << '\n';
		Octree gOctree(center, maxRadius, points);


		tw.start();
		double min[3], max[3];
		SMRF::getMinMaxCoordinates(points, min, max);
		SMRF::DTMGrid dtm = SMRF::smrf(points, gOctree, min, max);
		tw.stop();
		double seconds_dtm = tw.getElapsedDecimalSeconds();
		std::cout << "DTM computation completed in " << seconds_dtm << "s\n";
	} 
	else if (mainOptions.algorithm == Algorithm::CSF) 
	{
		Cfg cfg("csf.cfg");

		std::string slop_smooth(cfg.getValue("slop_smooth"));
		//cfg.readConfigFile("params.cfg", "slop_smooth", slop_smooth);

		bool ss = false;
		if (slop_smooth == "true" || slop_smooth == "True")
		{
			ss = true;
		}
		else if (slop_smooth == "false" || slop_smooth == "False")
		{
			ss = false;
		}
		else{
			if (atoi(slop_smooth.c_str()) == 0){
				ss = false;
			}
			else
			{
				ss = true;
			}
		}

		std::string class_threshold(cfg.getValue("class_threshold"));
		std::string cloth_resolution(cfg.getValue("cloth_resolution"));
		std::string iterations(cfg.getValue("iterations"));
		std::string rigidness(cfg.getValue("rigidness"));
		std::string time_step(cfg.getValue("time_step"));
		//std::string terr_pointClouds_filepath(cfg.getValue("terr_pointClouds_filepath"));

		CSF csf{};
		//step 1 Input the point cloud
		csf.readPointsFromFile(mainOptions.inputFile);
		tw.start();
		//In real application, the point cloud may be provided by main program
		//csf.setPointCloud(pc);//pc is PointCloud class

		//step 2 parameter setting
		csf.params.bSloopSmooth = ss;
		csf.params.class_threshold = atof(class_threshold.c_str());
		csf.params.cloth_resolution = atof(cloth_resolution.c_str());
		csf.params.interations = atoi(iterations.c_str());
		csf.params.rigidness = atoi(rigidness.c_str());
		csf.params.time_step = atof(time_step.c_str());

		//step3 do filtering
		std::vector<int> groundIndexes, offGroundIndexes;
		csf.do_filtering(groundIndexes, offGroundIndexes, true);	
		tw.stop();

		std::cout << "Use Time: " << tw.getElapsedDecimalSeconds() << '\n';

		csf.savePoints(groundIndexes, mainOptions.outputDirName + "/ground.txt");
		csf.savePoints(offGroundIndexes,  mainOptions.outputDirName + "/non-ground.txt");
	}
}
