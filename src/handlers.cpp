//
// Created by miguelyermo on 1/3/20.
//

#include "handlers.h"
#include <boost/algorithm/string.hpp>
#include <dirent.h>
#include "point_cloud.h"
#include <fstream>
#include <iostream>
#include <main_options.h>

unsigned int getNumberOfPoints(const char *filePath)
{
	std::ifstream inFile(filePath);
	unsigned int numLines = std::count(std::istreambuf_iterator<char>(inFile),
										std::istreambuf_iterator<char>(), '\n');

	return numLines;
}

void handleNumberOfPoints(std::vector<Lpoint> &points)
/**
 * Normalization of the number of points.
 * If userNumPoints > numPoints -> numPoints
 * If userNumPoints < numPoints -> userNumPoints
 * If no userNumPoints -> numPoints
 */
{
	unsigned int numPoints = points.size();

	if (mainOptions.userNumPoints)
	{
		if (mainOptions.numPoints < numPoints)
		{
			points.resize(mainOptions.numPoints);
		}
	}
}

unsigned int getNumberOfCols(std::string &filePath)
/**
 * Get the number of columns of the input file.
 * @param filePath
 * @return
 */
{
	std::ifstream file(filePath);
	std::string line, item;
	unsigned int numCols = 0;

	// First Line
	std::getline(file, line);
	std::stringstream ss(line);

	while (ss >> item)
		numCols++;

	file.close();
	return numCols;
}

void createDirectory(std::string &dirname)
/**
 * This function creates a directory if it does not exist.
 * @param dirname
 * @return
 */
{
	filesys::path dir = dirname;
	if (!filesys::is_directory(dir))
	{
		filesys::create_directories(dir);
	}
}

std::string getFileExtension(std::string &filename)
/**
 * 	//
 * https://thispointer.com/c-how-to-extract-file-extension-from-a-path-string-using-boost-c17-filesystem-library/
 * @param filename
 * @return
 */
{
	filesys::path pathObj(filename);
	std::string fExt;
	if (pathObj.has_extension())
	{
		fExt = pathObj.extension().string();
	}
	else
	{
		fExt = NULL_EXT;
	}

	return fExt;
}

std::string getAndReplaceLowerFileExtension(std::string &filename)
/**
 * This function gets the lowercase version of the extension of the file and
 * replace it in the filename.
 * @param filename
 * @return
 */
{
	filesys::path pathObj(filename);
	// Get File Extension
	std::string fExt = getFileExtension(filename);
	// Lowercase file extension
	boost::algorithm::to_lower(fExt);
	// Replace File Extension
	pathObj.replace_extension(fExt);
	filename = pathObj.string();

	return fExt;
}

void writePoints(std::string &filename, std::vector<Lpoint> &points) {
	std::ofstream f;
	f = openFile(filename);
	f << std::fixed << std::setprecision(2);

	for (Lpoint &p : points)
	{
		f << p << "\n";
	}

	f.close();
}

std::vector<Lpoint> readPointCloud(std::string filename) {
	// Get Input File extension
	std::string fExt = getAndReplaceLowerFileExtension(filename);

	std::vector<Lpoint> points{};

	if (fExt != NULL_EXT)
	{
		if (fExt == ".xyz" || fExt == ".txt") // The file is plain text
		{
			unsigned int numCols = getNumberOfCols(filename);
			std::cout << "Number of columns: " << numCols << "\n";
			mainOptions.numCols = numCols;
			points = readPoints(filename, numCols);
			return std::move(points);
		}
		else
		{
			std::cout << "Unrecognized file format. Exiting now.\n";
			exit(1);
		}
	}
	else
	{
		std::cout << "File has no extension. Files to be read must have .xyz, .txt "
					"or .las extensions. Exiting now.\n";
		exit(2);
	}
}

std::vector<Lpoint> readGroundTruth(std::string &filePath, unsigned int numCols) {
	std::ifstream inFile(filePath);
	double x, y;
	unsigned int fuelType;
	std::vector<Lpoint> points;

	switch (numCols)
	{
	case 3:
		while (inFile >> x >> y >> fuelType)
		{
			points.emplace_back(x, y, fuelType);
		}
		break;

	default:
		std::cout << "Unrecognized format\n";
		exit(1);
	}
	inFile.close();
	return points;
}

std::vector<Lpoint> readPoints(std::string &filePath, unsigned int numCols)
/**
 * This function read the LiDAR points directly from a plain text file.
 * @param filePath
 * @param numCols
 * @return
 */
{
	std::ifstream inFile(filePath);
	double x, y, z, realZ, I;
	unsigned int gId, classification, r, g, b, fuelType;
	unsigned short nor, rn, dir, edge;
	unsigned int idx = 0;
	std::vector<Lpoint> points;

	std::cout << "Number of read cols: " << numCols << "\n";
	switch (numCols)
	{
	// Classified point cloud
	case 3:
		while (inFile >> x >> y >> z)
		{
			points.emplace_back(x, y, z);
		}
		break;
	
	case 6:
		while (inFile >> x >> y >> z >> I >> gId >> classification)
		{
			points.emplace_back(idx, x, y, z, I, gId, classification);
			idx++;
		}
		break;

	case 7:
		while (inFile >> x >> y >> z >> realZ >> I >> gId >> classification)
		{
			points.emplace_back(idx, x, y, z, realZ, I, gId, classification);
			idx++;
		}
		break;

	case 8: // Data from https://www.itc.nl/isprs/wgIII-3/filtertest/downloadsites/
		double xx, yy, zz, II;
		while (inFile >> x >> y >> z >> I >> xx >> yy >> zz >> II)
		{
			points.emplace_back(idx++, x, y, z, I);
			points.emplace_back(idx++, xx, yy, zz, I);
		}
		break;

	// Raw point cloud without RGB
	case 9:
		while (inFile >> x >> y >> z >> I >> rn >> nor >> dir >> edge >>
				classification) {
			points.emplace_back(idx, x, y, z, I, rn, nor, dir, edge, classification);
			idx++;
		}
		break;
	// Over-segmented point cloud with group Ids
	case 10:
		while (inFile >> x >> y >> z >> I >> rn >> nor >> dir >> edge >>
				classification >> gId) {
			points.emplace_back(idx, x, y, z, I, rn, nor, dir, edge, classification,
								gId);
			idx++;
		}
		break;

	case 11:
		while (inFile >> x >> y >> z >> realZ >> I >> rn >> nor >> dir >> edge >>
				classification >> gId) {
			points.emplace_back(idx, x, y, z, realZ, I, rn, nor, dir, edge,
								classification, gId);
			idx++;
		}
		break;

	case 12:
		while (inFile >> x >> y >> z >> I >> rn >> nor >> dir >> edge >>
				classification >> r >> g >> b) {
			points.emplace_back(idx, x, y, z, I, rn, nor, dir, edge, classification,
								r, g, b);
			idx++;
		}
		break;

	default:
		std::cout << "Unrecognized format\n";
		exit(1);
	}
	inFile.close();
	return points;
}

bool isDir(const std::string &s)
{
	struct stat buffer;
	return (stat(s.c_str(), &buffer) == 0);
}

void createDir(std::string &pathToDirectory)
/**
 * Creates pathToDirectory directory if it does not exist.
 * @param pathToDirectory Relative path to directory
 */
{
	const char *cString = pathToDirectory.c_str();
	DIR *dir = opendir(cString);
	if (dir)
	{
		/* Directory exists. */
		closedir(dir);
	}
	else if (ENOENT == errno)
	{
		/* Directory does not exist. */
		mkdir(cString, 0777);
	}
	else
	{
		/* opendir() failed for some other reason. */
		mkdir(cString, 0777);
	}
}

void removeFilesInDir(std::string &pathToDirectory)
/**
 * Checks if a directory is empty
 * @param pathToDirectory
 */
{
	struct dirent *next_file;
	const char *cString = pathToDirectory.c_str();
	DIR *dir = opendir(cString);
	char filepath[256];

	while ((next_file = readdir(dir)) != nullptr)
	{
		// build the path for each file in the folder
		sprintf(filepath, "%s/%s", cString, next_file->d_name);
		remove(filepath);
	}
	closedir(dir);
}

std::ofstream openFile(const std::string &filePath)
/**
 * Safely open file. Filepath must be the absolute or relative path to the file.
 * File must be closed explicity outside this function.
 * @param filePath Absolute or relative route to the file to be opened.
 * @return
 */
{
	std::ofstream f;
	std::ofstream null;
	f.open(filePath);

	if (f.is_open())
	{
		return f;
	}
	else
	{
		std::cout << "Unable to open file. Bad file path?\n";
		return null; // Return unopened file.
	}
}

void deleteFileIfExists(const std::string &filePath) {
	std::ifstream file(filePath);
	if (file.is_open())
	{
		std::remove(filePath.c_str());
	}
}
