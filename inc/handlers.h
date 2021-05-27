//
// Created by miguelyermo on 1/3/20.
//

/*
* FILENAME :  handlers.h  
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION :
*  
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 03:07 1/3/20
*
*/

#ifndef CPP_HANDLERS_H
#define CPP_HANDLERS_H

#define NULL_EXT "null_ext" // To indicate that a file has no extension

#include "point.h"

#include <algorithm>
#include <array>
#include <experimental/filesystem> // File extensions
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace filesys = std::experimental::filesystem;


unsigned int getNumberOfPoints(const char * filePath);

void handleNumberOfPoints(std::vector<Lpoint> & points);

unsigned int getNumberOfCols(std::string & filePath);

std::string getFileExtension(std::string & filename);

void createDirectory(std::string & dirname);

std::vector<Lpoint> readPointCloud(std::string fileName);

std::vector<Lpoint> readPoints(std::string & filePath, unsigned int numCols);

std::vector<Lpoint> readPoints(std::string & filePath);

void writePoints(std::string & filePath, std::vector<Lpoint> & points);

bool isDir(const std::string & s);

void createDir(std::string & pathToDirectory);

std::ofstream openFile(const std::string & filePath);

void removeFilesInDir(std::string & pathToDirectory);

void deleteFileIfExists(const std::string & filePath);

std::vector<Lpoint> readGroundTruth(std::string& filePath, unsigned int numCols);

#endif //CPP_HANDLERS_H
