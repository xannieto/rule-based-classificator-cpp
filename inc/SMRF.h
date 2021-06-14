//
// Created by miguelyermo on 9/7/20.
//

/*
* FILENAME :  dtm.h  
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION : Handles a previously created DTM using dtm executable.
*  
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 09:20 9/7/20
*
*/

#ifndef RULE_BASED_CLASSIFIER_CPP_DTM_H
#define RULE_BASED_CLASSIFIER_CPP_DTM_H


#include "convexHull.hpp"
#include "main_options.h"
#include "octree.h"
#include "util.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <omp.h>
#include <set>
#include <utility>

namespace SMRF
{

const int          NO_DATA_VAL     = -999;
static double CELL_SIZE       = 1.0;
static double MAX_SLOPE       = 0.15;
static double MAX_WINDOW_SIZE = 16.0;
static double MAX_HEIGHT = 0.50;

typedef struct Coordinate
{
	double   value;
	long int index;

} Coord;

/** Struct to keep track of indices after qsort() */
typedef struct
{
	size_t index;
	size_t value;
} Pair;

typedef std::pair<int, int> indexOfCell_t;

class DTMCell
{
	public:
	unsigned int id{};
	bool         obj;
	bool         empty;
	bool         inhull;
	bool         inpainted;
	double       center[2];
	double       z;
	double       slope;

	std::vector<DTMCell *> neighbors{};

	inline double distanceToCell(DTMCell & cell)
	{
		return std::sqrt((cell.center[0] - center[0]) * (cell.center[0] - center[0]) +
		                 (cell.center[1] - center[1]) * (cell.center[1] - center[1]));
	}

	inline double heightDifference(DTMCell & cell) { return cell.z - z; }

	friend std::ostream & operator<<(std::ostream & out, const DTMCell & cell)
	{
		out << cell.center[0] << " " << cell.center[1] << " " << cell.z;
		return out;
	}

	double slope_2FD{};
	double slope_3FDWRD{};
	double slope_3FD{};
	double slope_3FDWRSD{};
	double slope_FFD{};
	double slope_MAX{};
	double slope_D{};
	double slope_ARCGIS{};
};

class DTMGrid
{
	// Attributes
	public:
	int                               rows{};
	int                               cols{};
	int                               size{};
	int                               numPts{};
	std::vector<int>                  ci;             // indice i da cela á que pertence un punto dado
	std::vector<int>                  cj;             // indice j da cela á que pertence un punto dado
	std::array<double, 2>             min{};
	std::vector<std::vector<DTMCell>> cells{};

	// Constructors
	// Default Constructor
	DTMGrid() = default;
	DTMGrid(double * min, double * max, int numPts, double cs);

	// Copy Constructor
	DTMGrid(const DTMGrid & dtm);

	void computeDTM(std::vector<Lpoint> & points, Octree & octree, double * min, double * max);
	void genMinSurface(Octree & octree, ConvexHull & hull);
	void identifyOutliers(int numPts, double minZ, double maxZ);
	void identifyCells(int numPts);
	void inpaintBE();
	void inpaintCellBE(int ci, int cj);
	std::vector<indexOfCell_t> getAdjCellsBE(int ci, int cj);
	void    clampNeighIdcs(int ci, int cj, int r, int min[2], int max[2]) const;
	size_t              getNumAdjBE(int ci, int cj);
	void    computeDtmZ(std::vector<Lpoint> & points, Octree & octree);
	double  nnInterpolation(Lpoint & p);
	double  slopeInterpolation(Lpoint & point);
	std::vector<size_t> getNeighCells(int ci, int cj, int r);
	void    inpaintMS(std::vector<indexOfCell_t> & vectorOfEmptyCells);
	void    inpaintCellMS(int ci, int cj);
	std::vector<indexOfCell_t> getAdjCellsMS(int ci, int cj);
	int     getNumAdjMS(int ci, int cj);
	void    calcSlopes();
	DTMGrid morphologicalOpening(int radius, int numPts);


	// Methods

	void computeCellNeighbors();
	
	using Cluster = std::vector<indexOfCell_t>;
	using CheckValueCellFn = bool(*)(DTMGrid*, indexOfCell_t);
	using ChangeCellStateFn = void(*)(DTMGrid*, indexOfCell_t&);
	using ListNeighsFn = std::vector<indexOfCell_t>(*)(DTMGrid*, std::vector<indexOfCell_t>&);
	using CheckNeighsFn = bool(*)(DTMGrid*, std::vector<indexOfCell_t>&);
	void createClusters(std::vector<indexOfCell_t>& inpaintCells, std::map<indexOfCell_t, std::vector<indexOfCell_t>>& neighbourCells,
		std::vector<Cluster>& clusters, ListNeighsFn listNeighsFn, CheckNeighsFn chNeighsFn);
	void solveSystems(std::vector<Cluster>& clusters, std::map<indexOfCell_t, std::vector<indexOfCell_t>>& neighbourCells,
		CheckValueCellFn chValueCellFn, ListNeighsFn listNeighsFn, CheckNeighsFn chNeighsFn, ChangeCellStateFn chCellStateFn);

	void writeToFile(const std::string & filename)
	{
		std::ofstream f(filename);
		f << std::fixed << std::setprecision(3);
		for (int i = 0; i < rows; i++)
			for (int j = 0; j < cols; j++)
				if (cells[i][j].inhull)
				{
					f << cells[i][j].center[0] << " " << cells[i][j].center[1] << " " << cells[i][j].z
					  << " " << cells[i][j].slope << "\n";
				}
	}

	void writeToFileForCompare(const std::string & filename)
	{
		std::ofstream f(filename);
		f << std::fixed << std::setprecision(3);
		for (int i = 0; i < rows; i++)
			for (int j = 0; j < cols; j++)
				if (cells[i][j].inhull)
				{
					f << cells[i][j].z << " " << cells[i][j].id << "\n";
				}
	}

	void createDTM(const std::string & filename)
	{
		std::ofstream f(filename);

		f << std::fixed << std::setprecision(2);

		f << "NCOLS " << cols << "\n";
		f << "NROWS " << rows << "\n";
		f << "XLLCENTER " << cells[0][0].center[0] << "\n";
		f << "YLLCENTER " << cells[0][0].center[1] << "\n";
		f << "CELLSIZE " << CELL_SIZE << "\n";
		f << "NODATA_VALUE " << NO_DATA_VAL << "\n";
		// NOTA: En C, for(i = grid->rows-1; i > 0; i--)
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				if (cells[i][j].z == std::numeric_limits<double>::max())
				{
					f << NO_DATA_VAL << " ";
				}
				else
				{
					f << cells[i][j].z << " ";
				}
			}
			f << "\n";
		}
	}
};


double distP2C(Lpoint & p, DTMCell * cell);

static int pairCmpDesc(const void * p1, const void * p2);

void getMinMaxCoordinates(std::vector<Lpoint> & points, double * min, double * max);

DTMGrid smrf(std::vector<Lpoint> & points, Octree & octree, double min[3], double max[3]);


} // namespace dtm

#endif //RULE_BASED_CLASSIFIER_CPP_DTM_H
