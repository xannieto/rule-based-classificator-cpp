//
// Created by miguelyermo on 23/7/20.
//

#include "convexHull.hpp"
#include "dtm.h"
#include "pseudoInverseMoorePenrose.h"

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <boost/exception/exception.hpp>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <ctime> 
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <functional>
#include <handlers.h>
#include <iostream>
#include <iterator>
#include <laspec.h>
#include <limits>
#include <map>
#include <memory>
#include <omp.h>
#include <ranges>
#include <ostream>
#include <queue>
#include <set>
#include <utility>
#include <vector>

//
// Created by miguelyermo on 9/7/20.
//

/*
* FILENAME :  dtm.h
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION : Creation and handling of a dtm.
*
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 09:20 9/7/20
*
*/

namespace dtm
{

constexpr double spring{ 0.707106781186547524 };
constexpr int MAX_NUM_CELL_SYSTEM{ 96 };

void classify(std::vector<Lpoint> & points, DTMGrid & dtm)
{
	int i = 0, count = 0;

	std::ofstream f(mainOptions.outputDirName + "/dtm/result.xyz");

	f << std::fixed << std::setprecision(3);

	printf("Labeling points...\n");

	for (Lpoint & p : points)
	{
		double dtmZ = dtm.nnInterpolation(p);
		p.setClass(p.z() < dtmZ + MAX_HEIGHT ? GROUND : UNCLASSIFIED);
		count += p.getClass() == GROUND;
		f << p.x() << " " << p.y() << " " << p.z() << " "
		  << "" << p.I() << " " << p.rn() << " " << p.nor() << " " << p.dir() << " " << p.edge()
		  << " " << p.getClass() << "\n";
	}
	std::cout << "Ground " << count / double(points.size()) * 100 << "\n";
}


/* Distance from point to cell center */
double distP2C(Lpoint & p, DTMCell * cell)
{
	return sqrt((p.x() - cell->center[0]) * (p.x() - cell->center[0]) +
	            (p.y() - cell->center[1]) * (p.y() - cell->center[1]));
}

/* Ensure neighbors indices are inside grid boundaries **/
inline void DTMGrid::clampNeighIdcs(int ci, int cj, int r, int min[2], int max[2]) const
{
	min[0] = ci - r >= 0 ? ci - r : 0;
	min[1] = cj - r >= 0 ? cj - r : 0;
	max[0] = ci + r < rows ? ci + r : rows - 1;
	max[1] = cj + r < cols ? cj + r : cols - 1;
}

/** Get non-empty neighboring cells in (row,col) pairs for a given radius (starts from left-bottom) */
// TODO: Funcion muy optimizable si se asignan los vecinos al crear la malla
// (8 vecinos salvo en esquinas y bordes) Ver computeCellNeighbors()
// Tambien son necesarios los vecinos de orden superior
inline std::vector<size_t> DTMGrid::getNeighCells(int Ci, int Cj, int r)
{
	int min[2], max[2], maxNeighs = 0;

	clampNeighIdcs(Ci, Cj, r, min, max);
	maxNeighs = (2 * r + 1) * (2 * r + 1) - 1;
	std::vector<size_t> neighs;
	neighs.reserve(2 * maxNeighs);
	for (size_t i = min[0]; i <= max[0]; i++)
		for (size_t j = min[1]; j <= max[1]; j++)
			if ((i != Ci || j != Cj) && (!cells[i][j].empty || cells[i][j].inpainted))
			{
				neighs.push_back(i);
				neighs.push_back(j);
			}
	return std::move(neighs);
}

double DTMGrid::nnInterpolation(Lpoint & p)
{
	{
		int    Ci = 0, Cj = 0, i = 0, j = 0, n = 0, numNeighs = 0;
		double dist = 0, minDist = 0, z = 0;
		Ci                         = ci[p.id()];
		Cj                         = cj[p.id()];
		z                          = cells[Ci][Cj].z;
		minDist                    = distP2C(p, &cells[Ci][Cj]);
		std::vector<size_t> neighs = getNeighCells(Ci, Cj, 1);
		for (n = 0; n < neighs.size(); n += 2)
		{
			i = neighs[n];
			j = neighs[n + 1];
			if ((int) cells[i][j].z == NO_DATA_VAL) continue;
			dist = distP2C(p, &cells[i][j]);
			if (dist < minDist)
			{
				z       = cells[i][j].z;
				minDist = dist;
			}
		}
		return z;
	}
}

double DTMGrid::slopeInterpolation(Lpoint & p)
{
	int    Ci = 0, Cj = 0, i = 0, j = 0, n = 0;
	double dist = 0, minDist = 0, slope = 0;
	Ci                         = ci[p.id()];
	Cj                         = cj[p.id()];
	slope                      = cells[Ci][Cj].slope;
	minDist                    = distP2C(p, &cells[Ci][Cj]);
	std::vector<size_t> neighs = getNeighCells(Ci, Cj, 1);
	for (n = 0; n < neighs.size(); n += 2)
	{
		i = neighs[n];
		j = neighs[n + 1];
		if ((int) cells[i][j].z == NO_DATA_VAL) continue;
		dist = distP2C(p, &cells[i][j]);
		if (dist < minDist)
		{
			slope   = cells[i][j].slope;
			minDist = dist;
		}
	}
	return slope;
}

void DTMGrid::computeDtmZ(std::vector<Lpoint> & points, Octree & octree)
{
	std::cout << "Computing dtm...\n";

	for (auto & p : points)
	{
		p.setSlopeDTM(slopeInterpolation(p));
		p.setDtmZ(nnInterpolation(p));
		// TODO: check DTM and solve the bug to avoid it
		if ((p.z() - p.getDtmZ()) < 0)
			p.setRealZ(0.01);
		else
			p.setRealZ(p.z() - p.getDtmZ());
	}
}

/** Compare pairs by value (descending order) */
static int pairCmpDesc(const void * p1, const void * p2)
{
	int v1 = ((Pair *) p1)->value;
	int v2 = ((Pair *) p2)->value;
	return v2 - v1;
}

static int cmpX(const void * a, const void * b)
{
	double v1 = ((struct Coordinate *) a)->value;
	double v2 = ((struct Coordinate *) b)->value;

	return (v1 > v2) - (v1 < v2);
}


extern DTMGrid * createGridDTM(double * min, double * max, int numPts, double cs)
{
	int       i = 0, j = 0;
	double    r    = cs / 2.0;
	DTMGrid * grid = nullptr;

	//grid         = (DTMGrid *) malloc(sizeof(DTMGrid));
	grid         = new DTMGrid;
	grid->min[0] = ceil(min[0]);
	grid->min[1] = ceil(min[1]);
	grid->rows   = ceil((floor(max[0]) - grid->min[0]) / cs);
	grid->cols   = ceil((floor(max[1]) - grid->min[1]) / cs);
	grid->size   = grid->rows * grid->cols;
	grid->ci.resize(numPts);
	grid->cj.resize(numPts);
	grid->cells.resize(grid->rows, std::vector<DTMCell>(grid->cols));
	for (i = 0; i < grid->rows; i++)
	{
		for (j = 0; j < grid->cols; j++)
		{
			grid->cells[i][j].id        = i * grid->cols + j;
			grid->cells[i][j].obj       = false;
			grid->cells[i][j].empty     = true;
			grid->cells[i][j].inpainted = false;
			grid->cells[i][j].center[0] = grid->min[0] + (i * cs) + r;
			grid->cells[i][j].center[1] = grid->min[1] + (j * cs) + r;
			grid->cells[i][j].z         = std::numeric_limits<double>::max();
		}
	}
	return grid;
}

/** Check if a cell is diagonal in relation to other cell */
bool isDiagonal(int ci, int cj, int ei, int ej)
{
	if (ci == (ei - 1) && cj == (ej - 1))
		return true;

	if (ci == (ei - 1) && cj == (ej + 1))
		return true;

	if (ci == (ei + 1) && cj == (ej - 1))
		return true;

	if (ci == (ei + 1) && cj == (ej + 1))
		return true;

	return false;	
}

/** Get the adjacent cells for BE Surface */
std::set<indexOfCell_t> DTMGrid::getAdjCellsBE(int ci, int cj)
{
	int min[2], max[2];

	clampNeighIdcs(ci, cj, 1, min, max);

	std::set<indexOfCell_t> neighs{};

	for (int i = min[0]; i <= max[0]; i++)
		for (int j = min[1]; j <= max[1]; j++)
		{
			if ((i != ci || j != cj) && cells[i][j].inhull)
			{
				neighs.emplace(std::make_pair(i, j));
			}
		}
	return std::move(neighs);
}

/** Get the adjacent cells for Minimun Surface */
std::set<indexOfCell_t> DTMGrid::getAdjCellsMS(int Ci, int Cj)
{
	int min[2], max[2]; // range of index for neighbours

	clampNeighIdcs(Ci, Cj, 1, min, max);
	std::set<indexOfCell_t> neighs{};

	for (int i = min[0]; i <= max[0]; i++)
		for (int j = min[1]; j <= max[1]; j++)
			if ( (i != Ci || j != Cj) && cells[i][j].inhull )
				neighs.emplace(std::make_pair(i, j));

	return std::move(neighs);
}

/** Check if a set have at least one cell which is not object */
bool checkFullObj(DTMGrid *grid, std::set<indexOfCell_t> & vec)
{
	for(const auto& cell : vec)
		if (!grid->cells[cell.first][cell.second].obj
			|| (grid->cells[cell.first][cell.second].obj && grid->cells[cell.first][cell.second].inpainted))
		{
			return false;
		}
	
	return true;
}

/** Filter the set taking object cells */
std::set<indexOfCell_t> checkForObjCells(DTMGrid *grid, std::set<indexOfCell_t> & vec)
{
	std::set<indexOfCell_t> tmp{};
	for (const auto& cell : vec)
		if (grid->cells[cell.first][cell.second].obj && !grid->cells[cell.first][cell.second].inpainted)
			tmp.emplace(cell);

	return std::move(tmp);
}

void DTMGrid::createClusters(std::vector<indexOfCell_t>& inpaintCells, std::map<indexOfCell_t, std::set<indexOfCell_t>>& neighbourCells,
		std::vector<Cluster>& clusters, ListNeighsFn listNeighsFn, CheckNeighsFn chNeighsFn)
{
	std::vector<indexOfCell_t> remainingCells(inpaintCells);

	while (!remainingCells.empty())
	{
		Cluster cluster{};
		// checkObj: cola que contén as celas baleiras adxacentes pendentes de comprobar
		// se teñen a súa vez máis celas adxacentes
		std::queue<indexOfCell_t> queueCheckCell{};

		queueCheckCell.emplace(remainingCells.front());
		// introducimos a primeira cela das que quedan por facer o inpainting
		// bucle para buscar veciños
		while (!queueCheckCell.empty() /*&& cluster.size() < MAX_NUM_CELL_SYSTEM*/)
		{
			auto cell(queueCheckCell.front());
			queueCheckCell.pop();

			// evita que caia en bucle infinito: para que non volva a revisar a mesma cela
			auto begin(remainingCells.begin());
			auto end(remainingCells.end());
			if (std::find(begin, end, cell) == end)
			{	
				continue;
			}

			auto neighbours(neighbourCells.at(cell));
			
			// quitamos esa cela da lista de pendentes por ver
			std::erase(remainingCells, cell);
			
            // anotamos esa cela para despois usala no sistema de ecuacións
			cluster.emplace_back(cell);
			
            // lista de veciños do mesmo tipo da cela extraida
			auto listNeighs(listNeighsFn(this, neighbours));

			for (auto empty{listNeighs.begin()}; empty != listNeighs.end(); ++empty)
			{
				queueCheckCell.emplace(*empty);
			}
		}
		clusters.emplace_back(cluster);
	}
}

void DTMGrid::solveSystems(std::vector<Cluster>& clusters, std::map<indexOfCell_t, std::set<indexOfCell_t>>& neighbourCells,
	CheckValueCellFn chValueCellFn, ListNeighsFn listNeighsFn, CheckNeighsFn chNeighsFn, ChangeCellStateFn chCellStateFn)
{
	/* resolución paralela dos clusters */
	#pragma omp parallel for shared(clusters, neighbourCells)
	for (int pos = 0; pos < clusters.size(); ++pos)
	{
		Cluster cluster(clusters.at(pos));
		
		while (!cluster.empty()) {
			int col{};
			std::map<indexOfCell_t, int> columns{};
			std::queue<indexOfCell_t> queueCells{};

			queueCells.emplace(cluster.front());
			
			while (!queueCells.empty())
			{
				auto cell(queueCells.front());
				queueCells.pop();

				// evita que caia en bucle infinito: para que non volva a revisar a mesma cela
				auto begin(cluster.begin());
				auto end(cluster.end());
				if (std::find(begin, end, cell) == end)
				{	
					continue;
				}

				auto neighbours(neighbourCells.at(cell));
				if (chNeighsFn(this, neighbours))
				{
					if (columns.empty())
					{
						std::erase(cluster, cell);
						cluster.emplace_back(cell);
						queueCells.emplace(cluster.front());
					}
					continue;
				}
				
				// quitamos esa cela da lista de pendentes por ver
				std::erase(cluster, cell);
				
				// anotamos esa cela para despois usala no sistema de ecuacións
				columns.emplace(cell, col++);
				
				// lista de veciños do mesmo tipo da cela extraida
				auto listNeighs(listNeighsFn(this, neighbours));

				for (auto empty{listNeighs.begin()}; empty != listNeighs.end(); ++empty)
				{
					queueCells.emplace(*empty);
				}
			}

			Eigen::MatrixXd M(columns.size() * 8, columns.size());
			M.setZero();
			Eigen::VectorXd b(columns.size() * 8);
			b.setZero();

			int currentRow{};
			for (auto pair{columns.begin()}; pair != columns.end(); ++pair)
			{
				auto cell(pair->first);
				auto local(neighbourCells.at(cell));

				int currentCol(columns.at(cell));

				// obtemos os valores Z das celas veciñas
				for (const auto& neighbour : local)
				{
					bool check(chValueCellFn(this, neighbour));

					if (check && !columns.count(neighbour))
						continue;

					double z(check ? 0.0 : cells[neighbour.first][neighbour.second].z);

					if (isDiagonal(neighbour.first, neighbour.second, cell.first, cell.second))
					{
						// se o veciño é unha cela do mesmo tipo (obj, empty), debe estar restando no sistema
						if (check /*&& adjacentCells.count(neighbour)*/)
						{
							M(currentRow, columns.at(neighbour)) = -spring; 
						}

						M(currentRow, currentCol) = spring;
						z *= spring; 
					}
					else
					{
						if (check /*&& adjacentCells.count(neighbour)*/)
						{
							M(currentRow, columns.at(neighbour)) = -1.0; 
						}
						
						M(currentRow, currentCol) = 1.0;
					}
					b(currentRow) = z;
					++currentRow;
				}
			}
			
			Eigen::VectorXd x = pseudoInverse(M) * b;

			for (auto element{columns.begin()}; element != columns.end(); ++element)
			{
				auto cell(element->first);
				int currentCol = columns.at(cell);
				cells[cell.first][cell.second].z = x(currentCol);
				chCellStateFn(this, cell);
			}
		}
	}
}

bool isObj(DTMGrid* grid, indexOfCell_t cell)
{
	return grid->cells[cell.first][cell.second].obj;
}

void changeObjToFalse(DTMGrid* grid, indexOfCell_t& cell)
{
	grid->cells[cell.first][cell.second].obj = false;
	grid->cells[cell.first][cell.second].inpainted = true;
}

/** Inpaint Bare Earth */
void DTMGrid::inpaintBE()
{
	std::cout << "Inpainting... " << std::flush;
	std::vector<indexOfCell_t> vectorOfObjCells{};

	// assign column for every cell
	//#pragma omp parallel for shared(vectorOfObjCells), collapse(2)
	for (int i = 0; i < rows; ++i)
		for (int j = 0; j < cols; ++j)
			if (cells[i][j].inhull && cells[i][j].obj)
			{
				vectorOfObjCells.emplace_back(std::make_pair(i, j));
			}

	const int numObj(vectorOfObjCells.size());

	// collemos tódolas celas baleiras e almacenamos os seus veciños nunha lista
	std::map<indexOfCell_t, std::set<indexOfCell_t>> neighbourOfObjCells{};
	// remainingCells contén as celas restantes para o inpainting (inicialmente todas)
	
	for (int pos = 0; pos < vectorOfObjCells.size(); ++pos)
	{
		auto cell(vectorOfObjCells.at(pos));
		neighbourOfObjCells.emplace(cell, getAdjCellsBE(cell.first, cell.second));
	}
	
	auto chValueFn{ &isObj };
	auto chCellStateFn{ &changeObjToFalse };
	auto lsNeighsFn{ &checkForObjCells };
	auto chNeighsFn{ &checkFullObj};

	std::vector<Cluster> clusters{};
	
	createClusters(vectorOfObjCells, neighbourOfObjCells, clusters, lsNeighsFn, chNeighsFn);
	solveSystems(clusters, neighbourOfObjCells, chValueFn, lsNeighsFn, chNeighsFn, chCellStateFn);

	std::cout << numObj << " inpainted" << '\n';
}

/** Get number of valid adjacent cells for Minimum Surface */
int DTMGrid::getNumAdjMS(int Ci, int Cj)
{
	int num = 0, min[2], max[2];

	clampNeighIdcs(Ci, Cj, 1, min, max);
	for (size_t i = min[0]; i <= max[0]; i++)
		for (size_t j = min[1]; j <= max[1]; j++)
			if ((i != Ci || j != Cj) && (!cells[i][j].empty || cells[i][j].inpainted)) num++;
	return num;
}

bool areAdjacent(indexOfCell_t c1, indexOfCell_t c2)
{
	// first, we check if c2 is in a near row
	if (c1.first == c2.first || c1.first == c2.first + 1 || c1.first == c2.first - 1)
		// second, the same with the column
		if (c1.second == c2.second || c1.second == c2.second + 1 || c1.second == c2.second - 1)
			return true;

	return false;
}

bool checkFullEmpty(DTMGrid *grid, std::set<indexOfCell_t> & vec)
{
	for(auto cell{vec.begin()}; cell != vec.end(); ++cell)
		if (!grid->cells[cell->first][cell->second].empty
			|| (grid->cells[cell->first][cell->second].empty && grid->cells[cell->first][cell->second].inpainted))
		{
			return false;
		}
	
	return true;
}

std::set<indexOfCell_t> checkForEmptyCells(DTMGrid *grid, std::set<indexOfCell_t> & vec)
{
	std::set<indexOfCell_t> tmp{};

	for (auto& cell : vec)
		if (grid->cells[cell.first][cell.second].empty && !grid->cells[cell.first][cell.second].inpainted)
			tmp.emplace(cell);

	return std::move(tmp);
}

void changeInpaintedToTrue(DTMGrid *grid, indexOfCell_t& cell)
{
	grid->cells[cell.first][cell.second].inpainted = true;
}

bool isEmpty(DTMGrid* grid, indexOfCell_t cell)
{
	return grid->cells[cell.first][cell.second].empty && !grid->cells[cell.first][cell.second].inpainted;
}

/** Inpaint Minimum Surface */
void DTMGrid::inpaintMS(std::vector<indexOfCell_t>& vectorOfEmptyCells)
{
	const int numEmpty(vectorOfEmptyCells.size());
	std::cout << "Inpainting... " << std::flush;

	// collemos tódolas celas baleiras e almacenamos os seus veciños nunnha lista
	std::map<indexOfCell_t, std::set<indexOfCell_t>> neighbourOfEmptyCells{};

	// TODO: paralelizar 
	for (int pos = 0; pos < vectorOfEmptyCells.size(); ++pos)
	{
		auto cell(vectorOfEmptyCells.at(pos));
		neighbourOfEmptyCells.emplace(cell, getAdjCellsMS(cell.first, cell.second));
	}

	auto chValueFn{ &isEmpty };
	auto chCellStateFn{ &changeInpaintedToTrue };
	auto lsNeighsFn{ &checkForEmptyCells };
	auto chNeighsFn{ &checkFullEmpty };
	
	std::vector<Cluster> clusters{};

	createClusters(vectorOfEmptyCells, neighbourOfEmptyCells, clusters, lsNeighsFn, chNeighsFn);
	solveSystems(clusters, neighbourOfEmptyCells, chValueFn, lsNeighsFn, chNeighsFn, chCellStateFn);
	
	std::cout << numEmpty << " inpainted" << '\n';
}

/** Generate a minimun surface with the lowest cell points */
void DTMGrid::genMinSurface(Octree & octree, ConvexHull & hull)
{
	int numEmpty = 0;
	Lpoint center(0, 0, 0, 0);
	std::vector<indexOfCell_t> emptyCells{};

	for (int i{}; i < rows; ++i)
		for (int j{}; j < cols; ++j)
		{
			center.id(-1);
			center.setX(cells[i][j].center[0]);
			center.setY(cells[i][j].center[1]);
			cells[i][j].inhull = hull.inHull(center);
			if (!cells[i][j].inhull) continue;
			std::vector<Lpoint *> points = octree.searchNeighbors2D(center, CELL_SIZE);
			if (!points.empty())
			{
				for (int k{}; k < points.size(); ++k)
				{
					ci[points[k]->id()] = i;
					cj[points[k]->id()] = j;
					if (points[k]->z() < cells[i][j].z) cells[i][j].z = points[k]->z();
				}
				cells[i][j].empty = false;
			}
			else
			{
				emptyCells.emplace_back(std::make_pair(i, j));
				numEmpty++;
			}
		}

	printf("%d empty cells %.2f%%\n", numEmpty, numEmpty * 100.0 / size);
	inpaintMS(emptyCells);
}

/** Perform morphological opening */
DTMGrid DTMGrid::morphologicalOpening(int radius, int numPts)
{
	int i = 0, j = 0, n = 0, ni = 0, nj = 0, numNeighs = 0;

	DTMGrid eGrid = *this;

#pragma omp parallel for default(none) private(i, j, n, ni, nj, numNeighs) \
    shared(radius, eGrid)
	for (i = 0; i < rows; i++) // Erosion
		for (j = 0; j < cols; j++)
		{
#if USE_DISK
			neighs = getDiskNeighCells(grid, i, j, radius, &numNeighs);
#else
			std::vector<size_t> neighs = getNeighCells(i, j, radius);
#endif
			for (n = 0; n < neighs.size(); n += 2)
			{
				ni = neighs[n];
				nj = neighs[n + 1];
				if (cells[ni][nj].z < eGrid.cells[i][j].z) eGrid.cells[i][j].z = cells[ni][nj].z;
			}
		}

	DTMGrid oGrid = eGrid;
#pragma omp parallel for default(none) private(i, j, n, ni, nj, numNeighs) \
    shared(radius, eGrid, oGrid)
	for (i = 0; i < eGrid.rows; i++) // Dilation
		for (j = 0; j < eGrid.cols; j++)
		{
#if USE_DISK
			neighs = getDiskNeighCells(eGrid, i, j, radius, &numNeighs);
#else
			std::vector<size_t> neighs = eGrid.getNeighCells(i, j, radius);
#endif
			for (n = 0; n < neighs.size(); n += 2)
			{
				ni = neighs[n];
				nj = neighs[n + 1];
				if (eGrid.cells[ni][nj].z > oGrid.cells[i][j].z)
					oGrid.cells[i][j].z = eGrid.cells[ni][nj].z;
			}
		}

	return oGrid;
}


void getMinMaxCoordinates(std::vector<Lpoint> & points, double * min, double * max)
{
	min[0] = min[1] = min[2] = std::numeric_limits<double>::max();
	max[0] = max[1] = max[2] = -std::numeric_limits<double>::max();
	for (auto & p : points)
	{
		if (p.x() < min[0]) min[0] = p.x();
		if (p.y() < min[1]) min[1] = p.y();
		if (p.z() < min[2]) min[2] = p.z();
		if (p.x() > max[0]) max[0] = p.x();
		if (p.y() > max[1]) max[1] = p.y();
		if (p.z() > max[2]) max[2] = p.z();
	}
}

/** Calculate plane slope by average maximum technique (ArGIS)
 *  @todo Handle slope when the cell it's at the grid boundary
 */
void DTMGrid::calcSlopes()
{
	double a = 0, b = 0, c = 0, d = 0, f = 0, g = 0, h = 0, I = 0, dzdx = 0, dzdy = 0,
	       rise = 0;

	for (size_t i = 0; i < rows; i++)
		for (size_t j = 0; j < cols; j++)
			if (!cells[i][j].empty || cells[i][j].inpainted)
			{
				std::vector<size_t> neighs = getNeighCells(i, j, 1);
				if (neighs.size() == 16)
				{
					g                 = cells[neighs[0]][neighs[1]].z;
					h                 = cells[neighs[2]][neighs[3]].z;
					I                 = cells[neighs[4]][neighs[5]].z;
					d                 = cells[neighs[6]][neighs[7]].z;
					f                 = cells[neighs[8]][neighs[9]].z;
					a                 = cells[neighs[10]][neighs[11]].z;
					b                 = cells[neighs[12]][neighs[13]].z;
					c                 = cells[neighs[14]][neighs[15]].z;
					dzdx              = ((c + 2 * f + I) - (a + 2 * d + g)) / (8 * CELL_SIZE);
					dzdy              = ((g + 2 * h + I) - (a + 2 * b + c)) / (8 * CELL_SIZE);
					rise              = sqrt(dzdx * dzdx + dzdy * dzdy);
					cells[i][j].slope = atan(rise) * 180 / M_PI;
				}
				else
				{
					// see todo
				}
			}
}


void DTMGrid::computeDTM(std::vector<Lpoint> & points, Octree & octree, double * Min,
                         double * max)
{
	ConvexHull hull(points);

	genMinSurface(octree, hull);
	identifyOutliers(points.size(), Min[2], max[2]);
	identifyCells(points.size());
	inpaintBE();
}

/** Improved Simple Morphological Filter (SMRF) (Pingel, 2013) */
DTMGrid smrf(std::vector<Lpoint> & points, Octree & octree, double min[3], double max[3])
{
	std::cout << "---- Computing DTM (SMRF) ----\n";
	DTMGrid dtm(min, max, points.size(), CELL_SIZE);
	dtm.computeDTM(points, octree, min, max);


	dtm.calcSlopes();
	dtm.computeDtmZ(points, octree);

	std::string outDir = mainOptions.outputDirName + "/dtm";
	createDirectory(outDir);
	dtm.writeToFile(outDir + "/dtm.xyz");
	dtm.createDTM(outDir + "/dtm.asc");

	//	createDTM(minSurface);
	classify(points, dtm);

	return dtm;
}

DTMGrid::DTMGrid(double * min, double * max, int numPts_, double cs)
{
	int    i = 0, j = 0;
	double r = cs / 2.0;

	min[0] = ceil(min[0]);
	min[1] = ceil(min[1]);
	cols   = ceil((floor(max[0]) - min[0]) / cs);
	rows   = ceil((floor(max[1]) - min[1]) / cs);
	printf("Cols: %d, Rows: %d\n", cols, rows);
	size   = rows * cols;
	numPts = numPts_;
	ci.resize(numPts);
	cj.resize(numPts);
	cells.resize(rows, std::vector<DTMCell>(cols));

	for (i = 0; i < rows; i++)
	{
		for (j = 0; j < cols; j++)
		{
			cells[i][j].id        = i * cols + j;
			cells[i][j].obj       = false;
			cells[i][j].empty     = true;
			cells[i][j].inpainted = false;
			cells[i][j].center[0] = min[0] + (j * cs) + r;
			cells[i][j].center[1] = min[1] + (i * cs) + r;
			cells[i][j].z         = std::numeric_limits<double>::max();
		}
	}
}

// Copy Constructor
DTMGrid::DTMGrid(const DTMGrid & dtm)
{
	rows  = dtm.rows;
	cols  = dtm.cols;
	size  = dtm.size;
	ci    = dtm.ci;
	cj    = dtm.cj;
	min   = dtm.min;
	cells = dtm.cells;
}

void DTMGrid::identifyOutliers(int numPts_, double minZ, double maxZ)
{
	int     i = 0, j = 0, count = 0;
	double  maxHeight = 0;
	DTMGrid oInvMinSurface;

	printf("Identifying outliers... ");
	fflush(stdout);

	DTMGrid invMinSurface = *this;

	for (i = 0; i < invMinSurface.rows; i++)
		for (j = 0; j < invMinSurface.cols; j++)
			invMinSurface.cells[i][j].z = (maxZ + minZ) - invMinSurface.cells[i][j].z;
	maxHeight      = 5 * MAX_SLOPE * 1 * CELL_SIZE;
	oInvMinSurface = invMinSurface.morphologicalOpening(1, numPts_);
	for (i = 0; i < invMinSurface.rows; i++)
		for (j = 0; j < invMinSurface.cols; j++)
			if (invMinSurface.cells[i][j].z - oInvMinSurface.cells[i][j].z > maxHeight)
			{
				cells[i][j].obj = true;
				count++;
			}
	printf("%d outliers\n", count);
}

void DTMGrid::identifyCells(int numPts_)
{
	int     i = 0, j = 0, window = 0, maxWindow = 0;
	double  maxHeight = 0;
	DTMGrid thisSurface;

	DTMGrid lastSurface = *this;

	maxWindow = std::ceil(MAX_WINDOW_SIZE / CELL_SIZE);
	for (window = 1; window <= maxWindow; window++)
	{
		maxHeight = MAX_SLOPE * (float) window * CELL_SIZE;

#if DEBUG
		printf("SMRF w=%d/%d h=%.2lf...\n", window, maxWindow, maxHeight);
#endif

		thisSurface = lastSurface.morphologicalOpening(window, numPts_);
		for (i = 0; i < thisSurface.rows; i++)
			for (j = 0; j < thisSurface.cols; j++)
				if (lastSurface.cells[i][j].z - thisSurface.cells[i][j].z > maxHeight)
					thisSurface.cells[i][j].obj = true;
		lastSurface = thisSurface;
	}
	for (i = 0; i < lastSurface.rows; i++)
		for (j = 0; j < lastSurface.cols; j++)
			cells[i][j].obj = lastSurface.cells[i][j].obj;
}

} // namespace dtm
