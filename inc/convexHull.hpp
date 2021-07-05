//
// Created by miguelyermo on 13/5/20.
//

/*
* FILENAME :  convexHull.h  
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION :
*  
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 09:36 13/5/20
*
*/

#ifndef RULE_BASED_CLASSIFIER_CPP_CONVEXHULL_HPP
#define RULE_BASED_CLASSIFIER_CPP_CONVEXHULL_HPP

#include <algorithm>
#include <iostream>
#include <omp.h>
#include <point.h>
#include <vector>

class ConvexHull
{
private:
  	std::vector<Lpoint*> points_{};
  	std::vector<Lpoint*> vertices_{};
	
  	double area_{};

public:
	explicit ConvexHull(std::vector<Lpoint*>& points) : points_(points)
  	{
	  	vertices_ = buildHull(points_);
		area_ = polyArea(vertices_);
 	};

    // NOTA: Conservar hasta que se haga un rework donde se lean los puntos directamente
    // a punteros. Si en algún momento el array de puntos exterior se destruye, estos
	// punteros dejan de tener validez.
	explicit ConvexHull(std::vector<Lpoint>& points)
	{
		for (auto & p : points)
    	{
			points_.push_back(&p);
		}
		
		vertices_ = buildHull(points_);
		area_ = polyArea(vertices_);
	}

	static std::vector<Lpoint*> buildHull(std::vector<Lpoint*> & points)
  	{
    	// Number of points
    	size_t n = points.size();
    	size_t k = 0;

    	// Preconditions
    	if (n <= 3) {std::cout << "Convex Hull cannot be computed with less than 3 points"; return points;}

    	// Reserve memory for the convex hull
    	std::vector<Lpoint*> H(2 * n);

    	// Sort Points (When overloading < operator in Lpoint, sort does strange things) // Solution (Ruben): I'm sorting
    	// pointer address, no actual points.
    	std::sort(points.begin(), points.end(), sortPoint);
			
    	// Build lower hull
    	for (std::size_t i = 0; i < n; i++)
    	{
      		while (k >= 2 && crossProduct(H[k - 2], H[k - 1], points[i]) <= 0) k--;
      		H[k++] = points[i];
    	}

		// Build upper hull
		for (std::size_t i = n - 1, t = k + 1; i > 0; --i)
		{
			while (k >= t && crossProduct(H[k - 2], H[k - 1], points[i - 1]) <= 0) k--;
			H[k++] = points[i-1];
		}

		H.resize(k - 1);
		return H;
	}

	[[nodiscard]] inline double area() const
	{
		return area_;
	}

	static double crossProduct(const Lpoint * O, const Lpoint * A, const Lpoint * B)
	/**
	* Vectorial product of two vectors formed by OA and OB
	* @param O
	* @param A
	* @param B
	* @return
	*/
	{
    	return (A->x() - O->x()) * (B->y() - O->y()) - (A->y() - O->y()) * (B->x() - O->x());
  	}

	static double polyArea(const std::vector<Lpoint*> & points)
	/**
	* Compute the area of a polynomial given the list of ordered vertices. The last vertex of the list must be the same as
	* the first one
	* @param points
	* @return
	*/
	{
		double area = 0.0;

		for (size_t i = 0, j = 1; i < points.size() - 1; i++, j++)
			area += 0.5 * (points[i]->x()*points[j]->y() - points[j]->x()*points[i]->y());

		return area;
	}

	static bool sortPoint(const Lpoint* p1, const Lpoint* p2)
	/**
	* Sorting of points lexicographically (by x coordinate and, if tied, by y coordinate)
	* @param p1
	* @param p2
	* @return
	*/
	{
		return p1->x() < p2->x();
	}

	bool inHull(const Lpoint & p)
	{
		for (size_t i = 0; i < vertices_.size() - 1; i++)
		{
			if (crossProduct(vertices_[i], vertices_[i+1], &p) < 0)
			{
				return false;
			}
		}
		return true;
	}
};

#endif //RULE_BASED_CLASSIFIER_CPP_CONVEXHULL_HPP
