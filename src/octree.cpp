//
// Created by miguel.yermo on 5/03/20.
//

#include "octree.h"
#include <cstdio>
#include <iostream>
#include <limits>

inline Vector mbbRadii(Vector & min, Vector & max, float & maxRadius)
{
	Vector radii;

	radii.setX((max.x() - min.x()) / 2.0);
	radii.setY((max.y() - min.y()) / 2.0);
	radii.setZ((max.z() - min.z()) / 2.0);

	if (radii.x() >= radii.y() && radii.x() >= radii.z())
	{
		maxRadius = radii.x();
	}
	else if (radii.y() >= radii.x() && radii.y() >= radii.z())
	{
		maxRadius = radii.y();
	}
	else
	{
		maxRadius = radii.z();
	}

	return radii;
}


inline Vector mbbCenter(Vector & min, Vector & radius)
{
	Vector center;

	center.setX(min.x() + radius.x());
	center.setY(min.y() + radius.y());
	center.setZ(min.z() + radius.z());

	return center;
}

//Vector mbb(Lpoint* points, unsigned int numPoints, float* maxRadius)
Vector mbb(const std::vector<Lpoint> & points, float & maxRadius)
/**
 * Computes the minimum bounding box of a set of points
 * @param points Array of points
 * @param numPoints Number of points
 * @param[out] maxRadius Maximum radius of the bounding box
 * @return (Vector) center of the bounding box
 */
{
	Vector center, min, max, radii;

	min.setX(std::numeric_limits<double>::max());
	min.setY(std::numeric_limits<double>::max());
	min.setZ(std::numeric_limits<double>::max());

	max.setX(-std::numeric_limits<double>::max());
	max.setY(-std::numeric_limits<double>::max());
	max.setZ(-std::numeric_limits<double>::max());

	for (const Lpoint & p : points)
	{
		if (p.x() < min.x()) min.setX(p.x());
		if (p.x() > max.x()) max.setX(p.x());
		if (p.y() < min.y()) min.setY(p.y());
		if (p.y() > max.y()) max.setY(p.y());
		if (p.z() < min.z()) min.setZ(p.z());
		if (p.z() > max.z()) max.setZ(p.z());
	}

	radii  = mbbRadii(min, max, maxRadius);
	center = mbbCenter(min, radii);
	printf("Octree Radius: %.2f\n", maxRadius);

	return center;
}
