//
// Created by miguelyermo on 26/3/20.
//

#include <algorithm>
#include "laspec.h"
#include "point.h"
#include "vector.h"

std::vector<Lpoint*> filterGroundRoadPoints(std::vector<Lpoint> & points)
{
  std::vector<Lpoint*> grPoints;
  for (Lpoint & p : points)
  {
    if (p.getClass() == GROUND || p.getClass() == ROAD)
    {
      grPoints.emplace_back(&p);
    }
  }

  return grPoints;
}

bool isVegetation(unsigned int type)
{
	return type == GRASS || type == LOW_VEG || type == MED_VEG || type == HIGH_VEG || type == TREE;
}

std::vector<Lpoint*> filterVegPoints(std::vector<Lpoint> & points)
{
  std::vector<Lpoint*> vegPoints;
  for (Lpoint & p : points)
  {
    if (isVegetation(p.getClass()))
    {
      vegPoints.emplace_back(&p);
    }
  }

  return vegPoints;
}

double Point::distance2D(const Point & p) const
{
  double diffX = x_ - p.x();
  double diffY = y_ - p.y();

  return sqrt(diffX * diffX + diffY * diffY);
}
double Point::distance2D(const Point * p) const { return distance2D(*p); }

double Point::distance3D(const Point & p) const
{
  double diffX = x_ - p.x();
  double diffY = y_ - p.y();
  double diffZ = z_ - p.z();

  return sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
}
double Point::distance3D(const Point * p) const { return distance3D(*p); }

double Point::distanceToLine(const Point & l1, const Point & l2)
/**
 * Computes the distance of this point to the line defined by two points
 * @param l1 Origin of the line
 * @param l2 End of the line
 * @return
 */
{
  double y2_y1 = l2.y() - l1.y();
  double x2_x1 = l2.x() - l1.x();

  return fabs(y2_y1 * x_ - x2_x1 * y_ + l2.x() * l1.y() - l2.y() * l1.x()) / sqrt(y2_y1 * y2_y1 + x2_x1 * x2_x1);
}

Point Point::getDest(Vector & v, double distance)
{
v.normalize2D();
Point dest(x_ + v.x() * distance, y_ + v.y() * distance, z_);
return dest;
}