//
// Created by miguelyermo on 9/3/20.
//

#include "vector.h"

void Vector::set(double x, double y, double z)
{
	setX(x);
	setY(y);
	setZ(z);
}

void Vector::set(Vector v)
{
	setX(v.x());
	setY(v.y());
	setZ(v.z());
}

void Vector::sum(Vector v)
{
	setX(x() + v.x());
	setY(y() + v.y());
	setZ(z() + v.z());
}

Vector Vector::perpenVector()
/**
* Returns the perpendicular vector.
* @return
*/
{
	Vector p(-y_, x_, z_);
	return p;
}

double Vector::norm2D()
/**
* Returns the 2D norm of the vector
* @return
*/
{
	return sqrt(x_ * x_ + y_ * y_);
}

double Vector::norm3D()
/**
* Returns the 3D norm of the vector
* @return
*/
{
	return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

void Vector::normalize2D()
/**
* Normalizes the vector in two dimensions (only x and y are taken into account)
*/
{
	double norm = norm2D();

	setX(x_ / norm);
	setY(y_ / norm);
	setZ(z_ / norm);
}

double Vector::dotProduct2D(Vector & vec)
{
	return x_ * vec.x() + y_ * vec.y();
}

Vector operator-(const Point & p2, const Point & p1)
{
	Vector v(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
	return v;
}


