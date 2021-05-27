//
// Created by miguel.yermo on 5/03/20.
//

/*
* FILENAME :  vector.h  
* PROJECT  :  rule-based-classifier-cpp
* DESCRIPTION :
*  
*
*
*
*
* AUTHOR :    Miguel Yermo        START DATE : 10:59 5/03/20
*
*/

#ifndef CPP_VECTOR_H
#define CPP_VECTOR_H

#include "point.h"

#include <cmath>
#include <iostream>

class Vector : public Point
{
	public:
	Vector() : Point(){};
	Vector(double x, double y, double z = 0.0)
	{
		x_ = x;
		y_ = y;
		z_ = z;
	}

	Vector(Point & origin, Point & dest)
	{
		x_ = dest.x() - origin.x();
		y_ = dest.y() - origin.y();
		z_ = dest.z() - origin.z();
	}

	void inline setX(double x) { x_ = x; };

	void inline setY(double y) { y_ = y; };

	void inline setZ(double z) { z_ = z; };

	double inline const getX() {return x_;};

	double inline const getY() {return y_;};

	double inline const getZ() {return z_;};

	void set(double x, double y, double z);

	void set(Vector v);

	void sum(Vector v);

	Vector perpenVector();

	double norm2D();

	double norm3D();

	void normalize2D();

	double dotProduct2D(Vector & vec);


	// Multiplication of vector and a scalar
	template<typename T>
	void operator*(const T x)
	{
		x_ *= x;
		y_ *= x;
		z_ *= x;
	}

	friend std::ostream & operator<<(std::ostream & out, const Vector & v)
	{
		out << "Vector: (" << v.x() << ", " << v.y() << ", " << v.z() << ")";
		return out;
	}
};

Vector operator-(const Point & p2, const Point & p1);
#endif //CPP_VECTOR_H
