//
// Created by miguel.yermo on 5/03/20.
//

#pragma clang diagnostic push
#pragma ide diagnostic   ignored "OCSimplifyInspection"
#ifndef CPP_OCTREE_H
#define CPP_OCTREE_H

#include "point.h"
#include "vector.h"

#include <array>
#include <iostream>
#include <vector>


class Box
{
	private:
	Vector min_{};
	Vector max_{};

	public:
	Box(const Point & p, const Vector & radii)
	{
		min_.setX(p.x() - radii.x());
		min_.setY(p.y() - radii.y());
		min_.setZ(p.z() - radii.z());
		max_.setX(p.x() + radii.x());
		max_.setY(p.y() + radii.y());
		max_.setZ(p.z() + radii.z());
	}

	inline bool isInside(const Point & p) const
	{
		if (p.x() > min_.x() && p.y() > min_.y() && p.z() > min_.z())
		{
			if (p.x() < max_.x() && p.y() < max_.y() && p.z() < max_.z())
			{
				return true;
			}
		}
		return false;
	}

	[[nodiscard]] const Vector & min() const { return min_; }

	[[nodiscard]] const Vector & max() const { return max_; }

	[[nodiscard]] double minX() const { return min_.x(); }

	[[nodiscard]] double minY() const { return min_.y(); }

	[[nodiscard]] double minZ() const { return min_.z(); }

	[[nodiscard]] double maxX() const { return max_.x(); }

	[[nodiscard]] double maxY() const { return max_.y(); }

	[[nodiscard]] double maxZ() const { return max_.z(); }
};

// Keep dividing the octree while octants have more points than these.
static constexpr unsigned int MAX_POINTS = 100;

class Octree
{
	private:
	std::vector<Octree>   octants_;
	Vector                center_;
	Point                 min_{};
	Point                 max_{};
	std::vector<Lpoint *> points_;
	unsigned int          numPoints_{};
	float                 radius_{};

	public:
	Octree(){};

	Octree(Vector & center, float radius) :
	  octants_(), center_(center), min_{}, max_{}, points_(), numPoints_(0), radius_(radius)
	{
		octants_.reserve(8);
	};


	Octree(Vector & center, float radius, std::vector<Lpoint> & points) :
	  octants_(), center_(center), min_{}, max_{}, points_(), numPoints_(0), radius_(radius)
	{
		octants_.reserve(8);
		buildOctree(points);
	};

	Octree(Vector & center, float radius, std::vector<Lpoint *> & points) :
	  octants_(), center_(center), min_{}, max_{}, points_(), numPoints_(0), radius_(radius)
	{
		octants_.reserve(8);
		buildOctree(points);
	}

	void computeOctreeLimits()
	/**
   * Compute the minimum and maximum coordinates of the octree bounding box.
   */
	{
		min_.setX(center_.x() - radius_);
		min_.setY(center_.y() - radius_);
		max_.setX(center_.x() + radius_);
		max_.setY(center_.y() + radius_);
	}

	inline bool isInside2D(Point & p) const
	/**
   * Checks if a point is inside the octree limits.
   * @param p
   * @return
   */
	{
		if (p.x() >= min_.x() && p.y() >= min_.y())
		{
			if (p.x() <= max_.x() && p.y() <= max_.y())
			{
				return true;
			}
		}

		return false;
	}

	[[nodiscard]] const Point & getMin() const { return min_; }

	[[nodiscard]] const Point & getMax() const { return max_; }

	void insertPoints(std::vector<Lpoint> & points)
	{
		for (Lpoint & p : points)
		{
			insertPoint(&p);
		}
	}

	void insertPoints(std::vector<Lpoint *> & points)
	{
		for (Lpoint * p : points)
		{
			insertPoint(p);
		}
	}

	void insertPoint(Lpoint * p)
	{
		unsigned int idx = 0;

		if (isLeaf())
		{
			if (numPoints_ > MAX_POINTS)
			{
				createOctants(); // Creation of children octree
				fillOctants();   // Move points from current Octree to its corresponding children.
				idx = octantIdx(p);
				octants_[idx].insertPoint(p);
			}
			else
			{
				points_.emplace_back(p);
				++numPoints_;
			}
		}
		else
		{
			idx = octantIdx(p);
			octants_[idx].insertPoint(p);
		}

	}

	static Octree createOctant(Vector & center, float radius) { return Octree(center, radius); }

	void createOctants()
	{
		Vector newCenter;
		for (int i = 0; i < 8; i++)
		{
			newCenter = center_;
			newCenter.setX(newCenter.x() + radius_ * (i & 4 ? 0.5f : -0.5f));
			newCenter.setY(newCenter.y() + radius_ * (i & 2 ? 0.5f : -0.5f));
			newCenter.setZ(newCenter.z() + radius_ * (i & 1 ? 0.5f : -0.5f));
			octants_.emplace_back(Octree(newCenter, 0.5f * radius_));
		}
	}

	void fillOctants()
	{
		int idx = 0; // Idx of the octant where a point should be inserted.

		for (Lpoint *& p : points_)
		{
			idx = octantIdx(p);
			octants_[idx].insertPoint(p);
		}

		numPoints_ = 0;
		points_.clear();
	}

	int octantIdx(Lpoint * p)
	{
		int child = 0;

		if (p->x() >= center_.x()) child |= 4;
		if (p->y() >= center_.y()) child |= 2;
		if (p->z() >= center_.z()) child |= 1;

		return child;
	}

	inline bool isLeaf() const { return octants_.empty(); }

	inline bool isEmpty() const { return numPoints_ == 0; }


	void buildOctree(std::vector<Lpoint> & points)
	/**
   * Build the Octree
   */
	{
		computeOctreeLimits();
		insertPoints(points);
	};

	void buildOctree(std::vector<Lpoint *> & points)
	/**
   * Build the Octree
   */
	{
		computeOctreeLimits();
		insertPoints(points);
	};

	inline Vector & getCenter() { return center_; }

	inline float getRadius() { return radius_; }

	static inline void makeBox(const Point & p, double radius, Vector & min, Vector & max)
	{
		min.setX(p.x() - radius);
		min.setY(p.y() - radius);
		min.setZ(p.z() - radius);
		max.setX(p.x() + radius);
		max.setY(p.y() + radius);
		max.setZ(p.z() + radius);
	}

	static inline void makeBoxUntilGround(const Lpoint & p, float radius, Vector & min, Vector & max)
	{
		min.setX(p.x() - radius);
		min.setY(p.y() - radius);
		min.setZ(p.getDtmZ());
		max.setX(p.x() + radius);
		max.setY(p.y() + radius);
		max.setZ(p.z());
	}

	static inline void makeBoxCylinder(const Point & p, double radius, Vector & min, Vector & max)
	{
		min.setX(p.x() - radius);
		min.setY(p.y() - radius);
		min.setZ(0);
		max.setX(p.x() + radius);
		max.setY(p.y() + radius);
		max.setZ(1000);
	}

	static inline void makeBox(const Point & p, Vector radius, Vector & min, Vector & max)
	/**
   * This functions defines the box whose inner points will be considered as neighs of Lpoint p.
   * @param p
   * @param radius Vector of radius. One per spatial direction.
   * @param min
   * @param max
   */
	{
		min.setX(p.x() - radius.x());
		min.setY(p.y() - radius.y());
		min.setZ(p.z() - radius.z());
		max.setX(p.x() + radius.x());
		max.setY(p.y() + radius.y());
		max.setZ(p.z() + radius.z());
	}

	inline bool boxOverlap2D(const Vector & boxMin, const Vector & boxMax) const
	{
		if (center_.x() + radius_ < boxMin.x() || center_.y() + radius_ < boxMin.y()) return false;
		if (center_.x() - radius_ > boxMax.x() || center_.y() - radius_ > boxMax.y()) return false;

		return true;
	}


	inline bool boxOverlap3D(const Vector & boxMin, const Vector & boxMax) const
	{
		if (center_.x() + radius_ < boxMin.x() || center_.y() + radius_ < boxMin.y() || center_.z() + radius_ < boxMin.z())
			return false;
		if (center_.x() - radius_ > boxMax.x() || center_.y() - radius_ > boxMax.y() || center_.z() - radius_ > boxMax.z())
			return false;

		return true;
	}

	static inline bool insideBox2D(const Lpoint & point, const Vector & min, const Vector & max)
	{
		if (point.x() > min.x() && point.y() > min.y())
		{
			if (point.x() < max.x() && point.y() < max.y())
			{
				return true;
			}
		}

		return false;
	}

	static inline bool insideCircle(const Lpoint & p, const Point & c, float r)
	{
		return (p.x() - c.x()) * (p.x() - c.x()) + (p.y() - c.y()) * (p.y() - c.y()) < r * r;
	}

	static inline bool insideBox3D(const Lpoint & point, const Vector & min, const Vector & max)
	{
		if (point.x() > min.x() && point.y() > min.y() && point.z() > min.z())
		{
			if (point.x() < max.x() && point.y() < max.y() && point.z() < max.z())
			{
				return true;
			}
		}
		return false;
	}

	static inline bool insideRing(const Lpoint & p, const Box & boxMin, const Box & boxMax)
	{
		if (!boxMin.isInside(p) && boxMax.isInside(p)) return true;

		return false;
	}

	std::vector<Lpoint *> neighbors2D(const Point & p, const Vector & boxMin, const Vector & boxMax,
	                                  std::vector<Lpoint *> & ptsInside) const
	{
		if (isLeaf())
		{
			if (!isEmpty())
			{
				for (Lpoint * point_ptr : points_)
				{
					const Lpoint & point = *point_ptr;
					if (insideBox2D(point, boxMin, boxMax) && p.id() != point.Id())
					{
						ptsInside.emplace_back(point_ptr);
					}
				}
			}
		}
		else
		{
			for (const Octree & octant : octants_)
			{
				if (!octant.boxOverlap2D(boxMin, boxMax))
				{
					continue;
				}
				else
				{
					ptsInside = octant.neighbors2D(p, boxMin, boxMax, ptsInside);
				}
			}
		}

		return std::move(ptsInside);
	}

	std::vector<Lpoint *> circleNeighbors(const Point & p, const Vector & boxMin, const Vector & boxMax,
		std::vector<Lpoint *> & ptsInside, float circleRadius) const
	{
		if(isLeaf())
		{
			if(!isEmpty())
			{
				for(Lpoint * point_ptr : points_)
				{
					const Lpoint & point = *point_ptr;
					if(insideCircle(point, p, circleRadius) && p.id() != point.id())
					{
						ptsInside.emplace_back(point_ptr);

					}
				}
			}
		}
		else
		{
			for (const Octree & octant : octants_)
			{
				if(!octant.boxOverlap3D(boxMin, boxMax))
				{
					continue;
				}
				else
				{
					ptsInside = octant.circleNeighbors(p, boxMin, boxMax, ptsInside, circleRadius);
				}
			}
		}

		return std::move(ptsInside);
	}

	std::vector<Lpoint *> neighbors3D(const Point & p, const Vector & boxMin, const Vector & boxMax,
	                                  std::vector<Lpoint *> & ptsInside) const
	{
		if (isLeaf())
		{
			if (!isEmpty())
			{
				for (Lpoint * point_ptr : points_)
				{
					const Lpoint & point = *point_ptr;
					// The neighbors do not inc the point itself.
					if (insideBox3D(point, boxMin, boxMax) && p.id() != point.Id())
					{
						ptsInside.emplace_back(point_ptr);
					}
				}
			}
		}
		else
		{
			for (const Octree & octant : octants_)
			{
				if (!octant.boxOverlap3D(boxMin, boxMax))
				{
					continue;
				}
				else
				{
					ptsInside = octant.neighbors3D(p, boxMin, boxMax, ptsInside);
				}
			}
		}

		return std::move(ptsInside);
	}

	std::vector<Lpoint *> neighbors3DFlagged(const Point & p, const Vector & boxMin, const Vector & boxMax,
																		std::vector<Lpoint *> & ptsInside, const std::vector<bool> & flags) const
	{
		if (isLeaf())
		{
			if (!isEmpty())
			{
				for (Lpoint * point_ptr : points_)
				{
					const Lpoint & point = *point_ptr;
					// The neighbors do not inc the point itself.
					if (!flags[point.Id()] && insideBox3D(point, boxMin, boxMax) && p.id() != point.Id())
					{
						ptsInside.emplace_back(point_ptr);
					}
				}
			}
		}
		else
		{
			for (const Octree & octant : octants_)
			{
				if (!octant.boxOverlap3D(boxMin, boxMax))
				{
					continue;
				}
				else
				{
					ptsInside = octant.neighbors3DFlagged(p, boxMin, boxMax, ptsInside, flags);
				}
			}
		}

		return std::move(ptsInside);
	}

	inline std::vector<Lpoint *> searchNeighbors2D(const Point & p, double radius)
	{
		Vector                boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBox(p, radius, boxMin, boxMax);
		ptsInside = neighbors2D(p, boxMin, boxMax, ptsInside);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchNeighbors2D(Point & p, double radius)
	{
		Vector                boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBox(p, radius, boxMin, boxMax);
		ptsInside = neighbors2D(p, boxMin, boxMax, ptsInside);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchNeighbors2DUntilGround(const Lpoint & p, double radius)
	{
		Vector                boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBoxUntilGround(p, radius, boxMin, boxMax);
		ptsInside = neighbors2D(p, boxMin, boxMax, ptsInside);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchCylinderNeighbors(const Lpoint & p, double radius)
	{
		Vector boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBoxUntilGround(p, radius, boxMin, boxMax);
		ptsInside = circleNeighbors(p, boxMin, boxMax, ptsInside, radius);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchCircleNeighbors(const Lpoint & p, double radius) const
	{
		Vector boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBox(p, radius, boxMin, boxMax);
		ptsInside = circleNeighbors(p, boxMin, boxMax, ptsInside, radius);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchGroundTruthNeighbors(const Point & p, double radius)
	{
		Vector boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBoxCylinder(p, radius, boxMin, boxMax);
		ptsInside = circleNeighbors(p, boxMin, boxMax, ptsInside, radius);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchNeighbors3D(const Point & p, Vector radius)
	/**
   * Searching neighbors in 3D using a different radius for each direction
   * @param p Point around the neighbors will be search
   * @param radius Vector of radiuses: one per spatial direction
   * @return
   */
	{
		Vector                boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBox(p, radius, boxMin, boxMax);
		ptsInside = neighbors3D(p, boxMin, boxMax, ptsInside);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchNeighbors3D(const Point & p, double radius)
	/**
   * Searching neighbors in 3D using a different radius for each direction
   * @param p Point around the neighbors will be search
   * @param radius Vector of radiuses: one per spatial direction
   * @return
   */
	{
		Vector                boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBox(p, radius, boxMin, boxMax);
		ptsInside = neighbors3D(p, boxMin, boxMax, ptsInside);

		return ptsInside;
	}

	inline std::vector<Lpoint *> searchNeighbors3D(const Point & p, double radius, const std::vector<bool> & flags)
  /**
   * Searching neighbors in 3D using a different radius for each direction
   * @param p Point around the neighbors will be search
   * @param radius Vector of radiuses: one per spatial direction
   * @param flags Vector of flags: return only points which flags[pointId] == false
   * @return
   */
	{
		Vector                boxMin, boxMax;
		std::vector<Lpoint *> ptsInside;

		makeBox(p, radius, boxMin, boxMax);
		ptsInside = neighbors3DFlagged(p, boxMin, boxMax, ptsInside, flags);

		return ptsInside;
	}

	std::vector<Lpoint *> neighborsRing(const Point & p, Box & innerBox, Box & outerBox,
	                                    std::vector<Lpoint *> & ptsInside) const
	/**
   * Function that searchs neighbors inside a ring.
   * @param p
   * @param innerBox Vector of Radii of the inside ring
   * @param outerBox Vector of Radii of the outside ring
   * @param ptsInside
   * @return
   */
	{
		if (isLeaf())
		{
			if (!isEmpty())
			{
				for (Lpoint * point_ptr : points_)
				{
					const Lpoint & point = *point_ptr;
					if (p.id() != point.Id() && insideRing(point, innerBox, outerBox))
					{
						ptsInside.emplace_back(point_ptr);
					}
				}
			}
		}
		else
		{
			for (const Octree & octant : octants_)
			{
				if (!octant.boxOverlap3D(outerBox.min(), outerBox.max()))
				{
					continue;
				}
				else
				{
					ptsInside = octant.neighborsRing(p, innerBox, outerBox, ptsInside);
				}
			}
		}

		return std::move(ptsInside);
	}

	inline std::vector<Lpoint *> searchNeighborsRing(const Lpoint & p, const Vector & innerRingRadii,
	                                                 const Vector & outerRingRadii)
	/**
* A point is considered to be inside a Ring around a point if its outside the innerRing and inside the outerRing
* @param p
* @param innerRingRadius
* @param outerRingRadius
* @return
*/
	{
		std::vector<Lpoint *> ptsInside;

		Box innerBox(p, innerRingRadii);
		Box outerBox(p, outerRingRadii);

		ptsInside = neighborsRing(p, innerBox, outerBox, ptsInside);
		return ptsInside;
	}
};


// Functions
Vector mbb(const std::vector<Lpoint> & points, float & maxRadius);

#endif //CPP_OCTREE_H