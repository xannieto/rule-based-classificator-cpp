//
// Created by miguelyermo on 1/3/20.
//

#ifndef CPP_POINT_H
#define CPP_POINT_H

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

class Vector;

class Point {
protected:
  unsigned int id_{}; // Id of the point (in order of reading)
  double x_{};        // X Coordinate
  double y_{};        // Y Coordinate
  double z_{};        // Z Coordinate

public:
  Point() = default;
  explicit Point(unsigned int id) : id_(id){};

  // 2D Geometric constructor ( Z = 0.0 )
  Point(double x, double y) : x_(x), y_(y), z_(0.0){};

  // 3D Geometric constructor
  Point(double x, double y, double z) : x_(x), y_(y), z_(z){};

  // 3D Geometric constructor with ID
  Point(unsigned int id, double x, double y, double z)
      : id_(id), x_(x), y_(y), z_(z){};

  double distance2D(const Point &p) const;
  double distance2D(const Point *p) const;
  double distance3D(const Point &p) const;
  double distance3D(const Point *p) const;
  double distanceToLine(const Point &l1, const Point &l2);
  Point getDest(Vector &v, double distance);

  // Overload << operator for Point.
  friend std::ostream &operator<<(std::ostream &out, const Point &p) {
    out << p.x_ << "\t" << p.y_ << "\t" << p.z_;
    return out;
  }

  // Overload << operator for Point*
  friend std::ostream &operator<<(std::ostream &out, const Point *p) {
    out << *p;
    return out;
  }

  Point &operator+=(const Point *p) {
    x_ += p->x();
    y_ += p->y();
    z_ += p->z();

    return *this;
  }

  Point &operator/=(const double val) {
    x_ /= val;
    y_ /= val;
    z_ /= val;

    return *this;
  }

  // Getters and setters
  inline unsigned int id() const { return id_; }

  inline void id(unsigned int id) { id_ = id; }

  inline double x() const { return x_; }

  inline void setX(double x) { x_ = x; }

  inline double y() const { return y_; }

  inline void setY(double y) { y_ = y; }

  inline double z() const { return z_; }

  inline void setZ(double z) { z_ = z; }
};

class Lpoint : public Point {
protected:
  double I_{};                      // Intensity
  unsigned short rn_{};             // Return Number
  unsigned short nor_{};            // Number of Returns (given pulse)
  unsigned short dir_{};            // Scan Direction Flag
  unsigned short edge_{};           // Edge of Flight Line
  unsigned short classification_{}; // Classification of the point
  char sar_{};                      // Scan Angle Rank
  unsigned char ud_{};              // User Data
  unsigned short psId_{};           // Point Source ID
  unsigned int gId_{};              // Group ID

  unsigned int r_{}; // Red channel (0-65535)
  unsigned int g_{}; // Green channel (0-65535)
  unsigned int b_{}; // Blue channel (0-65535)

  double dtmZ_{};
  double slope_{};
  bool hasSlope_{};

  double realZ_{}; // Real Z value

  double slopeDTM_{}; // Slope DTM value

  int fuelType_{}; // Prometheus fuel type

  // Flags to not duplicate points when drawing
  bool slopePainted = false;

  // Density of points in the volume around the point
  double pointDensity_{};

  // Quartil number. 1->1st quartil (< 25%), 2-> 2nd quartil, (>25 and <50)...
  unsigned short qn_{};

  // Is the point clusterized? (Belongs to a tower yet?)
  bool isClusterized_{};
  bool isNoise_{};

  // Is the point inside some polygon??
  bool isInPoly_{};

  // Powerlines
  bool wire_{}; // Is the point detected as a wire?
  bool cat_{};  // Is the point already included in a catenary?

  // Segmentation
  size_t regionId_{};

  // Classification
  bool boundUsed_ = false; // Flag to indicate if it is a boundary
  bool border_ = false;    // Flag to indicate if it is planar

public:
  // Default constructor
  Lpoint() : Point(){};

  Lpoint(size_t id, double x, double y, double z, double I)
      : Point(id, x, y, z)
  {
    I_ = I;
  }

            Lpoint(unsigned int id, double x, double y, double z)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = 0;
    classification_ = 0;
    gId_ = 0;
  }

  explicit Lpoint(unsigned int id) : Point(id) { id_ = id; }

  Lpoint(double x, double y) : Point(x, y) {
    x_ = x;
    y_ = y;
  }

  Lpoint(double x, double y, double z) : Point(x, y, z) {
    x_ = x;
    y_ = y;
    z_ = z;
  }

  Lpoint(double x, double y, unsigned int fuelType) : Point(x, y, fuelType) {
    x_ = x;
    y_ = y;
    z_ = 0.0;
    fuelType_ = fuelType;
  }

  // Reading points from classification
  Lpoint(unsigned int id, double x, double y, double z, double I,
         unsigned int gId, unsigned int classification)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = I;
    gId_ = gId;
    classification_ = classification;
  }

  // Reading points with Real Z
  Lpoint(unsigned int id, double x, double y, double z, double realZ, double I,
         unsigned int gId, unsigned int classification)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = I;
    realZ_ = realZ;
    gId_ = gId;
    classification_ = classification;
  }

  Lpoint(unsigned int id, double x, double y, double z, double I,
         unsigned short nor, unsigned short rn, unsigned char dir,
         unsigned char edge, unsigned short classification, unsigned int gId)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = I;
    nor_ = nor;
    rn_ = rn;
    dir_ = dir;
    edge_ = edge;
    classification_ = classification;
    gId_ = gId;
  }

  Lpoint(unsigned int id, double x, double y, double z, double realZ, double I,
         unsigned short nor, unsigned short rn, unsigned char dir,
         unsigned char edge, unsigned short classification, unsigned int gId)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    realZ_ = realZ;
    I_ = I;
    nor_ = nor;
    rn_ = rn;
    dir_ = dir;
    edge_ = edge;
    classification_ = classification;
    gId_ = gId;
  }

  Lpoint(unsigned int id, double x, double y, double z, double I,
         unsigned short nor, unsigned short rn, unsigned char dir,
         unsigned char edge, unsigned short classification)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = I;
    nor_ = nor;
    rn_ = rn;
    dir_ = dir;
    edge_ = edge;
    classification_ = classification;
  }

  Lpoint(unsigned int id, double x, double y, double z, double I,
         unsigned short nor, unsigned short rn, unsigned char dir,
         unsigned char edge, unsigned short classification, unsigned int r,
         unsigned int g, unsigned int b)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = I;
    nor_ = nor;
    rn_ = rn;
    dir_ = dir;
    edge_ = edge;
    classification_ = classification;
    r_ = r;
    g_ = g;
    b_ = b;
  }

  // Reading Point Data Record Format 2
  Lpoint(unsigned int id, double x, double y, double z, double I,
         unsigned short nor, unsigned short rn, unsigned char dir,
         unsigned char edge, unsigned short classification, char sar,
         unsigned char ud, unsigned short psId, unsigned int r, unsigned int g,
         unsigned int b)
      : Point(id, x, y, z) {
    id_ = id;
    x_ = x;
    y_ = y;
    z_ = z;
    I_ = I;
    nor_ = nor;
    rn_ = rn;
    dir_ = dir;
    edge_ = edge;
    classification_ = classification;
    sar_ = sar;
    ud_ = ud;
    psId_ = psId;
    r_ = r;
    g_ = g;
    b_ = b;
  }

  // Setters and getters
  inline double I() const { return I_; }

  inline unsigned int Id() const { return id_; }

  inline unsigned int gId() const { return gId_; }

  inline void setGId(unsigned int gId) { gId_ = gId; }

  inline unsigned short getClass() const { return classification_; }

  inline void setClass(unsigned short classification) {
    classification_ = classification;
  }

  inline double pointDensity() const { return pointDensity_; }

  inline void setPointDensity(double pointDensity) {
    pointDensity_ = pointDensity;
  }

  inline unsigned short qn() const { return qn_; }

  inline void setQn(unsigned short qn) { qn_ = qn; }

  inline unsigned short rn() const { return rn_; }

  inline unsigned short nor() const { return nor_; }

  inline unsigned short dir() const { return dir_; }

  inline unsigned short edge() const { return edge_; }

  inline bool isClusterized() const { return isClusterized_; }

  inline void isClusterized(bool isClusterized) {
    isClusterized_ = isClusterized;
  }

  inline bool isNoise() const { return isNoise_; }

  inline void isNoise(bool isNoise) { isNoise_ = isNoise; }

  inline bool isInPoly() const { return isInPoly_; }

  inline void isInPoly(bool isInPoly) { isInPoly_ = isInPoly; }

  inline double getDtmZ() const { return dtmZ_; }

  inline void setDtmZ(double dtmZ) { dtmZ_ = dtmZ; }

  inline double getSlope() const { return slope_; }

  inline void setSlope(double slope) { slope_ = slope; }

  inline void setSlopeDTM(double slopeDTM) { slopeDTM_ = slopeDTM; }

  inline double getSlopeDTM() const { return slopeDTM_; }

  inline void setDTMSlope(double slopeDTM) { slopeDTM_ = slopeDTM; }

  inline bool hasSlope() const { return hasSlope_; }

  inline void hasSlope(bool hasSlope) { hasSlope_ = hasSlope; }

  inline bool isSlopePainted() const { return slopePainted; }

  inline void setSlopePainted(bool slopePainted) {
    Lpoint::slopePainted = slopePainted;
  }

  inline double getRealZ() const { return realZ_; }

  inline void setRealZ(double realZ) { realZ_ = realZ; }

  inline const int getFuelType() const { return fuelType_; }

  inline void setFuelType(int fuelType) { fuelType_ = fuelType; }

  inline bool isWire() const { return wire_; }

  inline void setWire(bool wire) { wire_ = wire; }

  inline bool isCat() const { return cat_; }

  inline void setCat(bool cat) { cat_ = cat; }

  inline bool isBoundUsed() const { return boundUsed_; }

  inline void setBoundUsed(bool boundUser) { boundUsed_ = boundUser; }

  inline bool isBorder() const { return border_; }

  inline void setBorder(bool isBorder) { border_ = isBorder; }

  inline unsigned int getRegionId() const { return regionId_; };

  inline void setRegionId(unsigned int regionId) { regionId_ = regionId; };

  // Euclidean distance between two points
  inline double distance(const Lpoint &p) const {
    double diffX = x_ - p.x();
    double diffY = y_ - p.y();

    return sqrt(diffX * diffX + diffY * diffY);
  }
};

bool isVegetation(unsigned int type);
std::vector<Lpoint *> filterGroundRoadPoints(std::vector<Lpoint> &points);
std::vector<Lpoint *> filterVegPoints(std::vector<Lpoint> &points);

#endif // CPP_POINT_H
