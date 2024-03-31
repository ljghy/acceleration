#ifndef ACC_BOUNDING_BOX_HPP_
#define ACC_BOUNDING_BOX_HPP_

#include <limits>

#include <Eigen/Core>

#include <acc/Ray.hpp>

namespace acc {

struct BoundingBox {
  Eigen::Vector3d lb;
  Eigen::Vector3d ub;

  double minSqrDist(Eigen::Vector3d q) const;
  double minDist(Eigen::Vector3d p) const;

  BoundingBox trimLeft(int dim, double c) const;
  BoundingBox trimRight(int dim, double c) const;

  static BoundingBox merge(const BoundingBox &a, const BoundingBox &b);
  const BoundingBox operator+(const BoundingBox &other) const;
  BoundingBox &operator+=(const BoundingBox &other);

  void update(const Eigen::Vector3d &p);
  double area() const;
  Eigen::Vector3d center() const;

  bool contains(const Eigen::Vector3d &p) const {
    return (p.array() >= lb.array()).all() && (p.array() <= ub.array()).all();
  }

  double rayHit(const Ray &ray) const;
};

inline double BoundingBox::minSqrDist(Eigen::Vector3d q) const {
  Eigen::Vector3d center = (ub + lb) * 0.5;
  Eigen::Vector3d halfExtent = ub - center;
  q -= center;
  return (q.cwiseAbs() - halfExtent).cwiseMax(0.0).squaredNorm();
}

inline BoundingBox BoundingBox::trimLeft(int dim, double c) const {
  auto m = ub;
  m(dim) = c;
  return {lb, m};
}

inline BoundingBox BoundingBox::trimRight(int dim, double c) const {
  auto m = lb;
  m(dim) = c;
  return {m, ub};
}

inline void BoundingBox::update(const Eigen::Vector3d &p) {
  lb = lb.cwiseMin(p);
  ub = ub.cwiseMax(p);
}

inline BoundingBox BoundingBox::merge(const BoundingBox &a,
                                      const BoundingBox &b) {
  return {a.lb.cwiseMin(b.lb), a.ub.cwiseMax(b.ub)};
}

inline const BoundingBox
BoundingBox::operator+(const BoundingBox &other) const {
  return {lb.cwiseMin(other.lb), ub.cwiseMax(other.ub)};
}

inline BoundingBox &BoundingBox::operator+=(const BoundingBox &other) {
  lb = lb.cwiseMin(other.lb);
  ub = ub.cwiseMax(other.ub);
  return *this;
}

inline double BoundingBox::area() const {
  Eigen::Vector3d d = ub - lb;
  return d(0) * d(1) + d(1) * d(2) + d(2) * d(0);
}

inline double BoundingBox::minDist(Eigen::Vector3d p) const {
  Eigen::Vector3d center = (lb + ub) * 0.5;
  Eigen::Vector3d halfExtent = ub - center;
  p -= center;
  return (p.cwiseAbs() - halfExtent).cwiseMax(0.0).norm();
}

inline Eigen::Vector3d BoundingBox::center() const { return 0.5 * (lb + ub); }

inline double BoundingBox::rayHit(const Ray &ray) const {

  Eigen::Vector3d invDir = 1.0 / ray.dir.array(),
                  t1 = invDir.cwiseProduct(lb - ray.orig),
                  t2 = invDir.cwiseProduct(ub - ray.orig),
                  minT = t1.cwiseMin(t2), maxT = t1.cwiseMax(t2);

  double maxMinT = std::max(std::max(minT(0), minT(1)), minT(2)),
         minMaxT = std::min(std::min(maxT(0), maxT(1)), maxT(2));

  return ((minMaxT > 0.0) && (maxMinT < minMaxT))
             ? std::max(0.0, maxMinT)
             : std::numeric_limits<double>::max();
}

} // namespace acc

#endif
