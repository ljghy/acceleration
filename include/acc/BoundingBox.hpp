#ifndef ACC_BOUNDING_BOX_HPP_
#define ACC_BOUNDING_BOX_HPP_

#include <acc/Config.hpp>
#include <acc/Vec3Op.hpp>

namespace acc {

struct BoundingBox {
  vec3_t lb;
  vec3_t ub;

  ACC_HOST_DEVICE void init();

  ACC_HOST_DEVICE real_t minSqrDist(const vec3_t &) const;
  ACC_HOST_DEVICE real_t minDist(const vec3_t &) const;

  ACC_HOST_DEVICE BoundingBox operator+(const BoundingBox &other) const;
  ACC_HOST_DEVICE BoundingBox &operator+=(const BoundingBox &other);
  ACC_HOST_DEVICE bool containedIn(const BoundingBox &other) const;

  ACC_HOST_DEVICE void update(const vec3_t &);
  ACC_HOST_DEVICE real_t area() const;
  ACC_HOST_DEVICE vec3_t center() const;

  ACC_HOST_DEVICE bool contains(const vec3_t &) const;

  ACC_HOST_DEVICE real_t rayHit(const vec3_t &o, const vec3_t &d) const;

  ACC_HOST_DEVICE void enlarge(const real_t factor);
};

inline ACC_HOST_DEVICE void BoundingBox::init() {
  lb[0] = lb[1] = lb[2] = real_t_max;
  ub[0] = ub[1] = ub[2] = real_t_min;
}

inline ACC_HOST_DEVICE real_t BoundingBox::minSqrDist(const vec3_t &p) const {
  const vec3_t center = real_t{0.5} * (lb + ub);
  const vec3_t halfExtent = ub - center;
  const vec3_t q = p - center;
  return sqrNorm(max(abs(q) - halfExtent, real_t{}));
}

inline ACC_HOST_DEVICE real_t BoundingBox::minDist(const vec3_t &p) const {
  return std::sqrt(minSqrDist(p));
}

inline ACC_HOST_DEVICE void BoundingBox::update(const vec3_t &p) {
  lb = min(lb, p);
  ub = max(ub, p);
}

inline ACC_HOST_DEVICE BoundingBox
BoundingBox::operator+(const BoundingBox &other) const {
  return {min(lb, other.lb), max(ub, other.ub)};
}

inline ACC_HOST_DEVICE BoundingBox &
BoundingBox::operator+=(const BoundingBox &other) {
  lb = min(lb, other.lb);
  ub = max(ub, other.ub);
  return *this;
}

inline ACC_HOST_DEVICE bool
BoundingBox::containedIn(const BoundingBox &other) const {
  return (lb[0] >= other.lb[0]) && (lb[1] >= other.lb[1]) &&
         (lb[2] >= other.lb[2]) && (ub[0] <= other.ub[0]) &&
         (ub[1] <= other.ub[1]) && (ub[2] <= other.ub[2]);
}

inline ACC_HOST_DEVICE real_t BoundingBox::area() const {
  const vec3_t d = ub - lb;
  return d[0] * d[1] + d[1] * d[2] + d[2] * d[0];
}

inline ACC_HOST_DEVICE vec3_t BoundingBox::center() const {
  return real_t{0.5} * (lb + ub);
}

inline ACC_HOST_DEVICE bool BoundingBox::contains(const vec3_t &p) const {
  return (p[0] >= lb[0]) && (p[0] <= ub[0]) && (p[1] >= lb[1]) &&
         (p[1] <= ub[1]) && (p[2] >= lb[2]) && (p[2] <= ub[2]);
}

inline ACC_HOST_DEVICE real_t BoundingBox::rayHit(const vec3_t &o,
                                                  const vec3_t &d) const {

  const vec3_t invDir = inv(d), t1 = prod(invDir, lb - o),
               t2 = prod(invDir, ub - o), minT = min(t1, t2),
               maxT = max(t1, t2);

  const real_t maxMinT = max(minT), minMaxT = min(maxT);

  return ((minMaxT > real_t{}) && (maxMinT < minMaxT)) ? max(real_t{}, maxMinT)
                                                       : real_t_max;
}

inline ACC_HOST_DEVICE void BoundingBox::enlarge(const real_t factor) {
  const vec3_t center = real_t{0.5} * (lb + ub);
  vec3_t halfExtent = real_t{0.5} * (ub - lb);
  halfExtent *= factor;
  lb = center - halfExtent;
  ub = center + halfExtent;
}

} // namespace acc

#endif
