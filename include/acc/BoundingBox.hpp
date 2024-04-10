#ifndef ACC_BOUNDING_BOX_HPP_
#define ACC_BOUNDING_BOX_HPP_

#include <cmath>
#include <limits>

#include <acc/Common.hpp>

namespace acc {

struct BoundingBox {
  vec3_t lb;
  vec3_t ub;

  void init();

  real_t minSqrDist(vec3_t) const;
  real_t minDist(vec3_t) const;

  BoundingBox trimLeft(int dim, real_t c) const;
  BoundingBox trimRight(int dim, real_t c) const;

  static BoundingBox merge(const BoundingBox &a, const BoundingBox &b);
  const BoundingBox operator+(const BoundingBox &other) const;
  BoundingBox &operator+=(const BoundingBox &other);

  void update(const vec3_t &);
  real_t area() const;
  vec3_t center() const;

  bool contains(const vec3_t &) const;

  real_t rayHit(const vec3_t &o, const vec3_t &d) const;
};

inline void BoundingBox::init() {
  lb[0] = lb[1] = lb[2] = std::numeric_limits<real_t>::max();
  ub[0] = ub[1] = ub[2] = std::numeric_limits<real_t>::lowest();
}

inline real_t BoundingBox::minSqrDist(vec3_t p) const {
  vec3_t center = real_t{0.5} * (lb + ub);
  vec3_t halfExtent = ub - center;
  p -= center;
  return sqrNorm(max(abs(p) - halfExtent, real_t{}));
}

inline real_t BoundingBox::minDist(vec3_t p) const {
  return std::sqrt(minSqrDist(p));
}

inline BoundingBox BoundingBox::trimLeft(int dim, real_t c) const {
  auto m = ub;
  m[dim] = c;
  return {lb, m};
}

inline BoundingBox BoundingBox::trimRight(int dim, real_t c) const {
  auto m = lb;
  m[dim] = c;
  return {m, ub};
}

inline void BoundingBox::update(const vec3_t &p) {
  lb = min(lb, p);
  ub = max(ub, p);
}

inline BoundingBox BoundingBox::merge(const BoundingBox &a,
                                      const BoundingBox &b) {
  return {min(a.lb, b.lb), max(a.ub, b.ub)};
}

inline const BoundingBox
BoundingBox::operator+(const BoundingBox &other) const {
  return {min(lb, other.lb), max(ub, other.ub)};
}

inline BoundingBox &BoundingBox::operator+=(const BoundingBox &other) {
  lb = min(lb, other.lb);
  ub = max(ub, other.ub);
  return *this;
}

inline real_t BoundingBox::area() const {
  vec3_t d = ub - lb;
  return d[0] * d[1] + d[1] * d[2] + d[2] * d[0];
}

inline vec3_t BoundingBox::center() const { return real_t{0.5} * (lb + ub); }

inline bool BoundingBox::contains(const vec3_t &p) const {
  return (p[0] >= lb[0]) && (p[0] <= ub[0]) && (p[1] >= lb[1]) &&
         (p[1] <= ub[1]) && (p[2] >= lb[2]) && (p[2] <= ub[2]);
}

inline real_t BoundingBox::rayHit(const vec3_t &o, const vec3_t &d) const {

  vec3_t invDir = inv(d), t1 = prod(invDir, lb - o), t2 = prod(invDir, ub - o),
         minT = min(t1, t2), maxT = max(t1, t2);

  real_t maxMinT = max(minT), minMaxT = min(maxT);

  return ((minMaxT > real_t{}) && (maxMinT < minMaxT))
             ? std::max(real_t{}, maxMinT)
             : std::numeric_limits<real_t>::max();
}

} // namespace acc

#endif
