#ifndef ACC_TRI_UTILS_HPP_
#define ACC_TRI_UTILS_HPP_

#include <cmath>
#include <limits>

#include <acc/BoundingBox.hpp>

namespace acc::tri {

inline ACC_HOST_DEVICE BoundingBox
boundingBox(const vec3_t &a, const vec3_t &b, const vec3_t &c,
            const real_t eps = std::numeric_limits<real_t>::epsilon()) {
  BoundingBox aabb{a, a};
  aabb.update(b);
  aabb.update(c);
  for (int i = 0; i < 3; ++i) {
    if (std::abs(aabb.lb[i] - aabb.ub[i]) < eps) {
      aabb.lb[i] -= eps;
      aabb.ub[i] += eps;
    }
  }
  return aabb;
}

inline ACC_HOST_DEVICE vec3_t barycentricCoords(const vec3_t &a,
                                                const vec3_t &b,
                                                const vec3_t &c,
                                                const vec3_t &p) {
  const vec3_t e1 = b - a, e2 = c - a, s = p - a;
  const real_t a11 = sqrNorm(e1), a12 = dot(e1, e2), a22 = sqrNorm(e2);

  const real_t b1 = dot(s, e1), b2 = dot(s, e2);
  const real_t invDet = real_t{1.0} / (a11 * a22 - a12 * a12);

  vec3_t bc;
  bc[1] = (a22 * b1 - a12 * b2) * invDet;
  bc[2] = (-a12 * b1 + a11 * b2) * invDet;
  bc[0] = real_t{1.0} - bc[1] - bc[2];

  return bc;
}

namespace internal {

template <typename Norm>
inline ACC_HOST_DEVICE real_t pointDistance(const vec3_t &a, const vec3_t &b,
                                            const vec3_t &c, const vec3_t &p,
                                            const Norm norm_) {
  const vec3_t ab = b - a;
  const vec3_t ac = c - a;
  const vec3_t ap = p - a;

  const real_t d1 = dot(ap, ab);
  const real_t d2 = dot(ap, ac);

  if (d1 <= real_t{} && d2 <= real_t{})
    return norm_(ap);

  const vec3_t bp = p - b;

  const real_t d3 = dot(bp, ab);
  const real_t d4 = dot(bp, ac);

  if (d3 >= real_t{} && d4 <= d3)
    return norm_(bp);

  const vec3_t cp = p - c;

  const real_t d5 = dot(cp, ab);
  const real_t d6 = dot(cp, ac);

  if (d6 >= real_t{} && d5 <= d6)
    return norm_(cp);

  const real_t vc = d1 * d4 - d3 * d2;
  if (vc <= real_t{} && d1 >= real_t{} && d3 <= real_t{}) {
    const real_t v = d1 / (d1 - d3);
    return norm_(ap - v * ab);
  }

  const real_t vb = d5 * d2 - d1 * d6;
  if (vb <= real_t{} && d2 >= real_t{} && d6 <= real_t{}) {
    const real_t v = d2 / (d2 - d6);
    return norm_(ap - v * ac);
  }

  const real_t va = d3 * d6 - d5 * d4;
  if (va <= real_t{} && (d4 - d3) >= real_t{} && (d5 - d6) >= real_t{}) {
    const real_t v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return norm_(bp - v * (c - b));
  }

  const real_t denom = real_t{1.0} / (va + vb + vc);
  const real_t v = vb * denom;
  const real_t w = vc * denom;
  return norm_(ap - v * ab - w * ac);
}

} // namespace internal

inline ACC_HOST_DEVICE real_t pointDistance(const vec3_t &a, const vec3_t &b,
                                            const vec3_t &c, const vec3_t &p) {
  return internal::pointDistance(a, b, c, p, norm);
}

inline ACC_HOST_DEVICE real_t pointSquaredDistance(const vec3_t &a,
                                                   const vec3_t &b,
                                                   const vec3_t &c,
                                                   const vec3_t &p) {
  return internal::pointDistance(a, b, c, p, sqrNorm);
}

inline ACC_HOST_DEVICE vec3_t pointProjection(const vec3_t &a, const vec3_t &b,
                                              const vec3_t &c,
                                              const vec3_t &p) {
  const vec3_t ab = b - a;
  const vec3_t ac = c - a;
  const vec3_t ap = p - a;

  const real_t d1 = dot(ap, ab);
  const real_t d2 = dot(ap, ac);

  if (d1 <= real_t{} && d2 <= real_t{})
    return a;

  const vec3_t bp = p - b;

  const real_t d3 = dot(bp, ab);
  const real_t d4 = dot(bp, ac);

  if (d3 >= real_t{} && d4 <= d3)
    return b;

  const vec3_t cp = p - c;

  const real_t d5 = dot(cp, ab);
  const real_t d6 = dot(cp, ac);

  if (d6 >= real_t{} && d5 <= d6)
    return c;

  const real_t vc = d1 * d4 - d3 * d2;
  if (vc <= real_t{} && d1 >= real_t{} && d3 <= real_t{}) {
    const real_t v = d1 / (d1 - d3);
    return a + v * ab;
  }

  const real_t vb = d5 * d2 - d1 * d6;
  if (vb <= real_t{} && d2 >= real_t{} && d6 <= real_t{}) {
    const real_t v = d2 / (d2 - d6);
    return a + v * ac;
  }

  const real_t va = d3 * d6 - d5 * d4;
  if (va <= real_t{} && (d4 - d3) >= real_t{} && (d5 - d6) >= real_t{}) {
    const real_t v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + v * (c - b);
  }

  const real_t denom = real_t{1.0} / (va + vb + vc);
  const real_t v = vb * denom;
  const real_t w = vc * denom;
  return a + v * ab + w * ac;
}

inline ACC_HOST_DEVICE real_t rayHit(const vec3_t &a, const vec3_t &b,
                                     const vec3_t &c, const vec3_t &o,
                                     const vec3_t &d) {
  const vec3_t s = o - a;
  const vec3_t e1 = b - a;
  const vec3_t e2 = c - a;
  const vec3_t n = cross(e1, e2);

  const real_t k = -real_t{1.0} / dot(n, d);
  const vec3_t w = cross(s, d);

  const real_t u = dot(w, e2) * k;
  const real_t v = -dot(w, e1) * k;
  const real_t t = dot(n, s) * k;

  if (u >= real_t{} && v >= real_t{} && u + v <= real_t{1.0} && t >= real_t{})
    return t;
  else
    return real_t_max;
}

} // namespace acc::tri

#endif
