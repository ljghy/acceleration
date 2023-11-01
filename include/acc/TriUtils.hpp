#ifndef ACC_TRI_UTILS_HPP_
#define ACC_TRI_UTILS_HPP_

#include <limits>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <acc/BoundingBox.hpp>
#include <acc/Ray.hpp>

namespace acc::tri {

inline BoundingBox boundingBox(const Eigen::Vector3d &a,
                               const Eigen::Vector3d &b,
                               const Eigen::Vector3d &c) {
  BoundingBox aabb{a, a};
  aabb.update(b);
  aabb.update(c);
  for (int i = 0; i < 3; ++i) {
    if (aabb.lb(i) == aabb.ub(i)) {
      aabb.lb(i) -= 1e-3;
      aabb.ub(i) += 1e-3;
    }
  }
  return aabb;
}

template <typename VertexListType, typename IndicesType>
inline BoundingBox boundingBox(const VertexListType &vertices,
                               const IndicesType &indices) {
  return boundingBox(vertices[indices(0)].position,
                     vertices[indices(1)].position,
                     vertices[indices(2)].position);
}

inline Eigen::Vector3d barycentricCoords(const Eigen::Vector3d &a,
                                         const Eigen::Vector3d &b,
                                         const Eigen::Vector3d &c,
                                         const Eigen::Vector3d &p) {
  Eigen::Vector3d e1 = b - a, e2 = c - a, s = p - a;
  double a11 = e1.squaredNorm(), a12 = e1.dot(e2), a22 = e2.squaredNorm();

  double b1 = s.dot(e1), b2 = s.dot(e2);
  double invDet = 1.0 / (a11 * a22 - a12 * a12);

  Eigen::Vector3d bc;
  bc(1) = (a22 * b1 - a12 * b2) * invDet;
  bc(2) = (-a12 * b1 + a11 * b2) * invDet;
  bc(0) = 1.0 - bc(1) - bc(2);

  return bc;
}

template <typename VertexListType, typename IndicesType>
inline Eigen::Vector3d barycentricCoords(const VertexListType &vertices,
                                         const IndicesType &indices,
                                         const Eigen::Vector3d &p) {
  return barycentricCoords(vertices[indices(0)].position,
                           vertices[indices(1)].position,
                           vertices[indices(2)].position, p);
}

inline double pointDistance(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                            const Eigen::Vector3d &c,
                            const Eigen::Vector3d &p) {
  Eigen::Vector3d ab = b - a;
  Eigen::Vector3d ac = c - a;
  Eigen::Vector3d ap = p - a;

  double d1 = ap.dot(ab);
  double d2 = ap.dot(ac);

  if (d1 <= 0.0 && d2 <= 0.0)
    return ap.norm();

  Eigen::Vector3d bp = p - b;

  double d3 = bp.dot(ab);
  double d4 = bp.dot(ac);

  if (d3 >= 0.0 && d4 <= d3)
    return bp.norm();

  Eigen::Vector3d cp = p - c;

  double d5 = cp.dot(ab);
  double d6 = cp.dot(ac);

  if (d6 >= 0.0 && d5 <= d6)
    return cp.norm();

  double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    double v = d1 / (d1 - d3);
    return (ap - v * ab).norm();
  }

  double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    double v = d2 / (d2 - d6);
    return (ap - v * ac).norm();
  }

  double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    double v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return (bp - v * (c - b)).norm();
  }

  double denom = 1.0 / (va + vb + vc);
  double v = vb * denom;
  double w = vc * denom;
  return (ap - v * ab - w * ac).norm();
}

template <typename VertexListType, typename IndicesType>
inline double pointDistance(const VertexListType &vertices,
                            const IndicesType &indices,
                            const Eigen::Vector3d &p) {
  return pointDistance(vertices[indices(0)].position,
                       vertices[indices(1)].position,
                       vertices[indices(2)].position, p);
}

inline double rayHit(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                     const Eigen::Vector3d &c, const Ray &ray) {
  Eigen::Vector3d s = ray.orig - a;
  Eigen::Vector3d e1 = b - a;
  Eigen::Vector3d e2 = c - a;
  Eigen::Vector3d n = e1.cross(e2);

  double k = -1.0 / n.dot(ray.dir);
  Eigen::Vector3d w = s.cross(ray.dir);

  double u = w.dot(e2) * k;
  double v = -w.dot(e1) * k;
  double t = n.dot(s) * k;

  if (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0)
    return t;
  else
    return std::numeric_limits<double>::max();
}

template <typename VertexListType, typename IndicesType>
inline double rayHit(const VertexListType &vertices, const IndicesType &indices,
                     const Ray &ray) {
  return rayHit(vertices[indices(0)].position, vertices[indices(1)].position,
                vertices[indices(2)].position, ray);
}

} // namespace acc::tri

#endif
