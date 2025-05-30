#ifndef ACC_VEC3_OP_HPP_
#define ACC_VEC3_OP_HPP_

#include <acc/Config.hpp>

#if defined(ACC_BACKEND_EIGEN)
#include <Eigen/Dense>
#elif defined(ACC_BACKEND_GLM)
#if defined(__CUDACC__)
#include <cuda.h>
#define GLM_FORCE_CUDA
#endif
#include <glm/glm.hpp>
#endif

namespace acc {

#if defined(ACC_BACKEND_EIGEN)

using vec3_t = Eigen::Matrix<real_t, 3, 1>;

inline ACC_HOST_DEVICE vec3_t min(const vec3_t &a, const vec3_t &b) {
  return a.cwiseMin(b);
}
inline ACC_HOST_DEVICE vec3_t min(const vec3_t &a, const real_t b) {
  return a.cwiseMin(b);
}
inline ACC_HOST_DEVICE real_t min(const vec3_t &a) { return a.minCoeff(); }

inline ACC_HOST_DEVICE vec3_t max(const vec3_t &a, const vec3_t &b) {
  return a.cwiseMax(b);
}
inline ACC_HOST_DEVICE vec3_t max(const vec3_t &a, const real_t b) {
  return a.cwiseMax(b);
}
inline ACC_HOST_DEVICE real_t max(const vec3_t &a) { return a.maxCoeff(); }

inline ACC_HOST_DEVICE vec3_t prod(const vec3_t &a, const vec3_t &b) {
  return a.cwiseProduct(b);
}
inline ACC_HOST_DEVICE vec3_t inv(const vec3_t &a) { return a.cwiseInverse(); }

inline ACC_HOST_DEVICE vec3_t abs(const vec3_t &a) { return a.cwiseAbs(); }

inline ACC_HOST_DEVICE real_t norm(const vec3_t &a) { return a.norm(); }
inline ACC_HOST_DEVICE real_t sqrNorm(const vec3_t &a) {
  return a.squaredNorm();
}
inline ACC_HOST_DEVICE real_t dot(const vec3_t &a, const vec3_t &b) {
  return a.dot(b);
}
inline ACC_HOST_DEVICE vec3_t cross(const vec3_t &a, const vec3_t &b) {
  return a.cross(b);
}

#elif defined(ACC_BACKEND_GLM)

using vec3_t = glm::vec3;

inline ACC_HOST_DEVICE vec3_t min(const vec3_t &a, const vec3_t &b) {
  return glm::min(a, b);
}
inline ACC_HOST_DEVICE vec3_t min(const vec3_t &a, const real_t b) {
  return glm::min(a, vec3_t{b});
}
inline ACC_HOST_DEVICE real_t min(const vec3_t &a) {
  return glm::min(a.x, glm::min(a.y, a.z));
}

inline ACC_HOST_DEVICE vec3_t max(const vec3_t &a, const vec3_t &b) {
  return glm::max(a, b);
}
inline ACC_HOST_DEVICE vec3_t max(const vec3_t &a, const real_t b) {
  return glm::max(a, vec3_t{b});
}
inline ACC_HOST_DEVICE real_t max(const vec3_t &a) {
  return glm::max(a.x, glm::max(a.y, a.z));
}

inline ACC_HOST_DEVICE vec3_t prod(const vec3_t &a, const vec3_t &b) {
  return a * b;
}
inline ACC_HOST_DEVICE vec3_t inv(const vec3_t &a) { return real_t{1.0} / a; }

inline ACC_HOST_DEVICE vec3_t abs(const vec3_t &a) { return glm::abs(a); }

inline ACC_HOST_DEVICE real_t norm(const vec3_t &a) { return glm::length(a); }
inline ACC_HOST_DEVICE real_t sqrNorm(const vec3_t &a) {
  return glm::dot(a, a);
}
inline ACC_HOST_DEVICE real_t dot(const vec3_t &a, const vec3_t &b) {
  return glm::dot(a, b);
}
inline ACC_HOST_DEVICE vec3_t cross(const vec3_t &a, const vec3_t &b) {
  return glm::cross(a, b);
}

#endif

} // namespace acc

#endif
