#ifndef ACC_COMMON_HPP_
#define ACC_COMMON_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>

#if defined(ACC_BACKEND_EIGEN)
#include <Eigen/Dense>
#elif defined(ACC_BACKEND_GLM) || defined(ACC_BACKEND_GLM_CUDA)
#if defined(ACC_BACKEND_GLM_CUDA)
#include <cuda.h>
#define GLM_FORCE_CUDA
#endif
#include <glm/glm.hpp>
#include <type_traits>
#endif

namespace acc {

#if defined(ACC_INDEX_T)
using index_t = ACC_INDEX_T;
#else
using index_t = std::size_t;
#endif

inline constexpr index_t nullIndex = static_cast<index_t>(-1);

#if defined(ACC_BACKEND_EIGEN)

#define ACC_HOST_DEVICE

#if defined(ACC_PRECISION)
using real_t = ACC_PRECISION;
#else
using real_t = double;
#endif

using vec3_t = Eigen::Matrix<real_t, 3, 1>;

inline vec3_t min(const vec3_t &a, const vec3_t &b) { return a.cwiseMin(b); }
inline vec3_t min(const vec3_t &a, real_t b) { return a.cwiseMin(b); }
inline real_t min(const vec3_t &a) { return a.minCoeff(); }

inline vec3_t max(const vec3_t &a, const vec3_t &b) { return a.cwiseMax(b); }
inline vec3_t max(const vec3_t &a, real_t b) { return a.cwiseMax(b); }
inline real_t max(const vec3_t &a) { return a.maxCoeff(); }

inline vec3_t prod(const vec3_t &a, const vec3_t &b) {
  return a.cwiseProduct(b);
}
inline vec3_t inv(const vec3_t &a) { return a.cwiseInverse(); }

inline vec3_t abs(const vec3_t &a) { return a.cwiseAbs(); }

inline real_t norm(const vec3_t &a) { return a.norm(); }
inline real_t sqrNorm(const vec3_t &a) { return a.squaredNorm(); }
inline real_t dot(const vec3_t &a, const vec3_t &b) { return a.dot(b); }
inline vec3_t cross(const vec3_t &a, const vec3_t &b) { return a.cross(b); }

#elif defined(ACC_BACKEND_GLM) || defined(ACC_BACKEND_GLM_CUDA)

#if defined(ACC_BACKEND_GLM_CUDA)
#define ACC_HOST_DEVICE __host__ __device__
#else
#define ACC_HOST_DEVICE
#endif

using real_t = float;
using vec3_t = glm::vec3;

inline ACC_HOST_DEVICE vec3_t min(const vec3_t &a, const vec3_t &b) {
  return glm::min(a, b);
}
inline ACC_HOST_DEVICE vec3_t min(const vec3_t &a, real_t b) {
  return glm::min(a, vec3_t{b});
}
inline ACC_HOST_DEVICE real_t min(const vec3_t &a) {
  return glm::min(a.x, glm::min(a.y, a.z));
}

inline ACC_HOST_DEVICE vec3_t max(const vec3_t &a, const vec3_t &b) {
  return glm::max(a, b);
}
inline ACC_HOST_DEVICE vec3_t max(const vec3_t &a, real_t b) {
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

inline constexpr real_t real_t_max = std::numeric_limits<real_t>::max();
inline constexpr real_t real_t_min = std::numeric_limits<real_t>::lowest();

inline ACC_HOST_DEVICE real_t min(real_t a, real_t b) { return a < b ? a : b; }
inline ACC_HOST_DEVICE real_t max(real_t a, real_t b) { return a > b ? a : b; }

} // namespace acc

#endif
