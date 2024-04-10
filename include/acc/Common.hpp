#ifndef ACC_COMMON_HPP_
#define ACC_COMMON_HPP_

#if defined(ACC_BACKEND_EIGEN)
#include <Eigen/Dense>
#elif defined(ACC_BACKEND_GLM)
#include <glm/glm.hpp>
#elif defined(ACC_BACKEND_CUSTOM)
#else // Fallback to Eigen
#define ACC_BACKEND_EIGEN
#include <Eigen/Dense>
#endif

namespace acc {

#if defined(ACC_PRECISION)
using real_t = ACC_PRECISION;
#else
using real_t = double;
#endif

#if defined(ACC_BACKEND_EIGEN)
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

#elif defined(ACC_BACKEND_GLM)

using vec3_t = glm::vector<real_t, 3>;

inline vec3_t min(const vec3_t &a, const vec3_t &b) { return glm::min(a, b); }
inline vec3_t min(const vec3_t &a, real_t b) { return glm::min(a, vec3_t{b}); }
inline real_t min(const vec3_t &a) { return std::min(a.x, std::min(a.y, a.z)); }

inline vec3_t max(const vec3_t &a, const vec3_t &b) { return glm::max(a, b); }
inline vec3_t max(const vec3_t &a, real_t b) { return glm::max(a, vec3_t{b}); }
inline real_t max(const vec3_t &a) { return std::max(a.x, std::max(a.y, a.z)); }

inline vec3_t prod(const vec3_t &a, const vec3_t &b) { return a * b; }
inline vec3_t inv(const vec3_t &a) { return real_t{1.0} / a; }

inline vec3_t abs(const vec3_t &a) { return glm::abs(a); }

inline real_t norm(const vec3_t &a) { return glm::length(a); }
inline real_t sqrNorm(const vec3_t &a) { return glm::dot(a, a); }
inline real_t dot(const vec3_t &a, const vec3_t &b) { return glm::dot(a, b); }
inline vec3_t cross(const vec3_t &a, const vec3_t &b) {
  return glm::cross(a, b);
}

#endif

} // namespace acc

#endif
