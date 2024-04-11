#ifndef ACC_COMMON_HPP_
#define ACC_COMMON_HPP_

#include <cstddef>
#include <cstdint>

#if defined(ACC_BACKEND_EIGEN)
#include <Eigen/Dense>
#elif defined(ACC_BACKEND_GLM)
#include <glm/glm.hpp>
#include <type_traits>
#endif

namespace acc {

using index_t = std::size_t;
inline constexpr index_t nullIndex = static_cast<index_t>(-1);

#if defined(ACC_BACKEND_EIGEN)

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

#elif defined(ACC_BACKEND_GLM)

#if defined(ACC_PRECISION)
using real_t = ACC_PRECISION;
#else
using real_t = float;
#endif

template <bool IsDouble> struct glm_vec3 {
  using type = glm::vec3; // float
};
template <> struct glm_vec3<true> {
  using type = glm::dvec3; // double
};
using vec3_t = glm_vec3<std::is_same_v<real_t, double>>::type;

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
