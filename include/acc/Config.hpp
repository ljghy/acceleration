#ifndef ACC_CONFIG_HPP_
#define ACC_CONFIG_HPP_

#include <limits>

#ifndef ACC_HOST_DEVICE
#ifdef __CUDACC__
#define ACC_HOST_DEVICE __host__ __device__
#else
#define ACC_HOST_DEVICE
#endif
#endif

namespace acc {

#if defined(ACC_INDEX_T)
using index_t = ACC_INDEX_T;
#else
using index_t = int;
#endif

inline constexpr index_t nullIndex = static_cast<index_t>(-1);

#if defined(ACC_BACKEND_EIGEN)
#if defined(ACC_PRECISION)
using real_t = ACC_PRECISION;
#else
using real_t = double;
#endif
#elif defined(ACC_BACKEND_GLM)
using real_t = float;
#else
#error "no backend specified"
#endif

inline constexpr real_t real_t_max = std::numeric_limits<real_t>::max();
inline constexpr real_t real_t_min = std::numeric_limits<real_t>::lowest();

inline ACC_HOST_DEVICE real_t min(const real_t a, const real_t b) {
  return a < b ? a : b;
}
inline ACC_HOST_DEVICE real_t max(const real_t a, const real_t b) {
  return a > b ? a : b;
}

} // namespace acc

#endif
