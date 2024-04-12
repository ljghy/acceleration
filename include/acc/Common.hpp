#ifndef ACC_COMMON_HPP_
#define ACC_COMMON_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>

#if defined(ACC_BACKEND_GLM_CUDA)
#define ACC_HOST_DEVICE __host__ __device__
#else
#define ACC_HOST_DEVICE
#endif

namespace acc {

#if defined(ACC_INDEX_T)
using index_t = ACC_INDEX_T;
#else
using index_t = std::size_t;
#endif

inline constexpr index_t nullIndex = static_cast<index_t>(-1);

#if defined(ACC_BACKEND_EIGEN)

#if defined(ACC_PRECISION)
using real_t = ACC_PRECISION;
#else
using real_t = double;
#endif
#elif defined(ACC_BACKEND_GLM) || defined(ACC_BACKEND_GLM_CUDA)
using real_t = float;
#endif

inline constexpr real_t real_t_max = std::numeric_limits<real_t>::max();
inline constexpr real_t real_t_min = std::numeric_limits<real_t>::lowest();

inline ACC_HOST_DEVICE real_t min(real_t a, real_t b) { return a < b ? a : b; }
inline ACC_HOST_DEVICE real_t max(real_t a, real_t b) { return a > b ? a : b; }

} // namespace acc

#endif
