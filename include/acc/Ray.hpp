#ifndef ACC_RAY_HPP_
#define ACC_RAY_HPP_

#include <acc/Common.hpp>

namespace acc {

struct Ray {
  vec3_t orig;
  vec3_t dir;
};

} // namespace acc

#endif
