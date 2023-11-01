#ifndef ACC_RAY_HPP_
#define ACC_RAY_HPP_

#include <Eigen/Core>

namespace acc {

struct Ray {
  Eigen::Vector3d orig;
  Eigen::Vector3d dir;
};

} // namespace acc

#endif
