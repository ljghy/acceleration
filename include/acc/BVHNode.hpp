#ifndef ACC_BVH_NODE_HPP_
#define ACC_BVH_NODE_HPP_

#include <algorithm>
#include <array>

#include <acc/BoundingBox.hpp>

namespace acc {

struct BVHNode {
  BoundingBox aabb;
  index_t objId;
  index_t parent;
  index_t children[2]{nullIndex, nullIndex};
};

} // namespace acc

#endif
