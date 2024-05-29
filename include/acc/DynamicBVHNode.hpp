#ifndef ACC_DYNAMIC_BVH_NODE_HPP_
#define ACC_DYNAMIC_BVH_NODE_HPP_

#include <acc/BoundingBox.hpp>

namespace acc {

struct DynamicBVHNode {
  BoundingBox aabb;
  index_t objId = nullIndex;
  index_t parent = nullIndex;
  index_t children[2]{nullIndex, nullIndex};
};

} // namespace acc

#endif
