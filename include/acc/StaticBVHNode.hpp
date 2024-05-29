#ifndef ACC_STATIC_BVH_NODE_HPP_
#define ACC_STATIC_BVH_NODE_HPP_

#include <acc/BoundingBox.hpp>

namespace acc {

struct StaticBVHNode {
  BoundingBox aabb;
  index_t children[2]{nullIndex, nullIndex};

  ACC_HOST_DEVICE StaticBVHNode(const BoundingBox &aabb)
      : aabb(aabb), children{nullIndex, nullIndex} {}

  ACC_HOST_DEVICE StaticBVHNode(const BoundingBox &aabb, index_t objId)
      : aabb(aabb), children{objId, nullIndex} {}

  ACC_HOST_DEVICE index_t objId() const {
    return children[1] == nullIndex ? children[0] : nullIndex;
  }

  ACC_HOST_DEVICE bool isLeaf() const { return children[1] == nullIndex; }
};

} // namespace acc

#endif
