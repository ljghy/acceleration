#ifndef ACC_QUERY_HPP_
#define ACC_QUERY_HPP_

#include <array>

#include <acc/Config.hpp>
#include <acc/Vec3Op.hpp>

#ifdef __CUDACC__
#include <acc/StaticStack.hpp>
namespace acc {
using DefaultStackType = StaticStack<index_t, 64>;
}
#else
#include <stack>
namespace acc {
using DefaultStackType = std::stack<index_t>;
}
#endif

namespace acc {

struct RayHitQueryResult {
  index_t objId;
  real_t t;
};

template <typename Hit, typename StackType = DefaultStackType,
          typename BVHNodeType>
inline ACC_HOST_DEVICE RayHitQueryResult
rayHit(const index_t numNodes, const BVHNodeType *nodes, const vec3_t &o,
       const vec3_t &d, const Hit hit, const real_t minT = real_t{},
       const real_t maxT = real_t_max) {

  RayHitQueryResult ret{nullIndex, maxT};

  if (numNodes <= 0)
    return ret;

  StackType s;
  s.push(index_t{});

  while (!s.empty()) {
    const auto i = s.top();
    s.pop();

    const auto &node = nodes[i];
    const index_t objId = node.objId();
    if (!node.isLeaf()) {
      const real_t t0 = nodes[node.children[0]].aabb.rayHit(o, d);
      const real_t t1 = nodes[node.children[1]].aabb.rayHit(o, d);

      if (t0 < t1) {
        if (t1 < ret.t)
          s.push(node.children[1]);
        if (t0 < ret.t)
          s.push(node.children[0]);
      } else {
        if (t0 < ret.t)
          s.push(node.children[0]);
        if (t1 < ret.t)
          s.push(node.children[1]);
      }
    } else {
      real_t t = hit(objId);
      if (t < ret.t && t > minT) {
        ret.objId = objId;
        ret.t = t;
      }
    }
  }

  return ret;
}

template <typename Hit, typename Discard, typename StackType = DefaultStackType,
          typename BVHNodeType>
inline ACC_HOST_DEVICE RayHitQueryResult
rayHit(const index_t numNodes, const BVHNodeType *nodes, const vec3_t &o,
       const vec3_t &d, const Hit hit, const Discard discard,
       const real_t minT = real_t{}, const real_t maxT = real_t_max) {

  RayHitQueryResult ret{nullIndex, maxT};

  if (numNodes <= 0)
    return ret;

  StackType s;
  s.push(index_t{});

  while (!s.empty()) {
    const auto i = s.top();
    s.pop();

    const auto &node = nodes[i];
    const index_t objId = node.objId();
    if (!node.isLeaf()) {
      const real_t t0 = nodes[node.children[0]].aabb.rayHit(o, d);
      const real_t t1 = nodes[node.children[1]].aabb.rayHit(o, d);

      if (t0 < t1) {
        if (t1 < ret.t)
          s.push(node.children[1]);
        if (t0 < ret.t)
          s.push(node.children[0]);
      } else {
        if (t0 < ret.t)
          s.push(node.children[0]);
        if (t1 < ret.t)
          s.push(node.children[1]);
      }
    } else {
      real_t t = hit(objId);
      if (t < ret.t && t > minT && !discard(objId, t)) {
        ret.objId = objId;
        ret.t = t;
      }
    }
  }

  return ret;
}

struct NearestObjectQueryResult {
  index_t objId;
  real_t sqrDist;
};

template <typename SqrDist, typename StackType = DefaultStackType,
          typename BVHNodeType>
NearestObjectQueryResult inline ACC_HOST_DEVICE
nearestObject(const index_t numNodes, const BVHNodeType *nodes, const vec3_t &p,
              SqrDist sqrDist) {

  NearestObjectQueryResult ret{nullIndex, real_t_max};

  if (numNodes <= 0)
    return ret;

  StackType s;
  s.push(index_t{});

  while (!s.empty()) {
    const auto i = s.top();
    s.pop();

    const auto &node = nodes[i];
    const index_t objId = node.objId();
    if (!node.isLeaf()) {
      const real_t sd0 = nodes[node.children[0]].aabb.minSqrDist(p);
      const real_t sd1 = nodes[node.children[1]].aabb.minSqrDist(p);

      if (sd0 < sd1) {
        if (sd1 < ret.sqrDist)
          s.push(node.children[1]);
        if (sd0 < ret.sqrDist)
          s.push(node.children[0]);
      } else {
        if (sd0 < ret.sqrDist)
          s.push(node.children[0]);
        if (sd1 < ret.sqrDist)
          s.push(node.children[1]);
      }
    } else {
      const real_t sd = sqrDist(objId);
      if (sd < ret.sqrDist) {
        ret.objId = objId;
        ret.sqrDist = sd;
      }
    }
  }

  return ret;
}

template <index_t K, typename SqrDist, typename StackType = DefaultStackType,
          typename BVHNodeType>
std::array<NearestObjectQueryResult, K> inline ACC_HOST_DEVICE
kNearestObjects(const index_t numNodes, const BVHNodeType *nodes,
                const vec3_t &p, SqrDist sqrDist) {

  static_assert(K > 1, "K must be greater than 1");

  std::array<NearestObjectQueryResult, K> ret;
  for (auto &r : ret)
    r = {nullIndex, real_t_max};

  if (numNodes <= 0)
    return ret;

  StackType s;
  s.push(index_t{});

  while (!s.empty()) {
    const auto i = s.top();
    s.pop();

    const auto &node = nodes[i];
    const index_t objId = node.objId();
    if (!node.isLeaf()) {
      const real_t sd0 = nodes[node.children[0]].aabb.minSqrDist(p);
      const real_t sd1 = nodes[node.children[1]].aabb.minSqrDist(p);

      if (sd0 < sd1) {
        if (sd1 < ret[K - 1].sqrDist)
          s.push(node.children[1]);
        if (sd0 < ret[K - 1].sqrDist)
          s.push(node.children[0]);
      } else {
        if (sd0 < ret[K - 1].sqrDist)
          s.push(node.children[0]);
        if (sd1 < ret[K - 1].sqrDist)
          s.push(node.children[1]);
      }
    } else {
      const real_t sd = sqrDist(objId);
      if (sd < ret[K - 1].sqrDist) {
        ret[K - 1] = {objId, sd};
        for (index_t k = K - 1; k > 0; --k) {
          if (ret[k].sqrDist < ret[k - 1].sqrDist)
            std::swap(ret[k], ret[k - 1]);
          else
            break;
        }
      }
    }
  }

  return ret;
}

} // namespace acc

#endif
