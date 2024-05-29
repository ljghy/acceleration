#ifndef ACC_STATIC_BVH_HPP_
#define ACC_STATIC_BVH_HPP_

#include <algorithm>
#include <bit>
#include <functional>
#include <stack>
#include <vector>

#include <acc/MortonCode.hpp>
#include <acc/StaticBVHNode.hpp>

namespace acc {

class StaticBVH {
public:
  StaticBVH() = default;
  StaticBVH(index_t n, const std::function<BoundingBox(index_t)> &getAABB) {
    build(n, getAABB);
  }

  void build(index_t n, const std::function<BoundingBox(index_t)> &getAABB);

  index_t maxDepth() const;
  index_t numNodes() const;
  const StaticBVHNode *nodes() const;

  BoundingBox rootAABB() const {
    if (!m_nodes.empty())
      return m_nodes[0].aabb;
    BoundingBox b;
    b.init();
    return b;
  }

  std::pair<index_t, real_t> rayHit(const vec3_t &o, const vec3_t &d,
                                    const std::function<real_t(index_t)> &hit,
                                    real_t minT = {},
                                    real_t maxT = real_t_max) const;

  std::pair<index_t, real_t>
  nearestObject(const vec3_t &p,
                const std::function<real_t(index_t)> &dist) const;

  index_t
  pointIntersectionFirst(const vec3_t &p,
                         const std::function<bool(index_t)> &inside) const;

  template <typename OutputIt>
  void pointIntersectionAll(const vec3_t &p,
                            const std::function<bool(index_t)> &inside,
                            OutputIt out) const;

private:
  index_t buildRecursive(const std::vector<BoundingBox> &aabbs,
                         const std::vector<MortonCode> &mortonCodes,
                         index_t first, index_t last);

  static index_t findSplit(const std::vector<MortonCode> &mortonCodes,
                           index_t first, index_t last);

private:
  std::vector<StaticBVHNode> m_nodes;
};

inline index_t StaticBVH::findSplit(const std::vector<MortonCode> &mortonCodes,
                                    index_t first, index_t last) {
  uint32_t firstCode = mortonCodes[first].code;
  uint32_t lastCode = mortonCodes[last].code;
  if (firstCode == lastCode)
    return (first + last) / 2;

  auto commonPrefix = std::countl_zero(firstCode ^ lastCode);
  auto split = first;
  index_t step = last - first;

  do {
    step = (step + 1) >> 1;
    index_t newSplit = split + step;

    if (newSplit < last) {
      auto splitCode = mortonCodes[newSplit].code;
      auto splitPrefix = std::countl_zero(firstCode ^ splitCode);
      if (splitPrefix > commonPrefix)
        split = newSplit;
    }
  } while (step > 1);

  return split;
}

inline void
StaticBVH::build(index_t n,
                 const std::function<BoundingBox(index_t)> &getAABB) {
  if (n == 0)
    return;

  m_nodes.clear();
  m_nodes.reserve(2 * n);

  std::vector<BoundingBox> aabbs(n);
  BoundingBox rootAABB;
  rootAABB.init();
  for (index_t i = 0; i < n; ++i) {
    aabbs[i] = getAABB(i);
    rootAABB += aabbs[i];
  }

  std::vector<MortonCode> mortonCodes;
  mortonCodes.reserve(n);
  for (index_t i = 0; i < n; ++i)
    mortonCodes.emplace_back(i, prod(aabbs[i].center() - rootAABB.lb,
                                     inv(rootAABB.ub - rootAABB.lb)));

  std::sort(mortonCodes.begin(), mortonCodes.end());

  m_nodes.emplace_back(rootAABB);
  buildRecursive(aabbs, mortonCodes, 0, n - 1);
}

inline index_t
StaticBVH::buildRecursive(const std::vector<BoundingBox> &aabbs,
                          const std::vector<MortonCode> &mortonCodes,
                          index_t first, index_t last) {
  if (first == last) {
    auto i = mortonCodes[first].index;
    m_nodes.emplace_back(aabbs[i], i);
  } else {
    auto split = findSplit(mortonCodes, first, last);
    auto child0 = buildRecursive(aabbs, mortonCodes, first, split);
    auto child1 = buildRecursive(aabbs, mortonCodes, split + 1, last);
    auto &node =
        (first == 0 && last + 1 == aabbs.size())
            ? m_nodes[0]
            : m_nodes.emplace_back(m_nodes[child0].aabb + m_nodes[child1].aabb);
    node.children[0] = child0;
    node.children[1] = child1;
  }
  return m_nodes.size() - 1;
}

inline index_t StaticBVH::maxDepth() const {
  if (m_nodes.empty())
    return index_t{};
  index_t maxDepth = index_t{};
  std::stack<std::pair<index_t, index_t>> s;
  s.emplace(index_t{}, index_t{});
  while (!s.empty()) {
    auto [i, depth] = s.top();
    s.pop();
    const auto &node = m_nodes[i];
    if (node.objId() == nullIndex) {
      s.emplace(node.children[0], depth + 1);
      s.emplace(node.children[1], depth + 1);
    } else {
      maxDepth = std::max(maxDepth, depth);
    }
  }
  return maxDepth;
}

inline index_t StaticBVH::numNodes() const { return m_nodes.size(); }

inline const StaticBVHNode *StaticBVH::nodes() const { return m_nodes.data(); }

inline std::pair<index_t, real_t>
StaticBVH::rayHit(const vec3_t &o, const vec3_t &d,
                  const std::function<real_t(index_t)> &hit, real_t minT,
                  real_t maxT) const {
  if (m_nodes.empty())
    return {nullIndex, maxT};

  index_t currObjId = nullIndex;
  real_t currMinT = maxT;

  std::stack<index_t> s;
  s.emplace(index_t{});

  while (!s.empty()) {
    auto i = s.top();
    s.pop();

    const auto &node = m_nodes[i];
    index_t objId = node.objId();
    if (objId == nullIndex) {
      real_t t0 = m_nodes[node.children[0]].aabb.rayHit(o, d);
      real_t t1 = m_nodes[node.children[1]].aabb.rayHit(o, d);

      if (t0 < t1) {
        if (t1 < currMinT)
          s.emplace(node.children[1]);
        if (t0 < currMinT)
          s.emplace(node.children[0]);
      } else {
        if (t0 < currMinT)
          s.emplace(node.children[0]);
        if (t1 < currMinT)
          s.emplace(node.children[1]);
      }
    } else {
      real_t t = hit(objId);
      if (t < currMinT && t > minT) {
        currMinT = t;
        currObjId = objId;
      }
    }
  }

  return {currObjId, currMinT};
}

std::pair<index_t, real_t> inline StaticBVH::nearestObject(
    const vec3_t &p, const std::function<real_t(index_t)> &dist) const {
  real_t currMinDist = real_t_max;

  if (m_nodes.empty())
    return {nullIndex, currMinDist};

  index_t currObjId = nullIndex;

  std::stack<index_t> s;
  s.emplace(index_t{});

  while (!s.empty()) {
    auto i = s.top();
    s.pop();

    const auto &node = m_nodes[i];
    index_t objId = node.objId();
    if (objId == nullIndex) {
      real_t d0 = m_nodes[node.children[0]].aabb.minDist(p);
      real_t d1 = m_nodes[node.children[1]].aabb.minDist(p);

      if (d0 < d1) {
        if (d1 < currMinDist)
          s.emplace(node.children[1]);
        if (d0 < currMinDist)
          s.emplace(node.children[0]);
      } else {
        if (d0 < currMinDist)
          s.emplace(node.children[0]);
        if (d1 < currMinDist)
          s.emplace(node.children[1]);
      }
    } else {
      real_t d = dist(objId);
      if (d < currMinDist) {
        currMinDist = d;
        currObjId = objId;
      }
    }
  }

  return {currObjId, currMinDist};
}

inline index_t StaticBVH::pointIntersectionFirst(
    const vec3_t &p, const std::function<bool(index_t)> &inside) const {
  index_t objId = nullIndex;
  if (m_nodes.empty())
    return objId;

  std::stack<index_t> s;
  s.emplace(index_t{});

  while (!s.empty()) {
    auto i = s.top();
    s.pop();

    const auto &node = m_nodes[i];
    index_t objId = node.objId();
    if (objId == nullIndex) {
      if (m_nodes[node.children[0]].aabb.contains(p))
        s.emplace(node.children[0]);
      if (m_nodes[node.children[1]].aabb.contains(p))
        s.emplace(node.children[1]);
    } else {
      if (inside(objId))
        return objId;
    }
  }

  return objId;
}

template <typename OutputIt>
inline void
StaticBVH::pointIntersectionAll(const vec3_t &p,
                                const std::function<bool(index_t)> &inside,
                                OutputIt out) const {
  if (m_nodes.empty())
    return;

  std::stack<index_t> s;
  s.emplace(index_t{});

  while (!s.empty()) {
    auto i = s.top();
    s.pop();

    const auto &node = m_nodes[i];
    index_t objId = node.objId();
    if (objId == nullIndex) {
      if (m_nodes[node.children[0]].aabb.contains(p))
        s.emplace(node.children[0]);
      if (m_nodes[node.children[1]].aabb.contains(p))
        s.emplace(node.children[1]);
    } else {
      if (inside(objId))
        *out++ = objId;
    }
  }
}

} // namespace acc

#endif
