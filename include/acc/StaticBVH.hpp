#ifndef ACC_STATIC_BVH_HPP_
#define ACC_STATIC_BVH_HPP_

#include <algorithm>
#include <functional>
#include <numeric>
#include <stack>
#include <vector>

#include <acc/StaticBVHNode.hpp>

namespace acc {

class StaticBVH {
public:
  StaticBVH(index_t n, const std::function<BoundingBox(index_t)> &getAABB);

  std::pair<index_t, real_t>
  rayHit(const vec3_t &o, const vec3_t &d,
         const std::function<real_t(index_t)> &hit, real_t minT = {},
         real_t maxT = std::numeric_limits<real_t>::max()) const;

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
  std::vector<StaticBVHNode> m_nodes;
};

inline StaticBVH::StaticBVH(
    index_t n, const std::function<BoundingBox(index_t)> &getAABB) {
  if (n == 0)
    return;

  m_nodes.reserve(2 * n);
  std::vector<index_t> indexOrder(n);
  std::iota(indexOrder.begin(), indexOrder.end(), index_t{});

  std::vector<BoundingBox> aabbs(n);
  std::transform(indexOrder.begin(), indexOrder.end(), aabbs.begin(), getAABB);

  std::vector<vec3_t> centers(n);
  std::transform(aabbs.begin(), aabbs.end(), centers.begin(),
                 [](const BoundingBox &aabb) { return aabb.center(); });

  struct State {
    std::vector<index_t>::iterator first;
    std::vector<index_t>::iterator last;
    index_t parentIndex;
  };

  std::stack<State> s;
  s.emplace(indexOrder.begin(), indexOrder.end(), nullIndex);

  while (!s.empty()) {
    auto [first, last, p] = s.top();
    s.pop();

    auto n = std::distance(first, last);
    auto i = *first;
    if (n == 1) {
      if (p != nullIndex) {
        (m_nodes[p].children[0] == nullIndex ? m_nodes[p].children[0]
                                             : m_nodes[p].children[1]) =
            m_nodes.size();
      }
      m_nodes.emplace_back(aabbs[i], i);
      continue;
    }

    BoundingBox b = aabbs[i];
    for (auto it = first; it != last; ++it)
      b += aabbs[*it];

    vec3_t extent = b.ub - b.lb;
    int dim = 0;
    if (extent[1] > extent[0])
      dim = 1;
    if (extent[2] > extent[dim])
      dim = 2;

    auto m = first + n / 2;
    std::nth_element(first, m, last, [&](index_t a, index_t b) {
      return centers[a][dim] < centers[b][dim];
    });

    index_t index = m_nodes.size();
    m_nodes.emplace_back(b);
    if (p != nullIndex) {
      (m_nodes[p].children[0] == nullIndex ? m_nodes[p].children[0]
                                           : m_nodes[p].children[1]) = index;
    }

    s.emplace(first, m, index);
    s.emplace(m, last, index);
  }
}

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
  real_t currMinDist = std::numeric_limits<real_t>::max();

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
