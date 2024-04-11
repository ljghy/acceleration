#ifndef ACC_DYNAMIC_BVH_HPP_
#define ACC_DYNAMIC_BVH_HPP_

#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <stack>
#include <vector>

#include <acc/BVHNode.hpp>

namespace acc {

class DynamicBVH {
public:
  DynamicBVH();

  void staticConstruct(index_t n,
                       const std::function<BoundingBox(index_t)> &getAABB);

  void insert(const BoundingBox &aabb, index_t id);
  void reserve(index_t n);
  void clear();

  index_t nearestObject(const vec3_t &p,
                        const std::function<real_t(index_t)> &dist) const;

  index_t
  pointIntersectionFirst(const vec3_t &p,
                         const std::function<bool(index_t)> &inside) const;

  template <typename OutputIt>
  void pointIntersectionAll(const vec3_t &p,
                            const std::function<bool(index_t)> &inside,
                            OutputIt) const;

  std::pair<index_t, real_t>
  rayHit(const vec3_t &o, const vec3_t &d,
         const std::function<real_t(index_t)> &hit, real_t minT = {},
         real_t maxT = std::numeric_limits<real_t>::max()) const;

  void update(const std::function<BoundingBox(index_t)> &getNewAABB,
              real_t enlargementFactor = real_t{1.2});

private:
  real_t inheritedAreaDiff(BoundingBox aabb, index_t index);

  void removeAndInsert(index_t index, const BoundingBox &newAABB);
  void insertLeafAt(index_t leafIndex, index_t newParent,
                    const BoundingBox &aabb, index_t);
  void refit(index_t index);

private:
  struct AreaCost {
    index_t index;
    real_t cost;
    bool operator<(const AreaCost &other) const { return cost < other.cost; }
  };

private:
  std::vector<BVHNode> m_nodes;
  index_t m_rootIndex;
};

inline DynamicBVH::DynamicBVH() : m_rootIndex(nullIndex) {}

inline void DynamicBVH::reserve(index_t n) { m_nodes.reserve(2 * n); }

inline void DynamicBVH::clear() {
  m_nodes.clear();
  m_rootIndex = nullIndex;
}

inline void DynamicBVH::staticConstruct(
    index_t n, const std::function<BoundingBox(index_t)> &getAABB) {
  if (n == 0)
    return;

  clear();
  reserve(n);

  std::vector<index_t> indexOrder(n);
  std::iota(indexOrder.begin(), indexOrder.end(), index_t{});

  std::vector<BoundingBox> aabbs(n);
  std::transform(indexOrder.begin(), indexOrder.end(), aabbs.begin(), getAABB);

  std::vector<vec3_t> centers(aabbs.size());
  std::transform(aabbs.begin(), aabbs.end(), centers.begin(),
                 [](const auto &aabb) { return aabb.center(); });

  struct State {
    std::vector<index_t>::iterator first;
    std::vector<index_t>::iterator last;
    index_t parentIndex;
  };

  std::stack<State> s;
  s.push({indexOrder.begin(), indexOrder.end(), nullIndex});

  while (!s.empty()) {
    auto [first, last, parentIndex] = s.top();
    s.pop();

    if (first == last)
      continue;
    auto n = std::distance(first, last);
    if (n == 1) {
      if (parentIndex != nullIndex) {
        (m_nodes[parentIndex].children[0] == nullIndex
             ? m_nodes[parentIndex].children[0]
             : m_nodes[parentIndex].children[1]) = m_nodes.size();
      }
      m_nodes.push_back({aabbs[*first], *first, parentIndex});
      continue;
    }

    BoundingBox b = aabbs[*first];
    for (auto i = first + 1; i != last; ++i)
      b += aabbs[*i];

    vec3_t extent = b.ub - b.lb;
    auto dim = (extent[0] > extent[1] && extent[0] > extent[2] ? 0
                : extent[1] > extent[2]                        ? 1
                                                               : 2);

    auto m = first + n / 2;
    std::nth_element(first, m, last, [&centers, dim](index_t i, index_t j) {
      return centers[i][dim] < centers[j][dim];
    });

    index_t index = m_nodes.size();
    m_nodes.push_back({b, nullIndex, parentIndex});
    if (parentIndex != nullIndex) {
      (m_nodes[parentIndex].children[0] == nullIndex
           ? m_nodes[parentIndex].children[0]
           : m_nodes[parentIndex].children[1]) = index;
    }
    s.push({first, m, index});
    s.push({m, last, index});
  }
  m_rootIndex = 0;
}

inline real_t DynamicBVH::inheritedAreaDiff(BoundingBox aabb, index_t index) {
  aabb += m_nodes[index].aabb;
  real_t area = 0.f;
  index = m_nodes[index].parent;
  while (index != nullIndex) {
    aabb += m_nodes[index].aabb;
    area += aabb.area() - m_nodes[index].aabb.area();
    index = m_nodes[index].parent;
  }
  return area;
}

inline void DynamicBVH::refit(index_t index) {
  while (index != nullIndex) {
    m_nodes[index].aabb =
        BoundingBox::merge(m_nodes[m_nodes[index].children[0]].aabb,
                           m_nodes[m_nodes[index].children[1]].aabb);
    index = m_nodes[index].parent;
  }
}

inline void DynamicBVH::removeAndInsert(index_t index,
                                        const BoundingBox &newAABB) {
  assert(m_nodes[index].children[0] == nullIndex &&
         m_nodes[index].children[1] == nullIndex && index < m_nodes.size());
  if (m_nodes.size() == 1) {
    m_nodes[0].aabb = newAABB;
    return;
  }
  auto parent = m_nodes[index].parent;
  auto sibling =
      (index == m_nodes[parent].children[0] ? m_nodes[parent].children[1]
                                            : m_nodes[parent].children[0]);
  if (parent == m_rootIndex)
    m_rootIndex = sibling;
  auto grandparent = m_nodes[parent].parent;
  m_nodes[sibling].parent = grandparent;
  if (grandparent != nullIndex)
    (parent == m_nodes[grandparent].children[0]
         ? m_nodes[grandparent].children[0]
         : m_nodes[grandparent].children[1]) = sibling;
  refit(grandparent);

  insertLeafAt(index, parent, newAABB, m_nodes[index].objId);
}

inline void DynamicBVH::insertLeafAt(index_t leafIndex, index_t newParent,
                                     const BoundingBox &aabb, index_t objId) {
  m_nodes[leafIndex].aabb = aabb;
  m_nodes[leafIndex].objId = objId;

  auto sibling = m_rootIndex;
  real_t minCost = BoundingBox::merge(aabb, m_nodes[m_rootIndex].aabb).area();
  std::priority_queue<AreaCost> q;
  q.push({m_rootIndex, minCost});

  while (!q.empty()) {
    AreaCost curr = q.top();
    q.pop();
    if (curr.index != m_rootIndex) {
      curr.cost = BoundingBox::merge(aabb, m_nodes[curr.index].aabb).area() +
                  inheritedAreaDiff(aabb, curr.index);
      if (curr.cost < minCost) {
        minCost = curr.cost;
        sibling = curr.index;
      }
    }
    if (m_nodes[curr.index].children[0] == nullIndex) {
      continue;
    }
    real_t lowerBound =
        aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
    if (lowerBound < minCost) {
      q.push({m_nodes[curr.index].children[0], lowerBound});
      q.push({m_nodes[curr.index].children[1], lowerBound});
    }
  }

  auto oldParent = m_nodes[sibling].parent;
  m_nodes[newParent] = {BoundingBox::merge(aabb, m_nodes[sibling].aabb),
                        nullIndex, oldParent};

  if (oldParent != nullIndex) {
    (m_nodes[oldParent].children[0] == sibling
         ? m_nodes[oldParent].children[0]
         : m_nodes[oldParent].children[1]) = newParent;
    m_nodes[newParent].children[0] = sibling;
    m_nodes[newParent].children[1] = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
  } else {
    m_nodes[newParent].children[0] = sibling;
    m_nodes[newParent].children[1] = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    m_rootIndex = newParent;
  }

  refit(m_nodes[leafIndex].parent);
}

inline void DynamicBVH::insert(const BoundingBox &aabb, index_t objId) {
  auto leafIndex = m_nodes.size();
  m_nodes.push_back({aabb, objId});
  if (m_nodes.size() == 1) {
    m_rootIndex = leafIndex;
    return;
  }

  auto sibling = m_rootIndex;
  real_t minCost = BoundingBox::merge(aabb, m_nodes[m_rootIndex].aabb).area();
  std::priority_queue<AreaCost> q;
  q.push({m_rootIndex, minCost});

  while (!q.empty()) {
    AreaCost curr = q.top();
    q.pop();
    if (curr.index != m_rootIndex) {
      curr.cost = BoundingBox::merge(aabb, m_nodes[curr.index].aabb).area() +
                  inheritedAreaDiff(aabb, curr.index);
      if (curr.cost < minCost) {
        minCost = curr.cost;
        sibling = curr.index;
      }
    }
    if (m_nodes[curr.index].children[0] == nullIndex) {
      continue;
    }
    real_t lowerBound =
        aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
    if (lowerBound < minCost) {
      q.push({m_nodes[curr.index].children[0], lowerBound});
      q.push({m_nodes[curr.index].children[1], lowerBound});
    }
  }

  auto oldParent = m_nodes[sibling].parent;
  auto newParent = m_nodes.size();
  m_nodes.push_back(
      {BoundingBox::merge(aabb, m_nodes[sibling].aabb), nullIndex, oldParent});

  if (oldParent != nullIndex) {
    (m_nodes[oldParent].children[0] == sibling
         ? m_nodes[oldParent].children[0]
         : m_nodes[oldParent].children[1]) = newParent;
    m_nodes[newParent].children[0] = sibling;
    m_nodes[newParent].children[1] = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
  } else {
    m_nodes[newParent].children[0] = sibling;
    m_nodes[newParent].children[1] = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    m_rootIndex = newParent;
  }

  refit(m_nodes[leafIndex].parent);
}

inline index_t
DynamicBVH::nearestObject(const vec3_t &p,
                          const std::function<real_t(index_t)> &dist) const {
  real_t minDist = std::numeric_limits<real_t>::max();

  index_t currentNearestObj = nullIndex;

  std::stack<index_t> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    auto nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    real_t d = node.aabb.minDist(p);
    if (d >= minDist)
      continue;

    if (node.objId == nullIndex) {
      real_t d1 = m_nodes[node.children[0]].aabb.minDist(p);
      real_t d2 = m_nodes[node.children[1]].aabb.minDist(p);
      if (d1 < d2) {
        if (d2 < minDist)
          s.push(node.children[1]);
        if (d1 < minDist)
          s.push(node.children[0]);
      } else {
        if (d1 < minDist)
          s.push(node.children[0]);
        if (d2 < minDist)
          s.push(node.children[1]);
      }
    } else {
      real_t d = dist(node.objId);

      if (d < minDist) {
        minDist = d;
        currentNearestObj = node.objId;
      }
    }
  }

  return currentNearestObj;
}

inline index_t DynamicBVH::pointIntersectionFirst(
    const vec3_t &p, const std::function<bool(index_t)> &inside) const {
  index_t obj = nullIndex;

  std::stack<index_t> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    auto nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    if (!node.aabb.contains(p))
      continue;

    if (node.objId == nullIndex) {
      s.push(node.children[0]);
      s.push(node.children[1]);
    } else {
      if (inside(node.objId)) {
        obj = node.objId;
        break;
      }
    }
  }

  return obj;
}

template <typename OutputIt>
inline void
DynamicBVH::pointIntersectionAll(const vec3_t &p,
                                 const std::function<bool(index_t)> &inside,
                                 OutputIt first) const {
  std::stack<index_t> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    auto nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    if (!node.aabb.contains(p))
      continue;

    if (node.objId == nullIndex) {
      s.push(node.children[0]);
      s.push(node.children[1]);
    } else {
      if (inside(node.objId))
        *first++ = node.objId;
    }
  }
}

inline std::pair<index_t, real_t>
DynamicBVH::rayHit(const vec3_t &o, const vec3_t &d,
                   const std::function<real_t(index_t)> &hit, real_t minT,
                   real_t maxT) const {
  real_t currentMinT = maxT;

  index_t hitObj = nullIndex;

  std::stack<index_t> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    auto nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    if (node.objId == nullIndex) {
      real_t t1 = m_nodes[node.children[0]].aabb.rayHit(o, d);
      real_t t2 = m_nodes[node.children[1]].aabb.rayHit(o, d);

      if (t1 < t2) {
        if (t2 < currentMinT)
          s.push(node.children[1]);
        if (t1 < currentMinT)
          s.push(node.children[0]);
      } else {
        if (t1 < currentMinT)
          s.push(node.children[0]);
        if (t2 < currentMinT)
          s.push(node.children[1]);
      }
    } else {
      real_t t = hit(node.objId);
      if (t < currentMinT && t > minT) {
        currentMinT = t;
        hitObj = node.objId;
      }
    }
  }

  return {hitObj, currentMinT};
}

inline void
DynamicBVH::update(const std::function<BoundingBox(index_t)> &getNewAABB,
                   real_t enlargementFactor) {
  for (index_t i = 0; i < m_nodes.size(); ++i) {
    if (m_nodes[i].objId == nullIndex)
      continue;
    BoundingBox newAABB = getNewAABB(m_nodes[i].objId);
    if (!(newAABB <= m_nodes[i].aabb)) {
      newAABB.enlarge(enlargementFactor);
      removeAndInsert(i, newAABB);
    }
  }
}

} // namespace acc

#endif
