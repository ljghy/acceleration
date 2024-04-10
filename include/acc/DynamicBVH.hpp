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

template <typename ObjectType> class DynamicBVH {
public:
  DynamicBVH();

  void staticConstruct(
      size_t n,
      const std::function<std::pair<BoundingBox, ObjectType *>(size_t)>
          &getAABB);

  void insert(const BoundingBox &aabb, ObjectType *obj);
  void reserve(size_t n);
  void clear();

  ObjectType *
  nearestObject(const vec3_t &p,
                const std::function<real_t(ObjectType *)> &dist) const;

  ObjectType *
  pointIntersectionFirst(const vec3_t &p,
                         const std::function<bool(ObjectType *)> &inside) const;

  template <typename OutputIt>
  void pointIntersectionAll(const vec3_t &p,
                            const std::function<void(ObjectType *)> &inside,
                            OutputIt) const;

  std::pair<ObjectType *, real_t>
  rayHit(const vec3_t &o, const vec3_t &d,
         const std::function<real_t(ObjectType *)> &hit, real_t minT = {},
         real_t maxT = std::numeric_limits<real_t>::max()) const;

  void update(const std::function<BoundingBox(ObjectType *)> &getNewAABB,
              real_t enlargementFactor = real_t{1.2});

private:
  real_t inheritedAreaDiff(BoundingBox aabb, int index);

  void removeAndInsert(int index, const BoundingBox &newAABB);
  void insertLeafAt(int leafIndex, int newParent, const BoundingBox &aabb,
                    ObjectType *obj);
  void refit(int index);

private:
  struct AreaCost {
    int index;
    real_t cost;

    bool operator<(const AreaCost &other) const { return cost < other.cost; }
  };

private:
  std::vector<BVHNode<ObjectType>> m_nodes;
  int m_rootIndex;
};

template <typename ObjectType>
inline DynamicBVH<ObjectType>::DynamicBVH() : m_rootIndex(nullIndex) {}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::reserve(size_t n) {
  m_nodes.reserve(2 * n);
}

template <typename ObjectType> inline void DynamicBVH<ObjectType>::clear() {
  m_nodes.clear();
  m_rootIndex = nullIndex;
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::staticConstruct(
    size_t n, const std::function<std::pair<BoundingBox, ObjectType *>(size_t)>
                  &getAABB) {
  if (n == 0)
    return;

  clear();
  reserve(n);

  std::vector<size_t> indexOrder(n);
  std::iota(indexOrder.begin(), indexOrder.end(), 0u);

  std::vector<std::pair<BoundingBox, ObjectType *>> aabbs(n);
  std::transform(indexOrder.begin(), indexOrder.end(), aabbs.begin(), getAABB);

  std::vector<vec3_t> centers(aabbs.size());
  std::transform(aabbs.begin(), aabbs.end(), centers.begin(),
                 [](const auto &pair) { return pair.first.center(); });

  struct State {
    std::vector<size_t>::iterator first;
    std::vector<size_t>::iterator last;
    int parentIndex;
  };

  std::stack<State> s;
  s.emplace(indexOrder.begin(), indexOrder.end(), nullIndex);

  while (!s.empty()) {
    auto [first, last, parentIndex] = s.top();
    s.pop();

    if (first == last)
      continue;
    size_t n = std::distance(first, last);
    if (n == 1) {
      m_nodes.push_back(
          {aabbs[*first].first, aabbs[*first].second, parentIndex});
      if (parentIndex != nullIndex) {
        (m_nodes[parentIndex].child1 == nullIndex
             ? m_nodes[parentIndex].child1
             : m_nodes[parentIndex].child2) =
            static_cast<int>(m_nodes.size()) - 1;
      }
      continue;
    }

    BoundingBox b = aabbs[*first].first;
    for (auto i = first + 1; i != last; ++i)
      b += aabbs[*i].first;

    vec3_t extent = b.ub - b.lb;
    int dim = (extent[0] > extent[1] && extent[0] > extent[2] ? 0
               : extent[1] > extent[2]                        ? 1
                                                              : 2);

    auto m = first + n / 2;
    std::nth_element(first, m, last, [&centers, dim](size_t i, size_t j) {
      return centers[i][dim] < centers[j][dim];
    });

    int index = static_cast<int>(m_nodes.size());
    m_nodes.push_back(BVHNode<ObjectType>{b, nullptr, parentIndex});
    if (parentIndex != nullIndex) {
      (m_nodes[parentIndex].child1 == nullIndex ? m_nodes[parentIndex].child1
                                                : m_nodes[parentIndex].child2) =
          index;
    }
    s.emplace(first, m, index);
    s.emplace(m, last, index);
  }
  m_rootIndex = 0;
}

template <typename ObjectType>
inline real_t DynamicBVH<ObjectType>::inheritedAreaDiff(BoundingBox aabb,
                                                        int index) {
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

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::refit(int index) {
  while (index != nullIndex) {
    int child1 = m_nodes[index].child1;
    int child2 = m_nodes[index].child2;
    m_nodes[index].aabb =
        BoundingBox::merge(m_nodes[child1].aabb, m_nodes[child2].aabb);
    index = m_nodes[index].parent;
  }
}

template <typename ObjectType>
inline void
DynamicBVH<ObjectType>::removeAndInsert(int index, const BoundingBox &newAABB) {
  assert(m_nodes[index].child1 == nullIndex &&
         m_nodes[index].child2 == nullIndex && index < m_nodes.size());
  if (m_nodes.size() == 1) {
    m_nodes[0].aabb = newAABB;
    return;
  }
  int parent = m_nodes[index].parent;
  int sibling = (index == m_nodes[parent].child1 ? m_nodes[parent].child2
                                                 : m_nodes[parent].child1);
  if (parent == m_rootIndex)
    m_rootIndex = sibling;
  int grandparent = m_nodes[parent].parent;
  m_nodes[sibling].parent = grandparent;
  if (grandparent != nullIndex)
    (parent == m_nodes[grandparent].child1 ? m_nodes[grandparent].child1
                                           : m_nodes[grandparent].child2) =
        sibling;
  refit(grandparent);

  insertLeafAt(index, parent, newAABB, m_nodes[index].object);
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::insertLeafAt(int leafIndex, int newParent,
                                                 const BoundingBox &aabb,
                                                 ObjectType *obj) {
  m_nodes[leafIndex].aabb = aabb;
  m_nodes[leafIndex].object = obj;

  int sibling = m_rootIndex;
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
    if (m_nodes[curr.index].child1 == nullIndex) {
      continue;
    }
    real_t lowerBound =
        aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
    if (lowerBound < minCost) {
      q.push({m_nodes[curr.index].child1, lowerBound});
      q.push({m_nodes[curr.index].child2, lowerBound});
    }
  }

  int oldParent = m_nodes[sibling].parent;
  m_nodes[newParent] = {BoundingBox::merge(aabb, m_nodes[sibling].aabb),
                        nullptr, oldParent};

  if (oldParent != nullIndex) {
    (m_nodes[oldParent].child1 == sibling ? m_nodes[oldParent].child1
                                          : m_nodes[oldParent].child2) =
        newParent;
    m_nodes[newParent].child1 = sibling;
    m_nodes[newParent].child2 = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
  } else {
    m_nodes[newParent].child1 = sibling;
    m_nodes[newParent].child2 = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    m_rootIndex = newParent;
  }

  refit(m_nodes[leafIndex].parent);
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::insert(const BoundingBox &aabb,
                                           ObjectType *obj) {
  int leafIndex = static_cast<int>(m_nodes.size());
  m_nodes.push_back({aabb, obj});
  if (m_nodes.size() == 1) {
    m_rootIndex = leafIndex;
    return;
  }

  int sibling = m_rootIndex;
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
    if (m_nodes[curr.index].child1 == nullIndex) {
      continue;
    }
    real_t lowerBound =
        aabb.area() + curr.cost - m_nodes[curr.index].aabb.area();
    if (lowerBound < minCost) {
      q.push({m_nodes[curr.index].child1, lowerBound});
      q.push({m_nodes[curr.index].child2, lowerBound});
    }
  }

  int oldParent = m_nodes[sibling].parent;
  int newParent = static_cast<int>(m_nodes.size());
  m_nodes.push_back(
      {BoundingBox::merge(aabb, m_nodes[sibling].aabb), nullptr, oldParent});

  if (oldParent != nullIndex) {
    (m_nodes[oldParent].child1 == sibling ? m_nodes[oldParent].child1
                                          : m_nodes[oldParent].child2) =
        newParent;
    m_nodes[newParent].child1 = sibling;
    m_nodes[newParent].child2 = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
  } else {
    m_nodes[newParent].child1 = sibling;
    m_nodes[newParent].child2 = leafIndex;
    m_nodes[sibling].parent = m_nodes[leafIndex].parent = newParent;
    m_rootIndex = newParent;
  }

  refit(m_nodes[leafIndex].parent);
}

template <typename ObjectType>
inline ObjectType *DynamicBVH<ObjectType>::nearestObject(
    const vec3_t &p, const std::function<real_t(ObjectType *)> &dist) const {
  real_t minDist = std::numeric_limits<real_t>::max();

  ObjectType *currentNearestObj = nullptr;

  std::stack<int> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    int nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    real_t d = node.aabb.minDist(p);
    if (d >= minDist)
      continue;

    if (node.object == nullptr) {
      real_t d1 = m_nodes[node.child1].aabb.minDist(p);
      real_t d2 = m_nodes[node.child2].aabb.minDist(p);
      if (d1 < d2) {
        if (d2 < minDist)
          s.push(node.child2);
        if (d1 < minDist)
          s.push(node.child1);
      } else {
        if (d1 < minDist)
          s.push(node.child1);
        if (d2 < minDist)
          s.push(node.child2);
      }
    } else {
      real_t d = dist(node.object);

      if (d < minDist) {
        minDist = d;
        currentNearestObj = node.object;
      }
    }
  }

  return currentNearestObj;
}

template <typename ObjectType>
inline ObjectType *DynamicBVH<ObjectType>::pointIntersectionFirst(
    const vec3_t &p, const std::function<bool(ObjectType *)> &inside) const {
  ObjectType *obj = nullptr;

  std::stack<int> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    int nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    if (!node.aabb.contains(p))
      continue;

    if (node.object == nullptr) {
      s.push(node.child1);
      s.push(node.child2);
    } else {
      if (inside(node.object)) {
        obj = node.object;
        break;
      }
    }
  }

  return obj;
}

template <typename ObjectType>
template <typename OutputIt>
inline void DynamicBVH<ObjectType>::pointIntersectionAll(
    const vec3_t &p, const std::function<void(ObjectType *)> &inside,
    OutputIt first) const {
  std::stack<int> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    int nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    if (!node.aabb.contains(p))
      continue;

    if (node.object == nullptr) {
      s.push(node.child1);
      s.push(node.child2);
    } else {
      if (inside(node.object))
        *first++ = node.object;
    }
  }
}

template <typename ObjectType>
inline std::pair<ObjectType *, real_t>
DynamicBVH<ObjectType>::rayHit(const vec3_t &o, const vec3_t &d,
                               const std::function<real_t(ObjectType *)> &hit,
                               real_t minT, real_t maxT) const {
  real_t currentMinT = maxT;

  ObjectType *hitObj = nullptr;

  std::stack<int> s;
  s.push(m_rootIndex);

  while (!s.empty()) {
    int nodeIndex = s.top();
    s.pop();

    if (nodeIndex == nullIndex)
      continue;

    const auto &node = m_nodes[nodeIndex];

    if (node.object == nullptr) {
      real_t t1 = m_nodes[node.child1].aabb.rayHit(o, d);
      real_t t2 = m_nodes[node.child2].aabb.rayHit(o, d);

      if (t1 < t2) {
        if (t2 < currentMinT)
          s.push(node.child2);
        if (t1 < currentMinT)
          s.push(node.child1);
      } else {
        if (t1 < currentMinT)
          s.push(node.child1);
        if (t2 < currentMinT)
          s.push(node.child2);
      }
    } else {
      real_t t = hit(node.object);
      if (t < currentMinT && t > minT) {
        currentMinT = t;
        hitObj = node.object;
      }
    }
  }

  return {hitObj, currentMinT};
}

template <typename ObjectType>
inline void DynamicBVH<ObjectType>::update(
    const std::function<BoundingBox(ObjectType *)> &getNewAABB,
    real_t enlargementFactor) {
  for (size_t i = 0; i < m_nodes.size(); ++i) {
    if (m_nodes[i].object == nullptr)
      continue;
    BoundingBox newAABB = getNewAABB(m_nodes[i].object);
    if (!(newAABB <= m_nodes[i].aabb)) {
      newAABB.enlarge(enlargementFactor);
      removeAndInsert(i, newAABB);
    }
  }
}

} // namespace acc

#endif
