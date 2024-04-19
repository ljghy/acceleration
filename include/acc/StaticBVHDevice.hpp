#ifndef ACC_STATIC_BVH_DEVICE_HPP_
#define ACC_STATIC_BVH_DEVICE_HPP_

#include <acc/FixedStack.hpp>
#include <acc/StaticBVHNode.hpp>

namespace acc {

class StaticBVHDeviceView {
public:
  StaticBVHDeviceView(index_t maxDepth = 0,
                      const StaticBVHNode *nodes = nullptr)
      : m_maxDepth(maxDepth), m_nodes(nodes) {}

  using RayHitFunc = real_t (*)(void *, index_t, const vec3_t &,
                                const vec3_t &);
  using DiscardFunc = bool (*)(void *, index_t, const vec3_t &, const vec3_t &,
                               real_t);

  using PointIntersectionFunc = bool (*)(void *, index_t, const vec3_t &);

  using DistFunc = real_t (*)(void *, index_t, const vec3_t &);

  template <typename StackType = FixedStack<index_t, 32>>
  __device__ void rayHit(const vec3_t &o, const vec3_t &d, RayHitFunc hit,
                         void *userData, index_t *, real_t *, real_t minT = {},
                         real_t maxT = real_t_max) const;

  template <typename StackType = FixedStack<index_t, 32>>
  __device__ void rayHit(const vec3_t &o, const vec3_t &d, RayHitFunc hit,
                         DiscardFunc discard, void *userData, index_t *,
                         real_t *, real_t minT = {},
                         real_t maxT = real_t_max) const;

  template <typename StackType = FixedStack<index_t, 32>>
  __device__ void pointIntersectionFirst(const vec3_t &p,
                                         PointIntersectionFunc inside,
                                         void *userData, index_t *objId) const;

  template <typename StackType = FixedStack<index_t, 32>>
  __device__ void nearestObject(const vec3_t &p, DistFunc dist, void *userData,
                                index_t *objId, real_t *d) const;

private:
  index_t m_maxDepth;
  const StaticBVHNode *m_nodes;
};

class StaticBVHDevice {
public:
  StaticBVHDevice() : m_maxDepth(0), m_numNodes(0), m_nodes(nullptr) {}
  StaticBVHDevice(index_t maxDepth, index_t numNodes,
                  const StaticBVHNode *nodes) {
    fromHost(maxDepth, numNodes, nodes);
  }

  void fromHost(index_t maxDepth, index_t numNodes,
                const StaticBVHNode *nodes) {
    m_maxDepth = maxDepth;
    m_numNodes = numNodes;
    if (m_nodes)
      cudaFree(m_nodes);
    cudaMalloc(&m_nodes, numNodes * sizeof(StaticBVHNode));
    cudaMemcpy(m_nodes, nodes, numNodes * sizeof(StaticBVHNode),
               cudaMemcpyHostToDevice);
  }

  StaticBVHDevice(const StaticBVHDevice &) = delete;
  StaticBVHDevice &operator=(const StaticBVHDevice &) = delete;

  StaticBVHDevice(StaticBVHDevice &&other) noexcept
      : m_maxDepth(other.m_maxDepth), m_numNodes(other.m_numNodes),
        m_nodes(other.m_nodes) {
    other.m_nodes = nullptr;
  }
  StaticBVHDevice &operator=(StaticBVHDevice &&other) noexcept {
    if (this != &other) {
      m_maxDepth = other.m_maxDepth;
      m_numNodes = other.m_numNodes;
      m_nodes = other.m_nodes;
      other.m_nodes = nullptr;
    }
    return *this;
  }

  StaticBVHDeviceView view() const {
    return StaticBVHDeviceView{m_maxDepth, m_nodes};
  }

  ~StaticBVHDevice() { cudaFree(m_nodes); }

private:
  index_t m_maxDepth;
  index_t m_numNodes;
  StaticBVHNode *m_nodes;
};

template <typename StackType>
inline __device__ void
StaticBVHDeviceView::rayHit(const vec3_t &o, const vec3_t &d, RayHitFunc hit,
                            void *userData, index_t *currObjId,
                            real_t *currMinT, real_t minT, real_t maxT) const {

  *currObjId = nullIndex;
  *currMinT = maxT;

  StackType stack(m_maxDepth);
  stack.push(index_t{});

  while (!stack.empty()) {
    auto i = stack.top();
    stack.pop();

    const auto &node = m_nodes[i];
    auto objId = node.objId();
    if (objId == nullIndex) {
      real_t t0 = m_nodes[node.children[0]].aabb.rayHit(o, d);
      real_t t1 = m_nodes[node.children[1]].aabb.rayHit(o, d);

      if (t0 < t1) {
        if (t1 < *currMinT)
          stack.push(node.children[1]);
        if (t0 < *currMinT)
          stack.push(node.children[0]);
      } else {
        if (t0 < *currMinT)
          stack.push(node.children[0]);
        if (t1 < *currMinT)
          stack.push(node.children[1]);
      }
    } else {
      real_t t = hit(userData, objId, o, d);
      if (t < *currMinT && t > minT) {
        *currMinT = t;
        *currObjId = objId;
      }
    }
  }
}

template <typename StackType>
inline __device__ void
StaticBVHDeviceView::rayHit(const vec3_t &o, const vec3_t &d, RayHitFunc hit,
                            DiscardFunc discard, void *userData,
                            index_t *currObjId, real_t *currMinT, real_t minT,
                            real_t maxT) const {

  *currObjId = nullIndex;
  *currMinT = maxT;

  StackType stack(m_maxDepth);
  stack.push(index_t{});

  while (!stack.empty()) {
    auto i = stack.top();
    stack.pop();

    const auto &node = m_nodes[i];
    auto objId = node.objId();
    if (objId == nullIndex) {
      real_t t0 = m_nodes[node.children[0]].aabb.rayHit(o, d);
      real_t t1 = m_nodes[node.children[1]].aabb.rayHit(o, d);

      if (t0 < t1) {
        if (t1 < *currMinT)
          stack.push(node.children[1]);
        if (t0 < *currMinT)
          stack.push(node.children[0]);
      } else {
        if (t0 < *currMinT)
          stack.push(node.children[0]);
        if (t1 < *currMinT)
          stack.push(node.children[1]);
      }
    } else {
      real_t t = hit(userData, objId, o, d);
      if (t < *currMinT && t > minT && !discard(userData, objId, o, d, t)) {
        *currMinT = t;
        *currObjId = objId;
      }
    }
  }
}

template <typename StackType>
__device__ void StaticBVHDeviceView::pointIntersectionFirst(
    const vec3_t &p, PointIntersectionFunc inside, void *userData,
    index_t *objId) const {
  *objId = nullIndex;

  StackType stack(m_maxDepth);
  stack.push(index_t{});

  while (!stack.empty()) {
    auto i = stack.top();
    stack.pop();

    const auto &node = m_nodes[i];
    index_t id = node.objId();
    if (id == nullIndex) {
      if (m_nodes[node.children[0]].aabb.contains(p))
        stack.push(node.children[0]);
      if (m_nodes[node.children[1]].aabb.contains(p))
        stack.push(node.children[1]);
    } else {
      if (inside(userData, id, p)) {
        *objId = id;
        return;
      }
    }
  }
}

template <typename StackType>
__device__ void
StaticBVHDeviceView::nearestObject(const vec3_t &p, DistFunc dist,
                                   void *userData, index_t *objId,
                                   real_t *d) const {
  *objId = nullIndex;
  *d = real_t_max;

  StackType stack(m_maxDepth);
  stack.push(index_t{});

  while (!stack.empty()) {
    auto i = stack.top();
    stack.pop();

    const auto &node = m_nodes[i];
    index_t id = node.objId();
    if (id == nullIndex) {
      real_t d0 = m_nodes[node.children[0]].aabb.pointDistance(p);
      real_t d1 = m_nodes[node.children[1]].aabb.pointDistance(p);

      if (d0 < d1) {
        if (d1 < *d)
          stack.push(node.children[1]);
        if (d0 < *d)
          stack.push(node.children[0]);
      } else {
        if (d0 < *d)
          stack.push(node.children[0]);
        if (d1 < *d)
          stack.push(node.children[1]);
      }
    } else {
      real_t d0 = dist(userData, id, p);
      if (d0 < *d) {
        *d = d0;
        *objId = id;
      }
    }
  }
}

} // namespace acc

#endif
