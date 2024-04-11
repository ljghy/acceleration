#ifndef ACC_STATIC_BVH_DEVICE_HPP_
#define ACC_STATIC_BVH_DEVICE_HPP_

#include <acc/DynamicArrayDevice.hpp>
#include <acc/StaticBVHNode.hpp>

namespace acc {

class StaticBVHDeviceView {
  friend class StaticBVHDevice;

  StaticBVHDeviceView(index_t maxDepth, const StaticBVHNode *nodes)
      : m_maxDepth(maxDepth), m_nodes(nodes) {}

public:
  __device__ void rayHit(const vec3_t &o, const vec3_t &d,
                         real_t (*hit)(index_t, const vec3_t &, const vec3_t &),
                         index_t *, real_t *, real_t minT = {},
                         real_t maxT = real_t_max) const;

private:
  index_t m_maxDepth;
  const StaticBVHNode *m_nodes;
};

class StaticBVHDevice {
public:
  StaticBVHDevice(index_t maxDepth, index_t numNodes,
                  const StaticBVHNode *nodes)
      : m_maxDepth(maxDepth), m_numNodes(numNodes) {
    cudaMalloc((void **)&m_nodes, numNodes * sizeof(StaticBVHNode));
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

inline __device__ void StaticBVHDeviceView::rayHit(
    const vec3_t &o, const vec3_t &d,
    real_t (*hit)(index_t, const vec3_t &, const vec3_t &), index_t *currObjId,
    real_t *currMinT, real_t minT, real_t maxT) const {

  *currObjId = nullIndex;
  *currMinT = maxT;

  DynamicArrayDevice<index_t> stack(m_maxDepth);
  stack.push_back(index_t{});

  while (!stack.empty()) {
    auto i = stack.back();
    stack.pop_back();

    const auto &node = m_nodes[i];
    auto objId = node.objId();
    if (objId == nullIndex) {
      real_t t0 = m_nodes[node.children[0]].aabb.rayHit(o, d);
      real_t t1 = m_nodes[node.children[1]].aabb.rayHit(o, d);

      if (t0 < t1) {
        if (t1 < *currMinT)
          stack.push_back(node.children[1]);
        if (t0 < *currMinT)
          stack.push_back(node.children[0]);
      } else {
        if (t0 < *currMinT)
          stack.push_back(node.children[0]);
        if (t1 < *currMinT)
          stack.push_back(node.children[1]);
      }
    } else {
      real_t t = hit(objId, o, d);
      if (t < *currMinT && t > minT) {
        *currMinT = t;
        *currObjId = objId;
      }
    }
  }
}

} // namespace acc

#endif
