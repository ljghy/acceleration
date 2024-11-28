#ifndef ACC_STATIC_BVH_DEVICE_HPP_
#define ACC_STATIC_BVH_DEVICE_HPP_

#ifdef __CUDACC__

#include <acc/Config.hpp>
#include <acc/StaticBVHNode.hpp>

namespace acc {

struct StaticBVHDeviceView {
public:
  index_t maxDepth;
  index_t numNodes;
  const StaticBVHNode *nodes;
};

class StaticBVHDevice {
public:
  StaticBVHDevice() : m_maxDepth(0), m_numNodes(0), m_nodes(nullptr) {}
  StaticBVHDevice(const index_t maxDepth, const index_t numNodes,
                  const StaticBVHNode *nodes) {
    fromHost(maxDepth, numNodes, nodes);
  }

  void fromHost(const index_t maxDepth, const index_t numNodes,
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
    return StaticBVHDeviceView{m_maxDepth, m_numNodes, m_nodes};
  }

  ~StaticBVHDevice() { cudaFree(m_nodes); }

private:
  index_t m_maxDepth;
  index_t m_numNodes;
  StaticBVHNode *m_nodes;
};

} // namespace acc

#endif

#endif
