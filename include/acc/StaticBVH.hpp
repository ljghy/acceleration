#ifndef ACC_STATIC_BVH_HPP_
#define ACC_STATIC_BVH_HPP_

#include <stack>
#include <vector>

#include <acc/Builder.hpp>
#include <acc/StaticBVHNode.hpp>

namespace acc {

class StaticBVH {
public:
  StaticBVH() = default;

  template <typename GetAABB, typename Builder = MortonCodeBVHBuilder>
  StaticBVH(const index_t n, GetAABB getAABB) {
    build<GetAABB, Builder>(n, getAABB);
  }

  template <typename GetAABB, typename Builder = MortonCodeBVHBuilder>
  void build(const index_t n, GetAABB getAABB) {
    m_nodes = Builder{}.template build<GetAABB, StaticBVHNode>(n, getAABB);
  }

  index_t maxDepth() const;
  index_t numNodes() const { return m_nodes.size(); }
  const auto &nodes() const { return m_nodes; }
  const StaticBVHNode *data() const { return m_nodes.data(); }

  BoundingBox rootAABB() const {
    if (!m_nodes.empty())
      return m_nodes[0].aabb;
    BoundingBox b;
    b.init();
    return b;
  }

private:
  std::vector<StaticBVHNode> m_nodes;
};

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

} // namespace acc

#endif
