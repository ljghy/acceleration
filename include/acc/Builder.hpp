#ifndef ACC_BUILDER_HPP_
#define ACC_BUILDER_HPP_

#include <algorithm>

#include <acc/BoundingBox.hpp>
#include <acc/Config.hpp>
#include <acc/MortonCode.hpp>

#ifndef ACC_DONT_PARALLELIZE
#include <execution>
#endif

#if __cpp_lib_bitops
#include <bit>
namespace acc {
template <typename T> inline constexpr int clz(T x) {
  return std::countl_zero(x);
}
} // namespace acc
#else
namespace acc {
template <typename T> inline constexpr int clz(T x) {
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_clz(x);
#else
  int count = sizeof(T) * 8;
  while (x) {
    x >>= 1;
    --count;
  }
  return count;
#endif
}
} // namespace acc

#endif

namespace acc {

class MortonCodeBVHBuilder {
public:
  template <typename GetAABB, typename BVHNodeType>
  std::vector<BVHNodeType> build(const index_t n, const GetAABB getAABB);

private:
  template <typename BVHNodeType>
  index_t buildRecursive(std::vector<BVHNodeType> &nodes, const index_t first,
                         const index_t last);

  index_t findSplit(const index_t first, const index_t last);

private:
  std::vector<BoundingBox> m_aabbs;
  std::vector<MortonCode> m_mortonCodes;
};

template <typename GetAABB, typename BVHNodeType>
std::vector<BVHNodeType> MortonCodeBVHBuilder::build(const index_t n,
                                                     const GetAABB getAABB) {
  std::vector<BVHNodeType> nodes;
  if (n == 0)
    return nodes;

  nodes.reserve(2 * n);

  m_aabbs.resize(n);
  std::for_each(
#ifndef ACC_DONT_PARALLELIZE
      std::execution::par,
#endif
      m_aabbs.begin(), m_aabbs.end(), [&](BoundingBox &aabb) {
        const int i = &aabb - m_aabbs.data();
        aabb = getAABB(i);
      });

  BoundingBox rootAABB;
  rootAABB.init();

  rootAABB = std::reduce(
#ifndef ACC_DONT_PARALLELIZE
      std::execution::par,
#endif
      m_aabbs.begin(), m_aabbs.end(), rootAABB);

  m_mortonCodes.resize(n);
  std::transform(
#ifndef ACC_DONT_PARALLELIZE
      std::execution::par,
#endif
      m_aabbs.begin(), m_aabbs.end(), m_mortonCodes.begin(),
      [&](const BoundingBox &aabb) {
        const int i = &aabb - m_aabbs.data();
        return MortonCode(i, prod(aabb.center() - rootAABB.lb,
                                  inv(rootAABB.ub - rootAABB.lb)));
      });

  MortonCode::radixSort(m_mortonCodes);

  nodes.emplace_back(rootAABB);
  buildRecursive(nodes, 0, n - 1);

  return nodes;
}

template <typename BVHNodeType>
index_t MortonCodeBVHBuilder::buildRecursive(std::vector<BVHNodeType> &nodes,
                                             const index_t first,
                                             const index_t last) {
  if (first == last) {
    const auto i = m_mortonCodes[first].index;
    nodes.emplace_back(m_aabbs[i], i);
  } else {
    const auto split = findSplit(first, last);
    const auto child0 = buildRecursive(nodes, first, split);
    const auto child1 = buildRecursive(nodes, split + 1, last);
    auto &node =
        (first == 0 && last + 1 == m_aabbs.size())
            ? nodes[0]
            : nodes.emplace_back(nodes[child0].aabb + nodes[child1].aabb);
    node.children[0] = child0;
    node.children[1] = child1;
  }
  return nodes.size() - 1;
}

inline index_t MortonCodeBVHBuilder::findSplit(const index_t first,
                                               const index_t last) {
  const auto firstCode = m_mortonCodes[first].code;
  const auto lastCode = m_mortonCodes[last].code;
  if (firstCode == lastCode)
    return (first + last) / 2;

  const auto commonPrefix = clz(firstCode ^ lastCode);
  auto split = first;
  index_t step = last - first;

  do {
    step = (step + 1) >> 1;
    const index_t newSplit = split + step;

    if (newSplit < last) {
      const auto splitCode = m_mortonCodes[newSplit].code;
      const auto splitPrefix = clz(firstCode ^ splitCode);
      if (splitPrefix > commonPrefix)
        split = newSplit;
    }
  } while (step > 1);

  return split;
}

} // namespace acc

#endif
