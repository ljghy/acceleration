#ifndef ACC_KD_TREE_HPP_
#define ACC_KD_TREE_HPP_

#include <algorithm>
#include <functional>
#include <iterator>
#include <limits>
#include <numeric>
#include <vector>

#include <acc/BoundingBox.hpp>

namespace acc {

template <typename VertexType> struct KDNode {
  VertexType *vert;

  int cutDim;

  KDNode *left = nullptr;
  KDNode *right = nullptr;
};

template <typename VertexType> class KDTree {
  using KDNodeType = KDNode<VertexType>;

public:
  ~KDTree();

  template <typename InputIt>
  void construct(InputIt first, InputIt last,
                 const std::function<vec3_t(const VertexType &)> &getPos);
  void free();

  VertexType *nearestNeighbor(vec3_t q) const;

private:
  template <typename InputIt>
  static KDNodeType *
  construct(InputIt first, InputIt last,
            const std::function<vec3_t(const VertexType &)> &getPos, int cutDim,
            std::vector<size_t>::iterator firstIndex,
            std::vector<size_t>::iterator lastIndex);
  static void free(KDNodeType *);

  static void nearestNeighbor(KDNodeType *node, vec3_t q,
                              VertexType *&nearestVert, real_t &minSqrDist,
                              BoundingBox bb);

private:
  KDNodeType *m_root = nullptr;
  BoundingBox m_boundingBox;
};

template <typename VertexType>
template <typename InputIt>
inline void KDTree<VertexType>::construct(
    InputIt first, InputIt last,
    const std::function<vec3_t(const VertexType &)> &getPos) {
  std::vector<size_t> indexOrder(std::distance(first, last));
  std::iota(indexOrder.begin(), indexOrder.end(), size_t{});
  m_root =
      construct(first, last, getPos, 0, indexOrder.begin(), indexOrder.end());

  m_boundingBox.init();
  std::for_each(first, last, [this, &getPos](const auto &vert) {
    m_boundingBox.update(getPos(vert));
  });
}

template <typename VertexType>
template <typename InputIt>
inline auto KDTree<VertexType>::construct(
    InputIt first, InputIt last,
    const std::function<vec3_t(const VertexType &)> &getPos, int cutDim,
    std::vector<size_t>::iterator firstIndex,
    std::vector<size_t>::iterator lastIndex) -> KDNodeType * {

  auto n = std::distance(firstIndex, lastIndex);
  if (n == 0)
    return nullptr;

  if (n == 1)
    return new KDNodeType{&*(first + *firstIndex), cutDim, nullptr, nullptr};

  auto m = firstIndex + n / 2;
  std::nth_element(firstIndex, m, lastIndex,
                   [first, last, cutDim, &getPos](size_t i, size_t j) {
                     return getPos(*(first + i))[cutDim] <
                            getPos(*(first + j))[cutDim];
                   });

  return new KDNodeType{
      &*(first + *m), cutDim,
      construct(first, last, getPos, (cutDim + 1) % 3, firstIndex, m),
      construct(first, last, getPos, (cutDim + 1) % 3, m, lastIndex)};
}

template <typename VertexType> inline void KDTree<VertexType>::free() {
  free(m_root);
  m_root = nullptr;
}

template <typename VertexType>
inline void KDTree<VertexType>::free(KDNodeType *node) {
  if (node == nullptr)
    return;
  free(node->left);
  free(node->right);
  delete node;
}

template <typename VertexType> inline KDTree<VertexType>::~KDTree() { free(); }

template <typename VertexType>
inline VertexType *KDTree<VertexType>::nearestNeighbor(vec3_t q) const {
  VertexType *nearestVert = nullptr;
  real_t minSqrDist = real_t_max;

  nearestNeighbor(m_root, q, nearestVert, minSqrDist, m_boundingBox);
  return nearestVert;
}

template <typename VertexType>
inline void KDTree<VertexType>::nearestNeighbor(KDNodeType *node, vec3_t q,
                                                VertexType *&nearestVert,
                                                real_t &minSqrDist,
                                                BoundingBox bb) {
  if (node == nullptr || bb.minSqrDist(q) > minSqrDist)
    return;

  auto p = node->vert->position;
  real_t sqrDist = sqrNorm(p - q);
  if (sqrDist < minSqrDist) {
    nearestVert = node->vert;
    minSqrDist = sqrDist;
  }

  int cd = node->cutDim;
  if (q[cd] < p[cd]) {
    nearestNeighbor(node->left, q, nearestVert, minSqrDist,
                    bb.trimLeft(cd, p(cd)));
    nearestNeighbor(node->right, q, nearestVert, minSqrDist,
                    bb.trimRight(cd, p(cd)));
  } else {
    nearestNeighbor(node->right, q, nearestVert, minSqrDist,
                    bb.trimRight(cd, p(cd)));
    nearestNeighbor(node->left, q, nearestVert, minSqrDist,
                    bb.trimLeft(cd, p(cd)));
  }
}

} // namespace acc

#endif
