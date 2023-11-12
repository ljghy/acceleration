#ifndef ACC_KD_TREE_HPP_
#define ACC_KD_TREE_HPP_

#include <algorithm>
#include <limits>
#include <numeric>
#include <vector>

#include <Eigen/Core>

#include <acc/BoundingBox.hpp>

namespace acc {

template <typename VertexType> struct KDNode {
  VertexType *vert;

  int cutDim;

  KDNode *left = nullptr;
  KDNode *right = nullptr;
};

template <typename VertexType> class KDTree {
  using VertexListType = std::vector<VertexType>;
  using KDNodeType = KDNode<VertexType>;

public:
  ~KDTree();

  void construct(VertexListType &vertices);
  void free();

  VertexType *nearestNeighbor(Eigen::Vector3d q) const;

private:
  static KDNodeType *construct(VertexListType &vertices, int cutDim,
                               std::vector<size_t>::iterator first,
                               std::vector<size_t>::iterator last);
  static void free(KDNodeType *);

  static void nearestNeighbor(KDNodeType *node, Eigen::Vector3d q,
                              VertexType *&nearestVert, double &minSqrDist,
                              BoundingBox bb);

private:
  KDNodeType *m_root = nullptr;
  BoundingBox m_boundingBox;
};

template <typename VertexType>
inline void KDTree<VertexType>::construct(VertexListType &vertices) {
  std::vector<size_t> indexOrder(vertices.size());
  std::iota(indexOrder.begin(), indexOrder.end(), 0u);
  m_root = construct(vertices, 0, indexOrder.begin(), indexOrder.end());

  m_boundingBox.lb.array() = std::numeric_limits<double>::max();
  m_boundingBox.ub.array() = std::numeric_limits<double>::lowest();
  for (const auto &vert : vertices) {
    m_boundingBox.lb = m_boundingBox.lb.cwiseMin(vert.position);
    m_boundingBox.ub = m_boundingBox.ub.cwiseMax(vert.position);
  }
}

template <typename VertexType>
inline auto KDTree<VertexType>::construct(VertexListType &vertices, int cutDim,
                                          std::vector<size_t>::iterator first,
                                          std::vector<size_t>::iterator last)
    -> KDNodeType * {
  if (first == last)
    return nullptr;

  if (std::distance(first, last) == 1)
    return new KDNodeType{&vertices[*first], cutDim, nullptr, nullptr};

  auto m = first + std::distance(first, last) / 2;
  std::nth_element(first, m, last, [&vertices, cutDim](size_t i, size_t j) {
    return vertices[i].position(cutDim) < vertices[j].position(cutDim);
  });

  return new KDNodeType{&vertices[*m], cutDim,
                        construct(vertices, (cutDim + 1) % 3, first, m),
                        construct(vertices, (cutDim + 1) % 3, m, last)};
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
inline VertexType *
KDTree<VertexType>::nearestNeighbor(Eigen::Vector3d q) const {
  VertexType *nearestVert = nullptr;
  double minSqrDist = std::numeric_limits<double>::max();

  nearestNeighbor(m_root, q, nearestVert, minSqrDist, m_boundingBox);
  return nearestVert;
}

template <typename VertexType>
inline void
KDTree<VertexType>::nearestNeighbor(KDNodeType *node, Eigen::Vector3d q,
                                    VertexType *&nearestVert,
                                    double &minSqrDist, BoundingBox bb) {
  if (node == nullptr || bb.minSqrDist(q) > minSqrDist)
    return;

  auto p = node->vert->position;
  double sqrDist = (p - q).squaredNorm();
  if (sqrDist < minSqrDist) {
    nearestVert = node->vert;
    minSqrDist = sqrDist;
  }

  int cd = node->cutDim;
  if (q(cd) < p(cd)) {
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
