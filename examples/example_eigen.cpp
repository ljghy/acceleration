#include <chrono>
#include <iostream>

#include <Eigen/Core>

#include <tbb/parallel_for.h>

#include <acc/Query.hpp>
#include <acc/StaticBVH.hpp>

int main() {

  acc::StaticBVH bvh;

  const int nPoints = 1000000;
  Eigen::Matrix<double, -1, 3, Eigen::RowMajor> points(nPoints, 3);
  points.setRandom();

  {

    auto start = std::chrono::high_resolution_clock::now();

    bvh.build(nPoints, [&](int i) {
      return acc::BoundingBox(points.row(i).array() - 1e-5,
                              points.row(i).array() + 1e-5);
    });

    auto end = std::chrono::high_resolution_clock::now();

    std::cout << "BVH built in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << " ms for " << nPoints << " points" << std::endl;

    std::cout << "BVH depth: " << bvh.maxDepth() << std::endl;
  }

  {
    const int nQueries = 1000000;
    Eigen::Matrix<double, -1, 3, Eigen::RowMajor> queries(nQueries, 3);
    queries.setRandom();

    auto start = std::chrono::high_resolution_clock::now();

    tbb::parallel_for<int>(0, nQueries, [&](int i) {
      acc::nearestObject(
          bvh.numNodes(), bvh.data(), queries.row(i),
          [&](const int j) { return (points.row(j) - queries.row(i)).norm(); });
    });

    auto end = std::chrono::high_resolution_clock::now();

    std::cout << nQueries << " nearest object queries took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << " ms in parallel" << std::endl;
  }
}
