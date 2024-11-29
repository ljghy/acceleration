#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>

#include <glm/glm.hpp>

#include <tbb/parallel_for.h>

#include <acc/Query.hpp>
#include <acc/StaticBVH.hpp>

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> dis(-1.f, 1.f);

glm::vec3 randVec3() { return glm::vec3(dis(gen), dis(gen), dis(gen)); }

int main() {

  acc::StaticBVH bvh;

  const int nPoints = 1000000;
  std::vector<glm::vec3> points(nPoints);
  std::for_each(points.begin(), points.end(), [](auto &p) { p = randVec3(); });

  {
    auto start = std::chrono::high_resolution_clock::now();

    bvh.build(nPoints, [&](int i) {
      return acc::BoundingBox{points[i] - 1e-5f, points[i] + 1e-5f};
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
    std::vector<glm::vec3> queries(nQueries);
    std::for_each(queries.begin(), queries.end(),
                  [](auto &q) { q = randVec3(); });

    auto start = std::chrono::high_resolution_clock::now();

    tbb::parallel_for<int>(0, nQueries, [&](int i) {
      acc::nearestObject(
          bvh.numNodes(), bvh.data(), queries[i],
          [&](const int j) { return glm::distance(points[j], queries[i]); });
    });

    auto end = std::chrono::high_resolution_clock::now();

    std::cout << nQueries << " nearest object queries took "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << " ms in parallel" << std::endl;
  }
}
