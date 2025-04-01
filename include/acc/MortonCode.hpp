#ifndef ACC_MORTON_CODE_HPP_
#define ACC_MORTON_CODE_HPP_

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>

#ifndef ACC_DONT_PARALLELIZE
#include <execution>
#endif

#include <acc/Vec3Op.hpp>

namespace acc {

struct MortonCode {
  uint32_t code;
  index_t index;

  MortonCode() = default;

  MortonCode(const index_t i, const vec3_t &p) : index(i) {
    const vec3_t q = min(max(p * real_t{1024}, real_t{0}), real_t{1023});
    uint32_t x = expandBits(static_cast<uint32_t>(q[0]));
    uint32_t y = expandBits(static_cast<uint32_t>(q[1]));
    uint32_t z = expandBits(static_cast<uint32_t>(q[2]));
    code = x * 4 + y * 2 + z;
  }

  static uint32_t expandBits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
  }

  bool operator<(const MortonCode &other) const { return code < other.code; }

  static void radixSort(std::vector<MortonCode> &codes) {

    auto *curr = &codes;
    std::vector<MortonCode> swapCodes(codes.size());
    auto *out = &swapCodes;

#ifndef ACC_DONT_PARALLELIZE
    const int parts = std::max<int>(1, codes.size() / 4096);
#else
    const int parts = 1;
#endif
    std::vector<int> partIndices(parts);
    std::iota(partIndices.begin(), partIndices.end(), 0);

    constexpr int bitsPerPass = 8;
    constexpr int passes = 4;
    constexpr int buckets = 1 << bitsPerPass;
    constexpr uint32_t mask = buckets - 1;

    for (int pass = 0; pass < passes; ++pass) {

      std::vector<std::array<index_t, buckets>> count(parts);
      for (auto &part : count)
        std::fill(part.begin(), part.end(), 0);

      std::for_each(
#ifndef ACC_DONT_PARALLELIZE
          std::execution::par,
#endif
          partIndices.begin(), partIndices.end(), [&](const int part) {
            const index_t partSize = codes.size() / parts;
            index_t first = partSize * part;
            const index_t last =
                part == parts - 1 ? codes.size() : (first + partSize);

            for (; first < last; ++first) {
              const int bucket =
                  ((*curr)[first].code >> (bitsPerPass * pass)) & mask;
              ++count[part][bucket];
            }
          });

      index_t base = 0;
      for (int bucket = 0; bucket < buckets; ++bucket)
        for (int part = 0; part < parts; ++part) {
          const index_t c = count[part][bucket];
          count[part][bucket] = base;
          base += c;
        }

      std::for_each(
#ifndef ACC_DONT_PARALLELIZE
          std::execution::par,
#endif
          partIndices.begin(), partIndices.end(), [&](const int part) {
            const index_t partSize = codes.size() / parts;
            index_t first = partSize * part;
            const index_t last =
                part == parts - 1 ? codes.size() : (first + partSize);

            for (; first < last; ++first) {
              const int bucket =
                  ((*curr)[first].code >> (bitsPerPass * pass)) & mask;
              (*out)[count[part][bucket]++] = (*curr)[first];
            }
          });

      std::swap(curr, out);
    }

    if (curr != &codes)
      curr->swap(codes);
  }
};

} // namespace acc

#endif
