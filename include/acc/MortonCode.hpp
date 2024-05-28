#ifndef ACC_MORTON_CODE_HPP_
#define ACC_MORTON_CODE_HPP_

#include <cstdint>

#include <acc/Vec3Op.hpp>

namespace acc {

struct MortonCode {
  uint32_t code;
  index_t index;

  MortonCode(const index_t i, vec3_t p) : index(i) {
    p = min(max(p * real_t{1024}, real_t{0}), real_t{1023});
    uint32_t x = expandBits(static_cast<uint32_t>(p[0]));
    uint32_t y = expandBits(static_cast<uint32_t>(p[1]));
    uint32_t z = expandBits(static_cast<uint32_t>(p[2]));
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
};

} // namespace acc

#endif
