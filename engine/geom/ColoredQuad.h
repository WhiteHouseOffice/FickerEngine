#pragma once
#include <vector>
#include <cstdint>
#include "math/MiniMath.h"

// A single quad made of 2 triangles, CCW winding by default.
// Colors are per-vertex RGBA8 packed as 0xRRGGBBAA.
struct ColoredQuad
{
  struct Vtx
  {
    Vec3 pos;
    uint32_t rgba;
  };

  std::vector<Vtx> vertices;
  std::vector<uint32_t> indices;

  static constexpr uint32_t RGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a=255)
  {
    return (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | uint32_t(a);
  }

  static ColoredQuad make(const Vec3& center, float halfW, float halfH,
                          uint32_t c0, uint32_t c1, uint32_t c2, uint32_t c3,
                          bool ccw=true)
  {
    ColoredQuad q;

    // local quad in XZ plane (y constant)
    Vec3 p0 = center + Vec3(-halfW, 0.0f, -halfH);
    Vec3 p1 = center + Vec3(+halfW, 0.0f, -halfH);
    Vec3 p2 = center + Vec3(+halfW, 0.0f, +halfH);
    Vec3 p3 = center + Vec3(-halfW, 0.0f, +halfH);

    q.vertices = {
      {p0, c0}, {p1, c1}, {p2, c2}, {p3, c3}
    };

    // Two triangles
    if (ccw)
      q.indices = {0,1,2,  0,2,3};
    else
      q.indices = {0,2,1,  0,3,2};

    return q;
  }
};
