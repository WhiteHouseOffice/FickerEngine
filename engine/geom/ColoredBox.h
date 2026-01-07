#pragma once
#include <cstdint>
#include <vector>

namespace engine::geom {

// Minimal colored, closed box mesh (24 verts, 12 tris).
// CCW winding for OUTWARD faces.
struct ColoredBox {
  struct Vertex {
    float x, y, z;
    uint32_t rgba; // 0xRRGGBBAA
  };

  std::vector<Vertex>   vertices;
  std::vector<uint32_t> indices;

  static constexpr uint32_t RGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | uint32_t(a);
  }

  // center + half extents; per-face colors in this order:
  // +X, -X, +Y, -Y, +Z, -Z
  static ColoredBox make(
    float cx, float cy, float cz,
    float hx, float hy, float hz,
    uint32_t cPosX, uint32_t cNegX,
    uint32_t cPosY, uint32_t cNegY,
    uint32_t cPosZ, uint32_t cNegZ
  ) {
    ColoredBox m;
    m.vertices.reserve(24);
    m.indices.reserve(36);

    const float x0 = cx - hx, x1 = cx + hx;
    const float y0 = cy - hy, y1 = cy + hy;
    const float z0 = cz - hz, z1 = cz + hz;

    auto addFace = [&]( // 4 verts in CCW order when viewed from outside
      float ax, float ay, float az,
      float bx, float by, float bz,
      float cx_, float cy_, float cz_,
      float dx, float dy, float dz,
      uint32_t col
    ) {
      const uint32_t base = (uint32_t)m.vertices.size();
      m.vertices.push_back({ax, ay, az, col});
      m.vertices.push_back({bx, by, bz, col});
      m.vertices.push_back({cx_, cy_, cz_, col});
      m.vertices.push_back({dx, dy, dz, col});

      // Two triangles: (0,1,2) and (0,2,3)
      m.indices.push_back(base + 0);
      m.indices.push_back(base + 1);
      m.indices.push_back(base + 2);

      m.indices.push_back(base + 0);
      m.indices.push_back(base + 2);
      m.indices.push_back(base + 3);
    };

    // +X face (outside looks toward +X): CCW in Y/Z plane
    addFace(
      x1, y0, z0,
      x1, y0, z1,
      x1, y1, z1,
      x1, y1, z0,
      cPosX
    );

    // -X face (outside looks toward -X)
    addFace(
      x0, y0, z1,
      x0, y0, z0,
      x0, y1, z0,
      x0, y1, z1,
      cNegX
    );

    // +Y face (outside looks toward +Y)
    addFace(
      x0, y1, z0,
      x1, y1, z0,
      x1, y1, z1,
      x0, y1, z1,
      cPosY
    );

    // -Y face (outside looks toward -Y)
    addFace(
      x0, y0, z1,
      x1, y0, z1,
      x1, y0, z0,
      x0, y0, z0,
      cNegY
    );

    // +Z face (outside looks toward +Z)
    addFace(
      x1, y0, z1,
      x0, y0, z1,
      x0, y1, z1,
      x1, y1, z1,
      cPosZ
    );

    // -Z face (outside looks toward -Z)
    addFace(
      x0, y0, z0,
      x1, y0, z0,
      x1, y1, z0,
      x0, y1, z0,
      cNegZ
    );

    return m;
  }
};

} // namespace engine::geom
