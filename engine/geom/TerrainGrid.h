#pragma once
#include <cstdint>
#include <vector>
#include <cmath>

namespace engine::geom {

// Simple colored heightfield terrain mesh.
// Generates triangles with per-vertex grey color (slight variation by height).
struct TerrainGrid {
  struct Vertex {
    float x, y, z;
    uint32_t rgba; // 0xRRGGBBAA
  };

  std::vector<Vertex>   vertices;
  std::vector<uint32_t> indices;

  static constexpr uint32_t RGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | uint32_t(a);
  }

  // sizeX/sizeZ = full width in world units
  // step = grid spacing (e.g. 1.0f)
  // baseY = baseline height
  // amp = hill amplitude
  static TerrainGrid make(
    float sizeX, float sizeZ, float step,
    float baseY, float amp
  ) {
    TerrainGrid t;

    const int nx = (int)std::floor(sizeX / step) + 1;
    const int nz = (int)std::floor(sizeZ / step) + 1;

    t.vertices.reserve((size_t)nx * (size_t)nz);
    t.indices.reserve((size_t)(nx - 1) * (size_t)(nz - 1) * 6);

    const float halfX = 0.5f * sizeX;
    const float halfZ = 0.5f * sizeZ;

    auto height = [&](float x, float z) -> float {
      // Gentle, natural-ish terrain:
      // 1) broad hill (gaussian-ish)
      const float r2 = (x*x + z*z);
      const float hill = std::exp(-r2 / (2.0f * (0.45f*halfX)*(0.45f*halfX))); // wide bump

      // 2) small undulations
      const float wav = 0.35f * std::sin(0.18f * x) * std::cos(0.16f * z);

      return baseY + amp * (0.75f * hill + wav);
    };

    auto clamp01 = [](float v) -> float {
      if (v < 0.0f) return 0.0f;
      if (v > 1.0f) return 1.0f;
      return v;
    };

    // Build verts
    for (int iz = 0; iz < nz; ++iz) {
      for (int ix = 0; ix < nx; ++ix) {
        float x = -halfX + ix * step;
        float z = -halfZ + iz * step;
        float y = height(x, z);

        // Slight grey variation by normalized height
        float hn = clamp01((y - baseY) / (amp + 1e-5f));
        uint8_t g = (uint8_t)(130 + hn * 60); // 130..190
        t.vertices.push_back({ x, y, z, RGBA(g, g, g, 255) });
      }
    }

    // Build indices (two tris per cell)
    auto idx = [&](int ix, int iz) -> uint32_t {
      return (uint32_t)(iz * nx + ix);
    };

    for (int iz = 0; iz < nz - 1; ++iz) {
      for (int ix = 0; ix < nx - 1; ++ix) {
        uint32_t i00 = idx(ix,     iz);
        uint32_t i10 = idx(ix + 1, iz);
        uint32_t i01 = idx(ix,     iz + 1);
        uint32_t i11 = idx(ix + 1, iz + 1);

        // Winding depends on your engine front-face convention.
        // We will assume "CW is front" based on your last culling result.
        // Cell triangles (CW):
        t.indices.push_back(i00);
        t.indices.push_back(i01);
        t.indices.push_back(i10);

        t.indices.push_back(i10);
        t.indices.push_back(i01);
        t.indices.push_back(i11);
      }
    }

    return t;
  }
};

} // namespace engine::geom
