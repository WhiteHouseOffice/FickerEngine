#pragma once
#include <cstdint>
#include <vector>
#include <cmath>
#include <algorithm>

namespace engine::geom {

// Simple colored heightfield terrain mesh.
// Generates triangles with per-vertex grey color (slight variation by height).
// IMPORTANT: This mesh can also be used for physics queries via sampleHeight/sampleNormal.
// (No proxy simplification; uses the full grid vertex data.)
struct TerrainGrid {
  struct Vertex {
    float x, y, z;
    uint32_t rgba; // 0xRRGGBBAA
  };

  struct Normal {
    float x, y, z;
  };

  std::vector<Vertex>   vertices;
  std::vector<uint32_t> indices;

  // Stored so physics can sample the mesh later
  float sizeX = 0.f;
  float sizeZ = 0.f;
  float step  = 1.f;
  int   nx = 0;
  int   nz = 0;
  float halfX = 0.f;
  float halfZ = 0.f;

  static constexpr uint32_t RGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    return (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | uint32_t(a);
  }

  static TerrainGrid make(
    float inSizeX, float inSizeZ, float inStep,
    float baseY, float amp
  ) {
    TerrainGrid t;
    t.sizeX = inSizeX;
    t.sizeZ = inSizeZ;
    t.step  = (inStep <= 1e-6f) ? 1.0f : inStep;

    t.nx = (int)std::floor(t.sizeX / t.step) + 1;
    t.nz = (int)std::floor(t.sizeZ / t.step) + 1;

    t.halfX = 0.5f * t.sizeX;
    t.halfZ = 0.5f * t.sizeZ;

    t.vertices.reserve((size_t)t.nx * (size_t)t.nz);
    t.indices.reserve((size_t)(t.nx - 1) * (size_t)(t.nz - 1) * 6);

    auto height = [&](float x, float z) -> float {
      // Gentle, natural-ish terrain:
      // 1) broad hill (gaussian-ish)
      const float r2 = (x*x + z*z);
      const float sig = (0.45f * t.halfX);
      const float hill = std::exp(-r2 / (2.0f * sig * sig)); // wide bump

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
    for (int iz = 0; iz < t.nz; ++iz) {
      for (int ix = 0; ix < t.nx; ++ix) {
        float x = -t.halfX + ix * t.step;
        float z = -t.halfZ + iz * t.step;
        float y = height(x, z);

        float hn = clamp01((y - baseY) / (amp + 1e-5f));
        uint8_t g = (uint8_t)(130 + hn * 60); // 130..190
        t.vertices.push_back({ x, y, z, RGBA(g, g, g, 255) });
      }
    }

    auto idx = [&](int ix, int iz) -> uint32_t {
      return (uint32_t)(iz * t.nx + ix);
    };

    // Build indices (two tris per cell)
    for (int iz = 0; iz < t.nz - 1; ++iz) {
      for (int ix = 0; ix < t.nx - 1; ++ix) {
        uint32_t i00 = idx(ix,     iz);
        uint32_t i10 = idx(ix + 1, iz);
        uint32_t i01 = idx(ix,     iz + 1);
        uint32_t i11 = idx(ix + 1, iz + 1);

        // CW front
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

  // Bilinear height sampling from full vertex grid (physics ground)
  float sampleHeight(float x, float z) const {
    if (vertices.empty() || nx <= 1 || nz <= 1) return -1e30f;

    float fx = (x + halfX) / step;
    float fz = (z + halfZ) / step;

    // clamp to grid
    fx = std::clamp(fx, 0.0f, (float)(nx - 1));
    fz = std::clamp(fz, 0.0f, (float)(nz - 1));

    int ix = (int)std::floor(fx);
    int iz = (int)std::floor(fz);

    int ix1 = std::min(ix + 1, nx - 1);
    int iz1 = std::min(iz + 1, nz - 1);

    float tx = fx - (float)ix;
    float tz = fz - (float)iz;

    auto at = [&](int xg, int zg) -> float {
      return vertices[(size_t)zg * (size_t)nx + (size_t)xg].y;
    };

    float h00 = at(ix,  iz);
    float h10 = at(ix1, iz);
    float h01 = at(ix,  iz1);
    float h11 = at(ix1, iz1);

    float hx0 = h00 + (h10 - h00) * tx;
    float hx1 = h01 + (h11 - h01) * tx;
    return hx0 + (hx1 - hx0) * tz;
  }

  Normal sampleNormal(float x, float z) const {
    // finite differences on the sampled heightfield (still derived from mesh vertices)
    const float e = std::max(0.5f * step, 0.05f);
    float hL = sampleHeight(x - e, z);
    float hR = sampleHeight(x + e, z);
    float hD = sampleHeight(x, z - e);
    float hU = sampleHeight(x, z + e);

    float dhdx = (hR - hL) / (2.0f * e);
    float dhdz = (hU - hD) / (2.0f * e);

    float nx_ = -dhdx;
    float ny_ = 1.0f;
    float nz_ = -dhdz;

    float l2 = nx_*nx_ + ny_*ny_ + nz_*nz_;
    if (l2 < 1e-12f) return {0.f, 1.f, 0.f};
    float inv = 1.0f / std::sqrt(l2);
    return { nx_*inv, ny_*inv, nz_*inv };
  }
};

} // namespace engine::geom
