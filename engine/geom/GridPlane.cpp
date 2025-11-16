#include "geom/GridPlane.h"

namespace geom {

GridPlane GridPlane::MakeXZ(int halfExtent, float spacing) {
  GridPlane g;

  const int   N   = halfExtent;
  const float h   = 0.f;
  const float max = N * spacing;

  // Lines parallel to Z (varying X)
  for (int x = -N; x <= N; ++x) {
    float fx = x * spacing;

    g.positions.push_back(Vec3{fx, h, -max});
    g.positions.push_back(Vec3{fx, h,  max});

    uint32_t base = static_cast<uint32_t>(g.positions.size() - 2);
    g.indices.push_back(base);
    g.indices.push_back(base + 1);
  }

  // Lines parallel to X (varying Z)
  for (int z = -N; z <= N; ++z) {
    float fz = z * spacing;

    g.positions.push_back(Vec3{-max, h, fz});
    g.positions.push_back(Vec3{ max, h, fz});

    uint32_t base = static_cast<uint32_t>(g.positions.size() - 2);
    g.indices.push_back(base);
    g.indices.push_back(base + 1);
  }

  return g;
}

} // namespace geom
