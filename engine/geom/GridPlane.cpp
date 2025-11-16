#include "geom/GridPlane.h"

namespace geom {

void GridPlane::build(float size, int subdivisions) {
  positions.clear();
  indices.clear();

  const float half = size * 0.5f;
  const float step = size / subdivisions;

  // Create grid lines (simple debug grid, no triangles)
  for (int i = 0; i <= subdivisions; ++i) {
    float x = -half + i * step;

    // Vertical line
    positions.push_back(Vec3(x, 0.f, -half));
    positions.push_back(Vec3(x, 0.f,  half));

    // Horizontal line
    float z = -half + i * step;
    positions.push_back(Vec3(-half, 0.f, z));
    positions.push_back(Vec3( half, 0.f, z));
  }

  // Indices: each pair is a line segment
  for (int i = 0; i < positions.size(); i++)
    indices.push_back(i);
}

} // namespace geom
