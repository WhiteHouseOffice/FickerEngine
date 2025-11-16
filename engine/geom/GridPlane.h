#pragma once

#include <vector>
#include "math/MiniMath.h"

// Pure CPU grid geometry (XZ plane lines)
namespace geom {

struct GridPlane {
  std::vector<Vec3>     positions; // world-space vertices
  std::vector<uint32_t> indices;   // line list indices

  // Build a simple XZ grid centered at origin.
  // halfExtent: number of cells from center to one side
  // spacing: distance between grid lines
  static GridPlane MakeXZ(int halfExtent, float spacing);
};

} // namespace geom
