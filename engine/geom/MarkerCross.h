#pragma once

#include <vector>
#include "math/MiniMath.h"

// Simple origin marker cross (3 axis lines)
namespace geom {

struct MarkerCross {
  std::vector<Vec3>     positions;
  std::vector<uint32_t> indices;

  // size = half-length of each axis line
  static MarkerCross MakeOrigin(float size);
};

} // namespace geom
