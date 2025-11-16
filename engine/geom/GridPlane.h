#pragma once
#include <vector>
#include "math/MiniMath.h"

namespace geom {

struct GridPlane {
  std::vector<Vec3> positions;
  std::vector<int>  indices;

  void build(float size, int subdivisions);
};

} // namespace geom
