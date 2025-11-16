#pragma once
#include <vector>
#include "math/MiniMath.h"

namespace geom {

struct MarkerCross {
  std::vector<Vec3> positions;
  std::vector<int>  indices;

  void build(float size);
};

} // namespace geom
