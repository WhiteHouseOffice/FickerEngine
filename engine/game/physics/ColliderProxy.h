#pragma once

#include <vector>
#include "math/MiniMath.h"

namespace fe {

struct SphereProxy {
  Vec3  center{0.f,0.f,0.f};
  float radius = 0.1f;
};

// A collider proxy that can approximate ANY shape.
// For now: multi-sphere shell/cluster.
// Later: can add boxes/capsules/hulls.
struct ColliderProxy {
  std::vector<SphereProxy> spheres;

  // Recommended: keep this small.
  // If you want “~1 cm shell”, you set radius ~0.005 if 1 unit = 1 meter.
};

} // namespace fe
