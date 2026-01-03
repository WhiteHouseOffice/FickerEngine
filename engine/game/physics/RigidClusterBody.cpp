#include "game/physics/RigidClusterBody.h"

namespace fe {

Vec3 RigidClusterBody::centerOfMass() const {
  if (particles.empty()) return Vec3(0.f,0.f,0.f);
  Vec3 c(0.f,0.f,0.f);
  for (const auto& p : particles) c = c + p.pos;
  return c * (1.0f / (float)particles.size());
}

void RigidClusterBody::wake() {
  asleep = false;
  sleepTimer = 0.f;
}

} // namespace fe
