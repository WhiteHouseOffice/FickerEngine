#include "game/physics/ProxyGen.h"

namespace fe {

ColliderProxy ProxyGen::make8CornerProxyFromAABB(const Vec3& aabbMin, const Vec3& aabbMax, float radius) {
  ColliderProxy p;

  // AABB center and half-extents
  const Vec3 c = (aabbMin + aabbMax) * 0.5f;
  Vec3 e = (aabbMax - aabbMin) * 0.5f;

  // Shrink corner positions by radius so outer boundary matches original AABB.
  // (This avoids “making the object bigger”.)
  e.x = (e.x > radius) ? (e.x - radius) : 0.f;
  e.y = (e.y > radius) ? (e.y - radius) : 0.f;
  e.z = (e.z > radius) ? (e.z - radius) : 0.f;

  const float sx[2] = {-1.f, +1.f};
  const float sy[2] = {-1.f, +1.f};
  const float sz[2] = {-1.f, +1.f};

  for (float ix : sx) for (float iy : sy) for (float iz : sz) {
    SphereProxy s;
    s.center = Vec3(c.x + ix*e.x, c.y + iy*e.y, c.z + iz*e.z);
    s.radius = radius;
    p.spheres.push_back(s);
  }

  return p;
}

ColliderProxy ProxyGen::make8CornerProxyFromChunkAABB(const WorldMesh& world, const Chunk& ch, float radius) {
  Vec3 mn, mx;
  world.computeChunkAABB(ch, mn, mx);
  return make8CornerProxyFromAABB(mn, mx, radius);
}

} // namespace fe
