#pragma once

#include <cstdint>

#include "game/world/WorldMesh.h"
#include "game/physics/ColliderProxy.h"

namespace fe {

// Very simple proxy generator:
// - Uses chunk AABB
// - Places 8 spheres at “shrunken corners” so outer boundary matches the AABB
// This is perfect for your first “8-sphere cube” step AND for chunk placeholders.
//
// Later: replace with voxel/sample packing for arbitrary shapes.
struct ProxyGen {
  static ColliderProxy make8CornerProxyFromAABB(const Vec3& aabbMin, const Vec3& aabbMax, float radius);

  // Convenience: build proxy from chunk by using its AABB.
  static ColliderProxy make8CornerProxyFromChunkAABB(const WorldMesh& world, const Chunk& ch, float radius);
};

} // namespace fe
