#include "game/Scene.h"

#include <cstdio>

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

// ---------- ctor ----------

Scene::Scene() = default;

// ---------- helpers to build CPU geometry ----------

namespace {

// Build a simple XZ grid centered at origin.
// Lines go from -half..+half in both X and Z (world units).
void buildGridGeometry(geom::GridPlane& grid, float halfSize, float step) {
  grid.positions.clear();
  grid.indices.clear();

  // We treat the grid as a set of line segments.
  auto addLine = [&](const Vec3& a, const Vec3& b) {
    int base = static_cast<int>(grid.positions.size());
    grid.positions.push_back(a);
    grid.positions.push_back(b);
    grid.indices.push_back(base);
    grid.indices.push_back(base + 1);
  };

  // Vertical lines (varying X, Z fixed at +/-halfSize)
  for (float x = -halfSize; x <= halfSize + 0.001f; x += step) {
    addLine(Vec3{x, 0.0f, -halfSize}, Vec3{x, 0.0f, halfSize});
  }

  // Horizontal lines (varying Z, X fixed at +/-halfSize)
  for (float z = -halfSize; z <= halfSize + 0.001f; z += step) {
    addLine(Vec3{-halfSize, 0.0f, z}, Vec3{halfSize, 0.0f, z});
  }
}

// Build a little origin marker cross (3 orthogonal line segments).
void buildMarkerGeometry(geom::MarkerCross& marker, float len) {
  marker.positions.clear();
  marker.indices.clear();

  auto addAxis = [&](const Vec3& a, const Vec3& b) {
    int base = static_cast<int>(marker.positions.size());
    marker.positions.push_back(a);
    marker.positions.push_back(b);
    marker.indices.push_back(base);
    marker.indices.push_back(base + 1);
  };

  // X axis (red in a real renderer)
  addAxis(Vec3{-len, 0.0f, 0.0f}, Vec3{len, 0.0f, 0.0f});
  // Y axis (green)
  addAxis(Vec3{0.0f, -len, 0.0f}, Vec3{0.0f, len, 0.0f});
  // Z axis (blue)
  addAxis(Vec3{0.0f, 0.0f, -len}, Vec3{0.0f, 0.0f, len});
}

} // namespace

// ---------- public API ----------

void Scene::init() {
  // 1) Build CPU geometry.
  geom::GridPlane grid;
  geom::MarkerCross marker;

  buildGridGeometry(grid, /*halfSize*/ 10.0f, /*step*/ 1.0f);
  buildMarkerGeometry(marker, /*len*/ 0.5f);

  // 2) Upload into CPU RenderMeshes.
  gridMesh.uploadGrid(grid);
  markerMesh.uploadMarker(marker);

  // 3) Print stats ONCE so we know it worked.
  gridMesh.debugPrint("grid");
  markerMesh.debugPrint("marker");
}

void Scene::update(float /*dt*/) {
  // Nothing animated yet – this is just our test environment.
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // Will call into a real renderer once WebGPU/WebGL path exists.
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // For now, all debug happens via the one-time prints in init().
}
