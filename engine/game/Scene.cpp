#include "game/Scene.h"

#if defined(FE_WEB) || defined(__EMSCRIPTEN__)
  #include <GL/gl.h>
#endif

using engine::render::RenderMesh;

static std::vector<RenderMesh::VertexPC>
MakeVerticesPC(const std::vector<Vec3>& positions, float r, float g, float b, float a) {
  std::vector<RenderMesh::VertexPC> out;
  out.reserve(positions.size());
  for (const auto& p : positions) {
    out.push_back(RenderMesh::VertexPC{ p.x, p.y, p.z, r, g, b, a });
  }
  return out;
}

static std::vector<std::uint32_t>
MakeIndicesU32(const std::vector<int>& indices) {
  std::vector<std::uint32_t> out;
  out.reserve(indices.size());
  for (int i : indices) out.push_back(static_cast<std::uint32_t>(i));
  return out;
}

Scene::Scene() {
  grid_.build(/*size*/ 10.0f, /*subdivisions*/ 20);
  marker_.build(/*size*/ 0.5f);
}

void Scene::buildGroundQuad(float halfSize) {
  // 2 triangles on XZ plane at y=0
  std::vector<RenderMesh::VertexPC> v = {
    {-halfSize, 0.f, -halfSize, 0.20f, 0.20f, 0.20f, 1.f},
    { halfSize, 0.f, -halfSize, 0.20f, 0.20f, 0.20f, 1.f},
    { halfSize, 0.f,  halfSize, 0.20f, 0.20f, 0.20f, 1.f},
    {-halfSize, 0.f,  halfSize, 0.20f, 0.20f, 0.20f, 1.f},
  };
  std::vector<std::uint32_t> idx = {
    0, 1, 2,
    0, 2, 3
  };

  groundMesh_.Upload(v, idx, GL_TRIANGLES);
}

void Scene::init() {
  // Grid + marker as lines
  auto gridV = MakeVerticesPC(grid_.positions, 0.55f, 0.55f, 0.55f, 1.f);
  auto gridI = MakeIndicesU32(grid_.indices);
  gridMesh_.Upload(gridV, gridI, GL_LINES);

  auto markerV = MakeVerticesPC(marker_.positions, 1.f, 1.f, 1.f, 1.f);
  auto markerI = MakeIndicesU32(marker_.indices);
  markerMesh_.Upload(markerV, markerI, GL_LINES);

  // Solid ground plane (walkable visual target)
  buildGroundQuad(5.0f);
}

void Scene::update(float /*dt*/) {
  // no-op for now
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
#if defined(FE_WEB) || defined(__EMSCRIPTEN__)
  // For now we ignore view/proj and just draw in legacy pipeline.
  // Next step will be to apply transforms (camera) properly.
  groundMesh_.Draw();
  gridMesh_.Draw();
  markerMesh_.Draw();
#endif
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // keep clean for now
}
