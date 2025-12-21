#include "game/Scene.h"

#ifdef __EMSCRIPTEN__
  #include <GLES/gl.h>
#else
  #include <GL/gl.h>
#endif

using engine::render::RenderMesh;

static RenderMesh::VertexPC V(float x,float y,float z, float r,float g,float b,float a) {
  RenderMesh::VertexPC v{};
  v.x=x; v.y=y; v.z=z; v.r=r; v.g=g; v.b=b; v.a=a;
  return v;
}

static std::vector<std::uint16_t> ToU16Indices(const std::vector<int>& idx) {
  std::vector<std::uint16_t> out;
  out.reserve(idx.size());
  for (int i : idx) out.push_back((std::uint16_t)i);
  return out;
}

void Scene::init() {
  // Build CPU geom
  grid_.build(10.0f, 20);
  marker_.build(1.0f);

  // -------- Ground quad (two triangles) --------
  // A simple "walkable plane" centered at origin (y=0)
  std::vector<RenderMesh::VertexPC> groundV;
  groundV.reserve(4);
  groundV.push_back(V(-10.f, 0.f, -10.f, 0.25f, 0.25f, 0.25f, 1.f));
  groundV.push_back(V( 10.f, 0.f, -10.f, 0.25f, 0.25f, 0.25f, 1.f));
  groundV.push_back(V( 10.f, 0.f,  10.f, 0.25f, 0.25f, 0.25f, 1.f));
  groundV.push_back(V(-10.f, 0.f,  10.f, 0.25f, 0.25f, 0.25f, 1.f));

  std::vector<std::uint16_t> groundI = {0,1,2, 0,2,3};
  groundMesh_.Upload(groundV, groundI, GL_TRIANGLES);

  // -------- Grid (lines) --------
  std::vector<RenderMesh::VertexPC> gridV;
  gridV.reserve(grid_.positions.size());
  for (auto& p : grid_.positions) {
    gridV.push_back(V(p.x, p.y, p.z, 0.2f, 0.8f, 0.2f, 1.f));
  }
  std::vector<std::uint16_t> gridI = ToU16Indices(grid_.indices);
  gridMesh_.Upload(gridV, gridI, GL_LINES);

  // -------- Marker (lines) --------
  std::vector<RenderMesh::VertexPC> markerV;
  markerV.reserve(marker_.positions.size());
  for (auto& p : marker_.positions) {
    markerV.push_back(V(p.x, p.y, p.z, 1.0f, 0.2f, 0.2f, 1.f));
  }
  std::vector<std::uint16_t> markerI = ToU16Indices(marker_.indices);
  markerMesh_.Upload(markerV, markerI, GL_LINES);
}

void Scene::render() {
  // If you already set up camera matrices elsewhere, keep doing that there.
  // This is just drawing the meshes.
  groundMesh_.Draw();
  gridMesh_.Draw();
  markerMesh_.Draw();
}
