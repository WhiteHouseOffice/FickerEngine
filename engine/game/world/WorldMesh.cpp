#include "game/world/WorldMesh.h"

#include <algorithm>

namespace fe {

VertexID WorldMesh::addVertex(const Vec3& p) {
  Vertex v;
  v.id = m_nextV++;
  v.pos = p;
  m_vertices.push_back(v);
  return v.id;
}

EdgeID WorldMesh::addEdge(VertexID a, VertexID b) {
  Edge e;
  e.id = m_nextE++;
  e.a = a;
  e.b = b;
  m_edges.push_back(e);
  return e.id;
}

FaceID WorldMesh::addTri(VertexID v0, VertexID v1, VertexID v2, ColorRGBA c) {
  Face f;
  f.id = m_nextF++;
  f.v0 = v0;
  f.v1 = v1;
  f.v2 = v2;
  f.color = c;
  m_faces.push_back(f);
  return f.id;
}

Vertex* WorldMesh::getVertex(VertexID id) {
  for (auto& v : m_vertices) if (v.id == id) return &v;
  return nullptr;
}

const Vertex* WorldMesh::getVertex(VertexID id) const {
  for (auto& v : m_vertices) if (v.id == id) return &v;
  return nullptr;
}

Face* WorldMesh::getFace(FaceID id) {
  for (auto& f : m_faces) if (f.id == id) return &f;
  return nullptr;
}

const Face* WorldMesh::getFace(FaceID id) const {
  for (auto& f : m_faces) if (f.id == id) return &f;
  return nullptr;
}

void WorldMesh::computeChunkAABB(const Chunk& ch, Vec3& outMin, Vec3& outMax) const {
  outMin = Vec3( 1e30f,  1e30f,  1e30f);
  outMax = Vec3(-1e30f, -1e30f, -1e30f);

  auto expand = [&](const Vec3& p){
    if (p.x < outMin.x) outMin.x = p.x;
    if (p.y < outMin.y) outMin.y = p.y;
    if (p.z < outMin.z) outMin.z = p.z;
    if (p.x > outMax.x) outMax.x = p.x;
    if (p.y > outMax.y) outMax.y = p.y;
    if (p.z > outMax.z) outMax.z = p.z;
  };

  for (FaceID fid : ch.faces) {
    const Face* f = getFace(fid);
    if (!f) continue;

    const Vertex* a = getVertex(f->v0);
    const Vertex* b = getVertex(f->v1);
    const Vertex* c = getVertex(f->v2);
    if (!a || !b || !c) continue;

    expand(a->pos);
    expand(b->pos);
    expand(c->pos);
  }
}

void WorldMesh::deleteDanglingEdges() {
  // We don’t currently store face->edges, so we’ll approximate:
  // An edge is “used” if its (a,b) or (b,a) appears in any triangle edge set.
  // (Fast enough for now; we’ll optimize later.)

  auto used = [&](VertexID a, VertexID b, const Face& f)->bool{
    const VertexID t0 = f.v0, t1 = f.v1, t2 = f.v2;
    auto match = [&](VertexID x, VertexID y){
      return (x==a && y==b) || (x==b && y==a);
    };
    return match(t0,t1) || match(t1,t2) || match(t2,t0);
  };

  std::vector<Edge> kept;
  kept.reserve(m_edges.size());

  for (const auto& e : m_edges) {
    bool isUsed = false;
    for (const auto& f : m_faces) {
      if (used(e.a, e.b, f)) { isUsed = true; break; }
    }
    if (isUsed) kept.push_back(e);
  }

  m_edges.swap(kept);
}

} // namespace fe
