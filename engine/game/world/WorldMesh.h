#pragma once

#include <cstdint>
#include <vector>

#include "math/MiniMath.h"

namespace fe {

// Minimal “world as topology” storage.
// This is NOT a renderer mesh. It’s your canonical editable world data.
using VertexID = uint32_t;
using EdgeID   = uint32_t;
using FaceID   = uint32_t;

struct ColorRGBA {
  uint8_t r=255, g=255, b=255, a=255;
};

struct Vertex {
  VertexID id = 0;
  Vec3     pos{0.f,0.f,0.f};
};

struct Edge {
  EdgeID   id = 0;
  VertexID a = 0;
  VertexID b = 0;

  // If an edge has no faces attached, you can delete it during cleanup.
  // (We track adjacency in Face->edges; edges can be “dangling”.)
};

struct Face {
  FaceID id = 0;

  // Tri face for now (simple + fast). You can later generalize to polygons.
  VertexID v0=0, v1=0, v2=0;
  ColorRGBA color{};
};

struct Chunk {
  // A “piece” (e.g. stone blown from mountain) is a subset of faces.
  std::vector<FaceID> faces;
};

class WorldMesh {
public:
  VertexID addVertex(const Vec3& p);
  EdgeID   addEdge(VertexID a, VertexID b);
  FaceID   addTri(VertexID v0, VertexID v1, VertexID v2, ColorRGBA c = {});

  const std::vector<Vertex>& vertices() const { return m_vertices; }
  const std::vector<Edge>&   edges() const    { return m_edges; }
  const std::vector<Face>&   faces() const    { return m_faces; }

  Vertex*       getVertex(VertexID id);
  const Vertex* getVertex(VertexID id) const;

  Face*       getFace(FaceID id);
  const Face* getFace(FaceID id) const;

  // Utility: compute AABB for a chunk (used to seed a collider proxy)
  void computeChunkAABB(const Chunk& ch, Vec3& outMin, Vec3& outMax) const;

  // Cleanup: remove edges that are not referenced by any face.
  // (Optional; you said dangling edges get deleted.)
  void deleteDanglingEdges();

private:
  VertexID m_nextV = 1;
  EdgeID   m_nextE = 1;
  FaceID   m_nextF = 1;

  std::vector<Vertex> m_vertices;
  std::vector<Edge>   m_edges;
  std::vector<Face>   m_faces;
};

} // namespace fe
