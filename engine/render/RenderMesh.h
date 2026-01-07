#pragma once
#include <cstdint>
#include <vector>

namespace engine::render {

// Position + Color (legacy fixed-function friendly)
struct VertexPC {
  float x, y, z;
  float r, g, b, a;
};

class RenderMesh {
public:
  enum class Primitive {
    Triangles = 0,
    Lines     = 1
  };

  enum class Winding {
    CCW = 0,
    CW  = 1
  };

  void Clear();

  void SetPrimitive(Primitive prim);
  void SetVertices(const std::vector<VertexPC>& verts);
  void SetIndices(const std::vector<uint16_t>& inds);

  // New: culling control (recommended for surfaces)
  void SetBackfaceCulling(bool enabled);
  void SetFrontFaceWinding(Winding winding);

  void Draw() const;

private:
  Primitive m_primitive = Primitive::Triangles;

  // New: defaults
  bool    m_backfaceCull = true;           // ON by default for triangle surfaces
  Winding m_frontWinding = Winding::CCW;   // CCW front faces

  std::vector<VertexPC> m_vertices;
  std::vector<uint16_t> m_indices;
};

} // namespace engine::render
