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

  void Clear();

  void SetPrimitive(Primitive prim);
  void SetVertices(const std::vector<VertexPC>& verts);
  void SetIndices(const std::vector<uint16_t>& inds);

  void Draw() const;

private:
  Primitive m_primitive = Primitive::Triangles;
  std::vector<VertexPC> m_vertices;
  std::vector<uint16_t> m_indices;
};

} // namespace engine::render
