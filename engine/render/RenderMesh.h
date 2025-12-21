#pragma once
#include <vector>
#include <cstdint>

namespace engine::render {

class RenderMesh {
public:
  struct VertexPC {
    float x, y, z;
    float r, g, b, a;
  };

  RenderMesh() = default;
  ~RenderMesh() = default;

  RenderMesh(const RenderMesh&) = delete;
  RenderMesh& operator=(const RenderMesh&) = delete;

  void Upload(const std::vector<VertexPC>& vertices,
              const std::vector<std::uint32_t>& indices,
              int primitiveMode);

  void Clear();
  void Draw() const;

  std::size_t VertexCount() const { return m_vertices.size(); }
  std::size_t IndexCount()  const { return m_indices.size(); }

private:
  std::vector<VertexPC> m_vertices;
  std::vector<std::uint32_t> m_indices;
  int m_mode = 0; // GL_LINES / GL_TRIANGLES
};

} // namespace engine::render
