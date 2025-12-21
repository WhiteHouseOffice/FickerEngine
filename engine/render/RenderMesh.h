#pragma once
#include <cstdint>
#include <vector>

namespace engine::render {

class RenderMesh {
public:
  struct VertexPC {
    float x, y, z;
    float r, g, b, a;
  };

  RenderMesh() = default;
  ~RenderMesh() = default;

  void Upload(const std::vector<VertexPC>& vertices,
              const std::vector<std::uint16_t>& indices,
              unsigned int primitive);

  void Draw() const;
  void Clear();

private:
  std::vector<VertexPC> m_vertices;
  std::vector<std::uint16_t> m_indices;
  unsigned int m_primitive = 0;
};

} // namespace engine::render
