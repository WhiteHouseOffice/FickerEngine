#include "render/RenderMesh.h"

#ifdef __EMSCRIPTEN__
  #include <GLES/gl.h>
#else
  #include <GL/gl.h>
#endif

namespace engine::render {

void RenderMesh::Upload(const std::vector<VertexPC>& vertices,
                        const std::vector<std::uint16_t>& indices,
                        unsigned int primitive) {
  m_vertices = vertices;
  m_indices = indices;
  m_primitive = primitive;
}

void RenderMesh::Clear() {
  m_vertices.clear();
  m_indices.clear();
  m_primitive = 0;
}

void RenderMesh::Draw() const {
  if (m_vertices.empty() || m_indices.empty()) return;

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glVertexPointer(3, GL_FLOAT, sizeof(VertexPC), &m_vertices[0].x);
  glColorPointer(4, GL_FLOAT, sizeof(VertexPC), &m_vertices[0].r);

  glDrawElements(m_primitive,
                 (GLsizei)m_indices.size(),
                 GL_UNSIGNED_SHORT,
                 m_indices.data());

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

} // namespace engine::render
