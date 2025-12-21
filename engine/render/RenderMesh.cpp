#include "render/RenderMesh.h"

#if defined(FE_WEB)
  #include <GL/gl.h>
#endif

namespace engine::render {

void RenderMesh::Clear() {
  m_vertices.clear();
  m_indices.clear();
  m_mode = 0;
}

void RenderMesh::Upload(const std::vector<VertexPC>& vertices,
                        const std::vector<std::uint32_t>& indices,
                        int primitiveMode) {
  m_vertices = vertices;
  m_indices  = indices;
  m_mode     = primitiveMode;
}

void RenderMesh::Draw() const {
#if defined(FE_WEB)
  if (m_vertices.empty() || m_indices.empty() || m_mode == 0) return;

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glVertexPointer(3, GL_FLOAT, sizeof(VertexPC), &m_vertices[0].x);
  glColorPointer(4, GL_FLOAT, sizeof(VertexPC), &m_vertices[0].r);

  glDrawElements(static_cast<GLenum>(m_mode),
                 static_cast<GLsizei>(m_indices.size()),
                 GL_UNSIGNED_INT,
                 m_indices.data());

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
#endif
}

} // namespace engine::render
