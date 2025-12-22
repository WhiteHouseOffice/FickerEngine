#include "render/RenderMesh.h"

#include <GL/gl.h>   // legacy fixed-function declarations
#include <cstddef>   // offsetof

namespace engine::render {

void RenderMesh::Clear() {
  m_vertices.clear();
  m_indices.clear();
}

void RenderMesh::SetPrimitive(Primitive prim) {
  m_primitive = prim;
}

void RenderMesh::SetVertices(const std::vector<VertexPC>& verts) {
  m_vertices = verts;
}

void RenderMesh::SetIndices(const std::vector<uint16_t>& inds) {
  m_indices = inds;
}

static GLenum ToGLPrimitive(RenderMesh::Primitive p) {
  switch (p) {
    case RenderMesh::Primitive::Triangles: return GL_TRIANGLES;
    case RenderMesh::Primitive::Lines:     return GL_LINES;
    default:                               return GL_TRIANGLES;
  }
}

void RenderMesh::Draw() const {
  if (m_vertices.empty() || m_indices.empty()) return;

  const GLenum glPrim = ToGLPrimitive(m_primitive);

  // Legacy fixed-function vertex arrays (no shader files)
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  glVertexPointer(
    3, GL_FLOAT, sizeof(VertexPC),
    reinterpret_cast<const void*>(offsetof(VertexPC, x))
  );

  glColorPointer(
    4, GL_FLOAT, sizeof(VertexPC),
    reinterpret_cast<const void*>(offsetof(VertexPC, r))
  );

  glDrawElements(
    glPrim,
    static_cast<GLsizei>(m_indices.size()),
    GL_UNSIGNED_SHORT,
    m_indices.data()
  );

  glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

} // namespace engine::render
