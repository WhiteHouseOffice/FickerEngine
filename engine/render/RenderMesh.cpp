#include "render/RenderMesh.h"

#include <GL/gl.h>
#include <cstddef> // offsetof

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

void RenderMesh::SetBackfaceCulling(bool enabled) {
  m_backfaceCull = enabled;
}

void RenderMesh::SetFrontFaceWinding(Winding winding) {
  m_frontWinding = winding;
}

static GLenum ToGLPrimitive(RenderMesh::Primitive p) {
  switch (p) {
    case RenderMesh::Primitive::Triangles: return GL_TRIANGLES;
    case RenderMesh::Primitive::Lines:     return GL_LINES;
    default:                               return GL_TRIANGLES;
  }
}

static GLenum ToGLWinding(RenderMesh::Winding w) {
  switch (w) {
    case RenderMesh::Winding::CCW: return GL_CCW;
    case RenderMesh::Winding::CW:  return GL_CW;
    default:                       return GL_CCW;
  }
}

void RenderMesh::Draw() const {
  if (m_vertices.empty() || m_indices.empty()) return;

  const GLenum glPrim = ToGLPrimitive(m_primitive);

  // Backface culling for triangle surfaces
  if (m_primitive == Primitive::Triangles && m_backfaceCull) {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(ToGLWinding(m_frontWinding));
  } else {
    glDisable(GL_CULL_FACE);
  }

  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  // IMPORTANT:
  // We are using CPU-side arrays (std::vector). The pointer must reference m_vertices.data().
  glVertexPointer(
    3, GL_FLOAT, sizeof(VertexPC),
    (const void*)(&m_vertices[0].x)
  );

  glColorPointer(
    4, GL_FLOAT, sizeof(VertexPC),
    (const void*)(&m_vertices[0].r)
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
