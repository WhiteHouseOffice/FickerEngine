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

// NEW
void RenderMesh::SetBackfaceCulling(bool enabled) {
  m_backfaceCull = enabled;
}

// NEW
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

// NEW
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

  // NEW: backface culling (only meaningful for filled triangles)
  if (m_primitive == Primitive::Triangles && m_backfaceCull) {
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(ToGLWinding(m_frontWinding));
  } else {
    glDisable(GL_CULL_FACE);
  }

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
