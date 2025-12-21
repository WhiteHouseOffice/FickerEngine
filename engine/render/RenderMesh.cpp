#include "render/RenderMesh.h"

// Web build relies on Emscripten's LEGACY_GL_EMULATION fixed-function compatibility layer
// (glMatrixMode, glVertexPointer, glEnableClientState, etc.). Those APIs are declared in
// <GL/gl.h> in the Emscripten sysroot.
#if defined(__EMSCRIPTEN__) || defined(FE_WEB)
  #include <GL/gl.h>
#else
  #if defined(__APPLE__)
    #include <OpenGL/gl.h>
  #else
    #include <GL/gl.h>
  #endif
#endif

namespace engine::render {

void RenderMesh::Clear() {
    m_vertices.clear();
    m_indices.clear();
    m_vertexCount = 0;
    m_indexCount = 0;
    m_primitive = 0;
}

void RenderMesh::Upload(const std::vector<VertexPC>& vertices,
                        const std::vector<std::uint16_t>& indices,
                        unsigned int primitive) {
    m_vertices = vertices;
    m_indices  = indices;
    m_vertexCount = m_vertices.size();
    m_indexCount  = m_indices.size();
    m_primitive   = primitive;

    if (m_vertexCount == 0 || m_indexCount == 0) {
        Clear();
    }
}

void RenderMesh::Draw() const {
    if (m_vertices.empty() || m_indices.empty() || m_primitive == 0) return;

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_FLOAT, sizeof(VertexPC), &m_vertices[0].x);
    glColorPointer(4, GL_FLOAT, sizeof(VertexPC), &m_vertices[0].r);

    glDrawElements(static_cast<GLenum>(m_primitive),
                   static_cast<GLsizei>(m_indices.size()),
                   GL_UNSIGNED_SHORT,
                   m_indices.data());

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}

} // namespace engine::render
