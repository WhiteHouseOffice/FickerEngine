#include "render/RenderMesh.h"

#if defined(__EMSCRIPTEN__)
  #include <GLES2/gl2.h>
#else
  // If you build native with a GL loader, include it here instead.
  // For legacy desktop GL without a loader you can try:
  #if defined(__APPLE__)
    #include <OpenGL/gl.h>
  #else
    #include <GL/gl.h>
  #endif
#endif

#include <cstring>

namespace engine::render {

static void DestroyBuffer(unsigned int& id) {
    if (id != 0) {
        GLuint buf = static_cast<GLuint>(id);
        glDeleteBuffers(1, &buf);
        id = 0;
    }
}

RenderMesh::~RenderMesh() {
    Clear();
}

void RenderMesh::Clear() {
    DestroyBuffer(m_vbo);
    DestroyBuffer(m_ibo);
    m_vertexCount = 0;
    m_indexCount = 0;
}

void RenderMesh::Upload(const std::vector<VertexPC>& vertices,
                        const std::vector<std::uint32_t>& indices) {
    m_vertexCount = vertices.size();
    m_indexCount  = indices.size();

    if (m_vertexCount == 0 || m_indexCount == 0) {
        Clear();
        return;
    }

    if (m_vbo == 0) glGenBuffers(1, reinterpret_cast<GLuint*>(&m_vbo));
    if (m_ibo == 0) glGenBuffers(1, reinterpret_cast<GLuint*>(&m_ibo));

    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(m_vbo));
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexPC)),
                 vertices.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLuint>(m_ibo));
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(std::uint32_t)),
                 indices.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void RenderMesh::Draw() const {
    if (m_vbo == 0 || m_ibo == 0 || m_indexCount == 0) return;

    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(m_vbo));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLuint>(m_ibo));

    // Fixed-function style pointers. With -sLEGACY_GL_EMULATION=1 Emscripten
    // maps these to an internal shader pipeline for WebGL.
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_FLOAT, sizeof(VertexPC), reinterpret_cast<const void*>(0));
    glColorPointer(4, GL_FLOAT, sizeof(VertexPC), reinterpret_cast<const void*>(3 * sizeof(float)));

    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(m_indexCount),
                   GL_UNSIGNED_INT,
                   reinterpret_cast<const void*>(0));

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

} // namespace engine::render
