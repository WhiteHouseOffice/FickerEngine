#include "render/RenderMesh.h"

#include <cstring>

namespace engine::render {

static void DestroyBuffer(unsigned int& id) {
    if (id != 0) {
        glDeleteBuffers(1, reinterpret_cast<GLuint*>(&id));
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
    m_indexCount  = 0;
}

void RenderMesh::Upload(const std::vector<VertexPC>& vertices,
                        const std::vector<std::uint32_t>& indices,
                        GLenum primitive) {
    m_primitive   = primitive;
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
    Draw(m_primitive);
}

void RenderMesh::Draw(GLenum primitive) const {
    if (m_vbo == 0 || m_ibo == 0 || m_indexCount == 0) return;

    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(m_vbo));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLuint>(m_ibo));

    // Fixed-function client arrays (requires -sLEGACY_GL_EMULATION=1 on web)
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3, GL_FLOAT, sizeof(VertexPC), reinterpret_cast<const void*>(0));
    glColorPointer(4, GL_FLOAT, sizeof(VertexPC), reinterpret_cast<const void*>(3 * sizeof(float)));

    glDrawElements(primitive,
                   static_cast<GLsizei>(m_indexCount),
                   GL_UNSIGNED_INT,
                   reinterpret_cast<const void*>(0));

    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

} // namespace engine::render
