#pragma once
#include <vector>
#include <cstdint>

#if defined(FE_WEB) || defined(__EMSCRIPTEN__)
  #include <GL/gl.h>
#else
  #if defined(__APPLE__)
    #include <OpenGL/gl.h>
  #else
    #include <GL/gl.h>
  #endif
#endif

namespace engine::render {

class RenderMesh {
public:
    struct VertexPC {
        float x, y, z;
        float r, g, b, a;
    };

    RenderMesh() = default;
    ~RenderMesh();

    void Clear();

    // Upload vertex/index data and remember primitive type for Draw().
    void Upload(const std::vector<VertexPC>& vertices,
                const std::vector<std::uint32_t>& indices,
                GLenum primitive);

    // Uses the last primitive passed to Upload()
    void Draw() const;

    // Explicit override if you want
    void Draw(GLenum primitive) const;

    std::size_t VertexCount() const { return m_vertexCount; }
    std::size_t IndexCount()  const { return m_indexCount;  }

private:
    unsigned int m_vbo = 0;
    unsigned int m_ibo = 0;

    std::size_t m_vertexCount = 0;
    std::size_t m_indexCount  = 0;

    GLenum m_primitive = GL_TRIANGLES;
};

} // namespace engine::render
