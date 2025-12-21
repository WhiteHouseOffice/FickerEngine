#pragma once
#include <vector>
#include <cstdint>

namespace engine::render {

class RenderMesh {
public:
    struct VertexPC {
        float x, y, z;
        float r, g, b, a;
    };

    RenderMesh() = default;
    ~RenderMesh() = default;

    RenderMesh(const RenderMesh&) = delete;
    RenderMesh& operator=(const RenderMesh&) = delete;

    // Upload CPU data for drawing via Emscripten LEGACY_GL_EMULATION.
    // We intentionally keep this "CPU-side" for now to avoid needing GL loader headers
    // across platforms while we iterate on the scene.
    //
    // "primitive" is usually GL_LINES or GL_TRIANGLES.
    void Upload(const std::vector<VertexPC>& vertices,
                const std::vector<std::uint16_t>& indices,
                unsigned int primitive);

    // Draw with fixed-function emulation (LEGACY_GL_EMULATION).
    void Draw() const;

    void Clear();

    std::size_t VertexCount() const { return m_vertexCount; }
    std::size_t IndexCount()  const { return m_indexCount;  }

private:
    std::vector<VertexPC> m_vertices;
    std::vector<std::uint16_t> m_indices;

    std::size_t m_vertexCount = 0;
    std::size_t m_indexCount  = 0;

    unsigned int m_primitive = 0; // GLenum stored as uint
};

} // namespace engine::render
