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
    ~RenderMesh();

    RenderMesh(const RenderMesh&) = delete;
    RenderMesh& operator=(const RenderMesh&) = delete;

    // Upload CPU data to GPU buffers (VBO/IBO). Safe to call multiple times; re-uploads.
    void Upload(const std::vector<VertexPC>& vertices,
                const std::vector<std::uint32_t>& indices);

    // Draw with fixed-function emulation (LEGACY_GL_EMULATION).
    void Draw() const;

    void Clear();

    std::size_t VertexCount() const { return m_vertexCount; }
    std::size_t IndexCount()  const { return m_indexCount;  }

private:
    unsigned int m_vbo = 0;
    unsigned int m_ibo = 0;

    std::size_t m_vertexCount = 0;
    std::size_t m_indexCount  = 0;
};

} // namespace engine::render
