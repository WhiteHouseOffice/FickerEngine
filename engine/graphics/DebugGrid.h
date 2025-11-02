#pragma once
#include <vector>
#include <cstdint>

struct GridVertex {
    float x, y, z;
    float r, g, b;
};

class DebugGrid {
public:
    DebugGrid();
    ~DebugGrid();

    // Call at startup after GL/WGPU context exists
    void Init(int halfLines = 10, float step = 1.0f);

    // Per-frame render (expects a valid shader with uMVP and aPos/aColor)
    void Render(const float* mvp /* 16 floats, column-major */);

private:
    std::vector<GridVertex> vertices_;
    std::vector<uint32_t>   indices_;

    // GL objects (no-ops on non-GL backends)
    unsigned vao_ = 0, vbo_ = 0, ebo_ = 0;
    int indexCount_ = 0;

    void buildCPU(int halfLines, float step);
    void buildGL();   // create VAO/VBO/EBO for WebGL2/OpenGL
    void destroyGL();
};
