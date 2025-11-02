#pragma once
#include <vector>
#include <cstdint>

struct MeshData {
    std::vector<float> positions;    // xyz interleaved
    std::vector<uint32_t> indices;   // element indices
    enum class Topology { Lines, Triangles } topology = Topology::Lines;
};

class RenderMesh {
public:
    bool Upload(const MeshData& d);
    void Draw(float r, float g, float b) const; // solid color
private:
    unsigned vbo_ = 0, ebo_ = 0;
    int indexCount_ = 0;
    int mode_ = 0; // GL_LINES or GL_TRIANGLES (numeric values used)
};
