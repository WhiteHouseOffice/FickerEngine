#pragma once
#include <vector>
#include <cstdint>
struct MeshData {
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    enum class Topology { Lines, Triangles } topology = Topology::Lines;
};
class RenderMesh {
public:
    bool Upload(const MeshData& d);
    void Draw(float r, float g, float b) const;
private:
    unsigned vao_=0,vbo_=0,ebo_=0; int indexCount_=0; int mode_=0;
};
