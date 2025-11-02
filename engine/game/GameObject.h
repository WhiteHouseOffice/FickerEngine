#pragma once
#include "engine/render/RenderMesh.h"
#include <array>
struct GameObject {
    RenderMesh mesh;
    std::array<float,3> color{0.9f,0.9f,0.9f};
};
