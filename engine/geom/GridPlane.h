#pragma once
#include <vector>
#include <cstdint>

namespace geom {

// Builds a line-grid on XZ-plane centered at origin.
// size: total width/depth (meters), step: grid spacing (meters)
struct GridData {
    std::vector<float>   positions; // xyz triplets
    std::vector<uint32_t> indices;  // line list indices
};

GridData BuildGrid(float size = 20.f, float step = 1.f);

} // namespace geom
