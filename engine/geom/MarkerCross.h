#pragma once
#include <vector>
#include <cstdint>

namespace geom {

// Simple 3-axis marker centered at origin, length in meters
struct LinesData {
    std::vector<float>    positions; // xyz triplets
    std::vector<uint32_t> indices;   // line list
};

LinesData BuildMarkerCross(float length = 0.25f);

} // namespace geom
