#pragma once
#include <vector>
#include <cstdint>

namespace geom {
struct GridData {
    std::vector<float>    positions; // xyz triplets
    std::vector<uint32_t> indices;   // line list indices
};
GridData BuildGrid(float size = 20.f, float step = 1.f);
} // namespace geom
