#include "geom/GridPlane.h"
#include <cmath>

namespace geom {

GridData BuildGrid(float size, float step) {
    GridData g;
    if (step <= 0.f) step = 1.f;
    int lines = static_cast<int>(std::floor(size / step));
    if (lines < 1) lines = 1;
    float half = lines * step * 0.5f;

    // We'll generate separate vertices for each line end (simplifies)
    // Vertical lines (along Z), varying X
    for (int i = -lines; i <= lines; ++i) {
        float x = i * step;
        // from (x,0,-half) to (x,0,+half)
        g.positions.insert(g.positions.end(), { x, 0.f, -half,  x, 0.f, +half });
        uint32_t base = static_cast<uint32_t>(g.indices.size()); // not used; we push absolute indices below
        uint32_t idx0 = static_cast<uint32_t>((g.positions.size()/3) - 2);
        uint32_t idx1 = idx0 + 1;
        g.indices.push_back(idx0);
        g.indices.push_back(idx1);
    }

    // Horizontal lines (along X), varying Z
    for (int j = -lines; j <= lines; ++j) {
        float z = j * step;
        // from (-half,0,z) to (+half,0,z)
        g.positions.insert(g.positions.end(), { -half, 0.f, z,  +half, 0.f, z });
        uint32_t idx0 = static_cast<uint32_t>((g.positions.size()/3) - 2);
        uint32_t idx1 = idx0 + 1;
        g.indices.push_back(idx0);
        g.indices.push_back(idx1);
    }

    return g;
}

} // namespace geom
