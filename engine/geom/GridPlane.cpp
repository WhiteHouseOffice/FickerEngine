#include "geom/GridPlane.h"
#include <cmath>

namespace geom {

GridData BuildGrid(float size, float step) {
    GridData g;
    if (step <= 0.f) step = 1.f;
    int lines = static_cast<int>(std::floor(size / step));
    if (lines < 1) lines = 1;
    const float half = lines * step * 0.5f;

    // Z-directed lines at varying X
    for (int i = -lines; i <= lines; ++i) {
        const float x = i * step;
        g.positions.insert(g.positions.end(), { x, 0.f, -half,  x, 0.f, +half });
        uint32_t idx0 = static_cast<uint32_t>((g.positions.size()/3) - 2);
        uint32_t idx1 = idx0 + 1;
        g.indices.push_back(idx0);
        g.indices.push_back(idx1);
    }
    // X-directed lines at varying Z
    for (int j = -lines; j <= lines; ++j) {
        const float z = j * step;
        g.positions.insert(g.positions.end(), { -half, 0.f, z,  +half, 0.f, z });
        uint32_t idx0 = static_cast<uint32_t>((g.positions.size()/3) - 2);
        uint32_t idx1 = idx0 + 1;
        g.indices.push_back(idx0);
        g.indices.push_back(idx1);
    }
    return g;
}

} // namespace geom
