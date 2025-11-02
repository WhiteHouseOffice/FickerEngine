#include "geom/MarkerCross.h"

namespace geom {

LinesData BuildMarkerCross(float L) {
    LinesData d;
    // X axis: (-L,0,0) -> (L,0,0)
    d.positions.insert(d.positions.end(), { -L,0,0,  L,0,0 });
    d.indices.push_back(0); d.indices.push_back(1);
    // Y axis: (0,-L,0) -> (0,L,0)
    d.positions.insert(d.positions.end(), { 0,-L,0,  0,L,0 });
    d.indices.push_back(2); d.indices.push_back(3);
    // Z axis: (0,0,-L) -> (0,0,L)
    d.positions.insert(d.positions.end(), { 0,0,-L,  0,0,L });
    d.indices.push_back(4); d.indices.push_back(5);
    return d;
}

} // namespace geom
