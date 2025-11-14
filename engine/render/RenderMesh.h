#pragma once
#include <vector>
#include <cstdint>

// CPU geometry types – no namespace
#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

struct GridPlane;
struct MarkerCross;

namespace render {

class RenderMesh {
public:
  // For now this is purely CPU-side data that mirrors the geom structs
  std::vector<float>        positions;
  std::vector<std::uint32_t> indices;

  void clear();

  void uploadGrid(const GridPlane& grid);
  void uploadMarker(const MarkerCross& marker);
};

} // namespace render
