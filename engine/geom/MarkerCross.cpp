#include "geom/MarkerCross.h"

namespace geom {

void MarkerCross::build(float size) {
  positions.clear();
  indices.clear();

  float s = size;

  // X axis
  positions.push_back(Vec3(-s, 0, 0));
  positions.push_back(Vec3( s, 0, 0));

  // Y axis
  positions.push_back(Vec3(0, -s, 0));
  positions.push_back(Vec3(0,  s, 0));

  // Z axis
  positions.push_back(Vec3(0, 0, -s));
  positions.push_back(Vec3(0, 0,  s));

  for (int i = 0; i < positions.size(); i++)
    indices.push_back(i);
}

} // namespace geom
