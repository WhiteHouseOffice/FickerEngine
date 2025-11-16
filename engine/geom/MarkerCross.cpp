#include "geom/MarkerCross.h"

namespace geom {

MarkerCross MarkerCross::MakeOrigin(float size) {
  MarkerCross m;
  const float s = size;

  // X axis (red in your head)
  m.positions.push_back(Vec3{-s, 0.f, 0.f});
  m.positions.push_back(Vec3{ s, 0.f, 0.f});
  m.indices.push_back(0);
  m.indices.push_back(1);

  // Y axis (green)
  m.positions.push_back(Vec3{0.f, -s, 0.f});
  m.positions.push_back(Vec3{0.f,  s, 0.f});
  m.indices.push_back(2);
  m.indices.push_back(3);

  // Z axis (blue)
  m.positions.push_back(Vec3{0.f, 0.f, -s});
  m.positions.push_back(Vec3{0.f, 0.f,  s});
  m.indices.push_back(4);
  m.indices.push_back(5);

  return m;
}

} // namespace geom
