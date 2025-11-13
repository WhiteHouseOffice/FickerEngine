#pragma once

#include "math/MiniMath.h"  // Vec3, Mat4, lookAt, perspective

class Game {
public:
  // Initialise camera etc.
  void init();

  // Per-frame update with delta time in seconds
  void update(float dt);

  // Camera matrices
  Mat4 view() const;
  Mat4 proj(float aspect) const;

  // For debugging / other systems
  Vec3 cameraPos() const { return camPos; }

private:
  Vec3  camPos{0.f, 2.f, 5.f};
  float yaw   = 0.f;   // radians, rotation around Y
  float pitch = 0.f;   // radians, up/down

  float moveSpeed        = 5.f;      // units per second
  float mouseSensitivity = 0.0025f;  // not used yet, but nice to keep
  float logTimer         = 0.f;      // for periodic logging
};
