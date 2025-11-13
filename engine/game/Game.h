#pragma once

#include "math/MiniMath.h"

// Simple “game world” driver:
// - Holds camera position & orientation
// - Gets updated every frame by Engine
class Game {
public:
  // Called every frame from Engine with delta time in seconds
  void update(float dt);

  // Camera position in world space
  Vec3 camPos { 0.0f, 2.0f, 5.0f };

  // Yaw (around Y axis) and pitch (look up/down), in radians.
  float yaw   = 0.0f;
  float pitch = 0.0f;
};
