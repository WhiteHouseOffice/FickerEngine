#pragma once

// Plain placeholder GameObject for now.
// We'll expand this later when we actually need per-object logic.
class GameObject {
public:
  GameObject() = default;

  // Per-frame update; currently a no-op.
  void update(float dt);
};
