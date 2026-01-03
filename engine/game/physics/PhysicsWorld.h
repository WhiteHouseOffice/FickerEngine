#pragma once

#include <vector>

#include "math/MiniMath.h"
#include "game/physics/RigidBody.h"

class GameObject;

// Extremely simple physics world:
// - integrates lots of objects cheaply
// - optional infinite ground plane
// - optional AABB box colliders on GameObject
// - optional kinematic player sphere that can push dynamic boxes
class PhysicsWorld {
public:
  Vec3 gravity{0.f, -9.81f, 0.f};

  // Simple ground plane (y = groundY)
  bool  enableGroundPlane = true;
  float groundY = 0.0f;

  // 0 = no bounce
  float groundRestitution = 0.0f;

  // Simple friction applied on contact
  float groundFriction = 0.0f;

  // Box collision solver (keep small for speed)
  int solverIterations = 2;

  // Kinematic player sphere proxy (optional)
  void setPlayerSphere(const Vec3& center, float radius) {
    m_playerEnabled = true;
    m_playerCenter = center;
    m_playerRadius = radius;
  }
  void clearPlayerSphere() { m_playerEnabled = false; }

  void step(float dt, const std::vector<GameObject*>& objects);

private:
  void integrate(GameObject& obj, RigidBody& rb, float dt);
  void solveGroundPlane(GameObject& obj, RigidBody& rb);

  // AABB collisions (naive O(n^2))
  void solveBoxCollisions(std::vector<GameObject*>& objects);

  // Player sphere pushes boxes (does not move player)
  void solvePlayerSpherePush(GameObject& obj, RigidBody& rb);

  // Helpers
  static bool overlapAABB(const Vec3& aPos, const Vec3& aHalf,
                          const Vec3& bPos, const Vec3& bHalf,
                          Vec3& outMTV);

  bool m_playerEnabled = false;
  Vec3  m_playerCenter{0.f, 0.f, 0.f};
  float m_playerRadius = 0.5f;
};
