#include "game/physics/PhysicsWorld.h"
#include "game/GameObject.h"

#include <algorithm>
#include <cmath>

static inline float feAbs(float x) { return x < 0.f ? -x : x; }

void PhysicsWorld::step(float dt, const std::vector<GameObject*>& objects) {
  if (dt <= 0.0f) return;

  // Integrate dynamics
  for (GameObject* obj : objects) {
    if (!obj) continue;

    RigidBody* rb = obj->rigidBody();
    if (!rb || !rb->isDynamic()) continue;

    integrate(*obj, *rb, dt);

    if (enableGroundPlane) {
      solveGroundPlane(*obj, *rb);
    }

    // Push interaction with player sphere (optional)
    if (m_playerEnabled) {
      solvePlayerSpherePush(*obj, *rb);
    }

    rb->clearForces();
  }

  // Box-vs-box collisions (cheap, few iterations)
  if (solverIterations > 0) {
    std::vector<GameObject*> ptrs;
    ptrs.reserve(objects.size());
    for (auto* o : objects) if (o) ptrs.push_back(o);

    for (int it = 0; it < solverIterations; ++it) {
      solveBoxCollisions(ptrs);

      if (enableGroundPlane) {
        for (auto* o : ptrs) {
          if (!o) continue;
          if (RigidBody* rb = o->rigidBody()) {
            if (rb->isDynamic()) solveGroundPlane(*o, *rb);
          }
        }
      }
    }
  }
}

void PhysicsWorld::integrate(GameObject& obj, RigidBody& rb, float dt) {
  Vec3 accel{0.f, 0.f, 0.f};

  if (rb.useGravity) {
    accel = accel + gravity;
  }

  accel = accel + (rb.forceAccum * rb.invMass);

  // Semi-implicit Euler
  rb.velocity = rb.velocity + (accel * dt);

  if (rb.linearDamping > 0.0f) {
    float k = 1.0f - rb.linearDamping * dt;
    if (k < 0.0f) k = 0.0f;
    rb.velocity = rb.velocity * k;
  }

  obj.position = obj.position + (rb.velocity * dt);
}

void PhysicsWorld::solveGroundPlane(GameObject& obj, RigidBody& rb) {
  float bottomY = obj.position.y;
  if (obj.hasBoxCollider()) {
    bottomY = obj.position.y - obj.boxHalfExtents().y;
  }

  if (bottomY >= groundY) return;

  // move up to rest on plane
  const float pen = groundY - bottomY;
  obj.position.y += pen;

  if (rb.velocity.y < 0.0f) {
    rb.velocity.y = -rb.velocity.y * groundRestitution;
  }

  if (groundFriction > 0.0f) {
    float k = 1.0f - groundFriction;
    if (k < 0.0f) k = 0.0f;
    rb.velocity.x *= k;
    rb.velocity.z *= k;
  }
}

bool PhysicsWorld::overlapAABB(const Vec3& aPos, const Vec3& aHalf,
                              const Vec3& bPos, const Vec3& bHalf,
                              Vec3& outMTV)
{
  const Vec3 d = bPos - aPos;

  const float px = (aHalf.x + bHalf.x) - feAbs(d.x);
  if (px <= 0.f) return false;

  const float py = (aHalf.y + bHalf.y) - feAbs(d.y);
  if (py <= 0.f) return false;

  const float pz = (aHalf.z + bHalf.z) - feAbs(d.z);
  if (pz <= 0.f) return false;

  outMTV = Vec3(0.f, 0.f, 0.f);
  if (px < py && px < pz) {
    outMTV.x = (d.x < 0.f) ? -px : px;
  } else if (py < pz) {
    outMTV.y = (d.y < 0.f) ? -py : py;
  } else {
    outMTV.z = (d.z < 0.f) ? -pz : pz;
  }
  return true;
}

void PhysicsWorld::solveBoxCollisions(std::vector<GameObject*>& objects)
{
  const int n = (int)objects.size();
  for (int i = 0; i < n; ++i) {
    GameObject* a = objects[i];
    if (!a || !a->hasBoxCollider()) continue;

    for (int j = i + 1; j < n; ++j) {
      GameObject* b = objects[j];
      if (!b || !b->hasBoxCollider()) continue;

      Vec3 mtv;
      if (!overlapAABB(a->position, a->boxHalfExtents(), b->position, b->boxHalfExtents(), mtv)) {
        continue;
      }

      RigidBody* aRb = a->rigidBody();
      RigidBody* bRb = b->rigidBody();

      const float invA = (aRb && aRb->isDynamic()) ? aRb->invMass : 0.f;
      const float invB = (bRb && bRb->isDynamic()) ? bRb->invMass : 0.f;
      const float invSum = invA + invB;
      if (invSum <= 0.f) continue;

      // positional correction (split by inverse mass)
      a->position = a->position - (mtv * (invA / invSum));
      b->position = b->position + (mtv * (invB / invSum));

      // kill velocity along MTV axis (simple, stable)
      if (aRb && aRb->isDynamic()) {
        if (mtv.x != 0.f) aRb->velocity.x = 0.f;
        if (mtv.y != 0.f) aRb->velocity.y = 0.f;
        if (mtv.z != 0.f) aRb->velocity.z = 0.f;
      }
      if (bRb && bRb->isDynamic()) {
        if (mtv.x != 0.f) bRb->velocity.x = 0.f;
        if (mtv.y != 0.f) bRb->velocity.y = 0.f;
        if (mtv.z != 0.f) bRb->velocity.z = 0.f;
      }
    }
  }
}

void PhysicsWorld::solvePlayerSpherePush(GameObject& obj, RigidBody& rb)
{
  if (!obj.hasBoxCollider()) return;

  // Sphere vs AABB: push the box out along normal; do not move player.
  const Vec3 half = obj.boxHalfExtents();
  const Vec3 mn = obj.position - half;
  const Vec3 mx = obj.position + half;

  // closest point on box to sphere center
  const float cx = std::min(std::max(m_playerCenter.x, mn.x), mx.x);
  const float cy = std::min(std::max(m_playerCenter.y, mn.y), mx.y);
  const float cz = std::min(std::max(m_playerCenter.z, mn.z), mx.z);

  Vec3 closest(cx, cy, cz);
  Vec3 d = closest - m_playerCenter;
  const float dist2 = dot(d, d);
  const float r = m_playerRadius;
  if (dist2 > r * r) return;

  float dist = std::sqrt(std::max(dist2, 1e-8f));
  Vec3 n(0.f, 1.f, 0.f);
  if (dist > 1e-6f) {
    n = d * (1.0f / dist);
  }

  // move the box outward
  const float penetration = r - dist;
  obj.position = obj.position + (n * penetration);

  // push velocity away from player (only if moving into player)
  const float vn = dot(rb.velocity, n);
  if (vn < 0.f) {
    rb.velocity = rb.velocity - (n * vn);
  }
}
