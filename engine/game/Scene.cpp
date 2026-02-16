#include "game/Scene.h"
#include "game/GameObject.h"

#include "math/MiniMath.h"
#include "geom/TerrainGrid.h"
#include "render/RenderMesh.h"

#include <cstdio>
#include <cmath>
#include <memory>
#include <vector>

#ifdef FE_NATIVE
  #include <GL/gl.h>
#endif

// ------------------------------------------------------------
// Color unpack
// ------------------------------------------------------------
static void UnpackRGBA(uint32_t rgba, float& r, float& g, float& b, float& a) {
  r = float((rgba >> 24) & 0xFF) / 255.f;
  g = float((rgba >> 16) & 0xFF) / 255.f;
  b = float((rgba >>  8) & 0xFF) / 255.f;
  a = float((rgba >>  0) & 0xFF) / 255.f;
}

// ------------------------------------------------------------
// Spawn crates
// ------------------------------------------------------------
static void SpawnCrates(fe::PhysicsWorldRB& rb) {
  rb.clearDynamics();

  // two stacked crates
  {
    auto id = rb.createBox(Vec3(0.f, 2.0f, 0.f), Vec3(0.5f,0.5f,0.5f), 6.0f);
    if (auto* b = rb.get(id)) {
      b->linearDamping  = 0.10f;
      b->angularDamping = 0.10f;
      b->sleepVel       = 0.12f;
      b->sleepAngVel    = 0.18f;
      b->sleepTime      = 0.35f;
    }
  }
  {
    auto id = rb.createBox(Vec3(0.f, 3.2f, 0.f), Vec3(0.5f,0.5f,0.5f), 6.0f);
    if (auto* b = rb.get(id)) {
      b->linearDamping  = 0.10f;
      b->angularDamping = 0.10f;
      b->sleepVel       = 0.12f;
      b->sleepAngVel    = 0.18f;
      b->sleepTime      = 0.35f;
    }
  }
}

// ------------------------------------------------------------
// Terrain callbacks (physics queries use TerrainGrid vertex data)
// ------------------------------------------------------------
static float TerrainHeightCB(void* user, float x, float z) {
  auto* t = (engine::geom::TerrainGrid*)user;
  return t ? t->sampleHeight(x, z) : -1e30f;
}

static Vec3 TerrainNormalCB(void* user, float x, float z) {
  auto* t = (engine::geom::TerrainGrid*)user;
  if (!t) return Vec3(0.f, 1.f, 0.f);
  auto n = t->sampleNormal(x, z);
  return Vec3(n.x, n.y, n.z);
}

GameObject* Scene::createObject() {
  m_objects.push_back(std::make_unique<GameObject>());
  return m_objects.back().get();
}

void Scene::rebuildStaticAABBs() {
  m_static.clear();
  for (auto& obj : m_objects) {
    if (!obj->hasBoxCollider()) continue;
    fe::AABB a;
    const Vec3 he = obj->boxHalfExtents();
    a.min = obj->position - he;
    a.max = obj->position + he;
    m_static.push_back(a);
  }
  m_rb.setStaticAABBs(m_static);
}

void Scene::init() {
  m_objects.clear();

  // Terrain (render + physics sampling)
  m_terrain = engine::geom::TerrainGrid::make(
    80.0f, 80.0f, 1.0f,
    -0.25f, 0.6f
  );

  // Platforms (static AABBs)
  {
    auto* p = createObject();
    p->position = Vec3(3.75f, 1.35f, 0.0f);
    p->enableBoxCollider(Vec3(0.75f, 0.15f, 1.0f));
  }
  {
    auto* p = createObject();
    p->position = Vec3(-3.25f, 0.4f, 4.0f);
    p->enableBoxCollider(Vec3(0.75f, 0.2f, 1.0f));
  }

  rebuildStaticAABBs();

  // Physics tuning
  m_rb.gravity = Vec3(0.f, -18.0f, 0.f);

  // Terrain is the ground now
  m_rb.enableGround = false;
  m_rb.setTerrainCallbacks(&TerrainHeightCB, &TerrainNormalCB, &m_terrain);

  m_rb.friction = 0.7f;
  m_rb.restitution = 0.0f;
  m_rb.fixedDt = 1.0f / 120.0f;
  m_rb.maxSubsteps = 8;
  m_rb.velocityIters = 12;
  m_rb.positionIters = 6;

  SpawnCrates(m_rb);
}

void Scene::setPlayerSphere(const Vec3& center, float radius, const Vec3& velocity) {
  m_playerValid = true;
  m_playerCenter = center;
  m_playerRadius = radius;
  m_playerVel = velocity;
}

bool Scene::getPlayerSphere(Vec3& outCenter, Vec3& outVelocity, bool& outGrounded) const {
  outCenter = m_playerCenterOut;
  outVelocity = m_playerVelOut;
  outGrounded = m_playerGroundedOut;
  return true;
}

void Scene::update(float dt) {
  // Feed static colliders
  m_rb.setStaticAABBs(m_static);

  // Player collision
  if (m_playerValid) {
    m_rb.collidePlayerSphere(m_playerCenter, m_playerRadius, m_playerVel, &m_playerGroundedOut);
  }

  // Step physics (PhysicsWorldRB already uses its own accumulator)
  m_rb.step(dt);

  // Output player
  m_playerCenterOut = m_playerCenter;
  m_playerVelOut = m_playerVel;
}

void Scene::render(const Mat4& view, const Mat4& proj) {
  (void)view; (void)proj;
}

void Scene::renderDebug(const Mat4& view, const Mat4& proj) {
  (void)view; (void)proj;

#ifdef FE_NATIVE
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  // Load fixed-pipeline matrices (Mat4 assumed contiguous 16 floats)
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf((const float*)&proj);

  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf((const float*)&view);

  // Draw terrain triangles (already in world space)
  glBegin(GL_TRIANGLES);
  for (size_t i = 0; i < m_terrain.indices.size(); ++i) {
    const auto& v = m_terrain.vertices[m_terrain.indices[i]];
    float r,g,b,a;
    UnpackRGBA(v.rgba, r,g,b,a);
    glColor4f(r,g,b,a);
    glVertex3f(v.x, v.y, v.z);
  }
  glEnd();
#endif
}
