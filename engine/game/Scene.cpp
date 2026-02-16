#include "game/Scene.h"
#include "game/GameObject.h"

#include "geom/ColoredBox.h"
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
// Sanity helpers
// ------------------------------------------------------------
static bool fe_isfinite3(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static bool fe_isfiniteQuat(const fe::Quat& q) {
  return std::isfinite(q.w) && std::isfinite(q.x) &&
         std::isfinite(q.y) && std::isfinite(q.z);
}

static bool fe_isfiniteRigidBody(const fe::RigidBoxBody& b) {
  return fe_isfinite3(b.position)
      && fe_isfinite3(b.halfExtents)
      && fe_isfiniteQuat(b.orientation);
}

static fe::Quat IdentityQuat() {
  fe::Quat q; q.w = 1.f; q.x = q.y = q.z = 0.f; return q;
}

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
// Universal transformed mesh draw (local x,y,z,rgba + indices)
// ------------------------------------------------------------
template <typename V>
static void DrawTransformedMeshRGBA(
  const std::vector<V>& verts,
  const std::vector<uint32_t>& indices,
  const Mat4& M
) {
#ifdef FE_NATIVE
  glBegin(GL_TRIANGLES);
  for (size_t i = 0; i < indices.size(); ++i) {
    const V& v = verts[indices[i]];
    float r, g, b, a;
    UnpackRGBA(v.rgba, r,g,b,a);
    glColor4f(r,g,b,a);

    glVertex3f(v.x, v.y, v.z);
  }
  glEnd();
#else
  (void)verts; (void)indices; (void)M;
#endif
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
      b->linearDamping = 0.10f;
      b->angularDamping = 0.10f;
      b->sleepVel = 0.12f;
      b->sleepAngVel = 0.18f;
      b->sleepTime = 0.35f;
    }
  }
  {
    auto id = rb.createBox(Vec3(0.f, 3.2f, 0.f), Vec3(0.5f,0.5f,0.5f), 6.0f);
    if (auto* b = rb.get(id)) {
      b->linearDamping = 0.10f;
      b->angularDamping = 0.10f;
      b->sleepVel = 0.12f;
      b->sleepAngVel = 0.18f;
      b->sleepTime = 0.35f;
    }
  }
}

// ------------------------------------------------------------
// Terrain callbacks (physics queries use full TerrainGrid vertex data)
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

  // --- Terrain (build once; used for render + physics sampling) ---
  m_terrain = engine::geom::TerrainGrid::make(
    80.0f, 80.0f, 1.0f,
    -0.25f, 0.6f
  );

  // --- Example platforms ---
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

  // physics tuning
  m_rb.gravity = Vec3(0.f, -18.0f, 0.f);

  // Terrain is the ONLY ground now:
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
  m_accumDt += dt;

  // Feed static colliders
  m_rb.setStaticAABBs(m_static);

  // Feed player
  if (m_playerValid) {
    m_rb.collidePlayerSphere(m_playerCenter, m_playerRadius, m_playerVel, &m_playerGroundedOut);
  }

  // Fixed stepping
  static float s_accum = 0.0f;
  s_accum += dt;

  const float fixed = m_rb.fixedDt;
  int steps = 0;

  while (s_accum >= fixed && steps < m_rb.maxSubsteps) {
    m_rb.step(fixed);

    // IMPORTANT:
    // Removed the old “terrain floor constraint” Y-clamp.
    // Terrain collision is now handled through PhysicsWorldRB contacts using TerrainGrid vertices.

    s_accum -= fixed;
    steps++;
  }

  // Output player
  m_playerCenterOut = m_playerCenter;
  m_playerVelOut = m_playerVel;

  // sanity check
  for (const auto& b : m_rb.bodies()) {
    if (!fe_isfiniteRigidBody(b)) {
      std::printf("[physics] non-finite rigid body detected\n");
      break;
    }
  }
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

  // --- Terrain (grey hills) ---
  {
    // Draw mesh directly (already in world space)
    DrawTransformedMeshRGBA(m_terrain.vertices, m_terrain.indices, Mat4::identity());
  }

  // --- Platforms ---
  for (const auto& a : m_static) {
    // simple debug: draw as lines or omit (keeping as-is)
    (void)a;
  }
#endif
}
