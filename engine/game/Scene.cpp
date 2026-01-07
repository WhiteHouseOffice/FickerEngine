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
// Terrain height (must match TerrainGrid::make parameters)
// ------------------------------------------------------------
static float TerrainHeight(float x, float z) {
  const float baseY = -0.25f;
  const float amp   = 0.6f;

  // wide bump + gentle waves
  const float r2 = x*x + z*z;
  const float sigma = 0.45f * (0.5f * 80.0f); // matches sizeX=80
  const float hill  = std::exp(-r2 / (2.0f * sigma * sigma));
  const float wav   = 0.35f * std::sin(0.18f * x) * std::cos(0.16f * z);

  return baseY + amp * (0.75f * hill + wav);
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
  const std::vector<V>& localVerts,
  const std::vector<uint32_t>& localInds,
  const Vec3& pos,
  const fe::Quat& rot,
  const Vec3& scale,
  bool backfaceCull
) {
#ifdef FE_NATIVE
  using fe::quatNormalize;
  using fe::quatRotate;

  const fe::Quat qn = quatNormalize(rot);

  std::vector<engine::render::VertexPC> verts;
  verts.reserve(localVerts.size());

  for (const auto& v : localVerts) {
    // V must have: v.x v.y v.z v.rgba
    Vec3 pLocal(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    Vec3 pWorld = pos + quatRotate(qn, pLocal);

    float r,g,b,a;
    UnpackRGBA(v.rgba, r,g,b,a);
    verts.push_back({ pWorld.x, pWorld.y, pWorld.z, r,g,b,a });
  }

  std::vector<uint16_t> inds;
  inds.reserve(localInds.size());
  for (uint32_t i : localInds) inds.push_back((uint16_t)i);

  engine::render::RenderMesh mesh;
  mesh.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);
  mesh.SetBackfaceCulling(backfaceCull);
  mesh.SetFrontFaceWinding(engine::render::RenderMesh::Winding::CW); // your current convention
  mesh.SetVertices(verts);
  mesh.SetIndices(inds);
  mesh.Draw();
#else
  (void)localVerts; (void)localInds; (void)pos; (void)rot; (void)scale; (void)backfaceCull;
#endif
}

// ------------------------------------------------------------
// Cached unit meshes (centered at origin, half-extents=1)
// Uniform colors per face
// ------------------------------------------------------------
static const engine::geom::ColoredBox& UnitCrateBox() {
  static engine::geom::ColoredBox box =
    engine::geom::ColoredBox::make(
      0,0,0, 1,1,1,
      engine::geom::ColoredBox::RGBA(210,160,90,255),
      engine::geom::ColoredBox::RGBA(210,160,90,255),
      engine::geom::ColoredBox::RGBA(210,160,90,255),
      engine::geom::ColoredBox::RGBA(210,160,90,255),
      engine::geom::ColoredBox::RGBA(210,160,90,255),
      engine::geom::ColoredBox::RGBA(210,160,90,255)
    );
  return box;
}

static const engine::geom::ColoredBox& UnitPlatformBox() {
  static engine::geom::ColoredBox box =
    engine::geom::ColoredBox::make(
      0,0,0, 1,1,1,
      engine::geom::ColoredBox::RGBA(90,140,220,255),
      engine::geom::ColoredBox::RGBA(90,140,220,255),
      engine::geom::ColoredBox::RGBA(90,140,220,255),
      engine::geom::ColoredBox::RGBA(90,140,220,255),
      engine::geom::ColoredBox::RGBA(90,140,220,255),
      engine::geom::ColoredBox::RGBA(90,140,220,255)
    );
  return box;
}

// ------------------------------------------------------------
// Scene object lifecycle
// ------------------------------------------------------------
GameObject* Scene::createObject() {
  auto obj = std::make_unique<GameObject>();
  GameObject* out = obj.get();
  m_objects.emplace_back(std::move(obj));
  return out;
}

void Scene::rebuildStaticAABBs() {
  m_static.clear();
  m_static.reserve(m_objects.size());

  for (auto& o : m_objects) {
    if (!o || !o->hasBoxCollider()) continue;

    const Vec3 he = o->boxHalfExtents();
    fe::AABB a;
    a.min = o->position - he;
    a.max = o->position + he;
    m_static.push_back(a);
  }

  m_rb.setStaticAABBs(m_static);
}

template <typename RB>
static void SpawnCrates(RB& rb) {
  rb.createBox(Vec3(0.0f, 5.0f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
  rb.createBox(Vec3(0.0f, 6.2f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
  rb.createBox(Vec3(3.75f, 5.6f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
}

// ------------------------------------------------------------
// Required Scene API (Engine links these)
// ------------------------------------------------------------
void Scene::init() {
  // platforms (still colliders for rigid bodies)
  {
    auto* p = createObject();
    p->position = Vec3(0.0f, 0.65f, 0.0f);
    p->enableBoxCollider(Vec3(2.0f, 0.15f, 2.0f));
  }
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
  m_rb.enableGround = false; // terrain is the floor now
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
  if (!m_playerValid) return false;
  outCenter = m_playerCenterOut;
  outVelocity = m_playerVelOut;
  outGrounded = m_playerGroundedOut;
  return true;
}

void Scene::update(float dt) {
  static float s_accum = 0.0f;

  // gameplay update
  for (auto& obj : m_objects) {
    if (obj) obj->update(dt);
  }

  rebuildStaticAABBs();

  if (dt < 0.f) dt = 0.f;

  const float fixed = m_rb.fixedDt;
  const float maxDt = fixed * (float)m_rb.maxSubsteps;
  if (dt > maxDt) dt = maxDt;

  s_accum += dt;
  if (s_accum > maxDt) s_accum = maxDt;

  int steps = 0;
  while (s_accum >= fixed && steps < m_rb.maxSubsteps) {
    m_rb.step(fixed);

    // --- terrain floor constraint (simple but effective for gentle hills) ---
    for (auto& b : m_rb.bodiesMutable()) {
      const float floorY = TerrainHeight(b.position.x, b.position.z);
      const float bottom = b.position.y - b.halfExtents.y;

      if (bottom < floorY) {
        b.position.y += (floorY - bottom);
        if (b.linearVelocity.y < 0.f) b.linearVelocity.y = 0.f;
      }
    }

    s_accum -= fixed;
    steps++;
  }

  // non-finite guard (keep player stable)
  const auto& bodies = m_rb.bodies();
  bool bad = false;
  for (int i = 0; i < (int)bodies.size() && i < 3; ++i) {
    if (!fe_isfiniteRigidBody(bodies[i])) { bad = true; break; }
  }

  if (bad) {
    m_playerCenterOut = m_playerCenter;
    m_playerVelOut    = m_playerVel;
    m_playerGroundedOut = false;
    return;
  }

  // player collision (still uses RB world)
  m_playerCenterOut = m_playerCenter;
  m_playerVelOut = m_playerVel;
  m_playerGroundedOut = false;

  if (m_playerValid) {
    (void)m_rb.collidePlayerSphere(m_playerCenterOut, m_playerRadius, m_playerVelOut, &m_playerGroundedOut);
  }
}

void Scene::render(const Mat4& view, const Mat4& proj) {
  // keep your engine pattern: render via debug path
  renderDebug(view, proj);
}

// ------------------------------------------------------------
// Debug render (now: terrain + solid objects)
// ------------------------------------------------------------
static void LoadMat4_GL(int mode, const Mat4& M) {
#ifdef FE_NATIVE
  glMatrixMode(mode);
  glLoadMatrixf(M.m);
#else
  (void)mode; (void)M;
#endif
}

void Scene::renderDebug(const Mat4& view, const Mat4& proj) {
#ifdef FE_NATIVE
  LoadMat4_GL(GL_PROJECTION, proj);
  LoadMat4_GL(GL_MODELVIEW,  view);

  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  // --- Terrain (grey hills) ---
  {
    auto t = engine::geom::TerrainGrid::make(
      80.0f, 80.0f, 1.0f,
      -0.25f, 0.6f
    );

    std::vector<engine::render::VertexPC> tv;
    tv.reserve(t.vertices.size());
    for (const auto& v : t.vertices) {
      float r,g,b,a;
      UnpackRGBA(v.rgba, r,g,b,a);
      tv.push_back({ v.x, v.y, v.z, r,g,b,a });
    }

    std::vector<uint16_t> ti;
    ti.reserve(t.indices.size());
    for (uint32_t i : t.indices) ti.push_back((uint16_t)i);

    engine::render::RenderMesh tm;
    tm.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);

    // double-sided so you can't be "under it / wrong side"
    tm.SetBackfaceCulling(false);

    tm.SetFrontFaceWinding(engine::render::RenderMesh::Winding::CW);
    tm.SetVertices(tv);
    tm.SetIndices(ti);
    tm.Draw();
  }

  // --- Solid platforms (uniform blue) ---
  {
    const auto& unit = UnitPlatformBox();
    for (const auto& a : m_static) {
      Vec3 center = (a.min + a.max) * 0.5f;
      Vec3 half   = (a.max - a.min) * 0.5f;

      DrawTransformedMeshRGBA(unit.vertices, unit.indices,
                              center, IdentityQuat(), half,
                              true);
    }
  }

  // --- Solid crates (uniform orange, move+rotate with physics) ---
  {
    const auto& unit = UnitCrateBox();
    for (const auto& b : m_rb.bodies()) {
      DrawTransformedMeshRGBA(unit.vertices, unit.indices,
                              b.position, b.orientation, b.halfExtents,
                              true);
    }
  }

#else
  (void)view; (void)proj;
#endif
}
