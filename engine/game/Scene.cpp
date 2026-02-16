#include "game/Scene.h"
#include "game/GameObject.h"

#include "geom/ColoredBox.h"
#include "geom/TerrainGrid.h"
#include "render/RenderMesh.h"

#include <cstdio>
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

#ifdef FE_NATIVE
  #include <GL/gl.h>
#endif

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

// --- Terrain params (bigger hills so you can clearly ascend) ---
static constexpr float kTerrainSizeX = 80.0f;
static constexpr float kTerrainSizeZ = 80.0f;
static constexpr float kTerrainStep  = 1.0f;
static constexpr float kTerrainBaseY = -1.00f;
static constexpr float kTerrainAmp   = 3.00f;

static inline int TerrainNX() { return (int)std::floor(kTerrainSizeX / kTerrainStep) + 1; }
static inline int TerrainNZ() { return (int)std::floor(kTerrainSizeZ / kTerrainStep) + 1; }
static inline float TerrainHalfX() { return 0.5f * kTerrainSizeX; }
static inline float TerrainHalfZ() { return 0.5f * kTerrainSizeZ; }

static float TerrainSampleHeightFromMesh(const engine::geom::TerrainGrid& t, float x, float z) {
  const int nx = TerrainNX();
  const int nz = TerrainNZ();
  if ((int)t.vertices.size() < nx * nz || nx < 2 || nz < 2) return -1e30f;

  float fx = (x + TerrainHalfX()) / kTerrainStep;
  float fz = (z + TerrainHalfZ()) / kTerrainStep;

  fx = std::clamp(fx, 0.0f, (float)(nx - 1));
  fz = std::clamp(fz, 0.0f, (float)(nz - 1));

  int ix = (int)std::floor(fx);
  int iz = (int)std::floor(fz);
  int ix1 = std::min(ix + 1, nx - 1);
  int iz1 = std::min(iz + 1, nz - 1);

  float tx = fx - (float)ix;
  float tz = fz - (float)iz;

  auto atY = [&](int gx, int gz) -> float {
    return t.vertices[(size_t)gz * (size_t)nx + (size_t)gx].y;
  };

  float h00 = atY(ix,  iz);
  float h10 = atY(ix1, iz);
  float h01 = atY(ix,  iz1);
  float h11 = atY(ix1, iz1);

  float hx0 = h00 + (h10 - h00) * tx;
  float hx1 = h01 + (h11 - h01) * tx;
  return hx0 + (hx1 - hx0) * tz;
}

static Vec3 TerrainSampleNormalFromMesh(const engine::geom::TerrainGrid& t, float x, float z) {
  const float e = std::max(0.5f * kTerrainStep, 0.05f);

  float hL = TerrainSampleHeightFromMesh(t, x - e, z);
  float hR = TerrainSampleHeightFromMesh(t, x + e, z);
  float hD = TerrainSampleHeightFromMesh(t, x, z - e);
  float hU = TerrainSampleHeightFromMesh(t, x, z + e);

  float dhdx = (hR - hL) / (2.0f * e);
  float dhdz = (hU - hD) / (2.0f * e);

  Vec3 n(-dhdx, 1.0f, -dhdz);

  float l2 = n.x*n.x + n.y*n.y + n.z*n.z;
  if (l2 < 1e-12f) return Vec3(0.f, 1.f, 0.f);
  float inv = 1.0f / std::sqrt(l2);
  return n * inv;
}

static float TerrainHeightCB(void* user, float x, float z) {
  auto* t = (engine::geom::TerrainGrid*)user;
  return t ? TerrainSampleHeightFromMesh(*t, x, z) : -1e30f;
}

static Vec3 TerrainNormalCB(void* user, float x, float z) {
  auto* t = (engine::geom::TerrainGrid*)user;
  return t ? TerrainSampleNormalFromMesh(*t, x, z) : Vec3(0.f, 1.f, 0.f);
}

// --- Color unpack ---
static void UnpackRGBA(uint32_t rgba, float& r, float& g, float& b, float& a) {
  r = float((rgba >> 24) & 0xFF) / 255.f;
  g = float((rgba >> 16) & 0xFF) / 255.f;
  b = float((rgba >>  8) & 0xFF) / 255.f;
  a = float((rgba >>  0) & 0xFF) / 255.f;
}

// --- CPU transform into VertexPC and draw via RenderMesh ---
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
  const fe::Quat qn = fe::quatNormalize(rot);

  std::vector<engine::render::VertexPC> verts;
  verts.reserve(localVerts.size());

  for (const auto& v : localVerts) {
    Vec3 pLocal(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    Vec3 pWorld = pos + fe::quatRotate(qn, pLocal);

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
  mesh.SetFrontFaceWinding(engine::render::RenderMesh::Winding::CW);
  mesh.SetVertices(verts);
  mesh.SetIndices(inds);
  mesh.Draw();
#else
  (void)localVerts; (void)localInds; (void)pos; (void)rot; (void)scale; (void)backfaceCull;
#endif
}

// --- Cached unit meshes ---
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

// --- Scene object lifecycle ---
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

void Scene::init() {
  m_objects.clear();

  // Build terrain once (render + physics sample from mesh verts)
  m_terrain = engine::geom::TerrainGrid::make(
    kTerrainSizeX, kTerrainSizeZ, kTerrainStep,
    kTerrainBaseY, kTerrainAmp
  );

  // platforms — now placed ON TOP of terrain height at their (x,z)
  {
    auto* p = createObject();
    const Vec3 he(2.0f, 0.15f, 2.0f);
    const float x = 0.0f, z = 0.0f;
    const float h = TerrainSampleHeightFromMesh(m_terrain, x, z);
    p->position = Vec3(x, h + he.y + 0.05f, z);
    p->enableBoxCollider(he);
  }
  {
    auto* p = createObject();
    const Vec3 he(0.75f, 0.15f, 1.0f);
    const float x = 3.75f, z = 0.0f;
    const float h = TerrainSampleHeightFromMesh(m_terrain, x, z);
    p->position = Vec3(x, h + he.y + 0.05f, z);
    p->enableBoxCollider(he);
  }
  {
    auto* p = createObject();
    const Vec3 he(0.75f, 0.2f, 1.0f);
    const float x = -3.25f, z = 4.0f;
    const float h = TerrainSampleHeightFromMesh(m_terrain, x, z);
    p->position = Vec3(x, h + he.y + 0.05f, z);
    p->enableBoxCollider(he);
  }

  rebuildStaticAABBs();

  // physics tuning
  m_rb.gravity = Vec3(0.f, -18.0f, 0.f);

  // Terrain is the floor now (NO legacy plane)
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
  if (!m_playerValid) return false;
  outCenter = m_playerCenterOut;
  outVelocity = m_playerVelOut;
  outGrounded = m_playerGroundedOut;
  return true;
}

void Scene::update(float dt) {
  static float s_accum = 0.0f;

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
    s_accum -= fixed;
    steps++;
  }

  // non-finite guard
  const auto& bodies = m_rb.bodies();
  for (int i = 0; i < (int)bodies.size() && i < 3; ++i) {
    if (!fe_isfiniteRigidBody(bodies[i])) {
      std::printf("[physics] non-finite rigid body detected\n");
      break;
    }
  }

  m_playerCenterOut = m_playerCenter;
  m_playerVelOut = m_playerVel;
  m_playerGroundedOut = false;

  if (m_playerValid) {
    (void)m_rb.collidePlayerSphere(m_playerCenterOut, m_playerRadius, m_playerVelOut, &m_playerGroundedOut);
  }
}

void Scene::render(const Mat4& view, const Mat4& proj) {
  renderDebug(view, proj);
}

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

  // --- Terrain ---
  {
    std::vector<engine::render::VertexPC> tv;
    tv.reserve(m_terrain.vertices.size());
    for (const auto& v : m_terrain.vertices) {
      float r,g,b,a;
      UnpackRGBA(v.rgba, r,g,b,a);
      tv.push_back({ v.x, v.y, v.z, r,g,b,a });
    }

    std::vector<uint16_t> ti;
    ti.reserve(m_terrain.indices.size());
    for (uint32_t i : m_terrain.indices) ti.push_back((uint16_t)i);

    engine::render::RenderMesh tm;
    tm.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);
    tm.SetBackfaceCulling(false);
    tm.SetFrontFaceWinding(engine::render::RenderMesh::Winding::CW);
    tm.SetVertices(tv);
    tm.SetIndices(ti);
    tm.Draw();
  }

  // --- Platforms ---
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

  // --- Crates ---
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
