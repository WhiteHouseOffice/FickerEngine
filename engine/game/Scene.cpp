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

// ============================================================
// Math / sanity helpers
// ============================================================
static bool fe_isfinite3(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

static bool fe_isfiniteQuat(const fe::Quat& q) {
  return std::isfinite(q.w) && std::isfinite(q.x)
      && std::isfinite(q.y) && std::isfinite(q.z);
}

static fe::Quat IdentityQuat() {
  fe::Quat q;
  q.w = 1.f; q.x = q.y = q.z = 0.f;
  return q;
}

// ============================================================
// Terrain height (MUST match TerrainGrid::make params)
// ============================================================
static float TerrainHeight(float x, float z) {
  const float baseY = -0.25f;
  const float amp   = 0.6f;

  const float r2 = x*x + z*z;
  const float hill = std::exp(-r2 / (2.0f * 20.0f * 20.0f));
  const float wav  = 0.35f * std::sin(0.18f * x) * std::cos(0.16f * z);

  return baseY + amp * (0.75f * hill + wav);
}

// ============================================================
// Color unpack
// ============================================================
static void UnpackRGBA(uint32_t rgba, float& r, float& g, float& b, float& a) {
  r = float((rgba >> 24) & 0xFF) / 255.f;
  g = float((rgba >> 16) & 0xFF) / 255.f;
  b = float((rgba >>  8) & 0xFF) / 255.f;
  a = float((rgba >>  0) & 0xFF) / 255.f;
}

// ============================================================
// Universal transformed mesh draw
// ============================================================
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

  fe::Quat qn = quatNormalize(rot);

  std::vector<engine::render::VertexPC> verts;
  verts.reserve(localVerts.size());

  for (const auto& v : localVerts) {
    Vec3 pLocal(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    Vec3 pWorld = pos + quatRotate(qn, pLocal);

    float r,g,b,a;
    UnpackRGBA(v.rgba, r,g,b,a);
    verts.push_back({ pWorld.x, pWorld.y, pWorld.z, r,g,b,a });
  }

  std::vector<uint16_t> inds;
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

// ============================================================
// Cached meshes (unit size, centered at origin)
// ============================================================
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

// ============================================================
// Scene lifecycle
// ============================================================
GameObject* Scene::createObject() {
  auto obj = std::make_unique<GameObject>();
  GameObject* out = obj.get();
  m_objects.emplace_back(std::move(obj));
  return out;
}

void Scene::rebuildStaticAABBs() {
  m_static.clear();

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
  rb.createBox(Vec3(0,5,0), Vec3(0.5f,0.5f,0.5f), 2.f);
  rb.createBox(Vec3(0,6.2f,0), Vec3(0.5f,0.5f,0.5f), 2.f);
  rb.createBox(Vec3(3.5f,5.6f,0), Vec3(0.5f,0.5f,0.5f), 2.f);
}

// ============================================================
// Init
// ============================================================
void Scene::init() {
  {
    auto* p = createObject();
    p->position = Vec3(0,0.65f,0);
    p->enableBoxCollider(Vec3(2,0.15f,2));
  }

  rebuildStaticAABBs();

  m_rb.enableGround = false; // terrain is the floor
  m_rb.gravity = Vec3(0,-18,0);
  m_rb.fixedDt = 1.f / 120.f;
  m_rb.maxSubsteps = 8;

  SpawnCrates(m_rb);
}

// ============================================================
// Update
// ============================================================
void Scene::update(float dt) {
  static float acc = 0.f;
  acc += dt;

  while (acc >= m_rb.fixedDt) {
    m_rb.step(m_rb.fixedDt);

    // terrain floor constraint
    for (auto& b : m_rb.bodies()) {
      float floorY = TerrainHeight(b.position.x, b.position.z);
      float bottom = b.position.y - b.halfExtents.y;
      if (bottom < floorY) {
        b.position.y += (floorY - bottom);
        if (b.linearVelocity.y < 0) b.linearVelocity.y = 0;
      }
    }

    acc -= m_rb.fixedDt;
  }
}

// ============================================================
// Render
// ============================================================
static void LoadMat4_GL(int mode, const Mat4& M) {
#ifdef FE_NATIVE
  glMatrixMode(mode);
  glLoadMatrixf(M.m);
#else
  (void)mode; (void)M;
#endif
}

void Scene::render(const Mat4& view, const Mat4& proj) {
#ifdef FE_NATIVE
  LoadMat4_GL(GL_PROJECTION, proj);
  LoadMat4_GL(GL_MODELVIEW,  view);

  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  // --- terrain ---
  auto t = engine::geom::TerrainGrid::make(80,80,1,-0.25f,0.6f);

  std::vector<engine::render::VertexPC> tv;
  std::vector<uint16_t> ti;

  for (auto& v : t.vertices) {
    float r,g,b,a;
    UnpackRGBA(v.rgba,r,g,b,a);
    tv.push_back({v.x,v.y,v.z,r,g,b,a});
  }
  for (uint32_t i : t.indices) ti.push_back((uint16_t)i);

  engine::render::RenderMesh tm;
  tm.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);
  tm.SetBackfaceCulling(false);
  tm.SetVertices(tv);
  tm.SetIndices(ti);
  tm.Draw();

  // --- platforms ---
  for (auto& a : m_static) {
    Vec3 c = (a.min + a.max) * 0.5f;
    Vec3 h = (a.max - a.min) * 0.5f;

    DrawTransformedMeshRGBA(
      UnitPlatformBox().vertices,
      UnitPlatformBox().indices,
      c, IdentityQuat(), h, true
    );
  }

  // --- crates ---
  for (auto& b : m_rb.bodies()) {
    DrawTransformedMeshRGBA(
      UnitCrateBox().vertices,
      UnitCrateBox().indices,
      b.position, b.orientation, b.halfExtents, true
    );
  }
#endif
}
