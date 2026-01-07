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
// Finite checks
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

// ------------------------------------------------------------
// Debug draw helpers (immediate mode)
// ------------------------------------------------------------
static void DrawGrid(float halfSize, float step) {
#ifdef FE_NATIVE
  glBegin(GL_LINES);

  for (float x = -halfSize; x <= halfSize; x += step) {
    glVertex3f(x, 0.0f, -halfSize);
    glVertex3f(x, 0.0f,  halfSize);
  }
  for (float z = -halfSize; z <= halfSize; z += step) {
    glVertex3f(-halfSize, 0.0f, z);
    glVertex3f( halfSize, 0.0f, z);
  }

  glEnd();
#else
  (void)halfSize; (void)step;
#endif
}

static void DrawAABB(const Vec3& mn, const Vec3& mx) {
#ifdef FE_NATIVE
  glBegin(GL_LINES);

  // bottom
  glVertex3f(mn.x, mn.y, mn.z); glVertex3f(mx.x, mn.y, mn.z);
  glVertex3f(mx.x, mn.y, mn.z); glVertex3f(mx.x, mn.y, mx.z);
  glVertex3f(mx.x, mn.y, mx.z); glVertex3f(mn.x, mn.y, mx.z);
  glVertex3f(mn.x, mn.y, mx.z); glVertex3f(mn.x, mn.y, mn.z);

  // top
  glVertex3f(mn.x, mx.y, mn.z); glVertex3f(mx.x, mx.y, mn.z);
  glVertex3f(mx.x, mx.y, mn.z); glVertex3f(mx.x, mx.y, mx.z);
  glVertex3f(mx.x, mx.y, mx.z); glVertex3f(mn.x, mx.y, mx.z);
  glVertex3f(mn.x, mx.y, mx.z); glVertex3f(mn.x, mx.y, mn.z);

  // sides
  glVertex3f(mn.x, mn.y, mn.z); glVertex3f(mn.x, mx.y, mn.z);
  glVertex3f(mx.x, mn.y, mn.z); glVertex3f(mx.x, mx.y, mn.z);
  glVertex3f(mx.x, mn.y, mx.z); glVertex3f(mx.x, mx.y, mx.z);
  glVertex3f(mn.x, mn.y, mx.z); glVertex3f(mn.x, mx.y, mx.z);

  glEnd();
#else
  (void)mn; (void)mx;
#endif
}

// ------------------------------------------------------------
// Robust OBB drawing (lines)
// ------------------------------------------------------------
static void DrawOBB(const fe::RigidBoxBody& b) {
#ifdef FE_NATIVE
  if (!fe_isfinite3(b.position) || !fe_isfinite3(b.halfExtents)) return;

  if (!fe_isfiniteQuat(b.orientation)) {
    DrawAABB(b.position - b.halfExtents, b.position + b.halfExtents);
    return;
  }

  using fe::Quat;
  using fe::quatNormalize;
  using fe::quatRotate;

  Quat qn = quatNormalize(b.orientation);
  if (!fe_isfiniteQuat(qn)) {
    DrawAABB(b.position - b.halfExtents, b.position + b.halfExtents);
    return;
  }

  Vec3 ax = quatRotate(qn, Vec3(1,0,0));
  Vec3 ay = quatRotate(qn, Vec3(0,1,0));
  Vec3 az = quatRotate(qn, Vec3(0,0,1));

  if (!fe_isfinite3(ax) || !fe_isfinite3(ay) || !fe_isfinite3(az)) {
    DrawAABB(b.position - b.halfExtents, b.position + b.halfExtents);
    return;
  }

  auto corner = [&](int sx,int sy,int sz)->Vec3{
    return b.position
      + ax * (b.halfExtents.x * (float)sx)
      + ay * (b.halfExtents.y * (float)sy)
      + az * (b.halfExtents.z * (float)sz);
  };

  Vec3 c000 = corner(-1,-1,-1);
  Vec3 c100 = corner( 1,-1,-1);
  Vec3 c110 = corner( 1, 1,-1);
  Vec3 c010 = corner(-1, 1,-1);

  Vec3 c001 = corner(-1,-1, 1);
  Vec3 c101 = corner( 1,-1, 1);
  Vec3 c111 = corner( 1, 1, 1);
  Vec3 c011 = corner(-1, 1, 1);

  glBegin(GL_LINES);

  // bottom
  glVertex3f(c000.x,c000.y,c000.z); glVertex3f(c100.x,c100.y,c100.z);
  glVertex3f(c100.x,c100.y,c100.z); glVertex3f(c101.x,c101.y,c101.z);
  glVertex3f(c101.x,c101.y,c101.z); glVertex3f(c001.x,c001.y,c001.z);
  glVertex3f(c001.x,c001.y,c001.z); glVertex3f(c000.x,c000.y,c000.z);

  // top
  glVertex3f(c010.x,c010.y,c010.z); glVertex3f(c110.x,c110.y,c110.z);
  glVertex3f(c110.x,c110.y,c110.z); glVertex3f(c111.x,c111.y,c111.z);
  glVertex3f(c111.x,c111.y,c111.z); glVertex3f(c011.x,c011.y,c011.z);
  glVertex3f(c011.x,c011.y,c011.z); glVertex3f(c010.x,c010.y,c010.z);

  // verticals
  glVertex3f(c000.x,c000.y,c000.z); glVertex3f(c010.x,c010.y,c010.z);
  glVertex3f(c100.x,c100.y,c100.z); glVertex3f(c110.x,c110.y,c110.z);
  glVertex3f(c101.x,c101.y,c101.z); glVertex3f(c111.x,c111.y,c111.z);
  glVertex3f(c001.x,c001.y,c001.z); glVertex3f(c011.x,c011.y,c011.z);

  glEnd();
#else
  (void)b;
#endif
}

// ------------------------------------------------------------
// RenderMesh helpers
// ------------------------------------------------------------
static void UnpackRGBA(uint32_t rgba, float& r, float& g, float& b, float& a) {
  r = float((rgba >> 24) & 0xFF) / 255.0f;
  g = float((rgba >> 16) & 0xFF) / 255.0f;
  b = float((rgba >>  8) & 0xFF) / 255.0f;
  a = float((rgba >>  0) & 0xFF) / 255.0f;
}

static fe::Quat IdentityQuat() {
  fe::Quat q;
  q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
  return q;
}

// Generic: draw any local-space mesh (x,y,z,rgba + indices) with transform.
template <typename LocalVertex>
static void DrawTransformedMeshRGBA(
  const std::vector<LocalVertex>& localVerts,
  const std::vector<uint32_t>& localIndices,
  const Vec3& pos,
  const fe::Quat& rot,
  const Vec3& scale,
  bool backfaceCull,
  engine::render::RenderMesh::Winding frontWinding
) {
#ifdef FE_NATIVE
  using fe::Quat;
  using fe::quatNormalize;
  using fe::quatRotate;

  Quat qn = quatNormalize(rot);

  std::vector<engine::render::VertexPC> verts;
  verts.reserve(localVerts.size());

  for (const auto& v : localVerts) {
    // LocalVertex must provide: v.x v.y v.z v.rgba
    Vec3 pLocal(v.x * scale.x, v.y * scale.y, v.z * scale.z);
    Vec3 pWorld = pos + quatRotate(qn, pLocal);

    float r,g,b,a;
    UnpackRGBA(v.rgba, r,g,b,a);
    verts.push_back({ pWorld.x, pWorld.y, pWorld.z, r,g,b,a });
  }

  std::vector<uint16_t> inds;
  inds.reserve(localIndices.size());
  for (uint32_t i : localIndices) inds.push_back((uint16_t)i);

  engine::render::RenderMesh mesh;
  mesh.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);
  mesh.SetBackfaceCulling(backfaceCull);
  mesh.SetFrontFaceWinding(frontWinding);
  mesh.SetVertices(verts);
  mesh.SetIndices(inds);
  mesh.Draw();
#else
  (void)localVerts; (void)localIndices; (void)pos; (void)rot; (void)scale; (void)backfaceCull; (void)frontWinding;
#endif
}

// Cached unit box mesh (centered at origin, half-extents = 1)
// We scale it per-instance by (halfExtents).
static const engine::geom::ColoredBox& UnitColoredBoxMesh() {
  static engine::geom::ColoredBox s_box = engine::geom::ColoredBox::make(
    0.0f, 0.0f, 0.0f,  // center
    1.0f, 1.0f, 1.0f,  // half extents (unit)
    engine::geom::ColoredBox::RGBA(255,  80,  80, 255),  // +X
    engine::geom::ColoredBox::RGBA( 80, 255,  80, 255),  // -X
    engine::geom::ColoredBox::RGBA( 80,  80, 255, 255),  // +Y
    engine::geom::ColoredBox::RGBA(255, 255,  80, 255),  // -Y
    engine::geom::ColoredBox::RGBA( 80, 255, 255, 255),  // +Z
    engine::geom::ColoredBox::RGBA(255,  80, 255, 255)   // -Z
  );
  return s_box;
}

static void DrawTerrain() {
#ifdef FE_NATIVE
  // Make it hard to miss:
  // - larger area
  // - slightly below 0
  // - gentle hills
  auto t = engine::geom::TerrainGrid::make(
    80.0f, 80.0f, 1.0f,   // sizeX, sizeZ, step
    -0.25f, 0.6f          // baseY, amplitude
  );

  std::vector<engine::render::VertexPC> verts;
  verts.reserve(t.vertices.size());
  for (const auto& v : t.vertices) {
    float r,g,b,a;
    UnpackRGBA(v.rgba, r,g,b,a);
    verts.push_back({ v.x, v.y, v.z, r,g,b,a });
  }

  std::vector<uint16_t> inds;
  inds.reserve(t.indices.size());
  for (uint32_t i : t.indices) inds.push_back((uint16_t)i);

  engine::render::RenderMesh mesh;
  mesh.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);

  // Double-sided for now so you can never be "under it / wrong side".
  mesh.SetBackfaceCulling(false);

  // Your project currently behaves as if CW is front.
  mesh.SetFrontFaceWinding(engine::render::RenderMesh::Winding::CW);

  mesh.SetVertices(verts);
  mesh.SetIndices(inds);
  mesh.Draw();
#endif
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
    a.min = Vec3(o->position.x - he.x, o->position.y - he.y, o->position.z - he.z);
    a.max = Vec3(o->position.x + he.x, o->position.y + he.y, o->position.z + he.z);
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
// Required Scene API
// ------------------------------------------------------------
void Scene::init() {
  // ---- Static world platforms ----
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

  // ---- Rigid-body world tuning ----
  m_rb.gravity = Vec3(0.f, -18.0f, 0.f);
  m_rb.enableGround = true;
  m_rb.groundY = 0.0f;
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

  static int s_print = 0;
  if ((s_print++ % 60) == 0) {
    std::printf("[Scene] RB bodies=%zu\n", m_rb.bodies().size());
  }

  for (auto& obj : m_objects) {
    if (obj) obj->update(dt);
  }

  rebuildStaticAABBs();

  if (dt < 0.0f) dt = 0.0f;

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

  if ((s_print % 60) == 0 && !m_rb.bodies().empty()) {
    const auto& b0 = m_rb.bodies()[0];
    std::printf("[RB] b0 pos=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f)\n",
      b0.position.x, b0.position.y, b0.position.z,
      b0.linearVelocity.x, b0.linearVelocity.y, b0.linearVelocity.z);
  }

  const auto& bodies = m_rb.bodies();
  bool bad = false;
  for (int i = 0; i < (int)bodies.size() && i < 3; ++i) {
    if (!fe_isfiniteRigidBody(bodies[i])) { bad = true; break; }
  }

  if (bad) {
    static bool once = false;
    if (!once) {
      std::printf("[Scene] Non-finite rigid-body detected -> skipping player-vs-world collision\n");
      once = true;
    }

    m_playerCenterOut = m_playerCenter;
    m_playerVelOut    = m_playerVel;
    m_playerGroundedOut = false;
    return;
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

// ------------------------------------------------------------
// Debug render
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

  // --- 1) Terrain ---
  DrawTerrain();

  // --- 2) Solid platforms (static AABBs) ---
  {
    const auto& unit = UnitColoredBoxMesh();
    const uint32_t platCol = engine::geom::ColoredBox::RGBA(90, 140, 220, 255);

    for (const auto& a : m_static) {
      Vec3 center(0.5f*(a.min.x+a.max.x), 0.5f*(a.min.y+a.max.y), 0.5f*(a.min.z+a.max.z));
      Vec3 half  (0.5f*(a.max.x-a.min.x), 0.5f*(a.max.y-a.min.y), 0.5f*(a.max.z-a.min.z));

      // Use unit box colors for now; platforms are meant to read as blue.
      // (If you want strictly single-color faces, we can generate a UnitSingleColorBox mesh too.)
      (void)platCol;

      DrawTransformedMeshRGBA(
        unit.vertices, unit.indices,
        center, IdentityQuat(), half,
        true,
        engine::render::RenderMesh::Winding::CW
      );
    }
  }

  // --- 3) Solid rigid bodies (OBB surfaces; move+rotate with physics) ---
  {
    const auto& unit = UnitColoredBoxMesh();
    for (const auto& b : m_rb.bodies()) {
      DrawTransformedMeshRGBA(
        unit.vertices, unit.indices,
        b.position, b.orientation, b.halfExtents,
        true,
        engine::render::RenderMesh::Winding::CW
      );
    }
  }

  // --- 4) Keep wire debug overlays ---
  glDisable(GL_CULL_FACE);

  glColor3f(0.55f, 0.55f, 0.55f);
  DrawGrid(40.0f, 1.0f);

  glColor3f(0.2f, 0.9f, 0.2f);
  for (const auto& a : m_static) {
    DrawAABB(a.min, a.max);
  }

  glColor3f(0.9f, 0.7f, 0.2f);
  for (const auto& b : m_rb.bodies()) {
    DrawOBB(b);
  }

  glColor3f(1.f, 1.f, 1.f);
#else
  (void)view; (void)proj;
#endif
}
