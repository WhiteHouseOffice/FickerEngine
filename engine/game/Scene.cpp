#include "game/Scene.h"
#include "game/GameObject.h"

// NEW
#include "geom/ColoredQuad.h"
#include "render/RenderMesh.h"

#include <cstdio>
#include <cmath>
#include <memory>
#include <vector> // NEW

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
// Debug draw helpers
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
// Robust OBB drawing (never silently disappears)
// ------------------------------------------------------------
static void DrawOBB(const fe::RigidBoxBody& b) {
#ifdef FE_NATIVE
  if (!fe_isfinite3(b.position) || !fe_isfinite3(b.halfExtents)) return;

  // Orientation invalid → draw fallback AABB so box stays visible.
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

  if (!fe_isfinite3(c000)||!fe_isfinite3(c100)||!fe_isfinite3(c110)||!fe_isfinite3(c010)||
      !fe_isfinite3(c001)||!fe_isfinite3(c101)||!fe_isfinite3(c111)||!fe_isfinite3(c011)) {
    DrawAABB(b.position - b.halfExtents, b.position + b.halfExtents);
    return;
  }

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
// NEW: Colored surface quad using RenderMesh + ColoredQuad
// ------------------------------------------------------------
static void DrawColoredSurfaceQuad() {
#ifdef FE_NATIVE
  // Make sure fixed-function state doesn't override vertex colors.
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);

  // Create a colored quad slightly above y=0 to avoid z-fighting with grid lines.
  auto q = ColoredQuad::make(
    Vec3(0.0f, 0.01f, 0.0f),
    2.5f, 2.5f,
    ColoredQuad::RGBA(255,   0,   0, 255), // red
    ColoredQuad::RGBA(0,   255,   0, 255), // green
    ColoredQuad::RGBA(0,     0, 255, 255), // blue
    ColoredQuad::RGBA(255, 255,   0, 255), // yellow
    true // CCW
  );

  auto unpack = [](uint32_t rgba, float& r, float& g, float& b, float& a) {
    r = float((rgba >> 24) & 0xFF) / 255.0f;
    g = float((rgba >> 16) & 0xFF) / 255.0f;
    b = float((rgba >>  8) & 0xFF) / 255.0f;
    a = float((rgba >>  0) & 0xFF) / 255.0f;
  };

  std::vector<engine::render::VertexPC> verts;
  verts.reserve(q.vertices.size());

  for (const auto& v : q.vertices) {
    float r, g, b, a;
    unpack(v.rgba, r, g, b, a);
    verts.push_back(engine::render::VertexPC{ v.pos.x, v.pos.y, v.pos.z, r, g, b, a });
  }

  std::vector<uint16_t> inds;
  inds.reserve(q.indices.size());
  for (uint32_t idx : q.indices) inds.push_back((uint16_t)idx);

  engine::render::RenderMesh mesh;
  mesh.SetPrimitive(engine::render::RenderMesh::Primitive::Triangles);
  mesh.SetBackfaceCulling(true);
  mesh.SetFrontFaceWinding(engine::render::RenderMesh::Winding::CCW);
  mesh.SetVertices(verts);
  mesh.SetIndices(inds);
  mesh.Draw();
#else
  // no-op on web path for now
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
  // Lift slightly so we don't start in exact touching contact.
  rb.createBox(Vec3(0.0f, 5.0f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
  rb.createBox(Vec3(0.0f, 6.2f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
  rb.createBox(Vec3(3.75f, 5.6f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
}

// ------------------------------------------------------------
// Required Scene API (Engine links against these)
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

  // Crates spawn immediately (per your request)
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
  static int s_frames = 0;
  static float s_accum = 0.0f;

  // Debug: show whether bodies actually get deleted (they weren't)
  static int s_print = 0;
  if ((s_print++ % 60) == 0) {
    std::printf("[Scene] RB bodies=%zu\n", m_rb.bodies().size());
  }

  // Gameplay update
  for (auto& obj : m_objects) {
    if (obj) obj->update(dt);
  }

  rebuildStaticAABBs();

  if (dt < 0.0f) dt = 0.0f;

  const float fixed = m_rb.fixedDt;
  const float maxDt = fixed * (float)m_rb.maxSubsteps;

  // Clamp input dt to avoid huge catch-up after pauses
  if (dt > maxDt) dt = maxDt;

  // Fixed-step accumulator
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
  // Extended non-finite detection: check orientation too (catch invisible "despawn")
  static int s_nanCooldown = 0; // frames
  const auto& bodies = m_rb.bodies();

  bool bad = false;
  for (int i = 0; i < (int)bodies.size() && i < 3; ++i) {
    if (!fe_isfiniteRigidBody(bodies[i])) { bad = true; break; }
  }

  if (s_nanCooldown > 0) s_nanCooldown--;

  if (bad) {
    static bool once = false;
    if (!once) {
      std::printf("[Scene] Non-finite rigid-body detected -> skipping player-vs-world collision\n");
      once = true;
    }

    // Keep player outputs valid so controls still work.
    m_playerCenterOut = m_playerCenter;
    m_playerVelOut    = m_playerVel;
    m_playerGroundedOut = false;

    // Don't run collidePlayerSphere against a broken rigid world.
    return;
  }

  // Player collision
  m_playerCenterOut = m_playerCenter;
  m_playerVelOut = m_playerVel;
  m_playerGroundedOut = false;

  if (m_playerValid) {
    (void)m_rb.collidePlayerSphere(m_playerCenterOut, m_playerRadius, m_playerVelOut, &m_playerGroundedOut);
  }

  s_frames++;
}

void Scene::render(const Mat4& view, const Mat4& proj) {
  // Keep the existing approach: render via debug path so we always see something.
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

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  glColor3f(0.6f, 0.6f, 0.6f);
  DrawGrid(20.0f, 1.0f);

  glColor3f(0.2f, 0.9f, 0.2f);
  for (const auto& a : m_static) {
    DrawAABB(a.min, a.max);
  }

  glColor3f(0.9f, 0.7f, 0.2f);
  for (const auto& b : m_rb.bodies()) {
    DrawOBB(b);
  }

  // NEW: draw a colored surface quad (triangles + colors + culling)
  DrawColoredSurfaceQuad();

  glColor3f(1.f, 1.f, 1.f);
#else
  (void)view; (void)proj;
#endif
}
