#include "game/Scene.h"
#include "game/GameObject.h"
#include <cstdio>

#ifdef FE_NATIVE
  #include <GL/gl.h>
#endif

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

static void DrawOBB(const fe::RigidBoxBody& b) {
#ifdef FE_NATIVE
  using fe::Quat;
  using fe::quatNormalize;
  using fe::quatRotate;

  const Quat qn = quatNormalize(b.orientation);
  const Vec3 ax = quatRotate(qn, Vec3(1,0,0));
  const Vec3 ay = quatRotate(qn, Vec3(0,1,0));
  const Vec3 az = quatRotate(qn, Vec3(0,0,1));

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

void Scene::init() {
  // ---- Static world platforms (must match Game::init platform AABBs) ----
  {
    auto* p = createObject();
    p->position = Vec3(0.0f, 0.65f, 0.0f);        // (-2..2, 0.5..0.8, -2..2)
    p->enableBoxCollider(Vec3(2.0f, 0.15f, 2.0f)); // half extents
  }
  {
    auto* p = createObject();
    p->position = Vec3(3.75f, 1.35f, 0.0f);        // (3..4.5, 1.2..1.5, -1..1)
    p->enableBoxCollider(Vec3(0.75f, 0.15f, 1.0f));
  }
  {
    auto* p = createObject();
    p->position = Vec3(-3.25f, 0.4f, 4.0f);        // (-4..-2.5, 0.2..0.6, 3..5)
    p->enableBoxCollider(Vec3(0.75f, 0.2f, 1.0f));
  }

  // Build static collider list for rigid-body world
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
  m_rb.positionIters = 4;

  // ---- Spawn dynamic crates (stacked) ----
  m_rb.createBox(Vec3(0.0f, 1.30f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
  m_rb.createBox(Vec3(0.0f, 2.35f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
  m_rb.createBox(Vec3(3.75f, 2.10f, 0.0f), Vec3(0.50f, 0.50f, 0.50f), 2.0f);
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
  // ---- dt spike confirmation (prints first 60 frames only) ----
  static int s_frames = 0;

  // Gameplay update (static objects)
  for (auto& obj : m_objects) {
    if (obj) obj->update(dt);
  }

  // Static colliders might change later (destruction). For now, rebuild each frame is cheap.
  rebuildStaticAABBs();

  // Step rigid-body world (with SAFE clamp, no double-step risk)
  if (dt < 0.0f) dt = 0.0f;

  const float fixed = m_rb.fixedDt;
  const float maxDt = fixed * (float)m_rb.maxSubsteps;
  const float inDt  = dt;

  if (dt > maxDt) dt = maxDt;

  if (s_frames < 60) {
    // Prints enough to confirm if the first frames are huge.
    // If you see inDt >> maxDt early, that’s the spike.
    printf("[Scene] dt in=%f clamped=%f fixed=%f maxDt=%f\n", inDt, dt, fixed, maxDt);
  }
  s_frames++;

  m_rb.step(dt);

  // Collide player sphere against crates + platforms + ground
  m_playerCenterOut = m_playerCenter;
  m_playerVelOut = m_playerVel;
  m_playerGroundedOut = false;

  if (m_playerValid) {
    (void)m_rb.collidePlayerSphere(m_playerCenterOut, m_playerRadius, m_playerVelOut, &m_playerGroundedOut);
  }
}

void Scene::render(const Mat4& view, const Mat4& proj) {
  // Debug visualization lives in renderDebug().
  renderDebug(view, proj);
}

static void LoadMat4_GL(int mode, const Mat4& M) {
#ifdef FE_NATIVE
  glMatrixMode(mode);
  glLoadMatrixf(M.m);   // no transpose
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

  // Grid
  glColor3f(0.6f, 0.6f, 0.6f);
  DrawGrid(20.0f, 1.0f);

  // Static platforms
  glColor3f(0.2f, 0.9f, 0.2f);
  for (const auto& a : m_static) {
    DrawAABB(a.min, a.max);
  }

  // Dynamic crates (OBBs)
  glColor3f(0.9f, 0.7f, 0.2f);
  for (const auto& b : m_rb.bodies()) {
    DrawOBB(b);
  }

  glColor3f(1.f, 1.f, 1.f);
#else
  (void)view; (void)proj;
#endif
}
