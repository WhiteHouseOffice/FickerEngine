#include "game/Scene.h"
#include "game/GameObject.h"

#include <cstdio>
#include <cmath>

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

  // Orientation invalid → draw fallback AABB
  if (!fe_isfiniteQuat(b.orientation)) {
    Vec3 mn = b.position - b.halfExtents;
    Vec3 mx = b.position + b.halfExtents;
    DrawAABB(mn, mx);
    return;
  }

  using fe::Quat;
  using fe::quatNormalize;
  using fe::quatRotate;

  Quat qn = quatNormalize(b.orientation);
  if (!fe_isfiniteQuat(qn)) {
    Vec3 mn = b.position - b.halfExtents;
    Vec3 mx = b.position + b.halfExtents;
    DrawAABB(mn, mx);
    return;
  }

  Vec3 ax = quatRotate(qn, Vec3(1,0,0));
  Vec3 ay = quatRotate(qn, Vec3(0,1,0));
  Vec3 az = quatRotate(qn, Vec3(0,0,1));

  if (!fe_isfinite3(ax) || !fe_isfinite3(ay) || !fe_isfinite3(az)) {
    Vec3 mn = b.position - b.halfExtents;
    Vec3 mx = b.position + b.halfExtents;
    DrawAABB(mn, mx);
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
    Vec3 mn = b.position - b.halfExtents;
    Vec3 mx = b.position + b.halfExtents;
    DrawAABB(mn, mx);
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
// Scene update + render
// ------------------------------------------------------------
void Scene::update(float dt) {
  static int s_print = 0;
  if ((s_print++ % 60) == 0) {
    std::printf("[Scene] RB bodies=%zu\n", m_rb.bodies().size());
  }

  rebuildStaticAABBs();

  if (dt < 0.0f) dt = 0.0f;
  const float fixed = m_rb.fixedDt;
  const float maxDt = fixed * (float)m_rb.maxSubsteps;
  if (dt > maxDt) dt = maxDt;

  static float s_accum = 0.f;
  s_accum += dt;
  if (s_accum > maxDt) s_accum = maxDt;

  int steps = 0;
  while (s_accum >= fixed && steps < m_rb.maxSubsteps) {
    m_rb.step(fixed);
    s_accum -= fixed;
    steps++;
  }

  // Extended NaN detection (position + orientation)
  const auto& bodies = m_rb.bodies();
  for (int i = 0; i < (int)bodies.size() && i < 3; ++i) {
    if (!fe_isfiniteRigidBody(bodies[i])) {
      std::printf("[Scene] Non-finite rigid body detected (including orientation)\n");
      break;
    }
  }
}

void Scene::renderDebug(const Mat4& view, const Mat4& proj) {
#ifdef FE_NATIVE
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj.m);
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.m);

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  DrawGrid(20.0f, 1.0f);

  for (const auto& a : m_static) {
    DrawAABB(a.min, a.max);
  }

  for (const auto& b : m_rb.bodies()) {
    DrawOBB(b);
  }
#else
  (void)view; (void)proj;
#endif
}
