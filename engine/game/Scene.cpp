#include "game/Scene.h"
#include "game/GameObject.h"

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
#endif
}

static void DrawBox(const Vec3& mn, const Vec3& mx) {
#ifdef FE_NATIVE
  glBegin(GL_LINES);

  // bottom rectangle
  glVertex3f(mn.x, mn.y, mn.z); glVertex3f(mx.x, mn.y, mn.z);
  glVertex3f(mx.x, mn.y, mn.z); glVertex3f(mx.x, mn.y, mx.z);
  glVertex3f(mx.x, mn.y, mx.z); glVertex3f(mn.x, mn.y, mx.z);
  glVertex3f(mn.x, mn.y, mx.z); glVertex3f(mn.x, mn.y, mn.z);

  // top rectangle
  glVertex3f(mn.x, mx.y, mn.z); glVertex3f(mx.x, mx.y, mn.z);
  glVertex3f(mx.x, mx.y, mn.z); glVertex3f(mx.x, mx.y, mx.z);
  glVertex3f(mx.x, mx.y, mx.z); glVertex3f(mn.x, mx.y, mx.z);
  glVertex3f(mn.x, mx.y, mx.z); glVertex3f(mn.x, mx.y, mn.z);

  // vertical edges
  glVertex3f(mn.x, mn.y, mn.z); glVertex3f(mn.x, mx.y, mn.z);
  glVertex3f(mx.x, mn.y, mn.z); glVertex3f(mx.x, mx.y, mn.z);
  glVertex3f(mx.x, mn.y, mx.z); glVertex3f(mx.x, mx.y, mx.z);
  glVertex3f(mn.x, mn.y, mx.z); glVertex3f(mn.x, mx.y, mx.z);

  glEnd();
#endif
}

void Scene::setPlayerSphere(const Vec3& center, float radius) {
  m_playerSphereValid = true;
  m_playerSphereCenter = center;
  m_playerSphereRadius = radius;
}

GameObject* Scene::createObject() {
  m_objects.emplace_back(std::make_unique<GameObject>());
  return m_objects.back().get();
}

void Scene::init() {
  // Physics tuning (simple, stable)
  m_physics.gravity = Vec3(0.f, -18.0f, 0.f);
  m_physics.enableGroundPlane = true;
  m_physics.groundY = 0.0f;
  m_physics.groundRestitution = 0.0f;
  m_physics.groundFriction = 0.02f;
  m_physics.solverIterations = 2;

  // ---- Static platforms (match the debug boxes) ----
  {
    auto* p = createObject();
    p->position = Vec3(0.0f, 0.65f, 0.0f);          // center of box (-2..2, 0.5..0.8, -2..2)
    p->enableBoxCollider(Vec3(2.0f, 0.15f, 2.0f));  // half extents
    // no rigidbody => static
  }
  {
    auto* p = createObject();
    p->position = Vec3(3.75f, 1.35f, 0.0f);         // (3..4.5, 1.2..1.5, -1..1)
    p->enableBoxCollider(Vec3(0.75f, 0.15f, 1.0f));
  }
  {
    auto* p = createObject();
    p->position = Vec3(-3.25f, 0.4f, 4.0f);         // (-4..-2.5, 0.2..0.6, 3..5)
    p->enableBoxCollider(Vec3(0.75f, 0.2f, 1.0f));
  }

  // ---- Test cubes (dynamic) ----

  // One cube on the middle platform, so you can push it off.
  {
    auto* b = createObject();
    // platform top y=0.8, cube halfY=0.5 => center y = 1.3
    b->position = Vec3(0.0f, 1.3f, 0.0f);
    b->enableBoxCollider(Vec3(0.5f, 0.5f, 0.5f));
    b->enablePhysics(2.0f, true);
  }

  // A small pile on the ground to shove around
  for (int i = 0; i < 6; ++i) {
    auto* b = createObject();
    b->position = Vec3(-3.0f + i * 1.2f, 0.75f, -4.0f);
    b->enableBoxCollider(Vec3(0.5f, 0.5f, 0.5f));
    b->enablePhysics(2.0f, true);
  }

  // A drop test from above
  {
    auto* b = createObject();
    b->position = Vec3(4.0f, 6.0f, -2.0f);
    b->enableBoxCollider(Vec3(0.5f, 0.5f, 0.5f));
    b->enablePhysics(1.0f, true);
  }
}

void Scene::update(float dt) {
  // Gameplay update
  for (auto& obj : m_objects) {
    if (obj) obj->update(dt);
  }

  // Feed player sphere into physics (so you can push cubes)
  if (m_playerSphereValid) {
    m_physics.setPlayerSphere(m_playerSphereCenter, m_playerSphereRadius);
  } else {
    m_physics.clearPlayerSphere();
  }

  // Physics step
  std::vector<GameObject*> ptrs;
  ptrs.reserve(m_objects.size());
  for (auto& obj : m_objects) ptrs.push_back(obj.get());

  m_physics.step(dt, ptrs);
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // keep empty for now
}

void Scene::renderDebug(const Mat4& view, const Mat4& proj) {
#ifdef FE_NATIVE
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj.m);

  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.m);

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING);

  glLineWidth(1.0f);

  // Make sure lines are visible
  glColor3f(1.0f, 1.0f, 1.0f);

  // Grid
  DrawGrid(50.0f, 1.0f);

  // Draw all colliders as wire boxes
  glColor3f(1.0f, 1.0f, 0.0f);
  for (auto& obj : m_objects) {
    if (!obj) continue;
    if (!obj->hasBoxCollider()) continue;
    const Vec3 he = obj->boxHalfExtents();
    const Vec3 mn = obj->position - he;
    const Vec3 mx = obj->position + he;
    DrawBox(mn, mx);
  }

  // Reset color
  glColor3f(1.0f, 1.0f, 1.0f);
#endif
}
