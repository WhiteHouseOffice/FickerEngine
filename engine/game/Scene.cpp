#include "game/Scene.h"

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

void Scene::init() {
  // nothing
}

void Scene::update(float /*dt*/) {
  // nothing
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

  // Wire boxes (same coords you can collide with later)
  glColor3f(1.0f, 1.0f, 0.0f);
  DrawBox(Vec3(-2.0f, 0.5f, -2.0f), Vec3( 2.0f, 0.8f,  2.0f));
  DrawBox(Vec3( 3.0f, 1.2f, -1.0f), Vec3( 4.5f, 1.5f,  1.0f));
  DrawBox(Vec3(-4.0f, 0.2f,  3.0f), Vec3(-2.5f, 0.6f,  5.0f));

  // Reset color
  glColor3f(1.0f, 1.0f, 1.0f);
#endif
}
