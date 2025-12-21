#include "game/Scene.h"

#ifdef FE_NATIVE
  #include <GL/gl.h>
#endif

// Simple debug draw helpers (legacy OpenGL immediate mode)
// Works with your "no shader files" / fixed-function approach.

static void DrawGrid(float halfSize, float step) {
#ifdef FE_NATIVE
  glBegin(GL_LINES);

  // Lines parallel to Z (varying X)
  for (float x = -halfSize; x <= halfSize; x += step) {
    glVertex3f(x, 0.0f, -halfSize);
    glVertex3f(x, 0.0f,  halfSize);
  }

  // Lines parallel to X (varying Z)
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
  // No GPU uploads needed for this immediate-mode debug scene.
}

void Scene::update(float /*dt*/) {
  // Nothing animated yet.
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // Keep empty for now. We'll add real meshes here later.
}

void Scene::renderDebug(const Mat4& view, const Mat4& proj) {
#ifdef FE_NATIVE
  // NOTE:
  // This assumes Mat4 is laid out in a way compatible with glLoadMatrixf.
  // If you see nothing, the most likely issue is matrix layout (row/column major).

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf((const float*)&proj);

  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf((const float*)&view);

  glDisable(GL_TEXTURE_2D);
  glDisable(GL_CULL_FACE);
  glLineWidth(1.0f);

  // World grid on y = 0
  DrawGrid(50.0f, 1.0f);

  // A few platform boxes (wireframe) you can aim for / collide with later
  DrawBox(Vec3(-2.0f, 0.5f, -2.0f), Vec3( 2.0f, 0.8f,  2.0f));
  DrawBox(Vec3( 3.0f, 1.2f, -1.0f), Vec3( 4.5f, 1.5f,  1.0f));
  DrawBox(Vec3(-4.0f, 0.2f,  3.0f), Vec3(-2.5f, 0.6f,  5.0f));
#endif
}
