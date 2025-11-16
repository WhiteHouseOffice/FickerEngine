#include "game/Scene.h"

#include "math/MiniMath.h"

#if defined(FE_WEB)
  #include <GL/gl.h>
#endif

Scene::Scene() {
  // Build simple debug geometry once: a ground grid and an origin marker.
  grid_.build(/*size*/ 10.0f, /*subdivisions*/ 20);
  marker_.build(/*size*/ 0.5f);
}

void Scene::init() {
  // Copy CPU geometry into CPU-side "render meshes" once.
  gridMesh_.uploadGrid(grid_);
  markerMesh_.uploadMarker(marker_);

  // One-time debug dump so we see counts but avoid per-frame spam.
  gridMesh_.debugPrint("grid");
  markerMesh_.debugPrint("marker");
}

void Scene::update(float /*dt*/) {
  // No scene-level simulation yet.
}

void Scene::render(const Mat4& view, const Mat4& proj) {
#if defined(FE_WEB)
  // Basic fixed-function-style rendering via Emscripten's LEGACY_GL_EMULATION.
  glEnable(GL_DEPTH_TEST);

  // Background & clear
  glClearColor(0.05f, 0.05f, 0.06f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Upload projection and view matrices.
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj.m);

  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.m);

  glEnableClientState(GL_VERTEX_ARRAY);

  // Draw ground grid as lines.
  if (!gridMesh_.positions.empty() && !gridMesh_.indices.empty()) {
    glVertexPointer(3, GL_FLOAT, sizeof(Vec3), gridMesh_.positions.data());
    glDrawElements(GL_LINES,
                   static_cast<GLsizei>(gridMesh_.indices.size()),
                   GL_UNSIGNED_INT,
                   gridMesh_.indices.data());
  }

  // Draw marker cross at the origin.
  if (!markerMesh_.positions.empty() && !markerMesh_.indices.empty()) {
    glVertexPointer(3, GL_FLOAT, sizeof(Vec3), markerMesh_.positions.data());
    glDrawElements(GL_LINES,
                   static_cast<GLsizei>(markerMesh_.indices.size()),
                   GL_UNSIGNED_INT,
                   markerMesh_.indices.data());
  }

  glDisableClientState(GL_VERTEX_ARRAY);
#else
  (void)view;
  (void)proj;
#endif
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // No per-frame logging here – keeps the console clean.
}
