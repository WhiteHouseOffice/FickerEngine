// engine/game/Scene.cpp
#include "game/Scene.h"

#include <cstdio>

#include "geom/GridPlane.h"
#include "geom/MarkerCross.h"

using geom::GridPlane;
using geom::MarkerCross;

Scene::Scene() {
  // --- Build a tiny test grid directly on the CPU ---
  GridPlane grid;

  // Simple line from (-1,0,0) to (1,0,0)
  grid.positions.emplace_back(-1.0f, 0.0f, 0.0f);
  grid.positions.emplace_back( 1.0f, 0.0f, 0.0f);
  grid.indices.push_back(0);
  grid.indices.push_back(1);

  gridMesh.uploadGrid(grid);
  gridMesh.debugPrint("grid");

  // --- Build a tiny marker cross directly on the CPU ---
  MarkerCross marker;

  // Simple vertical line from (0,0,0) to (0,1,0)
  marker.positions.emplace_back(0.0f, 0.0f, 0.0f);
  marker.positions.emplace_back(0.0f, 1.0f, 0.0f);
  marker.indices.push_back(0);
  marker.indices.push_back(1);

  markerMesh.uploadMarker(marker);
  markerMesh.debugPrint("marker");
}

void Scene::render(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // GPU rendering will live here later.
}

void Scene::renderDebug(const Mat4& /*view*/, const Mat4& /*proj*/) {
  // For now just re-print stats so we see something happening.
  gridMesh.debugPrint("grid");
  markerMesh.debugPrint("marker");
}
