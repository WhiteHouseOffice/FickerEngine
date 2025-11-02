#include "game/Scene.h"
#include "geom/GridPlane.h"
#include "render/RenderMesh.h"

void Scene::Build() {
    geom::GridData g = geom::BuildGrid(40.f, 1.f); // 40×40 meters, 1m spacing
    render::MeshUpload up;
    up.positions = std::move(g.positions);
    up.indices   = std::move(g.indices);
    render::UploadGrid(up);
}
