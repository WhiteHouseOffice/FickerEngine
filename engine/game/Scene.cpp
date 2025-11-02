#include "game/Scene.h"
#include "geom/GridPlane.h"
#include "render/RenderMesh.h"

void Scene::Build() {
    auto g = geom::BuildGrid(40.f, 1.f);  // 40×40m, 1m spacing
    render::MeshUpload up;
    up.positions = std::move(g.positions);
    up.indices   = std::move(g.indices);
    render::UploadGrid(up);
}
