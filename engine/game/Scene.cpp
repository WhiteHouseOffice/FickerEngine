#include "Scene.h"
#include "engine/geom/GridPlane.h"
#include "engine/geom/MarkerCross.h"
void Scene::Build(){
    grid.mesh.Upload(GridPlane::Build(20,1.0f));
    grid.color={0.6f,0.6f,0.6f};
    marker.mesh.Upload(MarkerCross::Build(0.1f,0.25f));
    marker.color={1.0f,1.0f,1.0f};
    drawOrder={&grid,&marker};
}
