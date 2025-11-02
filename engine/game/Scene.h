#pragma once
#include "GameObject.h"
#include <vector>
struct Scene {
    GameObject grid;
    GameObject marker;
    std::vector<GameObject*> drawOrder;
    void Build();
};
