#pragma once
#include "engine/render/RenderMesh.h"
struct MarkerCross {
    static MeshData Build(float y=0.1f,float s=0.25f){
        MeshData m; m.topology=MeshData::Topology::Lines;
        auto add=[&](float x,float z){m.positions.insert(m.positions.end(),{x,y,z});};
        add(-s,0);add(s,0); add(0,-s);add(0,s); add(-s,-s);add(s,s); add(-s,s);add(s,-s);
        for(uint32_t i=0;i<8;i+=2){m.indices.push_back(i);m.indices.push_back(i+1);}
        return m;
    }
};
