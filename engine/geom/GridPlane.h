#pragma once
#include "engine/render/RenderMesh.h"
struct GridPlane {
    static MeshData Build(int halfLines=20,float step=1.0f){
        MeshData m; m.topology=MeshData::Topology::Lines;
        const float y=0.f,extent=halfLines*step;
        auto line=[&](float x0,float z0,float x1,float z1){
            uint32_t base=(uint32_t)(m.positions.size()/3);
            m.positions.insert(m.positions.end(),{x0,y,z0,x1,y,z1});
            m.indices.push_back(base);m.indices.push_back(base+1);
        };
        for(int i=-halfLines;i<=halfLines;++i){float z=i*step;line(-extent,z,+extent,z);}
        for(int j=-halfLines;j<=halfLines;++j){float x=j*step;line(x,-extent,x,+extent);}
        return m;
    }
};
