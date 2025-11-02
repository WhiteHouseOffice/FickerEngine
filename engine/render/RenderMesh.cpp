#include "RenderMesh.h"
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
  #include <GL/glew.h>
#endif
bool RenderMesh::Upload(const MeshData& d){
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
    mode_ = (d.topology==MeshData::Topology::Lines)?0x0001:0x0004;
    indexCount_ = (int)d.indices.size();
    if(!vao_) glGenVertexArrays(1,&vao_);
    if(!vbo_) glGenBuffers(1,&vbo_);
    if(!ebo_) glGenBuffers(1,&ebo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER,vbo_);
    glBufferData(GL_ARRAY_BUFFER,d.positions.size()*sizeof(float),d.positions.data(),GL_STATIC_DRAW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3,GL_FLOAT,0,0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,d.indices.size()*sizeof(uint32_t),d.indices.data(),GL_STATIC_DRAW);
    glBindVertexArray(0);
    return true;
#else
    (void)d; return false;
#endif
}
void RenderMesh::Draw(float r,float g,float b) const{
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
    if(!indexCount_) return;
    glColor3f(r,g,b);
    glBindVertexArray(vao_);
    glDrawElements(mode_,indexCount_,0x1405,0);
    glBindVertexArray(0);
#endif
}
