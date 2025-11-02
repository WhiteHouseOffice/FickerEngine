#include "RenderMesh.h"

#if defined(FE_WEB)
  #define GL_GLEXT_PROTOTYPES 1
  #include <GLES2/gl2.h>
  #include <GLES2/gl2ext.h>
  extern "C" {
    void glEnableClientState(unsigned int);
    void glDisableClientState(unsigned int);
    void glVertexPointer(int, unsigned int, int, const void*);
    void glColor3f(float, float, float);
  }
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
#endif

bool RenderMesh::Upload(const MeshData& d){
    mode_ = (d.topology==MeshData::Topology::Lines) ? 0x0001 /*GL_LINES*/ : 0x0004 /*GL_TRIANGLES*/;
    indexCount_ = (int)d.indices.size();

    if(!vbo_) glGenBuffers(1,&vbo_);
    if(!ebo_) glGenBuffers(1,&ebo_);

    glBindBuffer(0x8892 /*GL_ARRAY_BUFFER*/, vbo_);
    glBufferData(0x8892, d.positions.size()*sizeof(float), d.positions.data(), 0x88E4 /*GL_STATIC_DRAW*/);

    glBindBuffer(0x8893 /*GL_ELEMENT_ARRAY_BUFFER*/, ebo_);
    glBufferData(0x8893, d.indices.size()*sizeof(uint32_t), d.indices.data(), 0x88E4);
    return true;
}

void RenderMesh::Draw(float r,float g,float b) const{
    if(!indexCount_) return;

    glColor3f(r,g,b);

    glBindBuffer(0x8892 /*GL_ARRAY_BUFFER*/, vbo_);
    glEnableClientState(0x8074 /*GL_VERTEX_ARRAY*/);
    glVertexPointer(3, 0x1406 /*GL_FLOAT*/, 0, (const void*)0);

    glBindBuffer(0x8893 /*GL_ELEMENT_ARRAY_BUFFER*/, ebo_);
    glDrawElements(mode_, indexCount_, 0x1405 /*GL_UNSIGNED_INT*/, (const void*)0);

    glDisableClientState(0x8074 /*GL_VERTEX_ARRAY*/);
    glBindBuffer(0x8892, 0);
    glBindBuffer(0x8893, 0);
}
