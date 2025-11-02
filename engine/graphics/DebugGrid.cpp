#include "DebugGrid.h"
#include <cmath>
#include <cstring>

// If you have a platform abstraction, route these includes through it.
// For now, assume GL path is available when not on native-WGPU:
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
  #include <GL/glew.h>
#endif

DebugGrid::DebugGrid() {}
DebugGrid::~DebugGrid() { destroyGL(); }

void DebugGrid::buildCPU(int halfLines, float step) {
    vertices_.clear();
    indices_.clear();

    const float y = 0.0f;
    const int N = halfLines;
    const float extent = N * step;

    auto pushLine = [&](float x0,float z0,float x1,float z1, bool bold){
        float c = bold ? 0.9f : 0.5f;
        uint32_t base = (uint32_t)vertices_.size();
        vertices_.push_back({x0, y, z0, c,c,c});
        vertices_.push_back({x1, y, z1, c,c,c});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
    };

    // Lines parallel to X (varying Z)
    for (int i = -N; i <= N; ++i) {
        bool bold = (i == 0);
        float z = i * step;
        pushLine(-extent, z, +extent, z, bold);
    }

    // Lines parallel to Z (varying X)
    for (int j = -N; j <= N; ++j) {
        bool bold = (j == 0);
        float x = j * step;
        pushLine(x, -extent, x, +extent, bold);
    }

    indexCount_ = (int)indices_.size();
}

void DebugGrid::buildGL() {
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
    if (!vao_) glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    if (!vbo_) glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices_.size()*sizeof(GridVertex), vertices_.data(), GL_STATIC_DRAW);

    if (!ebo_) glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size()*sizeof(uint32_t), indices_.data(), GL_STATIC_DRAW);

    // layout(location=0) vec3 aPos
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)0);

    // layout(location=1) vec3 aColor
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GridVertex), (void*)(3*sizeof(float)));

    glBindVertexArray(0);
#endif
}

void DebugGrid::destroyGL() {
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
    if (ebo_) { glDeleteBuffers(1, &ebo_); ebo_ = 0; }
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
#endif
}

void DebugGrid::Init(int halfLines, float step) {
    buildCPU(halfLines, step);
    buildGL();
}

void DebugGrid::Render(const float* mvp) {
#if !defined(PLATFORM_WEB) || defined(FORCE_GL_PATH)
    // Assumes the current GL program has:
    //   uniform mat4 uMVP;
    //   layout(location=0) aPos; layout(location=1) aColor;
    GLint prog = 0; glGetIntegerv(GL_CURRENT_PROGRAM, &prog);
    if (!prog || indexCount_ == 0) return;

    GLint loc = glGetUniformLocation(prog, "uMVP");
    if (loc >= 0 && mvp) {
        glUniformMatrix4fv(loc, 1, GL_FALSE, mvp);
    }

    glBindVertexArray(vao_);
    glDrawElements(GL_LINES, indexCount_, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
#else
    (void)mvp; // TODO: implement for WebGPU path if needed
#endif
}
