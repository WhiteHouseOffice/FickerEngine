#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "math/MiniMath.h"
#include <memory>

#if defined(FE_WEB)
  #include <emscripten/emscripten.h>
  #include <GLES2/gl2.h>
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
#endif

struct Engine::Impl {
  Time  time;
  Scene scene;
  float eye[3]{6.0f,5.0f,8.0f};
  float ctr[3]{0.0f,0.0f,0.0f};
  float up [3]{0.0f,1.0f,0.0f};

  void render_frame(int w,int h){
    Mat4 P = perspective(60.f*3.1415926f/180.f, (float)w/(float)h, 0.01f, 200.f);
    Mat4 V = lookAt(eye, ctr, up);

    glViewport(0,0,w,h);
    glEnable(0x0B71 /*GL_DEPTH_TEST*/);
    glClearColor(0.05f,0.05f,0.06f,1.f);
    glClear(0x00004000 /*GL_COLOR_BUFFER_BIT*/ | 0x00000100 /*GL_DEPTH_BUFFER_BIT*/);

    glMatrixMode(0x1701 /*GL_PROJECTION*/);
    glLoadMatrixf(P.m);
    glMatrixMode(0x1700 /*GL_MODELVIEW*/);
    glLoadMatrixf(V.m);

    for (auto* obj : scene.drawOrder)
      obj->mesh.Draw(obj->color[0], obj->color[1], obj->color[2]);
  }
};

Engine& Engine::instance(){ static Engine g; return g; }
Engine::Engine():impl(std::make_unique<Impl>()){} 
Engine::~Engine()=default;

void Engine::init(){
#if !defined(FE_WEB)
  glewInit();
#endif
  impl->scene.Build();
}

void Engine::stepOnce(){
  impl->render_frame(1280, 720);
}

#if defined(FE_WEB)
extern "C" {
  EMSCRIPTEN_KEEPALIVE void fe_step_once(){ Engine::instance().stepOnce(); }
}
#endif
