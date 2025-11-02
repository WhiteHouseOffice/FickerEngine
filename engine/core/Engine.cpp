#include "core/Engine.h"
#include "core/Time.h"
#include "game/Scene.h"
#include "game/Game.h"
#include "math/MiniMath.h"
#include <memory>

#if defined(FE_WEB)
  #include <emscripten/emscripten.h>
  #define GL_GLEXT_PROTOTYPES 1
  #include <GLES2/gl2.h>
  #include <GLES2/gl2ext.h>
  extern "C" {
    void glMatrixMode(unsigned int);
    void glLoadMatrixf(const float*);
    void glColor3f(float,float,float);
    void glEnableClientState(unsigned int);
    void glDisableClientState(unsigned int);
    void glVertexPointer(int,unsigned int,int,const void*);
  }
#else
  #include <GL/glew.h>
  #include <GL/gl.h>
#endif

struct Engine::Impl {
  Time  time;
  Scene scene;
  Game  game;

  void render_frame(int w,int h){
    // Fixed-function projection
    Mat4 P = perspective(60.f*3.1415926f/180.f, (float)w/(float)h, 0.01f, 500.f);

    glViewport(0,0,w,h);
    glEnable(0x0B71 /*GL_DEPTH_TEST*/);
    glClearColor(0.05f,0.05f,0.06f,1.f);
    glClear(0x00004000 /*GL_COLOR_BUFFER_BIT*/ | 0x00000100 /*GL_DEPTH_BUFFER_BIT*/);

    glMatrixMode(0x1701 /*GL_PROJECTION*/);
    glLoadMatrixf(P.m);
    glMatrixMode(0x1700 /*GL_MODELVIEW*/);
    glLoadMatrixf(game.View().m);

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
  impl->game.Init();
}

void Engine::stepOnce(){
  const double dt = impl->time.tick();        // seconds
  const float  clamped = (float)std::min(dt, 0.1); // avoid giant steps
  impl->game.Update(clamped);
  impl->render_frame(1280, 720);
}

#if defined(FE_WEB)
extern "C" {
  EMSCRIPTEN_KEEPALIVE void fe_step_once(){ Engine::instance().stepOnce(); }
}
#endif
