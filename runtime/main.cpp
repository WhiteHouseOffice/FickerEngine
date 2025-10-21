
#include "engine/core/Engine.h"
#include "engine/render/IRenderer.h"
#include <memory>
#include <cstdio>

#ifdef FE_WEB
extern std::unique_ptr<IRenderer> CreateWebRenderer();
#include <emscripten.h>
#include <emscripten/bind.h>

static Engine* g_engine = nullptr;

static void stepOnce() {
  if(g_engine) g_engine->runOnce(1.0/60.0);
}

EMSCRIPTEN_BINDINGS(fe_bindings) {
  emscripten::function("stepOnce", &stepOnce);
}

int main() {
  EngineConfig cfg; cfg.width=800; cfg.height=600;
  auto r = CreateWebRenderer();
  if(!r->init(cfg.width, cfg.height)) {
    printf("Web init failed\\n");
    return 1;
  }
  static Engine eng(cfg, std::move(r));
  g_engine = &eng;
  return 0;
}

#else
int main() {
  printf("Native build not implemented in this bootstrap.\\n");
  return 0;
}
#endif
