#include "render/WebGPUContext.h"

#include <cstdio>

#ifdef __EMSCRIPTEN__
  #include <emscripten/html5.h>
#endif

namespace render {

WebGPUContext& WebGPUContext::Get() {
  static WebGPUContext instance;
  return instance;
}

void WebGPUContext::init() {
  if (initialized_) return;

#ifdef __EMSCRIPTEN__
  EmscriptenWebGLContextAttributes attr;
  emscripten_webgl_init_context_attributes(&attr);

  attr.alpha  = EM_FALSE;
  attr.depth  = EM_TRUE;
  attr.stencil = EM_FALSE;
  attr.antialias = EM_TRUE;

  // Request WebGL2
  attr.majorVersion = 2;
  attr.minorVersion = 0;
  attr.enableExtensionsByDefault = EM_TRUE;

  // Default Emscripten canvas has id="canvas"
  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE ctx =
      emscripten_webgl_create_context("#canvas", &attr);

  if (ctx <= 0) {
    std::printf("[WebGPUContext] failed to create WebGL2 context (error %d)\n", ctx);
    return;
  }

  if (emscripten_webgl_make_context_current(ctx) != EMSCRIPTEN_RESULT_SUCCESS) {
    std::printf("[WebGPUContext] failed to make WebGL context current\n");
    return;
  }

  std::printf("[WebGPUContext] WebGL2 context initialized\n");
#else
  std::printf("[WebGPUContext] init (native stub)\n");
#endif

  initialized_ = true;
}

} // namespace render
