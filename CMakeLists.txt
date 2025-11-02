cmake_minimum_required(VERSION 3.20)
project(FickerEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Sources
file(GLOB_RECURSE ENGINE_CORE     engine/core/*.cpp engine/core/*.h)
file(GLOB_RECURSE ENGINE_RENDER   engine/render/*.cpp engine/render/*.h)
file(GLOB_RECURSE ENGINE_GEOM     engine/geom/*.cpp   engine/geom/*.h)
file(GLOB_RECURSE ENGINE_GAME     engine/game/*.cpp   engine/game/*.h)
file(GLOB_RECURSE ENGINE_MATH     engine/math/*.h)
file(GLOB_RECURSE ENGINE_PLATFORM engine/platform/*.cpp engine/platform/*.h)
file(GLOB_RECURSE ENGINE_RUNTIME  runtime/*.cpp runtime/*.h)

set(ALL_ENGINE_SOURCES
  ${ENGINE_CORE}
  ${ENGINE_RENDER}
  ${ENGINE_GEOM}
  ${ENGINE_GAME}
  ${ENGINE_MATH}
  ${ENGINE_PLATFORM}
  ${ENGINE_RUNTIME}
)
list(REMOVE_DUPLICATES ALL_ENGINE_SOURCES)

add_executable(FickerEngine ${ALL_ENGINE_SOURCES})

# Our headers are included like "core/Time.h", "render/WebGPUContext.h", etc.
target_include_directories(FickerEngine PRIVATE ${CMAKE_SOURCE_DIR}/engine)

# --- Web (Emscripten) config
if (EMSCRIPTEN)
  message(STATUS "Configuring for WebAssembly (WebGPU via emdawnwebgpu)")
  set(CMAKE_EXECUTABLE_SUFFIX ".html")

  target_compile_definitions(FickerEngine PRIVATE
    FE_WEB=1
    FE_WEBGPU=1
    WGPU_TARGET_EMSCRIPTEN=1
  )

  # IMPORTANT: Apply to ALL files (compile & link)
  target_compile_options(FickerEngine PRIVATE
    --use-port=emdawnwebgpu
  )
  target_link_options(FickerEngine PRIVATE
    --use-port=emdawnwebgpu
    -sALLOW_MEMORY_GROWTH=1
    -sASSERTIONS=1
    -sASYNCIFY
  )
endif()
