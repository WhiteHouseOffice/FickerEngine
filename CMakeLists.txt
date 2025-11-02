cmake_minimum_required(VERSION 3.20)
project(FickerEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Collect sources (keep your canonical layout) ---
file(GLOB_RECURSE ENGINE_CORE      engine/core/*.[ch]pp engine/core/*.h)
file(GLOB_RECURSE ENGINE_RENDER    engine/render/*.[ch]pp engine/render/*.h)
file(GLOB_RECURSE ENGINE_GEOM      engine/geom/*.[ch]pp  engine/geom/*.h)
file(GLOB_RECURSE ENGINE_GAME      engine/game/*.[ch]pp  engine/game/*.h)
file(GLOB_RECURSE ENGINE_MATH      engine/math/*.h)
file(GLOB_RECURSE ENGINE_PLATFORM  engine/platform/*.[ch]pp engine/platform/*.h)
file(GLOB_RECURSE ENGINE_RUNTIME   runtime/*.[ch]pp runtime/*.h)

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
target_include_directories(FickerEngine PRIVATE ${CMAKE_SOURCE_DIR}/engine)

# --- Web (Emscripten) config: use emdawnwebgpu everywhere ---
if (EMSCRIPTEN OR DEFINED ENV{EMSCRIPTEN})
  message(STATUS "Configuring for WebAssembly (WebGPU via emdawnwebgpu)")
  set(CMAKE_EXECUTABLE_SUFFIX ".html")

  # Preprocessor switches expected by our code
  target_compile_definitions(FickerEngine PRIVATE
    FE_WEB=1
    FE_WEBGPU=1
    WGPU_TARGET_EMSCRIPTEN=1
  )

  # Apply Dawn (emdawnwebgpu) to ALL translation units and at link
  # (This provides <webgpu/webgpu.h> + libwebgpu, and JS glue.)
  target_compile_options(FickerEngine PRIVATE
    --use-port=emdawnwebgpu
  )
  target_link_options(FickerEngine PRIVATE
    --use-port=emdawnwebgpu
    -sALLOW_MEMORY_GROWTH=1
    -sASSERTIONS=1
    -sASYNCIFY
  )

  # Optional: set a friendlier HTML title
  set_target_properties(FickerEngine PROPERTIES OUTPUT_NAME "FickerEngine")
else()
  message(STATUS "Native build detected — minimal setup (no GL).")
  # If you later want native rendering, add it here explicitly.
endif()
