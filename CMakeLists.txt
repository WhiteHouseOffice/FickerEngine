cmake_minimum_required(VERSION 3.20)
project(FickerEngine)

# ---- Language ---------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Options ----------------------------------------------------------------
option(BUILD_WEB "Force web build config (Emscripten)" OFF)

# ---- Sources ----------------------------------------------------------------
# We keep the structure clean and simple: engine/* and runtime/*
file(GLOB_RECURSE ENGINE_CORE      engine/core/*.cpp      engine/core/*.h)
file(GLOB_RECURSE ENGINE_RENDER    engine/render/*.cpp    engine/render/*.h)
file(GLOB_RECURSE ENGINE_GEOM      engine/geom/*.cpp      engine/geom/*.h)
file(GLOB_RECURSE ENGINE_GAME      engine/game/*.cpp      engine/game/*.h)
file(GLOB_RECURSE ENGINE_MATH      engine/math/*.h)
file(GLOB_RECURSE ENGINE_PLATFORM  engine/platform/*.cpp  engine/platform/*.h)
file(GLOB_RECURSE ENGINE_RUNTIME   runtime/*.cpp          runtime/*.h)

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

# Expose "engine/" as an include root so you can #include "render/Whatever.h"
target_include_directories(FickerEngine PRIVATE
    ${CMAKE_SOURCE_DIR}/engine
)

# ---- Web / Emscripten (WebGPU via Dawn port) --------------------------------
if (EMSCRIPTEN OR BUILD_WEB)
    message(STATUS "Configuring for WebAssembly (WebGPU via emdawnwebgpu)")
    # Produce FickerEngine.html (plus .js/.wasm)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")

    # Compile-time macros the code uses to branch
    target_compile_definitions(FickerEngine PRIVATE
        FE_WEB=1
        FE_WEBGPU=1
        WGPU_TARGET_EMSCRIPTEN=1
    )

    # IMPORTANT: emscripten's legacy -sUSE_WEBGPU=1 is removed.
    # Use the Dawn WebGPU port instead:
    target_compile_options(FickerEngine PRIVATE
        "--use-port=emdawnwebgpu"
    )
    target_link_options(FickerEngine PRIVATE
        "--use-port=emdawnwebgpu"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASSERTIONS=1"
        "-sASYNCIFY"
    )

    # (Optional) If you later add assets/, you can preload them like this:
    # set_property(TARGET FickerEngine APPEND_STRING PROPERTY LINK_FLAGS
    #    " --preload-file assets@/assets")

else()
    message(STATUS "Native build is not configured in this minimal file. Use Emscripten or -DBUILD_WEB=ON.")
endif()
