cmake_minimum_required(VERSION 3.16)
project(FickerEngine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_WEB "Build with Emscripten/WebAssembly" ON)

# --------------------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------------------
file(GLOB_RECURSE ENGINE_CORE   CONFIGURE_DEPENDS "engine/core/*.cpp")
file(GLOB_RECURSE ENGINE_GEOM   CONFIGURE_DEPENDS "engine/geom/*.cpp")
file(GLOB_RECURSE ENGINE_RENDER CONFIGURE_DEPENDS "engine/render/*.cpp")
file(GLOB_RECURSE ENGINE_GAME   CONFIGURE_DEPENDS "engine/game/*.cpp")

set(RUNTIME_SRC "runtime/main.cpp")

set(ENGINE_SOURCES
    ${ENGINE_CORE}
    ${ENGINE_GEOM}
    ${ENGINE_RENDER}
    ${ENGINE_GAME}
    ${RUNTIME_SRC}
)

list(REMOVE_DUPLICATES ENGINE_SOURCES)

add_executable(FickerEngine ${ENGINE_SOURCES})
target_include_directories(FickerEngine PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/engine")

# --------------------------------------------------------------------------------------
# Web (Emscripten) vs Native
# --------------------------------------------------------------------------------------
if(EMSCRIPTEN OR CMAKE_SYSTEM_NAME STREQUAL "Emscripten" OR BUILD_WEB)
    message(STATUS "Configuring for WebAssembly (Emscripten)")

    target_compile_definitions(FickerEngine PRIVATE
        ENGINE_WEB=1
        FE_WEB=1
        # keep this if you use it elsewhere; it doesn't force WebGPU usage by itself:
        FE_WEBGPU=1
    )

    # IMPORTANT: fixed-function emulation for glVertexPointer/glEnableClientState etc.
    target_link_options(FickerEngine PRIVATE
        "-sLEGACY_GL_EMULATION=1"
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sGL_UNSAFE_OPTS=0"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASSERTIONS=1"
        "-sWASM=1"
        "-sEXIT_RUNTIME=0"
        # produces .html + .js + .wasm by default
    )

else()
    message(STATUS "Configuring native build")

    target_compile_definitions(FickerEngine PRIVATE
        FE_NATIVE=1
    )
endif()
