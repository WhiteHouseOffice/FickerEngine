cmake_minimum_required(VERSION 3.16)

project(FickerEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_WEB "Build with Emscripten/WebGL" OFF)

# --------------------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------------------
file(GLOB_RECURSE ENGINE_CORE    "engine/core/*.cpp")
file(GLOB_RECURSE ENGINE_GEOM    "engine/geom/*.cpp")
file(GLOB_RECURSE ENGINE_RENDER  "engine/render/*.cpp")
file(GLOB_RECURSE ENGINE_GAME    "engine/game/*.cpp")
set(RUNTIME_SRC "runtime/main.cpp")

set(SOURCES
    ${ENGINE_CORE}
    ${ENGINE_GEOM}
    ${ENGINE_RENDER}
    ${ENGINE_GAME}
    ${RUNTIME_SRC}
)
list(REMOVE_DUPLICATES SOURCES)

add_executable(FickerEngine ${SOURCES})

target_include_directories(FickerEngine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
)

# --------------------------------------------------------------------------------------
# Web build (Emscripten)
# --------------------------------------------------------------------------------------
if(EMSCRIPTEN OR BUILD_WEB)
    message(STATUS "Configuring for WebAssembly (Emscripten)")

    target_compile_definitions(FickerEngine PRIVATE
        ENGINE_WEB=1
        FE_WEB=1
    )

    # Emit an HTML launcher.
    set_target_properties(FickerEngine PROPERTIES SUFFIX ".html")

    # IMPORTANT:
    # - We use LEGACY_GL_EMULATION (fixed-function style).
    # - DO NOT enable FULL_ES2 at the same time (it conflicts and crashes link).
    target_link_options(FickerEngine PRIVATE
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sLEGACY_GL_EMULATION=1"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASYNCIFY"
        "-sASSERTIONS=1"
    )

else()
    message(STATUS "Configuring native build")

    target_compile_definitions(FickerEngine PRIVATE
        FE_NATIVE=1
    )
endif()
