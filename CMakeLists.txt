cmake_minimum_required(VERSION 3.20)
project(FickerEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Engine core files ---
file(GLOB_RECURSE ENGINE_CORE engine/core/*.cpp engine/core/*.h)
file(GLOB_RECURSE ENGINE_PLATFORM engine/platform/*.cpp engine/platform/*.h)

# --- Engine components ---
file(GLOB_RECURSE ENGINE_RENDER engine/render/*.cpp engine/render/*.h)
file(GLOB_RECURSE ENGINE_GEOM   engine/geom/*.cpp   engine/geom/*.h)
file(GLOB_RECURSE ENGINE_GAME   engine/game/*.cpp   engine/game/*.h)
file(GLOB_RECURSE ENGINE_MATH   engine/math/*.h)

add_executable(FickerEngine
    ${ENGINE_CORE}
    ${ENGINE_PLATFORM}
    ${ENGINE_RENDER}
    ${ENGINE_GEOM}
    ${ENGINE_GAME}
    ${ENGINE_MATH}
)

target_include_directories(FickerEngine PRIVATE
    ${CMAKE_SOURCE_DIR}/engine
)

# --- Platform conditions ---
if (EMSCRIPTEN)
    message(STATUS "Building for WebAssembly")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    target_compile_definitions(FickerEngine PRIVATE PLATFORM_WEB=1)
    target
