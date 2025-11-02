cmake_minimum_required(VERSION 3.20)
project(FickerEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_WEB "Force web build config" OFF)

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

target_include_directories(FickerEngine PRIVATE
    ${CMAKE_SOURCE_DIR}/engine
)

if (EMSCRIPTEN OR BUILD_WEB)
    message(STATUS "Configuring for WebAssembly (WebGPU)")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
    target_compile_definitions(FickerEngine PRIVATE FE_WEB=1 FE_WEBGPU=1)
    target_link_options(FickerEngine PRIVATE
        "-sUSE_WEBGPU=1"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASSERTIONS=1"
    )
endif()
