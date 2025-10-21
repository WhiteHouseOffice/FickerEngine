
cmake_minimum_required(VERSION 3.20)
project(FickerEngine C CXX)
set(CMAKE_CXX_STANDARD 20)

option(BUILD_WEB "Build Web (Emscripten) target" OFF)

add_library(engine_core
    engine/core/Time.cpp
    engine/core/Engine.cpp
)

target_include_directories(engine_core PUBLIC engine)

if(BUILD_WEB)
    add_compile_definitions(FE_WEB)
    add_library(renderer_web
        engine/render/WebGPURenderer.cpp
    )
    add_executable(engine
        runtime/main.cpp
    )
    target_link_libraries(engine PRIVATE engine_core renderer_web)

    # ✅ Ensure Emscripten links WebGPU support (provides webgpu headers/shim)
    target_link_options(engine PRIVATE "-sUSE_WEBGPU=1")

    set_target_properties(engine PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
else()
    add_executable(engine
        runtime/main.cpp
    )
    target_link_libraries(engine PRIVATE engine_core)
    set_target_properties(engine PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()
