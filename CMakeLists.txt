cmake_minimum_required(VERSION 3.20)
project(FickerEngine C CXX)
set(CMAKE_CXX_STANDARD 20)

option(BUILD_WEB "Build Web (Emscripten) target" OFF)

# Core engine library
add_library(engine_core
    engine/core/Time.cpp
    engine/core/Engine.cpp
)
target_include_directories(engine_core PUBLIC engine)

if(BUILD_WEB)
    add_compile_definitions(FE_WEB)

    # WebGPU renderer (Emscripten)
    add_library(renderer_web
        engine/render/WebGPURenderer.cpp
    )
    target_include_directories(renderer_web PUBLIC engine)

    # Executable
    add_executable(engine
        runtime/main.cpp
    )
    target_link_libraries(engine PRIVATE engine_core renderer_web)

    # ✅ Emscripten link settings:
    # - USE_WEBGPU: provide WebGPU shim/libs
    target_link_options(engine PRIVATE
        "-sUSE_WEBGPU=1"
        # If you ever need growth / debug:
        # "-sALLOW_MEMORY_GROWTH=1"
        # "-sASSERTIONS=1"
    )

    # ✅ Link embind explicitly (don’t pass -sEMBIND=1)
    # You can do this either way:
    target_link_libraries(engine PRIVATE embind)
    # (equivalent alternative would be:
    # target_link_options(engine PRIVATE "-lembind")
    # )

    set_target_properties(engine PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
else()
    # Native stub build (no renderer yet)
    add_executable(engine
        runtime/main.cpp
    )
    target_link_libraries(engine PRIVATE engine_core)
    set_target_properties(engine PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
target_include_directories(engine PRIVATE engine)
endif()
