cmake_minimum_required(VERSION 3.20)
project(FickerEngine)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE ENGINE_SOURCES
    "engine/core/*.cpp"
    "engine/game/*.cpp"
    "engine/geom/*.cpp"
    "engine/math/*.cpp"
    "engine/render/*.cpp"
    "runtime/*.cpp"
)

add_executable(FickerEngine ${ENGINE_SOURCES})

# --- Universal include path for all engine code ---
target_include_directories(FickerEngine PRIVATE
    ${CMAKE_SOURCE_DIR}/engine
)

# --- Platform specific flags ---
if (EMSCRIPTEN)
    message(STATUS "Configuring for WebAssembly (WebGPU via emdawnwebgpu)")
    set_target_properties(FickerEngine PROPERTIES OUTPUT_NAME "FickerEngine")
    target_compile_definitions(FickerEngine PRIVATE FE_WEB=1 FE_WEBGPU=1 WGPU_TARGET_EMSCRIPTEN=1)
    target_link_options(FickerEngine PRIVATE
        --use-port=emdawnwebgpu
        -sALLOW_MEMORY_GROWTH=1
        -sMAX_WEBGL_VERSION=2
        -sMIN_WEBGL_VERSION=2
    )
else()
    message(STATUS "Configuring native build")
    target_compile_definitions(FickerEngine PRIVATE FE_NATIVE=1)
endif()
