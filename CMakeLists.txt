cmake_minimum_required(VERSION 3.16)

project(FickerEngine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
# Emscripten / Web build
# --------------------------------------------------------------------------------------
if (EMSCRIPTEN)
    message(STATUS "Configuring for WebAssembly (Emscripten)")

    target_compile_definitions(FickerEngine PRIVATE
        FE_WEB=1
        FE_WEBGPU=1
        ENGINE_WEB=1
    )

    # Generate a full HTML page output
    set_target_properties(FickerEngine PROPERTIES
        SUFFIX ".html"
    )

    # Emscripten link options:
    target_link_options(FickerEngine PRIVATE
        "-sASYNCIFY"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASSERTIONS=1"
        "-sLEGACY_GL_EMULATION=1"
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sGL_UNSAFE_OPTS=0"

        # If you add these files back later, you can enable them:
        # "--pre-js=${CMAKE_CURRENT_SOURCE_DIR}/runtime/pre.js"
        # "--shell-file=${CMAKE_CURRENT_SOURCE_DIR}/runtime/shell.html"
    )

else()
    # ----------------------------------------------------------------------------------
    # Native build (minimal)
    # ----------------------------------------------------------------------------------
    message(STATUS "Configuring native build")

    target_compile_definitions(FickerEngine PRIVATE
        FE_NATIVE=1
    )

    # Add native libs here if/when you need them
    # target_link_libraries(FickerEngine PRIVATE ...)
endif()
