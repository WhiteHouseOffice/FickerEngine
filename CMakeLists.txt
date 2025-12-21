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
    message(STATUS "Configuring for WebAssembly (Emscripten + WebGPU)")

    # We are the web build
    target_compile_definitions(FickerEngine PRIVATE ENGINE_WEB=1
        FE_WEB=1
        FE_WEBGPU=1
    )

    # Generate a full HTML page
    set_target_properties(FickerEngine PROPERTIES
        SUFFIX ".html"
    )

    # Emscripten link options:
    "-sUSE_WEBGL2=1"
    "-sFULL_ES2=1"
    "-sLEGACY_GL_EMULATION=1"
    "-sALLOW_MEMORY_GROWTH=1"
    "--shell-file" "${CMAKE_SOURCE_DIR}/shell.html" # optional, only if you have it
    #  -sASSERTIONS       : runtime checks in debug builds
    target_link_options(FickerEngine PRIVATE
    #"-sUSE_WEBGPU"
    "-sASYNCIFY"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sASSERTIONS=1"
    "-sLEGACY_GL_EMULATION=1"
    "-sUSE_WEBGL2=1"
    "-sMIN_WEBGL_VERSION=2"
    "-sMAX_WEBGL_VERSION=2"
    "-sGL_UNSAFE_OPTS=0"


        # If you want custom shell / pre.js again, uncomment:
        # "--pre-js=${CMAKE_CURRENT_SOURCE_DIR}/runtime/pre.js"
        # "--shell-file=${CMAKE_CURRENT_SOURCE_DIR}/runtime/shell.html"
    )

else()
    # ----------------------------------------------------------------------------------
    # Native build (very minimal for now)
    # ----------------------------------------------------------------------------------
    message(STATUS "Configuring native build")

    target_compile_definitions(FickerEngine PRIVATE
        FE_NATIVE=1
    )

    # Add native libs here if/when you need them
    # target_link_libraries(FickerEngine PRIVATE ...)
endif()
