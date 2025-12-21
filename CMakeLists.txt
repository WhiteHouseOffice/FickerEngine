cmake_minimum_required(VERSION 3.16)

project(FickerEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --------------------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------------------
file(GLOB_RECURSE ENGINE_CORE   CONFIGURE_DEPENDS "engine/core/*.cpp")
file(GLOB_RECURSE ENGINE_GEOM   CONFIGURE_DEPENDS "engine/geom/*.cpp")
file(GLOB_RECURSE ENGINE_RENDER CONFIGURE_DEPENDS "engine/render/*.cpp")
file(GLOB_RECURSE ENGINE_GAME   CONFIGURE_DEPENDS "engine/game/*.cpp")
set(RUNTIME_SRC "runtime/main.cpp")

set(ALL_SRC
    ${ENGINE_CORE}
    ${ENGINE_GEOM}
    ${ENGINE_RENDER}
    ${ENGINE_GAME}
    ${RUNTIME_SRC}
)
list(REMOVE_DUPLICATES ALL_SRC)

add_executable(FickerEngine ${ALL_SRC})

target_include_directories(FickerEngine PRIVATE
    "${CMAKE_SOURCE_DIR}/engine"
)

# --------------------------------------------------------------------------------------
# Web build (Emscripten)
# --------------------------------------------------------------------------------------
if(EMSCRIPTEN)
    message(STATUS "Configuring for WebAssembly (Emscripten)")

    target_compile_definitions(FickerEngine PRIVATE
        ENGINE_WEB=1
        FE_WEB=1
    )

    # IMPORTANT:
    # - LEGACY_GL_EMULATION provides fixed-function style APIs (glVertexPointer etc).
    # - Do NOT combine it with -sFULL_ES2=1 (Emscripten asserts).
    target_link_options(FickerEngine PRIVATE
        "-sASYNCIFY"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASSERTIONS=1"
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sLEGACY_GL_EMULATION=1"
    )

    set_target_properties(FickerEngine PROPERTIES SUFFIX ".html")

else()
    message(STATUS "Configuring native build")

    target_compile_definitions(FickerEngine PRIVATE
        FE_NATIVE=1
    )
endif()
