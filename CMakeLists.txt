cmake_minimum_required(VERSION 3.16)

project(FickerEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_WEB "Build with Emscripten/WebGL" OFF)
option(BUILD_X11 "Native: build X11 + OpenGL (GLX) window" ON)

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
if(EMSCRIPTEN)
    message(STATUS "Configuring for WebAssembly (Emscripten)")

    target_compile_definitions(FickerEngine PRIVATE
        ENGINE_WEB=1
        FE_WEB=1
    )

    # Emit an HTML launcher.
    set_target_properties(FickerEngine PROPERTIES SUFFIX ".html")

    # WebGL2 + legacy fixed-function emulation (no shader files).
    # NOTE: do not combine FULL_ES2 with LEGACY_GL_EMULATION.
    target_link_options(FickerEngine PRIVATE
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sLEGACY_GL_EMULATION=1"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASYNCIFY"
        "-sASSERTIONS=1"
        # Optional: custom shell page (uncomment if you have one)
        # "--shell-file=${CMAKE_SOURCE_DIR}/shell.html"
    )

elseif(BUILD_WEB)
    message(FATAL_ERROR "BUILD_WEB=ON requires Emscripten. Use: emcmake cmake -S . -B webbuild -G Ninja")
else()
    # ----------------------------------------------------------------------------------
    # Native build (X11 + OpenGL via GLX)
    # ----------------------------------------------------------------------------------
    message(STATUS "Configuring native build (X11+OpenGL)")

    target_compile_definitions(FickerEngine PRIVATE FE_NATIVE=1)

    if(BUILD_X11)
        find_package(X11 REQUIRED)
        find_package(OpenGL REQUIRED)

        target_include_directories(FickerEngine PRIVATE ${X11_INCLUDE_DIR})
        target_link_libraries(FickerEngine PRIVATE ${X11_LIBRARIES} OpenGL::GL)

        # Needed for glX* on some toolchains.
        if(TARGET OpenGL::GLX)
            target_link_libraries(FickerEngine PRIVATE OpenGL::GLX)
        endif()
    endif()
endif()
