cmake_minimum_required(VERSION 3.16)

project(FickerEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_WEB "Build with Emscripten/WebGL" OFF)
option(BUILD_X11 "Native: build X11 + OpenGL window" ON)

# --------------------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------------------
file(GLOB_RECURSE ENGINE_CORE    CONFIGURE_DEPENDS "engine/core/*.cpp")
file(GLOB_RECURSE ENGINE_GEOM    CONFIGURE_DEPENDS "engine/geom/*.cpp")
file(GLOB_RECURSE ENGINE_RENDER  CONFIGURE_DEPENDS "engine/render/*.cpp")
file(GLOB_RECURSE ENGINE_GAME    CONFIGURE_DEPENDS "engine/game/*.cpp")

# Entry point (you decided to put it in repo root)
set(ENTRY_SRC "Main.cpp")

set(SOURCES
    ${ENGINE_CORE}
    ${ENGINE_GEOM}
    ${ENGINE_RENDER}
    ${ENGINE_GAME}
    ${ENTRY_SRC}
)
list(REMOVE_DUPLICATES SOURCES)

# --------------------------------------------------------------------------------------
# Target
# --------------------------------------------------------------------------------------
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
        FE_WEB=1
        ENGINE_WEB=1
    )

    # Emit an HTML launcher.
    set_target_properties(FickerEngine PROPERTIES SUFFIX ".html")

    # WebGL2 + legacy fixed-function emulation (no shader files).
    target_link_options(FickerEngine PRIVATE
        "-sUSE_GLFW=3"
        "-sUSE_WEBGL2=1"
        "-sMIN_WEBGL_VERSION=2"
        "-sMAX_WEBGL_VERSION=2"
        "-sLEGACY_GL_EMULATION=1"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sASYNCIFY"
        "-sASSERTIONS=1"
    )

elseif(BUILD_WEB)
    message(FATAL_ERROR "BUILD_WEB=ON requires Emscripten. Use: emcmake cmake -S . -B webbuild -G Ninja")

else()
    # ----------------------------------------------------------------------------------
    # Native build (Linux/X11 + OpenGL)
    # ----------------------------------------------------------------------------------
    message(STATUS "Configuring native build (GLFW+OpenGL)")

    target_compile_definitions(FickerEngine PRIVATE FE_NATIVE=1)

    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)

    target_link_libraries(FickerEngine PRIVATE
        OpenGL::GL
        glfw
    )

    # If you still want the toggle, keep it, but GLFW already pulls X11 on Linux.
    if(BUILD_X11)
        # Optional: some setups still like explicit X11 discovery
        find_package(X11 REQUIRED)
        target_include_directories(FickerEngine PRIVATE ${X11_INCLUDE_DIR})
        target_link_libraries(FickerEngine PRIVATE ${X11_LIBRARIES})

        # Needed for glX* on some toolchains.
        if(TARGET OpenGL::GLX)
            target_link_libraries(FickerEngine PRIVATE OpenGL::GLX)
        endif()
    endif()
endif()
