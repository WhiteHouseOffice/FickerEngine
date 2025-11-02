cmake_minimum_required(VERSION 3.20)
project(FickerEngine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
file(GLOB_RECURSE ENGINE_SOURCES
    engine/core/*.cpp
    engine/platform/*.cpp
)

add_executable(FickerEngine ${ENGINE_SOURCES})

# Include directories
target_include_directories(FickerEngine PRIVATE
    ${CMAKE_SOURCE_DIR}/engine
    ${CMAKE_SOURCE_DIR}/engine/core
    ${CMAKE_SOURCE_DIR}/engine/platform
)

# Web (Emscripten) vs native builds
if (EMSCRIPTEN)
    message(STATUS "Configuring for Emscripten (WebAssembly)")
    set(CMAKE_EXECUTABLE_SUFFIX ".html")

    target_compile_definitions(FickerEngine PRIVATE PLATFORM_WEB=1)
    target_link_options(FickerEngine PRIVATE
        "-sUSE_WEBGL2=1"
        "-sFULL_ES3=1"
        "-sALLOW_MEMORY_GROWTH=1"
        "-sMAX_WEBGL_VERSION=2"
        "-sMIN_WEBGL_VERSION=2"
        "-sASYNCIFY"
        "--shell-file" "${CMAKE_SOURCE_DIR}/engine/platform/web/shell.html"
    )

else()
    message(STATUS "Configuring for native build")
    target_compile_definitions(FickerEngine PRIVATE PLATFORM_NATIVE=1)
    find_package(OpenGL REQUIRED)
    target_link_libraries(FickerEngine PRIVATE ${OPENGL_LIBRARIES})
endif()
