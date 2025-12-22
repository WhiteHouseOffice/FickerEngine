cmake_minimum_required(VERSION 3.16)
project(FickerEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE ENGINE_CORE   "engine/core/*.cpp")
file(GLOB_RECURSE ENGINE_GEOM   "engine/geom/*.cpp")
file(GLOB_RECURSE ENGINE_RENDER "engine/render/*.cpp")
file(GLOB_RECURSE ENGINE_GAME   "engine/game/*.cpp")

set(SOURCES
  Main.cpp
  ${ENGINE_CORE}
  ${ENGINE_GEOM}
  ${ENGINE_RENDER}
  ${ENGINE_GAME}
)
list(REMOVE_DUPLICATES SOURCES)

add_executable(FickerEngine ${SOURCES})

target_include_directories(FickerEngine PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/engine
)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

target_compile_definitions(FickerEngine PRIVATE FE_NATIVE=1)

target_link_libraries(FickerEngine PRIVATE
  OpenGL::GL
  glfw
)
