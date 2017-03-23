cmake_minimum_required(VERSION 2.8.3)

project(gl_tools)

find_package(OpenGL REQUIRED)

include_directories(include/GLTools)
include_directories(${OPENGL_INCLUDE_DIR})

## Declare a cpp library
FILE(GLOB theSourceFiles src/*.cpp include/GLTools/*.h)

add_library(gl_tools STATIC ${theSourceFiles})

target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})