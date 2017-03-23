cmake_minimum_required(VERSION 2.8.3)
project(robot_gui)


# Qt编译扩展
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/RobotGUI)

## Declare a cpp library
FILE(GLOB theSourceFiles src/*.cpp include/RobotGUI/*.h)
message(STATUS "Moc file: ${MOCED_SOURCES}")
add_library(robot_gui ${theSourceFiles} ${MOCED_SOURCES})


## Specify libraries to link a library or executable target against
target_link_libraries(robot_gui
        freeimage
        ${OPENGL_LIBRARIES}
        gl_tools
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        )