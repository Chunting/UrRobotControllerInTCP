cmake_minimum_required(VERSION 2.8.3)
project(robot_simulator)


find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

FILE(GLOB theSourceFiles src/*.cpp)
add_executable(robot_simulator ${theSourceFiles})

message(STATUS "SourceFile: ${theSourceFiles}")
target_link_libraries(
        robot_simulator
        mathlib
        freeimage
        std_tools
        gl_tools
        udp_network
        robotlib
        robot_gui
        simulator_dynamics
        BulletDynamics
        BulletCollision
        BulletSoftBody
        LinearMath
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
)

