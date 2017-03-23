cmake_minimum_required(VERSION 2.8.3)
project(robotlib)


include_directories(include/RobotLib)


FILE(GLOB theSourceFiles src/*.cpp include/RobotLib/*.h)

add_library(robotlib STATIC ${theSourceFiles})

target_link_libraries(robotlib dl std_tools)
