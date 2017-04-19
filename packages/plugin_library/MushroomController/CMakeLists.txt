cmake_minimum_required(VERSION 2.8)

project(MushroomController)

# Enable Qt Support
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Network REQUIRED)

file(GLOB ${PROJECT_NAME}_files *.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

target_link_libraries(${PROJECT_NAME} cobotsys)

