cmake_minimum_required(VERSION 2.8)

project(Tutorial_create_robot)

# Enable Qt Support
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Network REQUIRED)


function(tutorial_app main_file)
    get_filename_component(f_base ${main_file} NAME_WE)
    add_executable(tutorial_${f_base} ${main_file})
    target_link_libraries(tutorial_${f_base} cobotsys)
endfunction()

tutorial_app(create_a_robot_mover.cpp)
tutorial_app(create_robot_and_show.cpp)

