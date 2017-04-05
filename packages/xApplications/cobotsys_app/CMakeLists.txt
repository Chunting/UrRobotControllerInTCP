
cmake_minimum_required(VERSION 3.6)

project(cobotsys_app)

file(GLOB ${PROJECT_NAME}_files src/*)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_files})

target_link_libraries(${PROJECT_NAME} cobotsys)