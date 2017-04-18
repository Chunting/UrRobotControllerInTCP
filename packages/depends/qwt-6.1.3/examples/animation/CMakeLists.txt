
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

get_filename_component(proj_dir_name ${CMAKE_CURRENT_SOURCE_DIR} NAME_WE)

file(GLOB src *.cpp *.h)

add_executable(${proj_dir_name} ${src})

target_link_libraries(${proj_dir_name} qwt)