project(ParameterTunner)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 貌似如果使用AUTOMOC，需要把Qt5Core这个引用进来。这个find_package是必须的。
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) # Fuck, 如果要使用autouic那么必须引用这个Qt5Widgets.

get_filename_component(proj_dir_name . NAME_WE)

file(GLOB src *.cpp *.h)

add_executable(${PROJECT_NAME} ${src} )

target_link_libraries(${PROJECT_NAME} cobotsys qwt)


