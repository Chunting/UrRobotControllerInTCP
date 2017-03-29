
project(QtRosListener)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 貌似如果使用AUTOMOC，需要把Qt5Core这个引用进来。这个find_package是必须的。
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) # Fuck, 如果要使用autouic那么必须引用这个Qt5Widgets.

FILE(GLOB ${PROJECT_NAME}_files *.cpp *.h res.qrc)
get_property(UseROS GLOBAL PROPERTY BUILD_WITH_ROS)
if (${UseROS} STREQUAL "ON")
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_files})

    target_link_libraries(${PROJECT_NAME} cobotsys cobotsys_ros_wrapper)
endif ()