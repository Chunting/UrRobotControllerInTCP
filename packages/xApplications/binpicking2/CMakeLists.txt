
project(binpicking2)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 貌似如果使用AUTOMOC，需要把Qt5Core这个引用进来。这个find_package是必须的。
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) # Fuck, 如果要使用autouic那么必须引用这个Qt5Widgets.

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

FILE(GLOB binpicking2_files *.cpp *.h res.qrc)

add_executable(${PROJECT_NAME} ${binpicking2_files})
target_link_libraries(${PROJECT_NAME} cobotsys cobotsys_common cobotsys_qt cobotsys_qt_cv)