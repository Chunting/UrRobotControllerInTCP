

project(Kinect2CameraAdapter)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 貌似如果使用AUTOMOC，需要把Qt5Core这个引用进来。这个find_package是必须的。
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) # Fuck, 如果要使用autouic那么必须引用这个Qt5Widgets.

find_package(freenect2 QUIET)
if (freenect2_FOUND)
    include_directories(${freenect2_INCLUDE_DIRS})

    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    FILE(GLOB ${PROJECT_NAME}_files src/*)

    add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

    target_link_libraries(${PROJECT_NAME}
            cobotsys
            ${freenect2_LIBRARY}
            ${OpenCL_LIBRARY}
            ${OpenCV_LIBRARIES})
endif ()

