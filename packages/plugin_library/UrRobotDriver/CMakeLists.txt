

project(UrRobotDriver)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 貌似如果使用AUTOMOC，需要把Qt5Core这个引用进来。这个find_package是必须的。
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) # Fuck, 如果要使用autouic那么必须引用这个Qt5Widgets.

include_directories(src)
include_directories(ur_adapter)

file(GLOB adapter ur_adapter/* ur_widget_utility/*)
set(${PROJECT_NAME}_files
        #        src/ur_ros_wrapper.cpp
        src/ur_driver.cpp
        src/ur_realtime_communication.cpp
        src/ur_communication.cpp
        src/robot_state.cpp
        src/robot_state_RT.cpp
        src/do_output.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files} ${adapter})

target_link_libraries(${PROJECT_NAME} cobotsys)
