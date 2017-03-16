

project(UrRobotDriver)

include_directories(src)
include_directories(ur_adapter)

file(GLOB adapter ur_adapter/*)
set(${PROJECT_NAME}_files
        #        src/ur_ros_wrapper.cpp
        src/ur_driver.cpp
        src/ur_realtime_communication.cpp
        src/ur_communication.cpp
        src/robot_state.cpp
        src/robot_state_RT.cpp
        src/do_output.cpp
        ur_adapter/UrAdapter.cpp
        ur_adapter/UrRobotDriverFactory.cpp
        ur_adapter/UrStatusWatcher.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

target_link_libraries(${PROJECT_NAME} cobotsys)
