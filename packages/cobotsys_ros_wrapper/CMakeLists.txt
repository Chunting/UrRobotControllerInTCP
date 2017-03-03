
project(cobotsys_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        genmsg
        image_transport
        geometry_msgs
        cv_bridge
        message_generation
        actionlib
        control_msgs
        sensor_msgs
        tf
        trac_ik_lib
        trajectory_msgs)

# Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

#add_message_files(FILES algorithm_result.msg)

#generate_messages(DEPENDENCIES std_msgs)

#catkin_package(
#        INCLUDE_DIRS include
#        LIBRARIES cobot_ros_wrapper
#          CATKIN_DEPENDS other_catkin_pkg
#          DEPENDS system_lib
#)

include_directories(src)

FILE(GLOB ${PROJECT_NAME}_files src/* include/*.h src/*.h)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets ${catkin_LIBRARIES} cobotsys)

set_target_properties(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
