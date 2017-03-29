cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)

# 编译器命令行参数
if (WIN32)
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")      # 为了生成可重定位的代码
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #
endif ()


# 工程名
project(cobotsys)

# Qt编译扩展
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Install depend
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;$ENV{QT5_INSTALL_PATH}/lib")

# 依赖Qt的库
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)

# 头文件引用
include_directories(framework/include)
include_directories(include)

# 用于外部扩展头文件引用#include <xxx>
global_include_append(${OpenCV_INCLUDE_DIRS})
global_include_append(${Boost_INCLUDE_DIRS})

# 内部扩展的代码，从其他地方集成过来的代码
set(extend_library_list "")
set(inc_list "")
function(add_extend_library path_to_package)
    if (EXISTS ${path_to_package}/CMakeLists.txt)
        include_directories(${path_to_package}/include)
        get_filename_component(f_base ${path_to_package} NAME_WE)
        MESSAGE(STATUS "ExtendLib   : " ${f_base})
        set(extend_library_list ${extend_library_list} ${f_base} PARENT_SCOPE)
        set(inc_list ${inc_list} ${path_to_package}/include PARENT_SCOPE)
        add_subdirectory(${path_to_package})
    endif ()
endfunction()
message("")
#message("${inc_list}")
#message("")
#set(ext_inc ${ext_inc} ${inc_list} PARENT_SCOPE)

#foreach (inc ${INCLUDE_DIRECTORIES})
#    message(STATUS "Include: " ${inc})
#endforeach ()
#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#    message(STATUS "dir='${dir}'")
#endforeach()
#message("")

# 源代码
file(GLOB ${PROJECT_NAME}_files src/* include/* framework/src/* framework/include/*)

# 主要的代码库部分
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

# 依赖链接库
target_link_libraries(${PROJECT_NAME}
        rt
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::Network
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES}
        ${extend_library_list})

#get_property(ros_ext_lib GLOBAL PROPERTY ROS_EXTEND)
#add_custom_command(TARGET ${PROJECT_NAME}
#        PRE_BUILD
#        COMMAND python ../../data/scripts/gen_ros_extend.py "${ros_ext_lib}"
#        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# 安装
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/cobotsys FILES_MATCHING PATTERN "*.h")
install(DIRECTORY framework/ DESTINATION include/cobotsys FILES_MATCHING PATTERN "*.h")