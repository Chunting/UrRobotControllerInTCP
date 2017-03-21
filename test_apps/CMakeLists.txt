cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")      # 为了生成可重定位的代码
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH})  # 这里定位并使用CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{QT5_INSTALL_PATH})   # Qt5

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 辅助宏，用于搜索对应目录所有子目录项
MACRO(sub_dir_list result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH (child ${children})
        IF (IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF ()
    ENDFOREACH ()
    SET(${result} ${dirlist})
ENDMACRO()

message("")
message("----------测试小程序---------------------------------------")
file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
set(cpplist "")
foreach (file ${children})
    set(file_path_full ${CMAKE_CURRENT_SOURCE_DIR}/${file})
    get_filename_component(f_extf ${file_path_full} EXT)
    get_filename_component(f_path ${file_path_full} ABSOLUTE)
    get_filename_component(f_base ${file_path_full} NAME_WE)

    if (${f_extf} STREQUAL ".cpp")
        message(STATUS "TEST APP   : " ${f_base})
        add_executable(${f_base} ${file_path_full})
        target_link_libraries(${f_base} cobotsys cobotsys_ros_wrapper)
    endif ()
endforeach ()