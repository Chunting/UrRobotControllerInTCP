cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")      # 为了生成可重定位的代码
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH})  # 这里定位并使用CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{QT5_INSTALL_PATH})   # Qt5


# 这里设置默认的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 一般信息打印
message("General cobotsys root config:")
message(STATUS "CMAKE PREFIX PATH: " ${CMAKE_PREFIX_PATH})
message(STATUS "DEBUG   Flags: ${CMAKE_CXX_FLAGS_DEBUG} ")
message(STATUS "RELEASE Flags: ${CMAKE_CXX_FLAGS_RELEASE} ")

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


#
#message(STATUS "Current Source Path: ${CMAKE_CURRENT_SOURCE_DIR}")
#
#function(scan_package_path path_to_scan)
#    message("")
#    message("PATH to Package Source: " ${path_to_scan})
#    sub_dir_list(PackagesList ${path_to_scan})
#
#    foreach (PackagePath ${PackagesList})
#        if (EXISTS "${path_to_scan}/${PackagePath}/CMakeLists.txt")
#            message(STATUS "PACKAGE: ${path_to_scan}/${PackagePath}")
#        endif ()
#    endforeach ()
#
#    foreach (PackagePath ${PackagesList})
#        if (EXISTS "${path_to_scan}/${PackagePath}/CMakeLists.txt")
#            add_subdirectory(${path_to_scan}/${PackagePath})
#        endif ()
#    endforeach ()
#endfunction()
#
#scan_package_path(${CMAKE_CURRENT_SOURCE_DIR}/packages)

function(build_package path_to_package)
    if (EXISTS "${path_to_package}/CMakeLists.txt")
        include_directories(${path_to_package}/src)
        include_directories(${path_to_package}/include)
        add_subdirectory(${path_to_package})
    endif ()
endfunction()

function(build_sample path_to_package)
    if (EXISTS "${path_to_package}/CMakeLists.txt")
        add_subdirectory(${path_to_package})
    endif ()
endfunction()

build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys)
build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys_qt)
build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys_qt_cv)
build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys_common)
build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys_ros_wrapper)

build_package(${CMAKE_SOURCE_DIR}/test_apps)

add_subdirectory(${CMAKE_SOURCE_DIR}/packages/xApplications)
add_subdirectory(${CMAKE_SOURCE_DIR}/samples)

message("")