cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)
option(BUILD_DOC "Build Document")

if (WIN32)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /source-charset:utf-8")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /source-charset:utf-8")
else ()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif ()

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{CMAKE_PREFIX_PATH})  # 这里定位并使用CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{QT5_INSTALL_PATH})   # Qt5
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_SOURCE_DIR}/cmake/Modules/")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# install rpath
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;$ENV{QT5_INSTALL_PATH}/lib")

# 这里设置默认的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 一般信息打印
message("General cobotsys root config:")
message(STATUS "CMAKE PREFIX PATH: " "${CMAKE_PREFIX_PATH}")
message(STATUS "DEBUG   Flags: ${CMAKE_CXX_FLAGS_DEBUG} ")
message(STATUS "RELEASE Flags: ${CMAKE_CXX_FLAGS_RELEASE} ")

set(ext_inc "")

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

# 文档生成
find_package(Doxygen QUIET)
if (BUILD_DOC)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_SOURCE_DIR}/docs/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target(doc_doxygen ALL
                COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                COMMENT "Generating API documentation with Doxygen"
                VERBATIM)
    else ()
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif ()
else ()
    message(STATUS "Document Generation ------------------------------------OFF")
endif ()


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

function(global_include_append include_path)
    include_directories(${include_path})
    set_property(GLOBAL PROPERTY CORE_INC ${include_path} APPEND)
endfunction()

function(search_inc path_dir)
    file(GLOB children ${path_dir}/*)
    foreach (child ${children})
        if (IS_DIRECTORY ${child})
            get_filename_component(f_base ${child} NAME_WE)
            if (${f_base} STREQUAL "include")
                include_directories(${child})
                message(STATUS "Include: " ${child})
                global_include_append(${child})
            else ()
                search_inc(${child})
            endif ()
        endif ()
    endforeach ()
endfunction()

function(build_package path_to_package)
    if (EXISTS "${path_to_package}/CMakeLists.txt")
        search_inc(${path_to_package})
        add_subdirectory(${path_to_package})
    endif ()
endfunction()

function(build_sample path_to_package)
    if (EXISTS "${path_to_package}/CMakeLists.txt")
        add_subdirectory(${path_to_package})
    endif ()
endfunction()

# 这里，后面的package是依赖前面的package的。
build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys)
build_package(${CMAKE_SOURCE_DIR}/packages/cobotsys_ros_wrapper)

# include core，有些头文件是在sub_directory里直接include_directories
# 这样，后面的其他工程如果引用了头文件，会导致不能编译通过。
# 所以，这里再次把那些头文件再次的引用
message("")
message("----------COBOTSYS Include--------------------------------------------")
get_property(core_inc_dirs GLOBAL PROPERTY CORE_INC)
foreach (inc ${core_inc_dirs})
    message(STATUS "Include: " ${inc})
    include_directories(${inc})
endforeach ()

# Plugin
add_subdirectory(${CMAKE_SOURCE_DIR}/packages/plugin_library)

# ROS extend
#set_property(GLOBAL PROPERTY ROS_EXTEND "")
add_subdirectory(${CMAKE_SOURCE_DIR}/packages/cobotsys_ros_extend)
get_property(ros_ext_lib GLOBAL PROPERTY ROS_EXTEND)
#message(STATUS "extlibs: ${ros_ext_lib}")

get_property(inc_dirs GLOBAL PROPERTY ROS_INCLUDE)
foreach (inc ${inc_dirs})
    message(STATUS "Include: " ${inc})
    include_directories(${inc})
endforeach ()

# Test
build_package(${CMAKE_SOURCE_DIR}/test_apps)

# App & Sample
add_subdirectory(${CMAKE_SOURCE_DIR}/packages/xApplications)
add_subdirectory(${CMAKE_SOURCE_DIR}/samples)

message("")

include(InstallRequiredSystemLibraries)
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "cobotsys SDK")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "0")
SET(CPACK_PACKAGE_VERSION_PATCH "1")
#SET(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "pan@cobotsys.com")
include(CPack)

