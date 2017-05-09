#

cmake_minimum_required(VERSION 2.8)
project(qwt)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Xml REQUIRED)
find_package(Qt5Svg REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Concurrent REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

global_include_append(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_subdirectory(src)
#add_subdirectory(examples)
#add_subdirectory(textengines)
#建议将例子注释，谁要看，谁自己将其打开。Sail
#MESSAGE("")
#MESSAGE("----------Qwt Samples-------------------------------------------------")
#FILE(GLOB children examples/*)
#FOREACH (child ${children})
#    IF (IS_DIRECTORY ${child})
#        if (EXISTS "${child}/CMakeLists.txt")
#            get_filename_component(f_base ${child} NAME_WE)
#            MESSAGE(STATUS "Qwt Samples: " ${f_base})
#            add_subdirectory(${child})
#        endif ()
#    ENDIF ()
#ENDFOREACH ()
#MESSAGE("")