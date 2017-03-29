

MESSAGE("")
MESSAGE("----------ROS EXTEND--------------------------------------------------")
FILE(GLOB children *)
FOREACH (child ${children})
    IF (IS_DIRECTORY ${child})
        if (EXISTS "${child}/CMakeLists.txt")
            get_filename_component(f_base ${child} NAME_WE)
            MESSAGE(STATUS "ROS Extend : " ${f_base})
            add_subdirectory(${child})
            set_property(GLOBAL PROPERTY ROS_INCLUDE ${child}/include APPEND)
        endif ()
    ENDIF ()
ENDFOREACH ()

# 在子目录的每个文件里需要增加如下
# set_property(GLOBAL PROPERTY ROS_EXTEND "${PROJECT_NAME}" APPEND)
#
#
