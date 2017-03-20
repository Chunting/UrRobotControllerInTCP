


MESSAGE("")
#MESSAGE("----------Applications---------------------------------------")
MESSAGE("----------示例程序  ---------------------------------------")
FILE(GLOB children *)
FOREACH (child ${children})
    IF (IS_DIRECTORY ${child})
        if (EXISTS "${child}/CMakeLists.txt")
            get_filename_component(f_base ${child} NAME_WE)
            MESSAGE(STATUS "Sample     : " ${f_base})
            add_subdirectory(${child})
        endif ()
    ENDIF ()
ENDFOREACH ()