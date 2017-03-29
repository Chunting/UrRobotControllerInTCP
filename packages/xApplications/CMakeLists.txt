

MESSAGE("")
MESSAGE("----------Applications------------------------------------------------")
FILE(GLOB children *)
FOREACH (child ${children})
    IF (IS_DIRECTORY ${child})
        if (EXISTS "${child}/CMakeLists.txt")
            get_filename_component(f_base ${child} NAME_WE)
            MESSAGE(STATUS "Application: " ${f_base})
            add_subdirectory(${child})
        endif ()
    ENDIF ()
ENDFOREACH ()