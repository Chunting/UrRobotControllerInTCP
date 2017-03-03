


MESSAGE("-------------------------------------------------")
FILE(GLOB children *)
FOREACH (child ${children})
    IF (IS_DIRECTORY ${child})
        if (EXISTS "${child}/CMakeLists.txt")
            MESSAGE(STATUS "Sample: " ${child})
            add_subdirectory(${child})
        endif ()
    ENDIF ()
ENDFOREACH ()