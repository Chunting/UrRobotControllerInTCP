

project(FotonicCameraAdapter)

set(OpenNI_INCLUDE_DIRS $ENV{OPENNI2_INCLUDE})
if (OpenNI_INCLUDE_DIRS)
    include_directories(${OpenNI_INCLUDE_DIRS})

    find_package(OpenCV REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})

    FILE(GLOB ${PROJECT_NAME}_files src/*)

    add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

    target_link_libraries(${PROJECT_NAME}
            cobotsys
            OpenNI2
            ${OpenCL_LIBRARY}
            ${OpenCV_LIBRARIES})

    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION plugins)
endif ()

