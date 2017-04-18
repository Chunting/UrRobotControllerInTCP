
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB qwt_HDRS *.h)
file(GLOB qwt_SRCS *.cpp)

add_library(qwt SHARED ${qwt_SRCS} ${qwt_HDRS})
target_link_libraries(qwt
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Xml
        Qt5::Svg
        Qt5::OpenGL
        Qt5::Concurrent
        Qt5::PrintSupport)