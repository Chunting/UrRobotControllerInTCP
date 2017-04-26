project(UrMover)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED) # 必须引用这个对象
find_package(Qt5Network REQUIRED)
find_package(Qt5WebSockets REQUIRED)

file(GLOB ${PROJECT_NAME}_files src/*)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

target_link_libraries(${PROJECT_NAME} cobotsys)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION plugins RUNTIME DESTINATION plugins)