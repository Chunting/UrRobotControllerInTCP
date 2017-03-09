cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)

# 编译器命令行参数
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")      # 为了生成可重定位的代码
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #

# 工程名
project(cobotsys_qt_cv)

# Qt编译扩展
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# 依赖Qt的库
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(OpenCV REQUIRED)

# 头文件引用
include_directories(src)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(framework)

# 源代码
file(GLOB ${PROJECT_NAME}_files src/*.cpp framework/*)

# 主要的代码库部分
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

# 依赖链接库
target_link_libraries(${PROJECT_NAME}
        cobotsys
        cobotsys_qt
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::Network
        ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)