cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)

# 编译器命令行参数
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")      # 为了生成可重定位的代码
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #

# 工程名
project(cobotsys)

# Qt编译扩展
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# Install depend
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;$ENV{QT5_INSTALL_PATH}/lib")

# 依赖Qt的库
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)

# 头文件引用
include_directories(framework/include)
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# 源代码
file(GLOB ${PROJECT_NAME}_files src/* include/* framework/src/* framework/include/*)

# 主要的代码库部分
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

# 依赖链接库
target_link_libraries(${PROJECT_NAME}
        rt
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
        Qt5::Network
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/cobotsys FILES_MATCHING PATTERN "*.h")
install(DIRECTORY framework/ DESTINATION include/cobotsys FILES_MATCHING PATTERN "*.h")