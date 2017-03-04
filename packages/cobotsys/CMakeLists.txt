cmake_minimum_required(VERSION 3.6)
# 默认编译标准
set(CMAKE_CXX_STANDARD 11)

# 编译器命令行参数
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")      # 为了生成可重定位的代码
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #

# 工程名
project(cobotsys)

# 头文件引用
include_directories(src)

# 源代码
file(GLOB ${PROJECT_NAME}_files src/*.cpp src/include/*.h)

# 主要的代码库部分
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})

# 依赖链接库
target_link_libraries(${PROJECT_NAME})

#
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)