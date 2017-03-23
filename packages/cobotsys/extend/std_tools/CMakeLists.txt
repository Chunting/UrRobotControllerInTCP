cmake_minimum_required(VERSION 2.8.3)
project(std_tools)


include_directories(include/StdTools)
#-Wno-deprecated
## Declare a cpp library
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated")      # 为了生成可重定位的代码
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")  #
FILE(GLOB theSourceFiles src/*.cpp include/StdTools/*.h)
add_library(std_tools STATIC ${theSourceFiles})

target_link_libraries(std_tools ncurses)