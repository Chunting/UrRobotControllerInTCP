# 这个软件包里的内容尽可能的少依赖库
# 可以依赖的库包括 boost, stl
# boost库里的内容，如果是C++11已经实现的部分，不要再次从boost里应用
# Created by 潘绪洋 itolfo@gmail.com/itolfo@126.com

# 在binpicking的第一个版本201703版本里，这里都不在做大的变化了
# 下一个版本里，会有一个巨大的变更。文件的组织形式需要做变化。

cmake_minimum_required(VERSION 3.6)

# 语言级别配置
set(CMAKE_CXX_STANDARD 11)

# 工程名
project(cobotsys_common)

# 依赖库
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)
find_package(OpenCV REQUIRED)

# 选项
option(BuildTestSample "Build test codes?" OFF)
option(EnableHighGUIShowInEasy "在easy_gui_show里调用cv::imshow函数" OFF)

if (EnableHighGUIShowInEasy)
    add_definitions(-DEASYGUISHOW_WITH_IMSHOW)
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 头文件路径
include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# 源代码文件
FILE(GLOB ${PROJECT_NAME}_files src/*.cpp include/*.h)

# 主要的代码库部分
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_files})
target_link_libraries(${PROJECT_NAME} cobotsys rt ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})


function(add_test_bin test_file_base)
    add_executable(${test_file_base} test/${test_file_base}.cpp)
    target_link_libraries(${test_file_base} ${PROJECT_NAME})
endfunction()


# test codes
if (BuildTestSample)
    add_test_bin(basic_test)
    add_test_bin(read_old_test)
    add_test_bin(ipc_easy_names)

    if (UseOpenCV)
        add_test_bin(ipc_mat_read_test)
        add_test_bin(ipc_mat_sema_read_test)
        add_test_bin(ipc_mat_write_test)
        add_test_bin(ipc_pub_mat)
        add_test_bin(ipc_rdr_pub)
    endif ()
endif ()

