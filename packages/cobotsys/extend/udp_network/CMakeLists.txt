cmake_minimum_required(VERSION 2.8.3)
project(udp_network)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/UDPNetwork)


FILE(GLOB theSourceFiles src/*.cpp include/UDPNetwork/*.h)
add_library(udp_network STATIC ${theSourceFiles})
target_link_libraries(udp_network std_tools)