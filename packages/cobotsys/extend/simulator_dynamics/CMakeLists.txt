cmake_minimum_required(VERSION 2.8.3)
project(simulator_dynamics)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include/SimulatorDynamics)

#message(STATUS "asdfsdfasf ${INCLUDE_DIRECTORIES}")

## Declare a cpp library
FILE(GLOB theSourceFiles src/*.cpp include/SimulatorDynamics/*.h)
add_library(simulator_dynamics ${theSourceFiles})

target_link_libraries(simulator_dynamics
        mathlib
        robotlib
        ode)



