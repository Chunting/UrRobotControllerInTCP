cmake_minimum_required(VERSION 2.8.3)
project(mathlib)


include_directories(include/MathLib)
FILE(GLOB theSourceFiles src/*.cpp include/MathLib/*.h)
add_library(mathlib STATIC ${theSourceFiles})
