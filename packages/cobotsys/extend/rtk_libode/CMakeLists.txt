cmake_minimum_required(VERSION 2.8.3)
project(rtk_libode)

###########
## Build ##
###########

add_custom_target(rtk_libode_setup_dummy_target ALL)
add_custom_command(TARGET rtk_libode_setup_dummy_target PRE_BUILD COMMAND "mkdir" "-p" "${PROJECT_BINARY_DIR}/scripts")
add_custom_command(TARGET rtk_libode_setup_dummy_target PRE_BUILD COMMAND "cp" "-f" "${PROJECT_SOURCE_DIR}/scripts/*" "${PROJECT_BINARY_DIR}/scripts")
add_custom_command(TARGET rtk_libode_setup_dummy_target PRE_BUILD COMMAND "python" "${PROJECT_SOURCE_DIR}/scripts/rtk_libode_setup.py")


add_library(ode SHARED IMPORTED)
set_property(TARGET ode PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libode.so)
set_target_properties(ode PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(ode rtk_libode_setup_dummy_target)
message(STATUS "libode installed: " ${PROJECT_SOURCE_DIR}/lib/libode.so)
