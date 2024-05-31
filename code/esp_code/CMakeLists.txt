# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_component("components/Tasks")
idf_build_component("components/driver_tof")
idf_build_component("components/mpu9250")
idf_build_component("components/motor_driver")
idf_build_component("components/ahrs")
idf_build_component("components/control_algos")

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}../esp-idf-lib/components)
project(MicroMouse)

