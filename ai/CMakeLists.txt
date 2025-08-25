cmake_minimum_required(VERSION 3.22)
project(drone LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "Minimum CMake policy version")
set(BUILD_WITH_CUDA ON CACHE BOOL "Enable CUDA build")
set(BUILD_GRAPHICAL_EXAMPLES OFF CACHE BOOL "Disable graphical examples")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable standard examples")
set(CMAKE_CUDA_COMPILER "/opt/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler")
set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "CUDA architectures")
set(BUILD_PYTORCH_WRAPPER OFF CACHE BOOL "Disable PyTorch wrapper")
set(BUILD_TESTING OFF CACHE BOOL "Disable testing")
set(USE_SYSTEM_GFLAGS ON CACHE BOOL "Use system gflags")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

include(realsense)
include(nvblox)

add_executable(main ${CMAKE_SOURCE_DIR}/src/main.cc)

target_link_libraries(main PRIVATE nvblox_lib realsense2)




