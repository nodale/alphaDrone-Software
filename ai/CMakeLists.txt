#TODO split into multiple files
cmake_minimum_required(VERSION 3.22)
project(drone LANGUAGES CXX CUDA)

set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_STANDARD 23)
add_compile_options(-fcoroutines)
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

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
file(GLOB_RECURSE HEADERS
    ${CMAKE_SOURCE_DIR}/include/*.h)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_executable(main ${CMAKE_SOURCE_DIR}/src/main.cc ${CMAKE_SOURCE_DIR}/src/realsenseloader.cc ${HEADERS})

target_link_libraries(main PUBLIC nvblox_lib realsense2 Eigen3::Eigen)




