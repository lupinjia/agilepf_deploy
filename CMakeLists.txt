cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(agilepf_deploy)

SET(CMAKE_CXX_STANDARD 17)

# Set build options
# choose neural network inference backend
option(INFER_LIBTORCH "Use LibTorch for neural network inference" ON)
option(INFER_OPENVINO "Use OpenVINO for neural network inference" OFF)
if(INFER_OPENVINO AND INFER_LIBTORCH)
    message(FATAL_ERROR "Cannot enable both LIBTORCH and OPENVINO options at the same time.")
elseif(INFER_LIBTORCH)
    message(STATUS "Using LibTorch for neural network inference.")
    add_compile_definitions(INFER_LIBTORCH)
elseif(INFER_OPENVINO)
    message(STATUS "Using OpenVINO for neural network inference.")
    add_compile_definitions(INFER_OPENVINO)
endif()
# whether to print debug information
option(PRINT_DEBUG "Print debug information" OFF)
if(PRINT_DEBUG)
    add_compile_definitions(PRINT_DEBUG)
endif()

if(INFER_LIBTORCH)
    list(APPEND CMAKE_PREFIX_PATH "/home/lupinjia/libtorch/share/cmake/Torch")
    find_package(Torch REQUIRED)
    set(NN_INF_INCS ${TORCH_INCLUDE_DIRS})
    set(NN_INF_LIBS ${TORCH_LIBRARIES})
elseif(INFER_OPENVINO)
    # make sure you have initialized OpenVINO through `source /opt/intel/openvino_2025/setupvars.sh`
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
    set(NN_INF_LIBS openvino::runtime)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/legged_sdk/lib/cmake")
find_package(legged_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include 
    ${NN_INF_INCS}
    /opt/legged_sdk/include)

add_executable(agilepf_deploy
src/main.cpp)

target_link_libraries(agilepf_deploy
    legged_sdk 
    yaml-cpp
    ${NN_INF_LIBS}
    ${OpenCV_LIBS}
    )
