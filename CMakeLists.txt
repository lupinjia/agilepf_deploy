cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(agilepf_deploy)

SET(CMAKE_CXX_STANDARD 17)

# set build options
option(LIBTORCH "Use LibTorch for neural network inference" ON)
option(PRINT_DEBUG "Print debug information" OFF)
if(PRINT_DEBUG)
    add_compile_definitions(PRINT_DEBUG)
endif()

if(LIBTORCH)
    list(APPEND CMAKE_PREFIX_PATH "/home/lupinjia/libtorch/share/cmake/Torch")
    find_package(Torch REQUIRED)
    set(NN_INF_INCS ${TORCH_INCLUDE_DIRS})
    set(NN_INF_LIBS ${TORCH_LIBRARIES})
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
