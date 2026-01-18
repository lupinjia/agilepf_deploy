cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(agilepf_deploy)

SET(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/home/lupinjia/libtorch/share/cmake/Torch")
list(APPEND CMAKE_PREFIX_PATH "/opt/legged_sdk/lib/cmake")
find_package(legged_sdk REQUIRED)
find_package(Torch REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include 
    ${TORCH_INCLUDE_DIRS}
    /opt/legged_sdk/include)

add_executable(agilepf_deploy
src/main.cpp)

target_link_libraries(agilepf_deploy
    legged_sdk 
    yaml-cpp
    ${TORCH_LIBRARIES})
