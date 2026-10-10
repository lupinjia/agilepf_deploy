cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(agilepf_deploy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Select the matching vendored ONNX Runtime library for this host architecture.
set(ONNXRUNTIME_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/onnxruntime")
set(ONNXRUNTIME_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT ONNXRUNTIME_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE ONNXRUNTIME_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(ONNXRUNTIME_ARCH MATCHES "^(AMD64|amd64|x86_64)$")
    set(ONNXRUNTIME_ARCH x86_64)
elseif(ONNXRUNTIME_ARCH MATCHES "^(aarch64|arm64)$")
    set(ONNXRUNTIME_ARCH arm64)
else()
    message(FATAL_ERROR "Unsupported ONNX Runtime architecture: ${ONNXRUNTIME_ARCH}")
endif()
message(STATUS "ONNX Runtime architecture: ${ONNXRUNTIME_ARCH}")

set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}/libonnxruntime.so")
if(NOT EXISTS "${ONNXRUNTIME_LIBRARY}")
    message(FATAL_ERROR "ONNX Runtime library not found: ${ONNXRUNTIME_LIBRARY}")
endif()

option(PRINT_DEBUG "Print debug information" OFF)
if(PRINT_DEBUG)
    add_compile_definitions(PRINT_DEBUG)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/legged_sdk/lib/cmake")
find_package(legged_sdk REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    "${ONNXRUNTIME_DIR}/include/core/session"
    "${ONNXRUNTIME_DIR}/include/core/common"
    /opt/legged_sdk/include)

add_executable(agilepf_deploy
    src/main.cpp)

target_link_libraries(agilepf_deploy
    legged_sdk
    yaml-cpp
    "${ONNXRUNTIME_LIBRARY}")

include(CTest)
if(BUILD_TESTING)
    add_executable(onnx_policy_smoke tests/onnx_policy_smoke.cpp)
    target_compile_definitions(onnx_policy_smoke PRIVATE
        POLICY_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/params/policy_params.yaml")
    target_link_libraries(onnx_policy_smoke
        legged_sdk
        yaml-cpp
        "${ONNXRUNTIME_LIBRARY}")
    set_target_properties(onnx_policy_smoke PROPERTIES
        BUILD_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}")
    add_test(NAME onnx_policy_smoke COMMAND onnx_policy_smoke)
endif()

# Keep the vendored ONNX Runtime shared library discoverable when running.
set_target_properties(agilepf_deploy PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH "${ONNXRUNTIME_DIR}/lib/${ONNXRUNTIME_ARCH}"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
