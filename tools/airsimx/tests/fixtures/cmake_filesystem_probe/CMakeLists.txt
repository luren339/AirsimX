cmake_minimum_required(VERSION 3.5)
project(AirSimFilesystemLinkProbe LANGUAGES CXX)

if(NOT DEFINED AIRSIM_SOURCE_ROOT)
    message(FATAL_ERROR "AIRSIM_SOURCE_ROOT is required")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(AIRSIM_TEST_DEFAULT_CXX14)
    # Simulate the compiler default used by older Clang releases while the
    # project minimum remains CMake 3.5 (before CMP0067).
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()
include("${AIRSIM_SOURCE_ROOT}/cmake/cmake-modules/CXX17Filesystem.cmake")

airsim_detect_cxx17_filesystem_extra_library(AIRSIM_CXX17_FILESYSTEM_EXTRA_LIBRARY)
file(WRITE "${CMAKE_BINARY_DIR}/filesystem-extra-library.txt"
    "${AIRSIM_CXX17_FILESYSTEM_EXTRA_LIBRARY}")

add_executable(directory_iterator_probe directory_iterator_probe.cpp)
if(AIRSIM_CXX17_FILESYSTEM_EXTRA_LIBRARY)
    target_link_libraries(directory_iterator_probe
        ${AIRSIM_CXX17_FILESYSTEM_EXTRA_LIBRARY})
endif()
