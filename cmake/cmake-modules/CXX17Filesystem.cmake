include(CheckCXXSourceCompiles)

# Choose the filesystem library from an actual C++17 compile/link probe.
# Older libc++ releases may keep filesystem in a separate libc++fs library.
function(airsim_detect_cxx17_filesystem_extra_library output_variable)
    set(_airsim_filesystem_probe [=[
#include <filesystem>
#include <system_error>

#if __cplusplus < 201703L
#error "filesystem link probe must compile as C++17"
#endif

int main()
{
    std::error_code error;
    std::filesystem::directory_iterator entry(std::filesystem::path("."), error);
    const std::filesystem::directory_iterator end;
    for (; entry != end && !error; entry.increment(error)) {
        (void)entry->path();
    }
    return error ? 1 : 0;
}
]=])

    set(_airsim_saved_required_flags "${CMAKE_REQUIRED_FLAGS}")
    set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -std=c++17")
    set(_airsim_saved_required_libraries "${CMAKE_REQUIRED_LIBRARIES}")
    if(CMAKE_CXX_STANDARD_LIBRARIES)
        separate_arguments(
            _airsim_standard_libraries UNIX_COMMAND
            "${CMAKE_CXX_STANDARD_LIBRARIES}")
        list(APPEND CMAKE_REQUIRED_LIBRARIES ${_airsim_standard_libraries})
    endif()

    check_cxx_source_compiles(
        "${_airsim_filesystem_probe}"
        AIRSIM_CXX17_FILESYSTEM_DIRECTORY_ITERATOR_LINKS_WITHOUT_EXTRA_LIB)

    if(AIRSIM_CXX17_FILESYSTEM_DIRECTORY_ITERATOR_LINKS_WITHOUT_EXTRA_LIB)
        set(CMAKE_REQUIRED_FLAGS "${_airsim_saved_required_flags}")
        set(CMAKE_REQUIRED_LIBRARIES "${_airsim_saved_required_libraries}")
        set(${output_variable} "" PARENT_SCOPE)
        return()
    endif()

    set(CMAKE_REQUIRED_LIBRARIES
        c++fs ${CMAKE_REQUIRED_LIBRARIES})
    check_cxx_source_compiles(
        "${_airsim_filesystem_probe}"
        AIRSIM_CXX17_FILESYSTEM_DIRECTORY_ITERATOR_LINKS_WITH_CXXFS)
    set(CMAKE_REQUIRED_FLAGS "${_airsim_saved_required_flags}")
    set(CMAKE_REQUIRED_LIBRARIES "${_airsim_saved_required_libraries}")

    if(NOT AIRSIM_CXX17_FILESYSTEM_DIRECTORY_ITERATOR_LINKS_WITH_CXXFS)
        message(FATAL_ERROR
            "C++17 std::filesystem::directory_iterator cannot be linked "
            "with libc++ alone or with the separate c++fs library")
    endif()

    set(${output_variable} "c++fs" PARENT_SCOPE)
endfunction()
