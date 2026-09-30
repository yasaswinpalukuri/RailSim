cmake_minimum_required(VERSION 3.16)
project(RailSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Writes build/compile_commands.json, which clang-tidy needs.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(RAILSIM_BUILD_TESTS "Build the GoogleTest test suite" ON)
option(RAILSIM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# Interface target that carries the warning flags. Our own targets link to it;
# third-party code (GoogleTest) does not, so its warnings cannot break our build.
add_library(railsim_warnings INTERFACE)
if(NOT MSVC)
    target_compile_options(railsim_warnings INTERFACE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    if(RAILSIM_WARNINGS_AS_ERRORS)
        target_compile_options(railsim_warnings INTERFACE -Werror)
    endif()
endif()

add_subdirectory(src)

if(RAILSIM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
