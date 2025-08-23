cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 11)

# googletest
include(FetchContent)
FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG release-1.11.0)
FetchContent_MakeAvailable(googletest)

add_library(common_library library_code.cpp)

add_executable(main_app main.cpp)
target_link_libraries(main_app common_library)

add_executable(unit_test_runner test_runner.cpp)
target_link_libraries(unit_test_runner common_library gtest_main gmock_main pthread)