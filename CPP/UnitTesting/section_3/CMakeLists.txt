cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 11)

find_package(GTest REQUIRED)
message("GTest_INCLUDE_DIRS = ${GTest_INCLUDE_DIRS}")

add_library(common_library library_code.cpp)

add_executable(main_app main.cpp)
target_link_libraries(main_app common_library)

add_executable(unit_test_runner test_runner.cpp)
target_link_libraries(unit_test_runner common_library $(GTEST_LIBRARIES) pthread)
