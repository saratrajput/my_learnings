cmake_minimum_required(VERSION 3.0.0)

project(Calculator_Project VERSION 1.0.0)


add_subdirectory(my_math)
add_subdirectory(my_print)

add_executable(calculator
    main.cpp)

add_executable(duplicate_calculator
    main.cpp)

target_link_libraries(calculator PRIVATE my_math my_print)
target_link_libraries(duplicate_calculator PRIVATE my_math my_print)

# Cache Variables
set(A "123" CACHE STRING "This command sets variable A in persistent cache.")

message($CACHE{A})

# Modification of Cache Variables
set(Name Alice CACHE STRING "The name variable.")
# set(Name Bob CACHE STRING "The modified name variable." FORCE)
message($CACHE{Name})

# Available Cache Variables
message(CMAKE_VERSION=${CMAKE_VERSION})
message(CMAKE_MAJOR_VERSION=${CMAKE_MAJOR_VERSION})
message(CMAKE_MINOR_VERSION=${CMAKE_MINOR_VERSION})
message(CMAKE_PATCH_VERSION=${CMAKE_PATCH_VERSION})

message("Root level CMakeLists: CMAKE_PROJECT_NAME=${")