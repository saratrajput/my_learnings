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
