project(Math_Project VERSION 1.0.0)

add_library(my_math
    src/addition.cpp
    src/division.cpp)

target_include_directories(my_math PUBLIC include)

message("Subdirectory level CMakeLists: CMAKE_PROJECT_NAME=${CMAKE_PROJECT_NAME}")
message("Subdirectory level CMakeLists: PROJECT_NAME=${PROJECT_NAME}")
