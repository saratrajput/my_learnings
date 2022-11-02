cmake_minimum_required(VERSION 3.0.0)

set(Name Charlie)

message("Root directory: Before adding subdirectory: ${Name}")
add_subdirectory(subfolder)
message("Root directory: After adding subdirectory: ${Name}")
