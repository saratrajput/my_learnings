cmake_minimum_required(VERSION 3.0.0)

set(VAR OFF)
set (VAR2 VAR)

# If-else
if(${VAR2})
    message("If block executed.")
else()
    message("Else block executed.")
endif()

# Unary tests
set(Name Alice)

# Unary tests DEFINED
if(DEFINED Name)
    message("Name: if block executed.")
else()
    message("Name: else block executed.")
endif()

if(DEFINED Age)
    message("Age: if block executed.")
else()
    message("Age: else block executed.")
endif()

# Unary tests COMMAND
if(COMMAND target_link_library)
    message("target_link_library is a command.")
else()
    message("target_link_library is NOT a command.")
endif()

if(COMMAND target_link_libraries)
    message("target_link_libraries is a command.")
else()
    message("target_link_libraries is NOT a command.")
endif()

# Unary tests EXISTS
if(EXISTS /home/sp/dev_root/my_learnings/cmake_tutorials/module5/CMakeLists.txt)
    message("Given file exists.")
else()
    message("File not found.")
endif()

# Binary tests
set(Name1 Charlie)
set(Name2 Charlies)

if(Name1 STRLESS Name2)
    message("${Name1} is less than ${Name2}.")
elseif(Name1 STRGREATER Name2)
    message("${Name1} is greater than ${Name2}.")
elseif(Name1 STREQUAL Name2)
    message("${Name1} is equal to ${Name2}.")
endif()

# While loop
set (VAR a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
# set (VAR ${VAR}a)
while(NOT VAR STREQUAL "aaaaaaaaaa")
    set(VAR ${VAR}a)
    message(${VAR})
endwhile()

# Foreach loop
## Variant 1
foreach(Name Alice Bob Charlie)
    message(${Name})
endforeach()

## Variant 2
foreach(x RANGE 100 105)
    message("Person_${x}")
endforeach()

## Variant 3
set(VAR1 0;1)
set(VAR2 2 3)
set(VAR3 "4 5")

foreach(x IN LISTS VAR1 VAR2 VAR3)
    message("x = ${x}")
endforeach()

# Function
function(print_detail name_var)
    message("1. My name is ${${name_var}}.")

endfunction()

function(print_detail name_var)
    message("2. My name is ${${name_var}}.")

    if(DEFINED ARGV1)
        message("Hello, my name is ${ARGV1}.")
    endif()

    message("ARGC= ${ARGC}")
    message("ARGV= ${ARGV}")
    message("ARGN= ${ARGN}")

    if(DEFINED ARGV0)
        message("ARGV0= ${ARGV0}")
    endif()

    if(DEFINED ARGV1)
        message("ARGV1= ${ARGV1}")
    endif()

    if(DEFINED ARGV2)
        message("ARGV2= ${ARGV2}")
    endif()

    if(DEFINED ARGV3)
        message("ARGV3= ${ARGV3}")
    endif()
endfunction()

set(Name Charlie)
# print_detail(${Name})
print_detail(Name)

_print_detail(Name)

# Optional arguments
print_detail(Name Bob)
print_detail(Name)
print_detail(Name Bob Alice)

# Scopes
set(Name Charlie)

function(print_detail_scope name_var)
    message("Inside function: Before modification: ${Name}.")
    set(Name Bob PARENT_SCOPE)
    message("Inside function: After modification: ${Name}.")
endfunction()

message("Outside function: before function call: ${Name}.")
print_detail_scope(Name)
message("Outside function: after function call: ${Name}.")

# Macros
macro(print_detail_macro name_var)
    message("My name is ${name_var}.")

    if(DEFINED name_var)
        message("If block executed.")
    endif()
    # set(name_var abc)
    # message("My name is ${name_var}.")
endmacro()

print_detail_macro(Charlie)

set(Name CMake)

macro(change_name1 temp_name)
    if(DEFINED ${temp_name})
        message("1: If block executed.")
    endif()
endmacro()

macro(change_name2 temp_name)
    if(DEFINED ${temp_name})
        message("2: If block executed.")
    endif()
endmacro()

change_name1(Name)
change_name2(CMake)

function(set_name temp_name)
    set(Name ${temp_name} PARENT_SCOPE)
endfunction()

set_name(CMake)

message(${Name})

set(A B)
set(B C)
set(C D)
set(D E)
set(E F)
set(F G)
set(G H)

function(print_char temp)
    message(${${${temp}}})
endfunction()

print_char(${${${A}}})
