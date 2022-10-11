cmake_minimum_required(VERSION 3.0.0)

message("Hello World")

set(NAME BOB Smith)
set(HEIGHT 190)

message("Hello, my name is ${NAME}, my height is ${HEIGHT}cm and my age is ${AGE} years.")

# Manipulating Variables
set(NAME2 Alice)

set(Alice Bob)

set(NAME2AliceBob Charlie)

message(${NAME2})

message(${${NAME2}})

message(NAME2 ${NAME2} ${${NAME2}})

message(NAME2${NAME2}${${NAME2}})

message("NAME2 ${NAME2} ${${NAME2}}")

message(${NAME2${NAME2}${${NAME2}}})

# Lists and Strings
set(VAR a b c;d "e;f" 2.7 "Hello There")
message (${VAR})

list(APPEND VAR 1.6 XX)
message (${VAR})

list(REMOVE_AT VAR 2 -3)
message (${VAR})

list(REMOVE_ITEM VAR a 2.7)
message(${VAR})

list(INSERT VAR 2 XX 2.7)
message(${VAR})

list(REMOVE_DUPLICATES VAR)
message(${VAR})

list(SORT VAR)
message(${VAR})

list(LENGTH VAR len_var)
list(GET VAR 2 5 6 sub_list)
list(SUBLIST VAR 2 3 sub_list2)
list(JOIN VAR ++ str_list)
list(FIND VAR XX find_var)

message("len_var: ${len_var}")
message("sub_list: ${sub_list}")
message("sub_list2: ${sub_list2}")
message("str_list: ${str_list}")
message("find_var: ${find_var}")
