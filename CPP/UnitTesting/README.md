# C++ Unit Testing

### Notes
* ```add_library``` command:
    * STATIC: ```add_library(sumLibrary STATIC library_code.cpp)```
        * The library code will get copied into the executable when you are building.
    * SHARED: ```add_library(sumLibrary STATIC library_code.cpp)```
        * The library code will not get copied into the executable when you are building, but that will mean that you need the library available at runtime.

* ```FetchContent_Declare()```
    * First parameter doesn't have to correspond to the actual name of the dependency. For example, either ```googletest``` or ```gtest``` is fine.
