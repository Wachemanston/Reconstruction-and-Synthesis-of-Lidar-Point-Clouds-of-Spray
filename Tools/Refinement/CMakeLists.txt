# cmake version to be used
cmake_minimum_required(VERSION 3.0)

# project name
project(dummy_proj)

set (CMAKE_CXX_STANDARD 17)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# target
add_executable(
    main 
    # List all the source files directly as suggested by 
    # https://stackoverflow.com/questions/41495301/c-compilation-error-multiple-definition-of-main-but-only-1-main-function-in
    ./src/main.cpp 
    ./src/CustomPointType.cpp
    ./src/FloatSerialization.cpp
    ./src/Command.cpp
    )

target_link_libraries(main ${PCL_LIBRARIES} "stdc++fs")
