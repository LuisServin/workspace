# YARP needs CMake 2.8.9 or greater
cmake_minimum_required(VERSION 2.8.9)

# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})

# set up our program 
add_executable(hello hello.cpp)

# link with YARP libraries
target_link_libraries(hello ${YARP_LIBRARIES})