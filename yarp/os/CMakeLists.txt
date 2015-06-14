cmake_minimum_required(VERSION 2.8.9)

find_package(YARP REQUIRED)

include_directories(${YARP_INCLUDE_DIRS})

add_executable(summer summer.cpp)
target_link_libraries(summer ${YARP_LIBRARIES})