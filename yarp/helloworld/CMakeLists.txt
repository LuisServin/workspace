cmake_minimum_required(VERSION 2.8.9)
project(helloworld)

include_directories("${CMAKE_SOURCE_DIR}/namer")

add_library(namer namer/world.cpp namer/world.h)

add_executable(helloworld main.cpp)
# target_link_libraries says the program "hello" needs
# library "namer"
target_link_libraries(helloworld namer)