cmake_minimum_required(VERSION 2.8.9)
project(Namer)

include_directories("${CMAKE_SOURCE_DIR}")
add_library(namer world.cpp world.h)