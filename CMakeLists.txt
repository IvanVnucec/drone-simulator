cmake_minimum_required(VERSION 3.13)

project(drone-simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O0")

add_subdirectory(src)
