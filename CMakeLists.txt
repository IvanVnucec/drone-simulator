cmake_minimum_required(VERSION 3.13)

project(drone-simulator)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -O0 -ggdb)

add_subdirectory(src)
target_include_directories(drone-simulator PRIVATE libs)
