cmake_minimum_required(VERSION 3.13)

project(drone-simulator)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -O0 -ggdb)

add_subdirectory(src)
target_include_directories(drone-simulator PRIVATE libs)

target_link_libraries(drone-simulator ${CONAN_LIBS})

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MYPROJECT_BUILD_TESTING) AND BUILD_TESTING)
    add_subdirectory(tests)
endif()
