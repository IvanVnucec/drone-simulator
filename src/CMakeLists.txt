cmake_minimum_required(VERSION 3.10)

project(drone-simulator)

find_package(Chrono REQUIRED
             COMPONENTS Irrlicht
             OPTIONAL_COMPONENTS PardisoMKL
             CONFIG)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
 
if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

include_directories(${CHRONO_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
    add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions( "-DNOMINMAX" )                # do not use MSVC's min/max macros
    add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
    add_compile_options(/wd4251)                   # class needs to have dll-interface
endif()

set(SOURCE_FILES main.cpp)
add_executable(${CMAKE_PROJECT_NAME} ${SOURCE_FILES})


target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"") 
target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC ${CHRONO_CXX_FLAGS})
target_link_options(${CMAKE_PROJECT_NAME} PUBLIC ${CHRONO_LINKER_FLAGS})

target_link_libraries(${CMAKE_PROJECT_NAME} ${CHRONO_LIBRARIES})
target_link_libraries(${CMAKE_PROJECT_NAME} Eigen3::Eigen)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_DLL_copy_command("${CHRONO_DLLS}")
