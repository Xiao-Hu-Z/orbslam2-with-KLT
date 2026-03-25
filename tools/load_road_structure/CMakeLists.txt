cmake_minimum_required(VERSION 3.10)
# Force g++ when clang++ is the default but libstdc++ headers are missing
if(NOT DEFINED CMAKE_CXX_COMPILER)
  find_program(GXX g++)
  if(GXX)
    set(CMAKE_CXX_COMPILER "${GXX}" CACHE STRING "" FORCE)
  endif()
endif()
project(load_road_structure CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(load_road_structure load_road_structure.cpp)
target_include_directories(load_road_structure PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
