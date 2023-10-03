cmake_minimum_required(VERSION 3.26)

project(CoveragePlan VERSION 1.0 
                     DESCRIPTION "Source for the coverage planner in CONVINCE WP3." 
                     LANGUAGES CXX)

# Compiler flags that seem to help speed things up a fair bit
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
                   
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -msse2 -mfpmath=sse -march=native -ffast-math -DNDEBUG")

# 3rd party packages
find_package(Boost 1.82 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Catch2 REQUIRED)
find_package(Despot CONFIG REQUIRED)

# Add the subdirectories
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)