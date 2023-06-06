cmake_minimum_required(VERSION 3.26)

project(CoveragePlan VERSION 1.0 
                     DESCRIPTION "Source for the coverage planner in CONVINCE WP3." 
                     LANGUAGES CXX)

# 3rd party packages
find_package(Boost REQUIRED COMPONENTS math)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Catch2 REQUIRED)

# Add the subdirectories
add_subdirectory(src)
add_subdirectory(apps)
add_subdirectory(tests)