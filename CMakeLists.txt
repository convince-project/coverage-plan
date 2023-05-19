cmake_minimum_required(VERSION 3.26)

project(CoveragePlan VERSION 1.0 
                     DESCRIPTION "Source for the coverage planner in CONVINCE WP3." 
                     LANGUAGES CXX)

# Maybe this needs to go into subdirectory CMake, I'm not sure
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Add the subdirectories
add_subdirectory(src)
add_subdirectory(apps)