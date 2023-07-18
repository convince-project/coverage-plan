# Create library for map of dynamics
add_library(mod STATIC mod/imac.cpp mod/imac_executor.cpp mod/bimac.cpp)
target_include_directories(mod PUBLIC ../include)
target_link_libraries(mod PUBLIC Eigen3::Eigen)
target_link_libraries(mod PUBLIC Boost::headers)

# Create library for coverage planner
add_library(planning STATIC planning/action.cpp 
                            planning/coverage_robot.cpp 
                            planning/random_coverage_robot.cpp)
target_include_directories(planning PUBLIC ../include)
target_link_libraries(planning PUBLIC mod)
target_link_libraries(planning PUBLIC Eigen3::Eigen)
