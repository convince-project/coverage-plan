# Create library for map of dynamics
add_library(mod STATIC mod/imac.cpp 
                       mod/imac_executor.cpp 
                       mod/bimac.cpp 
                       mod/imac_belief_sampler.cpp
                       mod/grid_cell.cpp
                       mod/fixed_imac_executor.cpp)
target_include_directories(mod PUBLIC ../include)
target_link_libraries(mod PUBLIC Eigen3::Eigen)
target_link_libraries(mod PUBLIC Boost::headers)
target_link_libraries(mod PUBLIC util)

# Create library for coverage planner
add_library(planning STATIC planning/action.cpp 
                            planning/coverage_robot.cpp 
                            planning/random_coverage_robot.cpp
                            planning/coverage_state.cpp
                            planning/coverage_observation.cpp
                            planning/coverage_belief.cpp
                            planning/coverage_pomdp.cpp
                            planning/coverage_world.cpp
                            planning/coverage_planner.cpp
                            planning/pomdp_coverage_robot.cpp
                            planning/coverage_bounds.cpp)
target_include_directories(planning PUBLIC ../include)
target_link_libraries(planning PUBLIC mod)
target_link_libraries(planning PUBLIC Eigen3::Eigen)
target_link_libraries(planning PUBLIC despot)
target_link_libraries(planning PUBLIC util)

# Create library for utility functions
add_library(util STATIC util/seed.cpp)
target_include_directories(util PUBLIC ../include)
