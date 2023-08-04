add_executable(unitTests mod/imac_tests.cpp 
                         mod/imac_executor_tests.cpp 
                         mod/bimac_tests.cpp
                         mod/grid_cell_tests.cpp
                         mod/imac_belief_sampler_tests.cpp
                         planning/action_tests.cpp
                         planning/coverage_robot_tests.cpp
                         planning/random_coverage_robot_tests.cpp
                         planning/coverage_state_tests.cpp
                         planning/coverage_observation_tests.cpp
                         planning/coverage_belief_tests.cpp
                         util/seed_tests.cpp)
target_link_libraries(unitTests PRIVATE Catch2::Catch2WithMain)
target_link_libraries(unitTests PUBLIC mod)
target_link_libraries(unitTests PUBLIC planning)
target_link_libraries(unitTests PUBLIC util)