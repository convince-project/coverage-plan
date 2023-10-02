
# Add executable for IMac example
add_executable(imacExample imac_example.cpp)
target_link_libraries(imacExample PUBLIC mod)

# Add executable for IMacExecutor example
add_executable(imacExecutorExample imac_executor_example.cpp)
target_link_libraries(imacExecutorExample PUBLIC mod)

# Add executable for BIMac learning experiment
add_executable(bimacLearningExperiment bimac_learning_experiment.cpp)
target_link_libraries(bimacLearningExperiment PUBLIC mod)

# Add executable for random coverage robot example
add_executable(randomCoverageRobotExample random_coverage_robot_example.cpp)
target_link_libraries(randomCoverageRobotExample PUBLIC mod planning baselines)

# Add executable for POMDP coverage example
add_executable(pomdpCoverageExample pomdp_coverage_example.cpp)
target_link_libraries(pomdpCoverageExample PUBLIC mod planning util)

# Add executable for multi-episode test
add_executable(multiEpisodeTest multi_episode_test.cpp)
target_link_libraries(multiEpisodeTest PUBLIC mod planning util)

# Add executable for random IMac environment generation
add_executable(randomIMacGen random_imac_gen.cpp)
target_link_libraries(randomIMacGen PUBLIC mod planning util)

# Add executable for preliminary experiments
add_executable(prelimExperiments prelim_experiments.cpp)
target_link_libraries(prelimExperiments PUBLIC mod planning util baselines)

# Add executable for five heavy pomdp example 
add_executable(fiveHeavyPOMDPExample five_heavy_pomdp_example.cpp)
target_link_libraries(fiveHeavyPOMDPExample PUBLIC mod planning util)

# Add executable for profiling test 
add_executable(profilingTest profiling_test.cpp)
target_link_libraries(profilingTest PUBLIC mod planning util)