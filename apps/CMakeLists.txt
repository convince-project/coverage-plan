
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
target_link_libraries(multiEpisodeTest PUBLIC mod planning util baselines)

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

# Add executable for numScenarios tuning
add_executable(numScenariosTuning num_scenarios_tuning.cpp)
target_link_libraries(numScenariosTuning PUBLIC mod planning util)

# Add executable for energy functional tuning
add_executable(energyFunctionalTuning energy_functional_tuning.cpp)
target_link_libraries(energyFunctionalTuning PUBLIC mod planning util baselines)

# Add executable for checkpoint generator
add_executable(lifelongTestCheckpointGenerator lifelong_test_checkpoint_generation.cpp)
target_link_libraries(lifelongTestCheckpointGenerator PUBLIC mod planning util)

# Add executable for testing performance at different checkpoints
add_executable(testCheckpointsExperiment test_checkpoints_experiment.cpp)
target_link_libraries(testCheckpointsExperiment PUBLIC mod planning util)

# Add executable for ICAPS planning experiment
add_executable(icapsPlanningExperiment icaps_planning_exp.cpp)
target_link_libraries(icapsPlanningExperiment PUBLIC mod planning util baselines)

# Add executable for ICAPS learning experiment
add_executable(icapsLearningExperiment icaps_learning_exp.cpp)
target_link_libraries(icapsLearningExperiment PUBLIC mod planning util baselines)