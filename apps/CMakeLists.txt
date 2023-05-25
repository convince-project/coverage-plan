
# Add executable for IMac example
add_executable(imacExample imac_example.cpp)
target_link_libraries(imacExample PUBLIC mod)

# Add executable for IMacExecutor example
add_executable(imacExecutorExample imac_executor_example.cpp)
target_link_libraries(imacExecutorExample PUBLIC mod)