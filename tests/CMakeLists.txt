add_executable(unitTests mod/imac_tests.cpp mod/imac_executor_tests.cpp)
target_link_libraries(unitTests PRIVATE Catch2::Catch2WithMain)
target_link_libraries(unitTests PUBLIC mod)