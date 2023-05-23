add_executable(tests mod/imac_tests.cpp)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
target_link_libraries(tests PUBLIC mod)