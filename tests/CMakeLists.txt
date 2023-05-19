add_executable(tests mod/imacTest.cpp)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)
target_link_libraries(tests PUBLIC mod)