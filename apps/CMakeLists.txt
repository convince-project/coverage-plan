
# Add executable for IMac test
add_executable(imacTest imac_test.cpp)
target_include_directories(imacTest PUBLIC ../include)
target_link_libraries(imacTest PUBLIC mod)