# Create library for map of dynamics
add_library(mod STATIC mod/imac.cpp)
target_include_directories(mod PUBLIC ../include)
target_link_libraries(mod PUBLIC Eigen3::Eigen)