
# Create library for map of dynamics
add_library(mod STATIC mod/imac.cpp)
target_include_directories(mod PUBLIC ../include)