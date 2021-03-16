add_executable(tsp tsp.cpp ../src/tsp.cpp)
target_include_directories(tsp PRIVATE ../src)
target_link_libraries(tsp PRIVATE Eigen3::Eigen)

add_test(NAME tsp COMMAND tsp)
