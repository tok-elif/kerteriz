# S2 duman testi: Eigen, manif ve GoogleTest gercekten baglandi mi.
# Gercek matematik testleri S5-S8'de gelir.
add_executable(test_dependencies test_dependencies.cpp)
target_link_libraries(test_dependencies PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_dependencies COMMAND test_dependencies)
