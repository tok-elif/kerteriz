# S2 duman testi: Eigen, manif ve GoogleTest gercekten baglandi mi.
add_executable(test_dependencies test_dependencies.cpp)
target_link_libraries(test_dependencies PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_dependencies COMMAND test_dependencies)

# S5 — INTERFACES §0 tipleri ve ArrayView.
add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_types COMMAND test_types)

# S5 DoD: types.hpp tek basina derlenir.
# Calistirilmaz, yalnizca DERLENIR. types.hpp bir gun baska bir kerteriz
# basligina baglanirsa veya Denetim (2)'nin static_assert'leri duserse bu
# hedef derlenmez ve build kirmizi olur.
add_library(kerteriz_types_standalone OBJECT compile_types_standalone.cpp)
target_link_libraries(kerteriz_types_standalone PRIVATE kerteriz::core)

# S6 — SE_2(3) sarmalayici ve konvansiyon bekcileri.
add_executable(test_lie test_lie.cpp)
target_link_libraries(test_lie PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_lie COMMAND test_lie)
