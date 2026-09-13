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

# S7 — sayisal residual Jacobian.
add_executable(test_numeric_residual_jacobian test_numeric_residual_jacobian.cpp)
target_link_libraries(test_numeric_residual_jacobian PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_numeric_residual_jacobian COMMAND test_numeric_residual_jacobian)

# S8 — CONVENTIONS §4 guncellemesi ve Denetim (3).
# ClosedFormScalarGaussian bu fazin en onemli testidir.
add_executable(test_linear_update test_linear_update.cpp)
target_link_libraries(test_linear_update PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_linear_update COMMAND test_linear_update)

# S10 — olcum registry'si ve Denetim (4).
#
# Iki ceviri birimi KASTEN ayridir: fabrika kaydi fake_measurements.cpp'de,
# Jacobian testleri test dosyasinda. Denetim ancak ikisi ayni binary'de
# bulustugunda anlamlidir.
add_executable(test_measurement_registry
  test_measurement_registry.cpp
  fake_measurements.cpp)
target_include_directories(test_measurement_registry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_measurement_registry PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_measurement_registry COMMAND test_measurement_registry)
