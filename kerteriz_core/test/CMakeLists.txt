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

# F1.1 — NavState: sabit cekirdek + sinirli kapasiteli augmentation.
add_executable(test_nav_state test_nav_state.cpp)
target_link_libraries(test_nav_state PRIVATE kerteriz::core GTest::gtest_main)
# minus()'un duzen on kosulu assert ile zorlanir. Release NDEBUG tanimlar ve
# assert'i sustururdu; o zaman on kosul testi calisir gorunup hicbir sey
# dogrulamazdi. Bu hedefte assert'ler BILEREK acik tutulur.
target_compile_options(test_nav_state PRIVATE -UNDEBUG)
# Eigen dinamik bellegi operator new ile DEGIL malloc ile alir; yalnizca
# operator new saymak Eigen tahsislerini kacirir ve sahte bir 0 uretirdi
# (F1.2'de kanitlandi). Eigen'in kendi kapisi de kullanilir.
target_compile_definitions(test_nav_state PRIVATE EIGEN_RUNTIME_NO_MALLOC)
add_test(NAME test_nav_state COMMAND test_nav_state)

# F1.2 — ImuPropagator: nominal yayilim, ayrik F/Q, surec gurultusu.
add_executable(test_imu_propagator test_imu_propagator.cpp)
target_link_libraries(test_imu_propagator PRIVATE kerteriz::core GTest::gtest_main)
# Eigen dinamik bellegi operator new ile DEGIL malloc ile alir; yalnizca
# operator new saymak Eigen tahsislerini kacirir ve sahte bir 0 uretirdi.
# Eigen'in kendi kapisi kullanilir. Kapi eigen_assert uzerinden calisir, o da
# NDEBUG altinda susar — bu yuzden assert'ler bu hedefte acik tutulur.
target_compile_definitions(test_imu_propagator PRIVATE EIGEN_RUNTIME_NO_MALLOC)
target_compile_options(test_imu_propagator PRIVATE -UNDEBUG)
add_test(NAME test_imu_propagator COMMAND test_imu_propagator)

# F1.3 — chi-kare CDF/quantile (kapi esigi).
add_executable(test_chi_square test_chi_square.cpp)
target_link_libraries(test_chi_square PRIVATE kerteriz::core GTest::gtest_main)
add_test(NAME test_chi_square COMMAND test_chi_square)

# F1.3 — EskfBackend: predict/update/snapshot/klon kovaryansi.
# Assert'ler ve Eigen malloc kapisi acik (olum testleri ve tahsis kapisi icin).
add_executable(test_eskf_backend test_eskf_backend.cpp)
target_link_libraries(test_eskf_backend PRIVATE kerteriz::core GTest::gtest_main)
target_compile_definitions(test_eskf_backend PRIVATE EIGEN_RUNTIME_NO_MALLOC)
target_compile_options(test_eskf_backend PRIVATE -UNDEBUG)
add_test(NAME test_eskf_backend COMMAND test_eskf_backend)

# F1.3 — klon yolunun RELEASE'de de fail-fast oldugu.
# KASTEN -DNDEBUG, -UNDEBUG YOK: assert'lerin kapali oldugu durum sinanir.
# Diger backend testleri assert'ler acikken kostugu icin bu senaryoyu goremez.
add_executable(test_clone_fail_fast test_clone_fail_fast.cpp)
target_link_libraries(test_clone_fail_fast PRIVATE kerteriz::core GTest::gtest_main)
target_compile_definitions(test_clone_fail_fast PRIVATE NDEBUG)
add_test(NAME test_clone_fail_fast COMMAND test_clone_fail_fast)
