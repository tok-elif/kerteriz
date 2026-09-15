# S9 testleri. Uc ayri hedef: biri sessizce kaybolursa ctest sayisi duser ve
# CI'daki "test gercekten kostu mu" kontrolu bunu yakalar.

add_executable(test_rng test_rng.cpp)
target_link_libraries(test_rng PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_rng COMMAND test_rng)

add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_trajectory COMMAND test_trajectory)

add_executable(test_synthesizers test_synthesizers.cpp)
target_link_libraries(test_synthesizers PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_synthesizers COMMAND test_synthesizers)

# F1.5 — sentetik ESKF duman testi (R6). Gercek uretim zinciri uctan uca kosar.
add_executable(test_eskf_smoke test_eskf_smoke.cpp)
target_link_libraries(test_eskf_smoke PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_eskf_smoke COMMAND test_eskf_smoke)
