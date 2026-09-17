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
# F2.1'den sonra senaryo ve dongu experiment.hpp'dedir; test onu cagirir.
add_executable(test_eskf_smoke test_eskf_smoke.cpp)
target_link_libraries(test_eskf_smoke PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_eskf_smoke COMMAND test_eskf_smoke)

# F2.1 — Monte Carlo kosum altyapisi: tohum turetme, determinizm, on ek
# kararliligi, kayit sozlesmesi. ISTATISTIK URETMEZ (o F2.2).
add_executable(test_monte_carlo test_monte_carlo.cpp)
target_link_libraries(test_monte_carlo PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_monte_carlo COMMAND test_monte_carlo)

# F2.3 — baslangic hatasi ornekleme makinesi ve dondurulmus E1 senaryosu.
add_executable(test_initial_sampling test_initial_sampling.cpp)
target_link_libraries(test_initial_sampling PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_initial_sampling COMMAND test_initial_sampling)

# F2.2 — NEES/NIS istatistik motoru ve chi-kare bantlari. CIZIM YAPMAZ.
add_executable(test_consistency test_consistency.cpp)
target_link_libraries(test_consistency PRIVATE kerteriz::sim GTest::gtest_main)
add_test(NAME test_consistency COMMAND test_consistency)
