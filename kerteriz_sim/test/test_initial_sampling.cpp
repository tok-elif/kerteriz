/// \file
/// F2.3 — baslangic hatasi ornekleme makinesi.
///
/// BU DOSYA ESKF'IN TUTARLI OLDUGUNU IDDIA ETMEZ. Sinanan sey ORNEKLEYICIDIR:
/// politika, tohum bagimsizligi, P0 carpanlastirmasi ve gecersiz P0 politikasi.
/// 500 kosumluk uretim deneyi normal birim testlerde KOSMAZ.

#include "kerteriz_sim/consistency.hpp"
#include "kerteriz_sim/e1_scenario.hpp"
#include "kerteriz_sim/monte_carlo.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <set>

namespace {

using kerteriz::kCoreDof;
using kerteriz::kMaxStateDof;
using kerteriz::NavCovariance;
using kerteriz::Scalar;
using kerteriz::StateVec;
using kerteriz_sim::derive_seeds;
using kerteriz_sim::MonteCarloConfig;
using kerteriz_sim::MonteCarloResult;
using kerteriz_sim::RunSeeds;
using kerteriz_sim::sample_initial_error;
using kerteriz_sim::ScenarioConfig;
using kerteriz_sim::SeededRng;

using CoreMat = Eigen::Matrix<Scalar, kCoreDof, kCoreDof>;

constexpr std::uint64_t kAna = 20260916ULL;

ScenarioConfig kisa_e1() {
  ScenarioConfig c = kerteriz_sim::e1_eskf_scenario();
  c.duration_s = 2.0; // testler hizli kalsin
  return c;
}

MonteCarloConfig mc(int runs, std::uint64_t master = kAna) {
  MonteCarloConfig m = kerteriz_sim::e1_eskf_monte_carlo();
  m.scenario = kisa_e1();
  m.runs = runs;
  m.master_seed = master;
  return m;
}

// -----------------------------------------------------------------------------
// 1. kFixed davranisi korunuyor
// -----------------------------------------------------------------------------

TEST(InitialPolicy, FixedModeIsTheDefaultAndDoesNotConsumeTheNewStream) {
  // F1.5 duman senaryosu kFixed'dir ve YENI akisi HIC tuketmez. Tuketseydi
  // IMU/GNSS dizileri kaymazdi ama politika degisiminin "olcume dokunmadigi"
  // iddiasi da dogrulanamazdi.
  const ScenarioConfig f15 = kerteriz_sim::noisy_scenario();
  EXPECT_EQ(f15.initial_error_policy, ScenarioConfig::InitialErrorPolicy::kFixed);

  RunSeeds a;
  a.imu = 20260915U;
  a.gnss = 777001U;
  RunSeeds b = a;
  b.initial_state = 123456789ULL; // kFixed'de ONEMSIZ olmali

  const auto ra = kerteriz_sim::run_single(f15, a);
  const auto rb = kerteriz_sim::run_single(f15, b);

  EXPECT_EQ(ra.final_state.minus(rb.final_state).norm(), 0.0)
      << "kFixed baslangic akisini tuketiyor";
  EXPECT_EQ(ra.nis_sum, rb.nis_sum);
  EXPECT_EQ((ra.initial_error_used - f15.initial_error).cwiseAbs().maxCoeff(), 0.0);
}

// -----------------------------------------------------------------------------
// 2-5. Tohum sozlesmesi
// -----------------------------------------------------------------------------

TEST(InitialPolicy, SameMasterAndRunIndexGiveIdenticalInitialError) {
  const auto a = kerteriz_sim::run_monte_carlo(mc(3));
  const auto b = kerteriz_sim::run_monte_carlo(mc(3));
  ASSERT_EQ(a.runs.size(), 3U);
  for (std::size_t i = 0; i < a.runs.size(); ++i) {
    EXPECT_EQ((a.runs[i].initial_error_used - b.runs[i].initial_error_used).cwiseAbs().maxCoeff(),
              0.0)
        << "kosu " << i;
  }
}

TEST(InitialPolicy, DifferentInitialStateSeedChangesTheInitialError) {
  const ScenarioConfig c = kisa_e1();
  RunSeeds a = derive_seeds(kAna, 0);
  RunSeeds b = a;
  b.initial_state = a.initial_state + 1; // YALNIZCA baslangic akisi degisti

  const auto ra = kerteriz_sim::run_single(c, a);
  const auto rb = kerteriz_sim::run_single(c, b);
  EXPECT_GT((ra.initial_error_used - rb.initial_error_used).norm(), 1e-9);
}

TEST(InitialPolicy, InitialErrorSamplingDoesNotMoveTheImuOrGnssStreams) {
  // Baslangic ornekleme politikasini degistirmek olcum gurultu dizilerini
  // KAYDIRMAMALI. Ayni IMU/GNSS tohumlariyla sentezleyici ciktisi birebir ayni
  // kalmali; degisen tek sey baslangic durumu olmali.
  ScenarioConfig sabit = kisa_e1();
  sabit.initial_error_policy = ScenarioConfig::InitialErrorPolicy::kFixed;
  sabit.initial_error = kerteriz_sim::default_initial_error();

  RunSeeds s = derive_seeds(kAna, 0);
  const auto r_sabit = kerteriz_sim::run_single(sabit, s);
  const auto r_ornek = kerteriz_sim::run_single(kisa_e1(), s);

  // Ayni sayida GNSS olayi: zamanlama ve sentezleyici tuketimi degismedi.
  EXPECT_EQ(r_sabit.counters.total, r_ornek.counters.total);
  EXPECT_EQ(r_sabit.final_stamp_ns, r_ornek.final_stamp_ns);
  // Baslangic hatasi ise GERCEKTEN degisti.
  EXPECT_GT((r_sabit.initial_error_used - r_ornek.initial_error_used).norm(), 1e-6);
}

TEST(InitialPolicy, GrowingNPreservesEarlierRunInitialErrors) {
  const auto kisa = kerteriz_sim::run_monte_carlo(mc(3));
  const auto uzun = kerteriz_sim::run_monte_carlo(mc(5));
  ASSERT_EQ(uzun.runs.size(), 5U);
  for (std::size_t i = 0; i < kisa.runs.size(); ++i) {
    EXPECT_EQ(
        (kisa.runs[i].initial_error_used - uzun.runs[i].initial_error_used).cwiseAbs().maxCoeff(),
        0.0)
        << "kosu " << i;
  }
}

TEST(InitialPolicy, ThreeSeedDomainsAreDistinctAndIndependent) {
  std::set<std::uint64_t> hepsi;
  for (int i = 0; i < 32; ++i) {
    const RunSeeds s = derive_seeds(kAna, i);
    EXPECT_NE(s.initial_state, s.imu) << "i = " << i;
    EXPECT_NE(s.initial_state, s.gnss) << "i = " << i;
    EXPECT_NE(s.imu, s.gnss) << "i = " << i;
    hepsi.insert(s.initial_state);
    hepsi.insert(s.imu);
    hepsi.insert(s.gnss);
  }
  EXPECT_EQ(hepsi.size(), 96U) << "uc alan arasinda cakisma var";
}

// -----------------------------------------------------------------------------
// 6. Teget bicimi
// -----------------------------------------------------------------------------

TEST(InitialSampler, TailBeyondCoreDofIsZero) {
  SeededRng rng(12345);
  const auto s = sample_initial_error(kerteriz_sim::e1_initial_covariance(), rng);
  ASSERT_TRUE(s.ok) << s.message;
  EXPECT_EQ(s.delta.tail(kMaxStateDof - kCoreDof).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_GT(s.delta.head<kCoreDof>().norm(), 0.0);
}

// -----------------------------------------------------------------------------
// 7-9. Gecersiz P0 — ONARILMAZ
// -----------------------------------------------------------------------------

TEST(InitialSampler, NonFiniteP0FailsExplicitly) {
  NavCovariance p = kerteriz_sim::default_initial_covariance();
  p(4, 4) = std::numeric_limits<Scalar>::quiet_NaN();
  SeededRng rng(1);
  const auto s = sample_initial_error(p, rng);
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.message.find("sonlu"), std::string::npos) << s.message;
  EXPECT_EQ(s.delta.cwiseAbs().maxCoeff(), 0.0);
}

TEST(InitialSampler, MateriallyNonSymmetricP0FailsExplicitly) {
  NavCovariance p = kerteriz_sim::default_initial_covariance();
  p(1, 5) = Scalar(0.3);
  p(5, 1) = Scalar(-0.3);
  SeededRng rng(1);
  const auto s = sample_initial_error(p, rng);
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.message.find("simetrik"), std::string::npos) << s.message;
}

TEST(InitialSampler, SingularOrIndefiniteP0FailsExplicitly) {
  NavCovariance tekil = kerteriz_sim::default_initial_covariance();
  tekil(7, 7) = Scalar(0);
  SeededRng r1(1);
  const auto a = sample_initial_error(tekil, r1);
  EXPECT_FALSE(a.ok);
  EXPECT_NE(a.message.find("pozitif tanimli"), std::string::npos) << a.message;

  NavCovariance belirsiz = kerteriz_sim::default_initial_covariance();
  belirsiz(3, 3) = Scalar(-1);
  SeededRng r2(1);
  const auto b = sample_initial_error(belirsiz, r2);
  EXPECT_FALSE(b.ok);
  EXPECT_NE(b.message.find("pozitif tanimli"), std::string::npos) << b.message;
}

TEST(InitialSampler, FailedSamplingMarksTheRunWithoutDroppingIt) {
  ScenarioConfig c = kisa_e1();
  c.initial_covariance(7, 7) = Scalar(0); // tekil
  MonteCarloConfig m = mc(3);
  m.scenario = c;

  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(m);
  ASSERT_EQ(r.runs.size(), 3U) << "basarisiz kosu SESSIZCE ATILMIS";
  EXPECT_EQ(r.failed_runs, 3);
  for (const auto& k : r.runs) {
    EXPECT_FALSE(k.ok);
    EXPECT_FALSE(k.failure_message.empty());
  }
}

// -----------------------------------------------------------------------------
// 10 + 12. Kosegen olmayan SPD P0 ve topluluk denetimi
// -----------------------------------------------------------------------------

CoreMat kosegen_olmayan_p0() {
  CoreMat a = CoreMat::Zero();
  for (int i = 0; i < kCoreDof; ++i) {
    for (int k = 0; k < kCoreDof; ++k) {
      a(i, k) = Scalar(0.05) * Scalar((i + 1) * (k + 3)) - Scalar(0.02) * Scalar(i * k);
    }
  }
  return a * a.transpose() + CoreMat::Identity() * Scalar(0.5);
}

TEST(InitialSampler, NonDiagonalSpdP0IsSupportedAndReproduced) {
  // KOSEGEN VARSAYILMAZ. Capraz terimler yok sayilsaydi ampirik kovaryans
  // kosegen cikardi ve bu test duserdi.
  const CoreMat hedef = kosegen_olmayan_p0();
  NavCovariance p0 = NavCovariance::Zero();
  p0.topLeftCorner<kCoreDof, kCoreDof>() = hedef;

  constexpr int kN = 20000;
  SeededRng rng(987654321ULL);
  Eigen::Matrix<Scalar, kCoreDof, 1> ort = Eigen::Matrix<Scalar, kCoreDof, 1>::Zero();
  std::vector<Eigen::Matrix<Scalar, kCoreDof, 1>> ornekler;
  ornekler.reserve(kN);
  for (int i = 0; i < kN; ++i) {
    const auto s = sample_initial_error(p0, rng);
    ASSERT_TRUE(s.ok) << s.message;
    ornekler.push_back(s.delta.head<kCoreDof>());
    ort += ornekler.back();
  }
  ort /= Scalar(kN);

  CoreMat kov = CoreMat::Zero();
  for (const auto& v : ornekler) {
    const auto c = v - ort;
    kov += c * c.transpose();
  }
  kov /= Scalar(kN - 1);

  // Tolerans gerekcesi: N = 20000 ornekte kovaryans ogesinin bagil standart
  // hatasi ~sqrt(2/N) ~ 1%. 6% sinir bunun ~6 katidir; TOHUM SABIT oldugu icin
  // test kararsiz degildir, sinir yalnizca makul bir pay birakir.
  for (int i = 0; i < kCoreDof; ++i) {
    for (int k = 0; k < kCoreDof; ++k) {
      const Scalar olcek = std::sqrt(hedef(i, i) * hedef(k, k));
      EXPECT_LT(std::abs(kov(i, k) - hedef(i, k)) / olcek, 0.06)
          << "(" << i << "," << k << ") ampirik " << kov(i, k) << " hedef " << hedef(i, k);
    }
  }
  // Capraz terimler GERCEKTEN sifirdan farkli: aksi halde bu test kosegen bir
  // ornekleyiciyi de gecirirdi.
  EXPECT_GT(std::abs(hedef(0, 1)) / std::sqrt(hedef(0, 0) * hedef(1, 1)), 0.1);
}

// -----------------------------------------------------------------------------
// 11. Dondurulmus E1 senaryosu
// -----------------------------------------------------------------------------

TEST(E1Scenario, IsFrozenWithTheExpectedContract) {
  const ScenarioConfig c = kerteriz_sim::e1_eskf_scenario();
  const MonteCarloConfig m = kerteriz_sim::e1_eskf_monte_carlo();

  EXPECT_EQ(c.initial_error_policy, ScenarioConfig::InitialErrorPolicy::kGaussianFromP0);
  EXPECT_EQ(m.runs, 500);
  EXPECT_EQ(m.master_seed, 20260916ULL);
  EXPECT_EQ(m.state_sample_stride, 20);
  EXPECT_TRUE(m.record_state_samples);
  EXPECT_EQ(kerteriz_sim::e1::kConsistencyConfidence, 0.95);

  // Yaw sigma = pi/6 = 30 derece; varyans karesi.
  EXPECT_NEAR(kerteriz_sim::e1::kYawSigmaRad, M_PI / 6.0, 1e-15);
  EXPECT_NEAR(c.initial_covariance(2, 2), (M_PI / 6.0) * (M_PI / 6.0), 1e-15);
  // Yuvarlanma/pitch DEGISMEDI.
  EXPECT_NEAR(c.initial_covariance(0, 0), 0.01, 1e-18);
  EXPECT_NEAR(c.initial_covariance(1, 1), 0.01, 1e-18);
  // Geri kalan P0 varsayilanla AYNI.
  const NavCovariance v = kerteriz_sim::default_initial_covariance();
  for (int i = 3; i < kCoreDof; ++i) {
    EXPECT_EQ(c.initial_covariance(i, i), v(i, i)) << "P0 kosegeni " << i << " degismis";
  }
  // Gurultu ve kapi `noisy_scenario` ile AYNI.
  const ScenarioConfig n = kerteriz_sim::noisy_scenario();
  EXPECT_EQ(c.chi2_confidence, n.chi2_confidence);
  EXPECT_EQ(c.gnss_position_noise_std, n.gnss_position_noise_std);
  EXPECT_EQ(c.imu_sim.gyro_noise_density, n.imu_sim.gyro_noise_density);
  EXPECT_EQ(c.imu_filter.accel_noise_density, n.imu_filter.accel_noise_density);
  EXPECT_EQ(c.duration_s, n.duration_s);
  EXPECT_EQ(c.imu_period_ns, n.imu_period_ns);
  EXPECT_EQ(c.gnss_rate_hz, n.gnss_rate_hz);
}

TEST(E1Scenario, YawVarianceActuallyReachesTheSampledEnsemble) {
  // Yaw sigma P0'a uygulanmazsa ampirik yaw dagilimi 0.1 rad'da kalirdi.
  MonteCarloConfig m = mc(400);
  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(m);
  ASSERT_EQ(r.runs.size(), 400U);

  Scalar toplam = 0;
  for (const auto& k : r.runs) {
    toplam += k.initial_error_used[2] * k.initial_error_used[2];
  }
  const Scalar yaw_std = std::sqrt(toplam / Scalar(r.runs.size()));
  EXPECT_GT(yaw_std, 0.35) << "yaw varyansi toplulaga ulasmiyor (0.1 rad'da kalmis olabilir)";
  EXPECT_LT(yaw_std, 0.75);

  // Yuvarlanma ise kucuk kalmali — stres YALNIZCA yaw uzerinde.
  Scalar roll = 0;
  for (const auto& k : r.runs) {
    roll += k.initial_error_used[0] * k.initial_error_used[0];
  }
  EXPECT_LT(std::sqrt(roll / Scalar(r.runs.size())), 0.2);
}

} // namespace
