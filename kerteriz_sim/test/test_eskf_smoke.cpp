/// \file
/// F1.5 — sentetik ESKF duman testi (R6).
///
/// GERCEK uretim zinciri sentetik veriyle uctan uca kosar:
///
///   TrajectoryGenerator -> ImuSynthesizer -> EskfBackend::predict
///                       -> GnssSynthesizer -> GnssPosition -> EskfBackend::update
///
/// F2.1'DEN SONRA: senaryo ve dongu artik BU DOSYADA DEGIL,
/// `kerteriz_sim/experiment.hpp` icindedir. Test o tekrar kullanilabilir
/// kosucuyu cagirir; Monte Carlo katmani da AYNISINI cagirir. Ikinci bir
/// filtre dongusu yoktur — iki kod yolunun zamanla ayrismasi boylece mumkun
/// degildir. Senaryo sayilari ve tohumlar DEGISMEDI.
///
/// UC KAYNAK BIRBIRINDEN BAGIMSIZDIR:
///   gercek      = analitik TrajectoryGenerator
///   olcumler    = sentezleyiciler(gercek)
///   kestirim    = EskfBackend
/// Gercek deger HICBIR ZAMAN filtre durumundan uretilmez.
///
/// Bu test bir MONTE CARLO veya NEES calismasi DEGILDIR. Tek realizasyondan
/// istatistiksel kapsama iddiasi cikarilmaz; NIS yalnizca teshis olarak
/// toplanir ve genis akil-sagligi sinirlariyla kontrol edilir.

#include "kerteriz_sim/experiment.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <gtest/gtest.h>

namespace {

using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;
using kerteriz_sim::kNanosecondsPerSecond;
using kerteriz_sim::RunResult;
using kerteriz_sim::RunSeeds;
using kerteriz_sim::ScenarioConfig;

// F1.5'in tohumlari — DEGISMEDI.
constexpr std::uint64_t kImuTohumu = 20260915U;
constexpr std::uint64_t kGnssTohumu = 777001U;

RunSeeds tohumlar(std::uint64_t gnss = kGnssTohumu) { return RunSeeds{kImuTohumu, gnss}; }

RunResult kos(const ScenarioConfig& cfg, const RunSeeds& s = tohumlar()) {
  return kerteriz_sim::run_single(cfg, s);
}

/// Teshis ciktisi. Bu test bir NEES/Monte Carlo calismasi degildir; asagidaki
/// sayilar yalnizca kosunun ne yaptigini gorunur kilar.
void yazdir(const char* etiket, const RunResult& s) {
  std::printf("  [%s]\n", etiket);
  std::printf("    konum hatasi    %.6g m  ->  %.6g m\n", s.initial_position_error,
              s.final_position_error);
  std::printf("    hiz hatasi      %.6g m/s ->  %.6g m/s\n", s.initial_velocity_error,
              s.final_velocity_error);
  std::printf("    GNSS toplam %d  kabul %d  red %d  sayisal basarisizlik %d\n", s.counters.total,
              s.counters.accepted, s.counters.rejected, s.counters.numerical_failures);
  std::printf("    NIS  min %.6g  ort %.6g  maks %.6g\n", s.nis_min, s.nis_mean(), s.nis_max);
  std::printf("    son damga %lld ns\n", static_cast<long long>(s.final_stamp_ns));
}

/// Ortak saglik kontrolleri.
void sonlu_ve_simetrik(const RunResult& s) {
  EXPECT_TRUE(s.final_state.extended_pose().translation().allFinite());
  EXPECT_TRUE(s.final_state.extended_pose().linearVelocity().allFinite());
  EXPECT_TRUE(s.final_state.extended_pose().rotation().allFinite());
  EXPECT_TRUE(s.final_state.gyro_bias().allFinite());
  EXPECT_TRUE(s.final_state.accel_bias().allFinite());

  const int n = s.final_state.active_dof();
  const auto blok = s.final_covariance.topLeftCorner(n, n);
  EXPECT_TRUE(blok.allFinite()) << "kovaryans sonlu degil";
  EXPECT_LT((blok - blok.transpose()).cwiseAbs().maxCoeff(), 1e-9) << "kovaryans simetrisi bozuk";
  EXPECT_GT(blok.diagonal().minCoeff(), 0.0) << "kovaryans kosegeni pozitif degil";
}

TimeNs beklenen_son_damga() {
  const ScenarioConfig c = kerteriz_sim::noisy_scenario();
  const auto adim = static_cast<int>(c.duration_s * static_cast<Scalar>(kNanosecondsPerSecond) /
                                     static_cast<Scalar>(c.imu_period_ns));
  return static_cast<TimeNs>(adim) * c.imu_period_ns;
}

// -----------------------------------------------------------------------------
// 1. Gurultusuz wiring
// -----------------------------------------------------------------------------

TEST(EskfSmoke, NoiselessChainConvergesTowardGroundTruth) {
  const RunResult s = kos(kerteriz_sim::noiseless_scenario());
  yazdir("gurultusuz", s);

  sonlu_ve_simetrik(s);

  // Baslangic kestirimi gercek deger OLMAMALI.
  EXPECT_GT(s.initial_position_error, 1.0) << "baslangic kestirimi gercege cok yakin";
  EXPECT_GT(s.initial_velocity_error, 0.2);

  EXPECT_EQ(s.counters.numerical_failures, 0);
  EXPECT_GT(s.counters.accepted, 0) << "hic GNSS guncellemesi uygulanmadi";
  EXPECT_EQ(s.counters.accepted + s.counters.rejected + s.counters.numerical_failures,
            s.counters.total);
  EXPECT_GT(s.counters.accepted, s.counters.total * 9 / 10)
      << "kosunun anlamli kismi kabul edilmedi";

  EXPECT_LT(s.final_position_error, s.initial_position_error);
  EXPECT_LT(s.final_velocity_error, s.initial_velocity_error);
  // Belirgin bir iyilesme beklenir, ama keyfi 1e-12 gibi bir kriter KONMAZ:
  // yayilim birinci mertebe ve hedef tam sifir hata DEGILDIR.
  EXPECT_LT(s.final_position_error, 0.25 * s.initial_position_error);

  EXPECT_EQ(s.final_stamp_ns, beklenen_son_damga()) << "damga tamsayi adimlardan uretilmemis";
}

// -----------------------------------------------------------------------------
// 2. Tohumlu gurultulu yakinsama
// -----------------------------------------------------------------------------

TEST(EskfSmoke, SeededNoisyChainStaysConsistentAndConverges) {
  const RunResult s = kos(kerteriz_sim::noisy_scenario());
  yazdir("gurultulu", s);

  sonlu_ve_simetrik(s);

  EXPECT_EQ(s.counters.numerical_failures, 0) << "sayisal basarisizlik olmamali";
  EXPECT_EQ(s.counters.accepted + s.counters.rejected, s.counters.total)
      << "kabul + red toplam GNSS sayisina esit degil";
  EXPECT_GT(s.counters.total, 100) << "senaryo beklenen kadar GNSS uretmedi";

  // Makul kabul orani. Bu bir chi-kare kapsama IDDIASI DEGILDIR — tek
  // realizasyon, genis akil-sagligi siniri.
  const Scalar oran =
      static_cast<Scalar>(s.counters.accepted) / static_cast<Scalar>(s.counters.total);
  EXPECT_GT(oran, 0.85) << "kabul orani cok dusuk: kabul=" << s.counters.accepted
                        << " toplam=" << s.counters.total;

  ASSERT_GT(s.counters.accepted + s.counters.rejected, 0);
  EXPECT_TRUE(std::isfinite(s.nis_min));
  EXPECT_TRUE(std::isfinite(s.nis_max));
  EXPECT_GE(s.nis_min, 0.0) << "NIS negatif olamaz";
  EXPECT_TRUE(std::isfinite(s.nis_mean()));
  EXPECT_LT(s.nis_mean(), 100.0) << "NIS ortalamasi akil disi buyuk — filtre kaciyor";

  EXPECT_GT(s.initial_position_error, 1.0);
  EXPECT_LT(s.final_position_error, s.initial_position_error);
  EXPECT_LT(s.final_velocity_error, s.initial_velocity_error);
  EXPECT_LT(s.final_position_error, 5.0) << "filtre kacti";

  EXPECT_EQ(s.final_stamp_ns, beklenen_son_damga());
}

TEST(EskfSmoke, NoisyRunIsActuallyNoisy) {
  // Gurultulu senaryonun gercekten gurultu tasidigi. Sentezleyici gurultusu
  // sessizce sifirlanirsa iki kosum ayni cikardi.
  const RunResult temiz = kos(kerteriz_sim::noiseless_scenario());
  const RunResult gurultulu = kos(kerteriz_sim::noisy_scenario());

  EXPECT_GT((gurultulu.final_state.extended_pose().translation() -
             temiz.final_state.extended_pose().translation())
                .norm(),
            1e-3)
      << "gurultulu kosum gurultusuzle ayni sonucu verdi";
  EXPECT_GT(gurultulu.nis_max, temiz.nis_max) << "gurultu NIS dagilimini genisletmedi";
}

/// S = 0 uretir: J_res ve R sifir, artik sifirdan farkli. GnssPosition bunu
/// yapamaz (konum blogu tam ranktir), bu yuzden sayim mantigini sinamak icin
/// test-yerel dejenere bir olcum gerekir.
class DejenereOlcum : public kerteriz::Measurement {
 public:
  TimeNs stamp_ns() const override { return 0; }
  int residual_dim() const override { return 1; }
  std::string_view name() const override { return "dejenere"; }
  void evaluate(const kerteriz::StateBundle&, kerteriz::MeasurementWorkspace& w) const override {
    w.dim = 1;
    w.r[0] = Scalar(1); // J_res ve R sifir kalir
  }
};

TEST(EskfSmoke, NumericalFailureIsCountedSeparatelyFromAccepted) {
  // Sayisal basarisizligin KABUL gibi sayilmadigini dogrular. Gercek zincirde
  // bu durum uretilemedigi icin sayim mantigi burada dogrudan sinanir.
  const ScenarioConfig cfg = kerteriz_sim::noisy_scenario();
  const auto gt0 = kerteriz_sim::TrajectoryGenerator(cfg.trajectory).at(0);
  const NavState kestirim0 = kerteriz_sim::truth_state(gt0).plus(cfg.initial_error);
  kerteriz::EskfBackend backend(kerteriz::EskfConfig{kestirim0, NavCovariance::Zero(), 0,
                                                     cfg.imu_filter, cfg.chi2_confidence});

  const DejenereOlcum z;
  const auto sonuc = backend.update(z);
  ASSERT_EQ(sonuc.status, UpdateStatus::kNumericalFailure);
  ASSERT_FALSE(sonuc.nis.has_value());

  RunResult s;
  kerteriz_sim::record_update(s, 0, sonuc);

  EXPECT_EQ(s.counters.numerical_failures, 1) << "sayisal basarisizlik ayri sayilmadi";
  EXPECT_EQ(s.counters.accepted, 0) << "sayisal basarisizlik KABUL gibi sayildi";
  EXPECT_EQ(s.counters.rejected, 0);
  EXPECT_EQ(s.counters.total, 1);
  EXPECT_EQ(s.nis_sum, 0.0) << "NIS yokken toplama katki yapildi";

  // Gozlem kaydi da tutulur ve NIS'in TANIMSIZ oldugu isaretlenir (ADR-24).
  ASSERT_EQ(s.nis_observations.size(), 1U);
  EXPECT_FALSE(s.nis_observations[0].has_nis);
  EXPECT_EQ(s.nis_observations[0].status, UpdateStatus::kNumericalFailure);
}

// -----------------------------------------------------------------------------
// 3. Determinizm
// -----------------------------------------------------------------------------

TEST(EskfSmoke, SameSeedsProduceIdenticalRuns) {
  const RunResult a = kos(kerteriz_sim::noisy_scenario());
  const RunResult b = kos(kerteriz_sim::noisy_scenario());

  // Ayni binary, ayni kod yolu, ayni girdi: bit-birebir esitlik beklenir.
  EXPECT_EQ(a.counters.accepted, b.counters.accepted);
  EXPECT_EQ(a.counters.rejected, b.counters.rejected);
  EXPECT_EQ(a.counters.numerical_failures, b.counters.numerical_failures);
  EXPECT_EQ(a.counters.total, b.counters.total);
  EXPECT_EQ(a.nis_sum, b.nis_sum);
  EXPECT_EQ(a.nis_min, b.nis_min);
  EXPECT_EQ(a.nis_max, b.nis_max);
  EXPECT_EQ(a.final_stamp_ns, b.final_stamp_ns);
  EXPECT_EQ(a.final_position_error, b.final_position_error);
  EXPECT_EQ(a.final_velocity_error, b.final_velocity_error);

  EXPECT_EQ(a.final_state.minus(b.final_state).norm(), 0.0) << "final durum ayrisiyor";
  EXPECT_EQ((a.final_covariance - b.final_covariance).cwiseAbs().maxCoeff(), 0.0)
      << "final kovaryans ayrisiyor";
}

TEST(EskfSmoke, DifferentSeedsProduceDifferentRuns) {
  // Determinizm testinin ANLAMLI oldugunu gosterir: tohum degisince sonuc da
  // degismeli, yoksa esitlik testi bos yere gecerdi.
  const RunResult a = kos(kerteriz_sim::noisy_scenario());
  const RunResult b = kos(kerteriz_sim::noisy_scenario(), tohumlar(kGnssTohumu + 1));

  EXPECT_NE(a.nis_sum, b.nis_sum) << "tohum degisti ama NIS toplami ayni";
  EXPECT_GT(a.final_state.minus(b.final_state).norm(), 1e-6);
}

} // namespace
