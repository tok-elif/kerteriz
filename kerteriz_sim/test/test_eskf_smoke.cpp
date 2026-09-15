/// \file
/// F1.5 — sentetik ESKF duman testi (R6).
///
/// GERCEK uretim zinciri sentetik veriyle uctan uca kosar:
///
///   TrajectoryGenerator -> ImuSynthesizer -> EskfBackend::predict
///                       -> GnssSynthesizer -> GnssPosition -> EskfBackend::update
///
/// Yeni filtre matematigi YOKTUR; ImuPropagator, EskfBackend ve GnssPosition
/// uretim kodu bypass EDILMEZ.
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

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz_sim/gnss.hpp"
#include "kerteriz_sim/imu.hpp"
#include "kerteriz_sim/rng.hpp"
#include "kerteriz_sim/trajectory.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <gtest/gtest.h>
#include <limits>

namespace {

using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::GnssPosition;
using kerteriz::ImuNoiseParams;
using kerteriz::kCoreDof;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::StateVec;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;
using kerteriz_sim::GnssParams;
using kerteriz_sim::GnssSynthesizer;
using kerteriz_sim::ImuParams;
using kerteriz_sim::ImuSynthesizer;
using kerteriz_sim::SeededRng;
using kerteriz_sim::TrajectoryGenerator;
using kerteriz_sim::TrajectoryKind;
using kerteriz_sim::TrajectoryParams;
using kerteriz_sim::TrajectorySample;

// -----------------------------------------------------------------------------
// Senaryo
// -----------------------------------------------------------------------------

constexpr TimeNs kSaniye = 1000000000LL;
constexpr TimeNs kImuPeriyodu = kSaniye / 100; ///< 100 Hz
constexpr Scalar kGnssHizi = 5.0;              ///< Hz
constexpr Scalar kSure = 30.0;                 ///< s
constexpr Scalar kGuven = 0.997;

constexpr std::uint64_t kImuTohumu = 20260915U;
constexpr std::uint64_t kGnssTohumu = 777001U;

struct SmokeConfig {
  ImuParams imu_sim;         ///< sentezleyici gurultusu
  ImuNoiseParams imu_filtre; ///< filtrenin varsaydigi gurultu
  Scalar gnss_std = 0.5;     ///< m, eksen basina
  Vec3 gnss_lever = Vec3::Zero();
  std::uint64_t imu_tohumu = kImuTohumu;
  std::uint64_t gnss_tohumu = kGnssTohumu;
};

struct SmokeSummary {
  Scalar initial_position_error = 0;
  Scalar final_position_error = 0;
  Scalar initial_velocity_error = 0;
  Scalar final_velocity_error = 0;
  int gnss_total = 0;
  int accepted = 0;
  int rejected = 0;
  int numerical_failures = 0;
  Scalar nis_sum = 0;
  Scalar nis_min = std::numeric_limits<Scalar>::infinity();
  Scalar nis_max = -std::numeric_limits<Scalar>::infinity();
  TimeNs final_stamp_ns = 0;
  NavState final_state;
  NavCovariance final_covariance = NavCovariance::Zero();

  Scalar nis_mean() const {
    const int n = accepted + rejected;
    return n > 0 ? nis_sum / static_cast<Scalar>(n) : Scalar(0);
  }
};

/// Sonucu ozete isler. AYRI bir fonksiyondur ki sayim mantigi dogrudan
/// sinanabilsin: gercek zincirde GnssPosition ile S = J P J^T + R her zaman
/// pozitif tanimlidir, yani sayisal basarisizlik DOGAL OLARAK tetiklenemez.
/// Yanlis siniflandirma ancak bu fonksiyon tek basina sinanarak yakalanabilir.
void say(SmokeSummary& s, const kerteriz::UpdateResult& sonuc) {
  ++s.gnss_total;
  switch (sonuc.status) {
  case UpdateStatus::kAccepted:
    ++s.accepted;
    break;
  case UpdateStatus::kChiSquareRejected:
    ++s.rejected;
    break;
  case UpdateStatus::kNumericalFailure:
    ++s.numerical_failures;
    break;
  }
  if (sonuc.nis.has_value()) {
    s.nis_sum += *sonuc.nis;
    s.nis_min = std::min(s.nis_min, *sonuc.nis);
    s.nis_max = std::max(s.nis_max, *sonuc.nis);
  }
}

TrajectoryGenerator sekiz() {
  TrajectoryParams p;
  p.kind = TrajectoryKind::kFigureEight;
  p.radius = 10.0;
  p.angular_rate = 0.2;
  p.height = 1.0;
  return TrajectoryGenerator(p);
}

/// Filtrenin varsaydigi IMU gurultusu. Sentezleyici ile AYNI fiziksel modeli
/// temsil eder; gurultusuz senaryoda sentezleyici sifir gurultu uretir ama
/// filtre yine bu degerleri kullanir — yayilim birinci mertebe oldugu icin
/// sifir surec gurultusu kovaryansi gercekci olmayan bicimde cokertirdi.
ImuNoiseParams filtre_gurultusu() {
  ImuNoiseParams n;
  n.gyro_noise_density = 1.0e-3;  // rad/s/sqrt(Hz)
  n.gyro_random_walk = 1.0e-5;    // rad/s^2/sqrt(Hz)
  n.accel_noise_density = 2.0e-2; // m/s^2/sqrt(Hz)
  n.accel_random_walk = 3.0e-4;   // m/s^3/sqrt(Hz)
  return n;
}

/// Sentezleyici tarafinda AYNI model.
ImuParams sim_gurultusu() {
  ImuParams p;
  p.gyro_noise_density = 1.0e-3;
  p.accel_noise_density = 2.0e-2;
  p.gyro_bias_walk = 1.0e-5;
  p.accel_bias_walk = 3.0e-4;
  return p;
}

/// Gercek durumdan NavState. SE_2_3 kurucusu (konum, kuaterniyon, hiz).
///
/// KUATERNIYON ACIKCA NORMALLESTIRILIR. Yorunge rotasyon MATRISI uretir; ondan
/// turetilen kuaterniyon birim normdan ~1e-16 sapabilir ve manif bunu reddeder
/// ("SE_2_3 assigned data not normalized !"). Bu kontrol yalnizca assert'ler
/// acikken calisir, dolayisiyla Release build'de gorunmez.
NavState gercek_durum(const TrajectorySample& gt) {
  Eigen::Quaterniond q(gt.rotation);
  q.normalize();
  NavState x;
  x.extended_pose() = SE23(gt.position, q, gt.velocity);
  return x;
}

/// Kasitli baslangic hatasi. NavState::plus ile uygulanir — projenin KENDI
/// sag perturbasyon konvansiyonu.
StateVec baslangic_hatasi() {
  StateVec d = StateVec::Zero();
  d.segment<3>(0) = Vec3(0.010, -0.015, 0.050);  // dtheta, yaw agirlikli ~2.9 derece
  d.segment<3>(3) = Vec3(0.30, -0.20, 0.10);     // dv
  d.segment<3>(6) = Vec3(1.50, -1.00, 0.50);     // dp
  d.segment<3>(9) = Vec3(0.002, -0.001, 0.003);  // db_g
  d.segment<3>(12) = Vec3(0.020, 0.030, -0.010); // db_a
  return d;
}

/// Baslangic kovaryansi gercek hatayi RAHATCA kapsar; ilk GNSS olcumunun
/// yalnizca kotu ayar yuzunden kapiya takilmasi ISTENMEZ.
NavCovariance baslangic_kovaryansi() {
  NavCovariance p = NavCovariance::Zero();
  p.block<3, 3>(0, 0).diagonal().setConstant(0.01);   // (0.1 rad)^2
  p.block<3, 3>(3, 3).diagonal().setConstant(0.25);   // (0.5 m/s)^2
  p.block<3, 3>(6, 6).diagonal().setConstant(9.0);    // (3 m)^2
  p.block<3, 3>(9, 9).diagonal().setConstant(1.0e-4); // (0.01 rad/s)^2
  p.block<3, 3>(12, 12).diagonal().setConstant(0.01); // (0.1 m/s^2)^2
  return p;
}

// -----------------------------------------------------------------------------
// Kosum
// -----------------------------------------------------------------------------

SmokeSummary kos(const SmokeConfig& cfg) {
  TrajectoryGenerator yorunge = sekiz();

  // AYRI RNG nesneleri ve AYRI tohumlar: GNSS ornek sayisi degisse bile IMU
  // gurultu dizisi degismez.
  SeededRng imu_rng(cfg.imu_tohumu);
  SeededRng gnss_rng(cfg.gnss_tohumu);
  ImuSynthesizer imu_sentez(cfg.imu_sim, imu_rng);
  GnssSynthesizer gnss_sentez(GnssParams{cfg.gnss_std, kGnssHizi}, gnss_rng);

  const TrajectorySample gt0 = yorunge.at(0);
  const NavState gercek0 = gercek_durum(gt0);
  const NavState kestirim0 = gercek0.plus(baslangic_hatasi());

  SmokeSummary s;
  s.initial_position_error = (kestirim0.extended_pose().translation() - gt0.position).norm();
  s.initial_velocity_error = (kestirim0.extended_pose().linearVelocity() - gt0.velocity).norm();

  EskfBackend backend(EskfConfig{kestirim0, baslangic_kovaryansi(), 0, cfg.imu_filtre, kGuven});

  const TimeNs gnss_periyodu = gnss_sentez.period_ns();
  const Eigen::Matrix3d gnss_kovaryansi =
      Eigen::Matrix3d::Identity() * (cfg.gnss_std * cfg.gnss_std);

  const auto adim_sayisi = static_cast<int>(kSure * kSaniye / kImuPeriyodu);
  TimeNs onceki_ns = 0;

  for (int k = 1; k <= adim_sayisi; ++k) {
    // MUTLAK ZAMAN TAMSAYI ADIMLARDAN URETILIR; dt biriktirilmez.
    const TimeNs t = static_cast<TimeNs>(k) * kImuPeriyodu;
    const Scalar dt = static_cast<Scalar>(t - onceki_ns) * 1e-9;
    const TrajectorySample gt = yorunge.at(t);

    backend.predict(imu_sentez.sample(gt, dt), dt);
    onceki_ns = t;

    if (gnss_periyodu > 0 && t % gnss_periyodu == 0) {
      const auto olcum = gnss_sentez.sample(gt);
      const GnssPosition z(olcum.stamp_ns, "gnss_main", olcum.position, gnss_kovaryansi,
                           cfg.gnss_lever);
      const auto sonuc = backend.update(z);

      say(s, sonuc);
    }
  }

  const TrajectorySample gt_son = yorunge.at(onceki_ns);
  s.final_position_error = (backend.state().extended_pose().translation() - gt_son.position).norm();
  s.final_velocity_error =
      (backend.state().extended_pose().linearVelocity() - gt_son.velocity).norm();
  s.final_stamp_ns = backend.stamp_ns();
  s.final_state = backend.state();
  s.final_covariance = backend.covariance();
  return s;
}

SmokeConfig gurultusuz_config() {
  SmokeConfig c;
  c.imu_sim = ImuParams{}; // tum gurultu ve bias yuruyusu SIFIR
  c.imu_filtre = filtre_gurultusu();
  c.gnss_std = 0.0;
  return c;
}

SmokeConfig gurultulu_config() {
  SmokeConfig c;
  c.imu_sim = sim_gurultusu();
  c.imu_filtre = filtre_gurultusu();
  c.gnss_std = 0.5;
  return c;
}

/// Teshis ciktisi. Bu test bir NEES/Monte Carlo calismasi degildir; asagidaki
/// sayilar yalnizca kosunun ne yaptigini gorunur kilar.
void yazdir(const char* etiket, const SmokeSummary& s) {
  std::printf("  [%s]\n", etiket);
  std::printf("    konum hatasi    %.6g m  ->  %.6g m\n", s.initial_position_error,
              s.final_position_error);
  std::printf("    hiz hatasi      %.6g m/s ->  %.6g m/s\n", s.initial_velocity_error,
              s.final_velocity_error);
  std::printf("    GNSS toplam %d  kabul %d  red %d  sayisal basarisizlik %d\n", s.gnss_total,
              s.accepted, s.rejected, s.numerical_failures);
  std::printf("    NIS  min %.4f  ort %.4f  maks %.4f\n", s.nis_min, s.nis_mean(), s.nis_max);
  std::printf("    son damga %lld ns\n", static_cast<long long>(s.final_stamp_ns));
}

/// Ortak saglik kontrolleri.
void sonlu_ve_simetrik(const SmokeSummary& s) {
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
  return static_cast<TimeNs>(static_cast<int>(kSure * kSaniye / kImuPeriyodu)) * kImuPeriyodu;
}

// -----------------------------------------------------------------------------
// 1. Gurultusuz wiring
// -----------------------------------------------------------------------------

TEST(EskfSmoke, NoiselessChainConvergesTowardGroundTruth) {
  const SmokeSummary s = kos(gurultusuz_config());
  yazdir("gurultusuz", s);

  sonlu_ve_simetrik(s);

  // Baslangic kestirimi gercek deger OLMAMALI.
  EXPECT_GT(s.initial_position_error, 1.0) << "baslangic kestirimi gercege cok yakin";
  EXPECT_GT(s.initial_velocity_error, 0.2);

  EXPECT_EQ(s.numerical_failures, 0);
  EXPECT_GT(s.accepted, 0) << "hic GNSS guncellemesi uygulanmadi";
  EXPECT_EQ(s.accepted + s.rejected + s.numerical_failures, s.gnss_total);
  EXPECT_GT(s.accepted, s.gnss_total * 9 / 10) << "kosunun anlamli kismi kabul edilmedi";

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
  const SmokeSummary s = kos(gurultulu_config());
  yazdir("gurultulu", s);

  sonlu_ve_simetrik(s);

  EXPECT_EQ(s.numerical_failures, 0) << "sayisal basarisizlik olmamali";
  EXPECT_EQ(s.accepted + s.rejected, s.gnss_total) << "kabul + red toplam GNSS sayisina esit degil";
  EXPECT_GT(s.gnss_total, 100) << "senaryo beklenen kadar GNSS uretmedi";

  // Makul kabul orani. Bu bir chi-kare kapsama IDDIASI DEGILDIR — tek
  // realizasyon, genis akil-sagligi siniri.
  const Scalar oran = static_cast<Scalar>(s.accepted) / static_cast<Scalar>(s.gnss_total);
  EXPECT_GT(oran, 0.85) << "kabul orani cok dusuk: kabul=" << s.accepted
                        << " toplam=" << s.gnss_total;

  ASSERT_GT(s.accepted + s.rejected, 0);
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
  const SmokeSummary temiz = kos(gurultusuz_config());
  const SmokeSummary gurultulu = kos(gurultulu_config());

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
  const TrajectorySample gt0 = sekiz().at(0);
  const NavState kestirim0 = gercek_durum(gt0).plus(baslangic_hatasi());
  EskfBackend backend(EskfConfig{kestirim0, NavCovariance::Zero(), 0, filtre_gurultusu(), kGuven});

  const DejenereOlcum z;
  const auto sonuc = backend.update(z);
  ASSERT_EQ(sonuc.status, UpdateStatus::kNumericalFailure);
  ASSERT_FALSE(sonuc.nis.has_value());

  SmokeSummary s;
  say(s, sonuc);

  EXPECT_EQ(s.numerical_failures, 1) << "sayisal basarisizlik ayri sayilmadi";
  EXPECT_EQ(s.accepted, 0) << "sayisal basarisizlik KABUL gibi sayildi";
  EXPECT_EQ(s.rejected, 0);
  EXPECT_EQ(s.gnss_total, 1);
  EXPECT_EQ(s.nis_sum, 0.0) << "NIS yokken toplama katki yapildi";
}

// -----------------------------------------------------------------------------
// 3. Determinizm
// -----------------------------------------------------------------------------

TEST(EskfSmoke, SameSeedsProduceIdenticalRuns) {
  const SmokeSummary a = kos(gurultulu_config());
  const SmokeSummary b = kos(gurultulu_config());

  // Ayni binary, ayni kod yolu, ayni girdi: bit-birebir esitlik beklenir.
  EXPECT_EQ(a.accepted, b.accepted);
  EXPECT_EQ(a.rejected, b.rejected);
  EXPECT_EQ(a.numerical_failures, b.numerical_failures);
  EXPECT_EQ(a.gnss_total, b.gnss_total);
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
  SmokeConfig farkli = gurultulu_config();
  farkli.gnss_tohumu = kGnssTohumu + 1;

  const SmokeSummary a = kos(gurultulu_config());
  const SmokeSummary b = kos(farkli);

  EXPECT_NE(a.nis_sum, b.nis_sum) << "tohum degisti ama NIS toplami ayni";
  EXPECT_GT(a.final_state.minus(b.final_state).norm(), 1e-6);
}

} // namespace
