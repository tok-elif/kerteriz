#pragma once

/// \file
/// Tekrar kullanilabilir sentetik deney kosucusu — F2.1.
///
/// ========================== NEDEN VAR =======================================
///
/// F1.5'in duman testi GERCEK uretim zincirini sentetik veriyle ucтan uca
/// kosturuyordu, ama senaryo ve dongu testin ICINDE hapsolmustu. Faz 2 ayni
/// deneyi N kez kosturup istatistik uretecek; dongunun ikinci bir kopyasini
/// yazmak iki kod yolunun zamanla ayrismasi demektir. Bu dosya o donguyu
/// TEK yere tasir; duman testi de artik BURAYI cagirir.
///
/// ========================== ZINCIR DEGISMEDI ================================
///
///   TrajectoryGenerator -> ImuSynthesizer  -> EskfBackend::predict
///                       -> GnssSynthesizer -> GnssPosition -> EskfBackend::update
///
/// Yeni filtre matematigi YOKTUR. Gercek deger HICBIR ZAMAN filtre durumundan
/// uretilmez: `TrajectoryGenerator` analitiktir ve filtreyi gormez.
///
/// ======================= BU DOSYA ISTATISTIK URETMEZ ========================
///
/// Burada NEES, NIS bandi veya kapsama HESAPLANMAZ (F2.2). Kayit tutulur,
/// yorum yapilmaz. Hata da burada hesaplanmaz: kayitlar gercek ve kestirim
/// durumlarini NavState olarak tasir, boylece F2.2 hatayi projenin KENDI sag
/// perturbasyon konvansiyonuyla (`NavState::minus`) alir — elde uydurulmus bir
/// Euler farkiyla degil.

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz_sim/gnss.hpp"
#include "kerteriz_sim/imu.hpp"
#include "kerteriz_sim/rng.hpp"
#include "kerteriz_sim/trajectory.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace kerteriz_sim {

using kerteriz::ImuNoiseParams;
using kerteriz::kCoreDof;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::SE23;
using kerteriz::StateVec;
using kerteriz::UpdateStatus;

inline constexpr TimeNs kNanosecondsPerSecond = 1000000000LL;

/// Cekirdek kovaryans blogu. TAM 63x63 `NavCovariance` KAYDEDILMEZ: ornek
/// basina ~32 KB tutar ve 500 realizasyonda gigabaytlara cikardi. Faz 2
/// senaryosunda augmentation yoktur, aktif duzen tam olarak cekirdektir.
using CoreCovariance = Eigen::Matrix<Scalar, kCoreDof, kCoreDof>;

// -----------------------------------------------------------------------------
// Senaryo
// -----------------------------------------------------------------------------

/// Deneyin TUM ayarlanabilir buyuklukleri. Varsayilanlar F1.5'in gurultulu
/// duman senaryosunu birebir uretir; sayilar API'yi sadelestirmek icin
/// DEGISTIRILMEDI.
struct ScenarioConfig {
  TrajectoryParams trajectory; ///< varsayilan: sekiz, r=10, w=0.2, h=1
  Scalar duration_s = Scalar(30);
  TimeNs imu_period_ns = kNanosecondsPerSecond / 100; ///< 100 Hz
  Scalar gnss_rate_hz = Scalar(5);

  ImuParams imu_sim;         ///< sentezleyicinin URETTIGI gurultu
  ImuNoiseParams imu_filter; ///< filtrenin VARSAYDIGI gurultu
  Scalar gnss_position_noise_std = Scalar(0.5);
  Vec3 gnss_lever_arm_b = Vec3::Zero();

  /// Baslangic hatasi politikasi.
  ///
  /// `kFixed`      : `initial_error` her realizasyonda AYNI. F1.5 duman
  ///                 senaryosunun davranisi budur ve VARSAYILANDIR.
  /// `kGaussianFromP0`: her realizasyon icin delta0 ~ N(0, P0_core) ornekle.
  ///                 Topluluk tabanli NEES yorumunun gerektirdigi budur —
  ///                 sabit bir sapma ile 500 kosu ayni onyargiyi tasir ve
  ///                 ANEES gercek kovaryans dogrulugunu olcmez.
  enum class InitialErrorPolicy { kFixed, kGaussianFromP0 };

  InitialErrorPolicy initial_error_policy = InitialErrorPolicy::kFixed;
  StateVec initial_error = StateVec::Zero(); ///< yalnizca kFixed; plus ile uygulanir
  NavCovariance initial_covariance = NavCovariance::Zero();
  Scalar chi2_confidence = Scalar(0.997);

  /// Zaman serisi kaydi. Tek kosuda acik olmasi ucuzdur; Monte Carlo'da
  /// varsayilan KAPALIDIR (bkz. monte_carlo.hpp).
  bool record_state_samples = true;
  /// Her kaciнci IMU adiminda ornek kaydedilecegi. 1 = hepsi.
  int state_sample_stride = 1;
};

/// Filtrenin varsaydigi IMU gurultusu (F1.5 ile AYNI sayilar).
///
/// Gurultusuz senaryoda bile kullanilir: yayilim birinci mertebedir ve sifir
/// surec gurultusu kovaryansi gercekci olmayan bicimde cokertirdi.
inline ImuNoiseParams default_filter_imu_noise() {
  ImuNoiseParams n;
  n.gyro_noise_density = 1.0e-3;  // rad/s/sqrt(Hz)
  n.gyro_random_walk = 1.0e-5;    // rad/s^2/sqrt(Hz)
  n.accel_noise_density = 2.0e-2; // m/s^2/sqrt(Hz)
  n.accel_random_walk = 3.0e-4;   // m/s^3/sqrt(Hz)
  return n;
}

/// Sentezleyici tarafinda AYNI fiziksel model.
inline ImuParams default_sim_imu_noise() {
  ImuParams p;
  p.gyro_noise_density = 1.0e-3;
  p.accel_noise_density = 2.0e-2;
  p.gyro_bias_walk = 1.0e-5;
  p.accel_bias_walk = 3.0e-4;
  return p;
}

/// Kasitli baslangic hatasi — yaw agirlikli ~2.9 derece, 1.87 m konum.
inline StateVec default_initial_error() {
  StateVec d = StateVec::Zero();
  d.segment<3>(0) = Vec3(0.010, -0.015, 0.050);  // dtheta
  d.segment<3>(3) = Vec3(0.30, -0.20, 0.10);     // dv
  d.segment<3>(6) = Vec3(1.50, -1.00, 0.50);     // dp
  d.segment<3>(9) = Vec3(0.002, -0.001, 0.003);  // db_g
  d.segment<3>(12) = Vec3(0.020, 0.030, -0.010); // db_a
  return d;
}

/// Baslangic kovaryansi gercek hatayi RAHATCA kapsar; ilk GNSS'in yalnizca
/// kotu ayar yuzunden kapiya takilmasi ISTENMEZ.
inline NavCovariance default_initial_covariance() {
  NavCovariance p = NavCovariance::Zero();
  p.block<3, 3>(0, 0).diagonal().setConstant(0.01);   // (0.1 rad)^2
  p.block<3, 3>(3, 3).diagonal().setConstant(0.25);   // (0.5 m/s)^2
  p.block<3, 3>(6, 6).diagonal().setConstant(9.0);    // (3 m)^2
  p.block<3, 3>(9, 9).diagonal().setConstant(1.0e-4); // (0.01 rad/s)^2
  p.block<3, 3>(12, 12).diagonal().setConstant(0.01); // (0.1 m/s^2)^2
  return p;
}

inline TrajectoryParams default_trajectory() {
  TrajectoryParams p;
  p.kind = TrajectoryKind::kFigureEight;
  p.radius = 10.0;
  p.angular_rate = 0.2;
  p.height = 1.0;
  return p;
}

/// F1.5'in GURULTULU duman senaryosu.
inline ScenarioConfig noisy_scenario() {
  ScenarioConfig c;
  c.trajectory = default_trajectory();
  c.imu_sim = default_sim_imu_noise();
  c.imu_filter = default_filter_imu_noise();
  c.gnss_position_noise_std = 0.5;
  c.initial_error = default_initial_error();
  c.initial_covariance = default_initial_covariance();
  return c;
}

/// F1.5'in GURULTUSUZ duman senaryosu: sentezleyici sifir gurultu uretir,
/// filtre yine de gercekci surec gurultusu varsayar.
inline ScenarioConfig noiseless_scenario() {
  ScenarioConfig c = noisy_scenario();
  c.imu_sim = ImuParams{}; // tum gurultu ve bias yuruyusu SIFIR
  c.gnss_position_noise_std = 0.0;
  return c;
}

// -----------------------------------------------------------------------------
// Kosum sonucu
// -----------------------------------------------------------------------------

/// IMU ve GNSS akislari AYRI tohumludur: GNSS ornek sayisi degisse bile IMU
/// gurultu dizisi degismez.
/// ALAN SIRASI SOZLESMEDIR. `initial_state` SONA eklenmistir: mevcut
/// `RunSeeds{imu, gnss}` bicimindeki toplu ilk-degerler anlamini korusun diye.
/// Basa eklendiginde bu tur her cagri sessizce kayar — nitekim ilk denemede
/// oyle oldu ve F1.5'in bit-birebir korumasi yakaladi. Yeni cagrilarda yine de
/// alan alan atama tercih edilmelidir.
struct RunSeeds {
  std::uint64_t imu = 0;
  std::uint64_t gnss = 0;
  /// Baslangic durumu hatasi. IMU/GNSS akislarindan BAGIMSIZDIR: baslangic
  /// ornekleme politikasini degistirmek olcum gurultu dizilerini KAYDIRMAMALI,
  /// yoksa iki degisiklik birbirine karisir ve kiyas anlamini yitirir.
  std::uint64_t initial_state = 0;
};

/// Tek bir andaki gercek/kestirim cifti. HATA BURADA HESAPLANMAZ — F2.2 onu
/// `NavState::minus` ile alir (CONVENTIONS §3.1 sag perturbasyon).
///
/// `truth` TAM 15 DoF'tur: poz/hiz/konumun yani sira jiro ve ivme bias'inin
/// GERCEK simule edilmis degerini de tasir. Bias'lar sifir birakilsaydi 15
/// boyutlu hata vektoru bilimsel olarak yanlis olurdu.
struct StateSample {
  TimeNs stamp_ns = 0;
  NavState truth;
  NavState estimate;
  CoreCovariance covariance = CoreCovariance::Zero(); ///< aktif 15x15 blok
  int active_dof = kCoreDof;
};

/// Tek bir guncellemenin NIS gozlemi. `dof` = residual_dim; chi-kare siniri
/// F2.2'de buradan kurulur.
struct NisObservation {
  TimeNs stamp_ns = 0;
  Scalar nis = 0;
  int dof = 0;
  UpdateStatus status = UpdateStatus::kAccepted;
  bool has_nis = false; ///< kNumericalFailure'da NIS TANIMSIZDIR (ADR-24)
};

struct RunCounters {
  int total = 0;
  int accepted = 0;
  int rejected = 0;
  int numerical_failures = 0;
};

struct RunResult {
  int run_index = 0;
  RunSeeds seeds;
  RunCounters counters;

  /// Bu realizasyonda GERCEKTEN uygulanan baslangic hatasi. Uretim deneyi
  /// toplulugu boylece denetlenebilir: ornekleyici P0'i dogru temsil ediyor mu
  /// sorusu, filtrenin sonucuna bakmadan yanitlanabilir.
  StateVec initial_error_used = StateVec::Zero();
  bool ok = true;              ///< ornekleme veya kosum basarili mi
  std::string failure_message; ///< bos degilse sebep

  Scalar initial_position_error = 0;
  Scalar final_position_error = 0;
  Scalar initial_velocity_error = 0;
  Scalar final_velocity_error = 0;

  Scalar nis_sum = 0;
  Scalar nis_min = std::numeric_limits<Scalar>::infinity();
  Scalar nis_max = -std::numeric_limits<Scalar>::infinity();

  TimeNs final_stamp_ns = 0;
  NavState final_state;
  NavCovariance final_covariance = NavCovariance::Zero();

  std::vector<StateSample> samples;
  std::vector<NisObservation> nis_observations;

  Scalar nis_mean() const {
    const int n = counters.accepted + counters.rejected;
    return n > 0 ? nis_sum / static_cast<Scalar>(n) : Scalar(0);
  }
};

/// Guncelleme sonucunu sayaclara isler.
///
/// AYRI bir fonksiyondur ki sayim mantigi dogrudan sinanabilsin: gercek
/// zincirde `GnssPosition` ile S = J P J^T + R HER ZAMAN pozitif tanimlidir,
/// yani sayisal basarisizlik dogal olarak tetiklenemez. Yanlis siniflandirma
/// ancak bu fonksiyon tek basina sinanarak yakalanabilir.
inline void record_update(RunResult& r, TimeNs stamp_ns, const kerteriz::UpdateResult& sonuc) {
  ++r.counters.total;
  switch (sonuc.status) {
  case UpdateStatus::kAccepted:
    ++r.counters.accepted;
    break;
  case UpdateStatus::kChiSquareRejected:
    ++r.counters.rejected;
    break;
  case UpdateStatus::kNumericalFailure:
    ++r.counters.numerical_failures;
    break;
  }

  NisObservation g;
  g.stamp_ns = stamp_ns;
  g.dof = sonuc.dof;
  g.status = sonuc.status;
  g.has_nis = sonuc.nis.has_value();
  if (g.has_nis) {
    g.nis = *sonuc.nis;
    r.nis_sum += g.nis;
    r.nis_min = std::min(r.nis_min, g.nis);
    r.nis_max = std::max(r.nis_max, g.nis);
  }
  r.nis_observations.push_back(g);
}

/// Gercek durumdan NavState.
///
/// BIAS'LAR ZORUNLU PARAMETREDIR, varsayilanlari YOKTUR. Gercek durum 15
/// serbestlik derecelidir ve jiro/ivme bias'i o 15'in altisidir; sifir birakmak
/// F2.2'nin `estimate.minus(truth)` ile kuracagi hata vektorunu bilimsel olarak
/// YANLIS yapardi. Varsayilan deger konsaydi bir cagri yerinde unutmak sessizce
/// mumkun olurdu — imza bunu imkansiz kilar.
///
/// Bias'in KAYNAGI `ImuSynthesizer`'dir. Rastgele yuruyus burada YENIDEN
/// KURULMAZ; sentezleyicinin o andaki degeri okunur (bkz. run_single).
///
/// KUATERNIYON ACIKCA NORMALLESTIRILIR. Yorunge rotasyon MATRISI uretir; ondan
/// turetilen kuaterniyon birim normdan ~1e-16 sapabilir ve manif bunu reddeder.
inline NavState truth_state(const TrajectorySample& gt, const Vec3& gyro_bias,
                            const Vec3& accel_bias) {
  Eigen::Quaternion<Scalar> q(gt.rotation);
  q.normalize();
  NavState x;
  x.extended_pose() = SE23(gt.position, q, gt.velocity);
  x.gyro_bias() = gyro_bias;
  x.accel_bias() = accel_bias;
  return x;
}

/// P0'in aktif 15x15 blogundan sifir ortalamali bir teget ornegi.
///
///   P0 = L L^T,  delta0 = L z,  z ~ N(0, I_15)
///
/// Euler acilari AYRI ornekLENMEZ ve NavState::plus disinda bir yonelim hatasi
/// KURULMAZ: ornek dogrudan projenin teget sirasindadir
/// [dtheta, dv, dp, db_g, db_a] ve 15'ten sonraki kuyruk SIFIR kalir.
///
/// P0'in acik tersi ALINMAZ. P0 sonlu degilse, maddeten simetrik degilse veya
/// pozitif tanimli degilse ornekleme ACIKCA BASARISIZ olur: titresim eklenmez,
/// ozdeger kirpilmaz, sessizce simetriklestirilmez. Bu deney YAPILANDIRMASIDIR;
/// gorunmez bicimde onarilacak bir sey degildir.
struct InitialSample {
  bool ok = false;
  std::string message;
  StateVec delta = StateVec::Zero();
};

inline InitialSample sample_initial_error(const NavCovariance& p0, SeededRng& rng,
                                          Scalar symmetry_tolerance = Scalar(1e-9)) {
  InitialSample out;
  const Eigen::Matrix<Scalar, kCoreDof, kCoreDof> pc = p0.topLeftCorner<kCoreDof, kCoreDof>();

  if (!pc.allFinite()) {
    out.message = "P0 sonlu degil";
    return out;
  }
  const Scalar olcek = std::max(Scalar(1), pc.cwiseAbs().maxCoeff());
  if ((pc - pc.transpose()).cwiseAbs().maxCoeff() > symmetry_tolerance * olcek) {
    out.message = "P0 maddeten simetrik degil";
    return out;
  }
  const Eigen::LLT<Eigen::Matrix<Scalar, kCoreDof, kCoreDof>> llt(pc);
  if (llt.info() != Eigen::Success) {
    out.message = "P0 pozitif tanimli degil";
    return out;
  }
  const auto l = llt.matrixL();
  // Kosegenin KESIN pozitifligi ayrica zorlanir: LLT tekil bir matriste de
  // Success dondurebilir ve sifir pivot sessizce gecerdi.
  for (int i = 0; i < kCoreDof; ++i) {
    if (!(l(i, i) > Scalar(0))) {
      out.message = "P0 pozitif tanimli degil (sifir pivot)";
      return out;
    }
  }

  Eigen::Matrix<Scalar, kCoreDof, 1> z;
  for (int i = 0; i < kCoreDof; ++i) {
    z[i] = rng.gaussian();
  }
  out.delta.setZero(); // 15'ten sonraki kuyruk SIFIR
  out.delta.head<kCoreDof>() = l * z;
  out.ok = true;
  return out;
}

/// Tek realizasyon. Sentezleyici -> GERCEK backend -> kayit.
inline RunResult run_single(const ScenarioConfig& cfg, const RunSeeds& seeds, int run_index = 0) {
  assert(cfg.imu_period_ns > 0 && "IMU periyodu pozitif olmali");
  assert(cfg.state_sample_stride >= 1 && "ornek adimi en az 1 olmali");

  TrajectoryGenerator yorunge(cfg.trajectory);

  // AYRI RNG nesneleri ve AYRI tohumlar.
  SeededRng imu_rng(seeds.imu);
  SeededRng gnss_rng(seeds.gnss);
  ImuSynthesizer imu_sentez(cfg.imu_sim, imu_rng);
  GnssSynthesizer gnss_sentez(GnssParams{cfg.gnss_position_noise_std, cfg.gnss_rate_hz}, gnss_rng);

  // t = 0'daki gercek durum, sentezleyicinin BASLANGIC bias'larini tasir;
  // baslangic hatasi bunun UZERINE uygulanir. Sifir varsayilsaydi, sifirdan
  // farkli bir baslangic bias'i yapilandirildiginda kestirim gercek degerden
  // ongorulmeyen bir kadar sapardi.
  const TrajectorySample gt0 = yorunge.at(0);
  const NavState gercek0 =
      truth_state(gt0, cfg.imu_sim.initial_gyro_bias, cfg.imu_sim.initial_accel_bias);

  RunResult r;
  r.run_index = run_index;
  r.seeds = seeds;

  StateVec baslangic_hatasi = cfg.initial_error;
  if (cfg.initial_error_policy == ScenarioConfig::InitialErrorPolicy::kGaussianFromP0) {
    // UCUNCU, BAGIMSIZ akis. kFixed bu akisi HIC TUKETMEZ, bu yuzden F1.5'in
    // acik IMU/GNSS tohumlari birebir ayni ciktiyi vermeye devam eder.
    SeededRng baslangic_rng(seeds.initial_state);
    const InitialSample ornek = sample_initial_error(cfg.initial_covariance, baslangic_rng);
    if (!ornek.ok) {
      r.ok = false;
      r.failure_message = ornek.message;
      return r;
    }
    baslangic_hatasi = ornek.delta;
  }
  r.initial_error_used = baslangic_hatasi;

  const NavState kestirim0 = gercek0.plus(baslangic_hatasi);
  r.initial_position_error = (kestirim0.extended_pose().translation() - gt0.position).norm();
  r.initial_velocity_error = (kestirim0.extended_pose().linearVelocity() - gt0.velocity).norm();

  kerteriz::EskfBackend backend(kerteriz::EskfConfig{kestirim0, cfg.initial_covariance, 0,
                                                     cfg.imu_filter, cfg.chi2_confidence});

  const TimeNs gnss_periyodu = gnss_sentez.period_ns();
  const Eigen::Matrix3d gnss_kovaryansi =
      Eigen::Matrix3d::Identity() * (cfg.gnss_position_noise_std * cfg.gnss_position_noise_std);

  // Donusumler ACIK: TimeNs -> Scalar zimni daralmasi birakilmaz. Islem sirasi
  // F1.5'teki ile AYNIDIR, dolayisiyla adim sayisi da aynidir.
  const auto adim_sayisi =
      static_cast<int>(cfg.duration_s * static_cast<Scalar>(kNanosecondsPerSecond) /
                       static_cast<Scalar>(cfg.imu_period_ns));

  // Sicak dongu icinde yeniden tahsis olmasin diye kapasite ONCEDEN ayrilir.
  if (cfg.record_state_samples) {
    r.samples.reserve(static_cast<std::size_t>(adim_sayisi) /
                          static_cast<std::size_t>(cfg.state_sample_stride) +
                      1U);
  }
  if (gnss_periyodu > 0) {
    const TimeNs toplam_ns = static_cast<TimeNs>(adim_sayisi) * cfg.imu_period_ns;
    r.nis_observations.reserve(static_cast<std::size_t>(toplam_ns / gnss_periyodu) + 1U);
  }

  TimeNs onceki_ns = 0;

  for (int k = 1; k <= adim_sayisi; ++k) {
    // MUTLAK ZAMAN TAMSAYI ADIMLARDAN URETILIR; dt biriktirilmez.
    const TimeNs t = static_cast<TimeNs>(k) * cfg.imu_period_ns;
    const Scalar dt = static_cast<Scalar>(t - onceki_ns) * 1e-9;
    const TrajectorySample gt = yorunge.at(t);

    backend.predict(imu_sentez.sample(gt, dt), dt);
    onceki_ns = t;

    if (gnss_periyodu > 0 && t % gnss_periyodu == 0) {
      const auto olcum = gnss_sentez.sample(gt);
      const kerteriz::GnssPosition z(olcum.stamp_ns, "gnss_main", olcum.position, gnss_kovaryansi,
                                     cfg.gnss_lever_arm_b);
      record_update(r, t, backend.update(z));
    }

    if (cfg.record_state_samples && (k % cfg.state_sample_stride) == 0) {
      StateSample s;
      s.stamp_ns = t;
      // SIRA ONEMLIDIR: `ImuSynthesizer::sample()` bias'i olcumu uretmeden ONCE
      // ilerletir. Kayit `sample()` cagrisindan SONRA yapildigi icin buradaki
      // deger, tam olarak BU damganin olcumunu ureten bias'tir. Once okunsaydi
      // bir adim geride kalirdi.
      s.truth = truth_state(gt, imu_sentez.gyro_bias(), imu_sentez.accel_bias());
      s.estimate = backend.state();
      s.active_dof = backend.state().active_dof();
      // Faz 2 senaryosunda augmentation yoktur; aktif duzen cekirdektir.
      assert(s.active_dof == kCoreDof && "Faz 2 senaryosu augmentation beklemiyor");
      s.covariance = backend.covariance().topLeftCorner<kCoreDof, kCoreDof>();
      r.samples.push_back(std::move(s));
    }
  }

  const TrajectorySample gt_son = yorunge.at(onceki_ns);
  r.final_position_error = (backend.state().extended_pose().translation() - gt_son.position).norm();
  r.final_velocity_error =
      (backend.state().extended_pose().linearVelocity() - gt_son.velocity).norm();
  r.final_stamp_ns = backend.stamp_ns();
  r.final_state = backend.state();
  r.final_covariance = backend.covariance();
  return r;
}

} // namespace kerteriz_sim
