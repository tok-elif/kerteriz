/// \file
/// E1 — Faz 2 ESKF tutarlilik deneyi CLI'si.
///
/// CIZIM YAPMAZ. Yalnizca CSV + manifest uretir ve ham ozeti yazdirir; sekil
/// `kerteriz_eval` tarafindadir. Genel bir deney cercevesi de KURULMAZ —
/// `make results` sarmalayicisi F2.5'tir.
///
/// Zincir uretim kodudur:
///   TrajectoryGenerator -> ImuSynthesizer -> EskfBackend::predict
///                       -> GnssSynthesizer -> GnssPosition -> EskfBackend::update
///                       -> F2.1 kayitlari -> F2.2 NEES/NIS toplamasi

#include "kerteriz_sim/consistency.hpp"
#include "kerteriz_sim/e1_scenario.hpp"

#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

namespace {

using kerteriz::kCoreDof;
using kerteriz::Scalar;
using kerteriz::StateVec;
using kerteriz_sim::MonteCarloResult;

using CoreVec = Eigen::Matrix<Scalar, kCoreDof, 1>;
using CoreMat = Eigen::Matrix<Scalar, kCoreDof, kCoreDof>;

const char* kTangentAdlari[kCoreDof] = {"dtheta_x", "dtheta_y", "dtheta_z(yaw)", "dv_x",  "dv_y",
                                        "dv_z",     "dp_x",     "dp_y",          "dp_z",  "dbg_x",
                                        "dbg_y",    "dbg_z",    "dba_x",         "dba_y", "dba_z"};

/// Baslangic toplulugunun DENETIMI. Filtre sonucuna bakmadan, ornekleyicinin
/// P0'i dogru temsil edip etmedigini sorar.
struct EnsembleAudit {
  int count = 0;
  CoreVec mean = CoreVec::Zero();
  CoreMat covariance = CoreMat::Zero();
  Scalar mean_norm = 0;
  Scalar yaw_std = 0;
  Scalar aq0 = 0; ///< ortalama delta0^T P0^-1 delta0, beklenen merkez 15
  bool ok = false;
};

EnsembleAudit denetle(const MonteCarloResult& mc, const kerteriz::NavCovariance& p0) {
  EnsembleAudit a;
  std::vector<CoreVec> d;
  d.reserve(mc.runs.size());
  for (const auto& r : mc.runs) {
    d.push_back(r.initial_error_used.head<kCoreDof>());
  }
  a.count = static_cast<int>(d.size());
  if (a.count < 2) {
    return a;
  }
  for (const auto& v : d) {
    a.mean += v;
  }
  a.mean /= static_cast<Scalar>(a.count);
  a.mean_norm = a.mean.norm();

  for (const auto& v : d) {
    const CoreVec c = v - a.mean;
    a.covariance += c * c.transpose();
  }
  a.covariance /= static_cast<Scalar>(a.count - 1); // ornek kovaryansi
  a.yaw_std = std::sqrt(a.covariance(2, 2));

  // ACIK TERS ALINMAZ: LLT cozumu.
  const CoreMat pc = p0.topLeftCorner<kCoreDof, kCoreDof>();
  const Eigen::LLT<CoreMat> llt(pc);
  if (llt.info() != Eigen::Success) {
    return a;
  }
  Scalar toplam = 0;
  for (const auto& v : d) {
    toplam += v.dot(llt.solve(v));
  }
  a.aq0 = toplam / static_cast<Scalar>(a.count);
  a.ok = true;
  return a;
}

struct SeriesSummary {
  Scalar min_v = 0, median_v = 0, max_v = 0;
  int below = 0, inside = 0, above = 0;
  int longest_above = 0;
};

template <typename Nokta, typename Deger>
SeriesSummary ozetle(const std::vector<Nokta>& p, Deger deger) {
  SeriesSummary s;
  if (p.empty()) {
    return s;
  }
  std::vector<Scalar> v;
  v.reserve(p.size());
  int seri = 0;
  for (const auto& n : p) {
    const Scalar x = deger(n);
    v.push_back(x);
    if (x < n.lower_95) {
      ++s.below;
      seri = 0;
    } else if (x > n.upper_95) {
      ++s.above;
      ++seri;
      s.longest_above = std::max(s.longest_above, seri);
    } else {
      ++s.inside;
      seri = 0;
    }
  }
  const auto sinirlar = std::minmax_element(v.begin(), v.end());
  s.min_v = *sinirlar.first;
  s.max_v = *sinirlar.second;
  s.median_v = kerteriz_sim::median(v); // cift N'de orta IKI degerin ortalamasi
  return s;
}

bool nees_csv_yaz(const std::string& yol, const kerteriz_sim::NeesSeries& s) {
  std::ofstream f(yol);
  if (!f) {
    return false;
  }
  f << "timestamp_ns,time_s,mean_nees,expected_mean,lower_95,upper_95,valid_count,invalid_count\n";
  f.setf(std::ios::fixed);
  f.precision(9);
  for (const auto& p : s.points) {
    f << p.stamp_ns << ',' << static_cast<Scalar>(p.stamp_ns) * 1e-9 << ',' << p.mean_nees << ','
      << p.expected_mean << ',' << p.lower_95 << ',' << p.upper_95 << ',' << p.valid_count << ','
      << p.invalid_count << '\n';
  }
  return f.good();
}

bool nis_csv_yaz(const std::string& yol, const kerteriz_sim::NisSeries& s) {
  std::ofstream f(yol);
  if (!f) {
    return false;
  }
  f << "timestamp_ns,time_s,mean_nis,expected_mean,lower_95,upper_95,valid_count,"
       "undefined_count,total_dof\n";
  f.setf(std::ios::fixed);
  f.precision(9);
  for (const auto& p : s.points) {
    f << p.stamp_ns << ',' << static_cast<Scalar>(p.stamp_ns) * 1e-9 << ',' << p.mean_nis << ','
      << p.expected_mean << ',' << p.lower_95 << ',' << p.upper_95 << ',' << p.valid_count << ','
      << p.undefined_count << ',' << p.total_dof << '\n';
  }
  return f.good();
}

} // namespace

int main(int argc, char** argv) {
  std::string cikti_dizini = "results/e1";
  std::string git_commit = "bilinmiyor";
  int kosu_sayisi = kerteriz_sim::e1::kRuns;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto sonraki = [&]() -> std::string {
      return (i + 1 < argc) ? argv[++i] : std::string();
    };
    if (a == "--output-dir") {
      cikti_dizini = sonraki();
    } else if (a == "--git-commit") {
      git_commit = sonraki();
    } else if (a == "--runs") {
      kosu_sayisi = std::atoi(sonraki().c_str());
    } else {
      std::fprintf(stderr,
                   "kullanim: kerteriz_e1_eskf [--output-dir <dizin>] [--git-commit <sha>]\n"
                   "                           [--runs N]\n");
      return 2;
    }
  }

  kerteriz_sim::MonteCarloConfig cfg = kerteriz_sim::e1_eskf_monte_carlo();
  cfg.runs = kosu_sayisi;

  const auto t0 = std::chrono::steady_clock::now();
  const MonteCarloResult mc = kerteriz_sim::run_monte_carlo(cfg);
  const Scalar sure_s =
      std::chrono::duration<Scalar>(std::chrono::steady_clock::now() - t0).count();

  const auto nees = kerteriz_sim::aggregate_nees(mc, kerteriz_sim::e1::kConsistencyConfidence);
  const auto nis = kerteriz_sim::aggregate_nis(mc, kerteriz_sim::e1::kConsistencyConfidence);
  if (!nees.status.ok) {
    std::fprintf(stderr, "NEES toplama hatasi: %s\n", nees.status.message.c_str());
    return 1;
  }
  if (!nis.status.ok) {
    std::fprintf(stderr, "NIS toplama hatasi: %s\n", nis.status.message.c_str());
    return 1;
  }

  const auto denetim = denetle(mc, cfg.scenario.initial_covariance);
  const auto q0_band =
      kerteriz_sim::anees_band(denetim.count, kCoreDof, kerteriz_sim::e1::kConsistencyConfidence);

  int kabul = 0, red = 0, sayisal = 0, toplam_guncelleme = 0;
  for (const auto& r : mc.runs) {
    kabul += r.counters.accepted;
    red += r.counters.rejected;
    sayisal += r.counters.numerical_failures;
    toplam_guncelleme += r.counters.total;
  }

  const auto nees_ozet = ozetle(nees.points, [](const auto& p) { return p.mean_nees; });
  const auto nis_ozet = ozetle(nis.points, [](const auto& p) { return p.mean_nis; });

  const std::string nees_yolu = cikti_dizini + "/phase2_eskf_anees.csv";
  const std::string nis_yolu = cikti_dizini + "/phase2_eskf_anis.csv";
  const std::string manifest_yolu = cikti_dizini + "/phase2_eskf_manifest.txt";
  if (!nees_csv_yaz(nees_yolu, nees) || !nis_csv_yaz(nis_yolu, nis)) {
    std::fprintf(stderr, "CSV yazilamadi — dizin var mi? %s\n", cikti_dizini.c_str());
    return 1;
  }

  // ---------------------------------------------------------------------------
  // Manifest — deneyi yeniden uretmeye yetecek kadar.
  // ---------------------------------------------------------------------------
  {
    std::ofstream m(manifest_yolu);
    if (!m) {
      std::fprintf(stderr, "manifest yazilamadi\n");
      return 1;
    }
    const auto& sc = cfg.scenario;
    m << std::setprecision(17);
    m << "# E1 — Faz 2 ESKF tutarlilik deneyi\n"
      << "# URETILMIS DOSYA. Sonuc gorulduKTEN SONRA parametre degistirilmez.\n"
      << "#\n"
      << "# KAPSAM: SPEC §8'deki nihai E1 IKI BACKEND kiyasidir. Bugun yalnizca\n"
      << "# ESKF vardir; bu dosya FAZ 2 ESKF TABAN CIZGISIDIR. InEKF Faz 3'te\n"
      << "# AYNI sozlesmeden gecirilecektir.\n\n";
    m << "estimator = ESKF\n";
    m << "runs = " << mc.runs.size() << "\n";
    m << "master_seed = " << cfg.master_seed << "\n";
    m << "git_commit = " << git_commit << "\n\n";

    m << "[senaryo]\n";
    m << "trajectory = figure_eight\n";
    m << "trajectory_radius_m = " << sc.trajectory.radius << "\n";
    m << "trajectory_angular_rate_rad_s = " << sc.trajectory.angular_rate << "\n";
    m << "trajectory_height_m = " << sc.trajectory.height << "\n";
    m << "duration_s = " << sc.duration_s << "\n";
    m << "imu_period_ns = " << sc.imu_period_ns << "\n";
    m << "imu_rate_hz = " << 1e9 / static_cast<Scalar>(sc.imu_period_ns) << "\n";
    m << "gnss_rate_hz = " << sc.gnss_rate_hz << "\n";
    m << "state_sample_stride = " << cfg.state_sample_stride << "\n\n";

    m << "[simulasyon gurultusu]\n";
    m << "sim_gyro_noise_density = " << sc.imu_sim.gyro_noise_density << "\n";
    m << "sim_accel_noise_density = " << sc.imu_sim.accel_noise_density << "\n";
    m << "sim_gyro_bias_walk = " << sc.imu_sim.gyro_bias_walk << "\n";
    m << "sim_accel_bias_walk = " << sc.imu_sim.accel_bias_walk << "\n";
    m << "sim_initial_gyro_bias = " << sc.imu_sim.initial_gyro_bias.transpose() << "\n";
    m << "sim_initial_accel_bias = " << sc.imu_sim.initial_accel_bias.transpose() << "\n";
    m << "gnss_position_noise_std_m = " << sc.gnss_position_noise_std << "\n";
    m << "gnss_lever_arm_b = " << sc.gnss_lever_arm_b.transpose() << "\n\n";

    m << "[filtrenin varsaydigi gurultu]\n";
    m << "filter_gyro_noise_density = " << sc.imu_filter.gyro_noise_density << "\n";
    m << "filter_gyro_random_walk = " << sc.imu_filter.gyro_random_walk << "\n";
    m << "filter_accel_noise_density = " << sc.imu_filter.accel_noise_density << "\n";
    m << "filter_accel_random_walk = " << sc.imu_filter.accel_random_walk << "\n";
    m << "gate_chi2_confidence = " << sc.chi2_confidence << "\n";
    m << "consistency_confidence = " << kerteriz_sim::e1::kConsistencyConfidence << "\n\n";

    m << "[baslangic belirsizligi]\n";
    m << "initial_error_policy = GaussianFromP0\n";
    m << "initial_error_mean = zero\n";
    m << "yaw_sigma_rad = " << kerteriz_sim::e1::kYawSigmaRad << "\n";
    m << "yaw_sigma_deg = " << kerteriz_sim::e1::kYawSigmaRad * 180.0 / 3.14159265358979323846
      << "\n";
    m << "roll_pitch_sigma_rad = " << kerteriz_sim::e1::kRollPitchSigmaRad << "\n";
    for (int i = 0; i < kCoreDof; ++i) {
      m << "P0_diag[" << i << "] " << kTangentAdlari[i] << " = " << sc.initial_covariance(i, i)
        << "\n";
    }
    m << "\n";

    m << "[baslangic toplulugu denetimi]\n";
    m << "ensemble_count = " << denetim.count << "\n";
    m << "empirical_mean_norm = " << denetim.mean_norm << "\n";
    m << "empirical_yaw_std_rad = " << denetim.yaw_std << "\n";
    m << "aq0_mean = " << denetim.aq0 << "  # beklenen merkez " << kCoreDof << "\n";
    m << "aq0_band_95 = [" << q0_band.lower << ", " << q0_band.upper << "]\n";
    for (int i = 0; i < kCoreDof; ++i) {
      m << "emp_cov_diag[" << i << "] " << kTangentAdlari[i] << " = " << denetim.covariance(i, i)
        << "  (P0 " << sc.initial_covariance(i, i) << ")\n";
    }
    m << "\n";

    m << "[sonuc sayilari]\n";
    m << "runtime_s = " << sure_s << "\n";
    m << "failed_runs = " << mc.failed_runs << "\n";
    m << "gnss_updates_total = " << toplam_guncelleme << "\n";
    m << "gnss_accepted = " << kabul << "\n";
    m << "gnss_chi_square_rejected = " << red << "\n";
    m << "numerical_failures = " << sayisal << "\n";
    m << "runs_with_numerical_failure = " << mc.runs_with_numerical_failure << "\n";
    m << "nees_timestamps = " << nees.points.size() << "\n";
    m << "nees_invalid_total = " << nees.invalid_total << "\n";
    m << "nis_timestamps = " << nis.points.size() << "\n";
    m << "nis_undefined_total = " << nis.undefined_total << "\n\n";

    m << "[komutlar]\n";
    m << "# depo kokunden, goreli yollarla\n";
    m << "cmake -S kerteriz_sim -B build/sim -DCMAKE_BUILD_TYPE=Release\n";
    m << "cmake --build build/sim -j\n";
    m << "./build/sim/kerteriz_e1_eskf --output-dir results/e1 --git-commit <sha>\n";
    m << "python3 -m kerteriz_eval.e1_plot --input-dir results/e1\n";
  }

  // ---------------------------------------------------------------------------
  // Ham ozet
  // ---------------------------------------------------------------------------
  std::printf("=== E1 · Faz 2 ESKF tutarlilik deneyi ===\n");
  std::printf("  kosu %zu   ana tohum %llu   sure %.2f s   basarisiz kosu %d\n", mc.runs.size(),
              static_cast<unsigned long long>(cfg.master_seed), sure_s, mc.failed_runs);
  std::printf("  GNSS toplam %d  kabul %d  chi-kare red %d  sayisal basarisizlik %d "
              "(kosu %d)\n",
              toplam_guncelleme, kabul, red, sayisal, mc.runs_with_numerical_failure);
  std::printf("  NEES damga %zu  gecersiz %d    NIS damga %zu  tanimsiz %d\n", nees.points.size(),
              nees.invalid_total, nis.points.size(), nis.undefined_total);

  std::printf("\n--- baslangic toplulugu denetimi (ORNEKLEYICI, filtre degil) ---\n");
  std::printf("  ornek %d   ortalama normu %.6f   yaw ampirik std %.6f rad (P0 %.6f)\n",
              denetim.count, denetim.mean_norm, denetim.yaw_std,
              std::sqrt(cfg.scenario.initial_covariance(2, 2)));
  std::printf("  AQ0 %.4f   beklenen %d   band [%.4f, %.4f]\n", denetim.aq0, kCoreDof,
              q0_band.lower, q0_band.upper);
  std::printf("  ampirik kovaryans kosegeni / P0 kosegeni:\n");
  for (int i = 0; i < kCoreDof; ++i) {
    std::printf("    %-14s %12.6g / %12.6g   oran %.4f\n", kTangentAdlari[i],
                denetim.covariance(i, i), cfg.scenario.initial_covariance(i, i),
                denetim.covariance(i, i) / cfg.scenario.initial_covariance(i, i));
  }

  const auto bant = [](const auto& p) {
    std::printf("  band [%.4f, %.4f]   beklenen %.4f\n", p.lower_95, p.upper_95, p.expected_mean);
  };
  std::printf("\n--- ANEES ---\n");
  if (!nees.points.empty()) {
    bant(nees.points.front());
  }
  std::printf("  min %.4f  medyan %.4f  maks %.4f\n", nees_ozet.min_v, nees_ozet.median_v,
              nees_ozet.max_v);
  std::printf("  bandin ALTINDA %d   ICINDE %d   USTUNDE %d   (toplam %zu damga)\n",
              nees_ozet.below, nees_ozet.inside, nees_ozet.above, nees.points.size());
  std::printf("  en uzun kesintisiz UST-ASIM %d damga\n", nees_ozet.longest_above);

  std::printf("\n--- ANIS ---\n");
  if (!nis.points.empty()) {
    bant(nis.points.front());
  }
  std::printf("  min %.4f  medyan %.4f  maks %.4f\n", nis_ozet.min_v, nis_ozet.median_v,
              nis_ozet.max_v);
  std::printf("  bandin ALTINDA %d   ICINDE %d   USTUNDE %d   (toplam %zu damga)\n", nis_ozet.below,
              nis_ozet.inside, nis_ozet.above, nis.points.size());
  std::printf("  en uzun kesintisiz UST-ASIM %d damga\n", nis_ozet.longest_above);

  std::printf("\nNOT: ardisik damgalar KORELEDIR. Yukaridaki 'icinde' sayisi bir kapsama\n"
              "olasiligi veya bagimsiz hipotez testi orani DEGILDIR; betimleyici bir\n"
              "zaman serisi ozetidir.\n");
  std::printf("cikti: %s  %s  %s\n", nees_yolu.c_str(), nis_yolu.c_str(), manifest_yolu.c_str());
  return 0;
}
