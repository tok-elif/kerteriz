/// \file
/// KITTI kosucusu CLI — F1.8 Checkpoint B.
///
/// Yollar HARDCODE EDILMEZ; veri seti ve cikti CLI ile gelir:
///
///   kerteriz_kitti_runner --dataset <yol> --output <csv> [--reference <csv>]
///                         [--emit-baseline-params <yaml>]
///                         [--no-gnss-velocity] [--ate-max-dt-ms N]
///                         [--gnss-position-stride N]
///                         [--withheld-reference <csv>]
///                         [--sampling-manifest <csv>]
///
/// Gercek veri seti DEPOYA GIRMEZ ve cikti deponun disina yazilir.
///
/// SEYRELTME OPT-IN'DIR (F2.4-C). `--gnss-position-stride` verilmezse davranis
/// LEGACY'dir ve tam hizli sonuclar bit-birebir korunur.

#include "kerteriz_bringup/ate.hpp"
#include "kerteriz_bringup/kitti_oxts.hpp"
#include "kerteriz_bringup/kitti_runner.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace {

using kerteriz_bringup::DatasetEvent;
using kerteriz_bringup::KittiConfig;
using kerteriz_bringup::KittiRunnerConfig;
using kerteriz_bringup::TrajectoryRow;

int kullanim() {
  std::fprintf(stderr, "kullanim: kerteriz_kitti_runner --dataset <yol> --output <csv>\n"
                       "          [--reference <csv>] [--emit-baseline-params <yaml>]\n"
                       "          [--no-gnss-velocity] [--ate-max-dt-ms N]\n");
  return 2;
}

bool csv_yaz(const std::string& yol, const std::vector<TrajectoryRow>& satirlar) {
  std::ofstream f(yol);
  if (!f) {
    return false;
  }
  f << "timestamp_ns,px,py,pz,vx,vy,vz\n";
  f.setf(std::ios::fixed);
  f.precision(9);
  for (const auto& s : satirlar) {
    f << s.stamp_ns << ',' << s.position_w.x() << ',' << s.position_w.y() << ',' << s.position_w.z()
      << ',' << s.velocity_w.x() << ',' << s.velocity_w.y() << ',' << s.velocity_w.z() << '\n';
  }
  return f.good();
}

bool referans_yaz(const std::string& yol,
                  const std::vector<kerteriz_bringup::TrajectorySample>& ornekler) {
  std::ofstream f(yol);
  if (!f) {
    return false;
  }
  f << "timestamp_ns,px,py,pz\n";
  f.setf(std::ios::fixed);
  f.precision(9);
  for (const auto& s : ornekler) {
    f << s.stamp_ns << ',' << s.position_w.x() << ',' << s.position_w.y() << ',' << s.position_w.z()
      << '\n';
  }
  return f.good();
}

/// HARICI taban cizgisine verilecek baslangic parametreleri. Kerteriz'in
/// kullandigi AYNI `InitialState`'ten uretilir; iki tarafin baslangic bilgisi
/// elle senkronize edilen iki dosyaya degil TEK KOD YOLUNA dayanir.
///
/// robot_localization durum sirasi:
///   [x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw, ax ay az]
/// Dogrusal hizlar GOVDE cercevesindedir, bu yuzden `velocity_b` yazilir.
bool baslangic_parametreleri_yaz(const std::string& yol,
                                 const kerteriz_bringup::InitialState& init) {
  std::ofstream f(yol);
  if (!f) {
    return false;
  }
  f.setf(std::ios::fixed);
  f.precision(9);
  f << "# URETILMIS DOSYA — elle duzenlemeyin.\n"
    << "# kerteriz_kitti_runner --emit-baseline-params ile uretildi.\n"
    << "# Kerteriz'in baslatmasiyla AYNI kaynaktan gelir (veri setinin ilk\n"
    << "# ara-degerlenmemis OXTS kaydi).\n"
    << "ekf_filter_node:\n"
    << "  ros__parameters:\n"
    << "    initial_state: [" << init.position_w.x() << ", " << init.position_w.y() << ", "
    << init.position_w.z() << ", " << init.roll << ", " << init.pitch << ", " << init.yaw << ", "
    << init.velocity_b.x() << ", " << init.velocity_b.y() << ", " << init.velocity_b.z() << ", "
    << "0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  return f.good();
}

/// Seyreltme manifesti — hangi damganin hangi rolde oldugu denetlenebilsin.
bool manifest_yaz(const std::string& yol, const kerteriz_bringup::SamplingPlan& p) {
  std::ofstream f(yol);
  if (!f) {
    return false;
  }
  f << "timestamp_ns,role,ordinal\n";
  for (const auto& k : p.records) {
    f << k.stamp_ns << ',' << kerteriz_bringup::to_string(k.role) << ',' << k.ordinal << '\n';
  }
  return f.good();
}

void sensor_yaz(const char* ad, const kerteriz_bringup::SensorCounts& c) {
  std::printf("  %-14s toplam %d  ara-degerlenmis-atlanan %d  secilmeyen %d  verilen %d  "
              "kabul %d  chi-kare-kapisi %d  sayisal-basarisizlik %d  tampon-reddi %d\n",
              ad, c.total, c.skipped_interpolated, c.skipped_not_selected, c.submitted, c.accepted,
              c.chi_square_gated, c.numerical_failure, c.buffer_rejected);
}

} // namespace

int main(int argc, char** argv) {
  std::string veri_yolu;
  std::string cikti_yolu;
  std::string referans_yolu;
  std::string baseline_param_yolu;
  std::string withheld_yolu;
  std::string manifest_yolu;
  kerteriz::TimeNs ate_max_dt_ns = 20000000; // 20 ms

  KittiRunnerConfig kosucu;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto sonraki = [&]() -> std::string {
      return (i + 1 < argc) ? argv[++i] : std::string();
    };
    if (a == "--dataset") {
      veri_yolu = sonraki();
    } else if (a == "--output") {
      cikti_yolu = sonraki();
    } else if (a == "--reference") {
      referans_yolu = sonraki();
    } else if (a == "--emit-baseline-params") {
      baseline_param_yolu = sonraki();
    } else if (a == "--gnss-position-stride") {
      const std::string v = sonraki();
      kosucu.gnss_sampling.enabled = true;
      kosucu.gnss_sampling.stride = std::atoi(v.c_str());
      // Gecersiz deger SESSIZCE duzeltilmez: raporlanan sozlesme ile gercekte
      // kosulan sozlesme ayrisirdi.
      if (kosucu.gnss_sampling.stride <= 0) {
        std::fprintf(stderr, "--gnss-position-stride pozitif olmalidir: %s\n", v.c_str());
        return 2;
      }
    } else if (a == "--withheld-reference") {
      withheld_yolu = sonraki();
    } else if (a == "--sampling-manifest") {
      manifest_yolu = sonraki();
    } else if (a == "--no-gnss-velocity") {
      kosucu.use_gnss_velocity = false;
    } else if (a == "--ate-max-dt-ms") {
      ate_max_dt_ns = static_cast<kerteriz::TimeNs>(std::atoll(sonraki().c_str())) * 1000000;
    } else {
      std::fprintf(stderr, "bilinmeyen secenek: %s\n", a.c_str());
      return kullanim();
    }
  }
  if (veri_yolu.empty() || cikti_yolu.empty()) {
    return kullanim();
  }

  KittiConfig veri_cfg;
  veri_cfg.dataset_dir = veri_yolu;
  std::vector<DatasetEvent> olaylar;
  const auto ayristirma = kerteriz_bringup::load_kitti_oxts(veri_cfg, olaylar);
  if (!ayristirma.ok) {
    std::fprintf(stderr, "KITTI ayristirma hatasi: %s\n", ayristirma.message.c_str());
    return 1;
  }

  const auto sonuc = kerteriz_bringup::run_kitti(olaylar, kosucu);
  if (!sonuc.ok) {
    std::fprintf(stderr, "kosu hatasi: %s\n", sonuc.message.c_str());
    return 1;
  }
  if (!csv_yaz(cikti_yolu, sonuc.trajectory)) {
    std::fprintf(stderr, "cikti yazilamadi: %s\n", cikti_yolu.c_str());
    return 1;
  }
  if (!referans_yolu.empty() && !referans_yaz(referans_yolu, sonuc.reference)) {
    std::fprintf(stderr, "referans yazilamadi: %s\n", referans_yolu.c_str());
    return 1;
  }
  if (!withheld_yolu.empty()) {
    if (!kosucu.gnss_sampling.enabled) {
      std::fprintf(stderr, "--withheld-reference yalnizca --gnss-position-stride ile "
                           "anlamlidir (legacy modda withheld kume bostur)\n");
      return 2;
    }
    if (!referans_yaz(withheld_yolu, sonuc.withheld_reference)) {
      std::fprintf(stderr, "withheld referans yazilamadi: %s\n", withheld_yolu.c_str());
      return 1;
    }
  }
  if (!manifest_yolu.empty() && !manifest_yaz(manifest_yolu, sonuc.sampling)) {
    std::fprintf(stderr, "seyreltme manifesti yazilamadi: %s\n", manifest_yolu.c_str());
    return 1;
  }
  if (!baseline_param_yolu.empty() &&
      !baslangic_parametreleri_yaz(baseline_param_yolu, sonuc.init)) {
    std::fprintf(stderr, "taban cizgisi parametreleri yazilamadi: %s\n",
                 baseline_param_yolu.c_str());
    return 1;
  }

  const auto& s = sonuc.stats;
  std::printf("=== KITTI kosusu ===\n");
  std::printf("  girdi karesi           %d\n", s.input_frames);
  std::printf("  IMU olayi              %d  (veri setinin ara-degerledigi: %d)\n", s.imu_events,
              s.imu_interpolated);
  sensor_yaz("GNSS konum", s.gnss_position);
  sensor_yaz("GNSS hiz", s.gnss_velocity);
  std::printf("  referans               toplam %d  ATE'ye giren %d\n", s.reference_total,
              s.reference_used);
  if (kosucu.gnss_sampling.enabled) {
    const auto& p = sonuc.sampling;
    std::printf("  --- GNSS konum seyreltmesi (stride %d, ~%.2f Hz) ---\n",
                kosucu.gnss_sampling.stride,
                9.6554 / static_cast<double>(kosucu.gnss_sampling.stride));
    std::printf("    baslatma sonrasi aday       %d\n", p.candidate_count);
    std::printf("    secili slot                 %d\n", p.selected_slot_count);
    std::printf("    secili kullanilabilir olcum %d\n", p.selected_usable_count);
    std::printf("    secili ama ara-degerlenmis  %d\n", p.selected_interpolated_skipped);
    std::printf("    withheld referans           %d\n", p.withheld_reference_count);
  }
  std::printf("  baslatma damgasi       %lld\n", static_cast<long long>(s.init_stamp_ns));
  std::printf("  baslangic konumu       [%.3f %.3f %.3f]\n", sonuc.init.position_w.x(),
              sonuc.init.position_w.y(), sonuc.init.position_w.z());
  std::printf("  baslangic hizi (W)     [%.3f %.3f %.3f]  |v| %.3f m/s\n",
              sonuc.init.velocity_w.x(), sonuc.init.velocity_w.y(), sonuc.init.velocity_w.z(),
              sonuc.init.velocity_w.norm());
  std::printf("  baslangic hizi (B)     [%.3f %.3f %.3f]\n", sonuc.init.velocity_b.x(),
              sonuc.init.velocity_b.y(), sonuc.init.velocity_b.z());
  std::printf("  baslangic RPY          [%.4f %.4f %.4f] rad\n", sonuc.init.roll, sonuc.init.pitch,
              sonuc.init.yaw);
  std::printf("  son damga              %lld\n", static_cast<long long>(s.last_stamp_ns));
  std::printf("  yorunge satiri         %zu\n", sonuc.trajectory.size());
  std::printf("  son durum sonlu        %s\n", s.final_state_finite ? "evet" : "HAYIR");
  std::printf("  son kovaryans sonlu    %s\n", s.final_covariance_finite ? "evet" : "HAYIR");
  std::printf("  kovaryans simetri hata %.3e\n", s.final_covariance_symmetry_error);
  std::printf("  kovaryans min kosegen  %.6e\n", s.final_covariance_min_diagonal);

  std::vector<kerteriz_bringup::TrajectorySample> kestirim;
  kestirim.reserve(sonuc.trajectory.size());
  for (const auto& t : sonuc.trajectory) {
    kestirim.push_back(kerteriz_bringup::TrajectorySample{t.stamp_ns, t.position_w});
  }
  const auto ate = kerteriz_bringup::translational_ate(sonuc.reference, kestirim, ate_max_dt_ns);
  std::printf("=== ATE (oteleme, hizalama YOK) ===\n");
  if (!ate.ok) {
    std::printf("  eslesen ornek yok\n");
  } else {
    std::printf("  RMSE %.6f m   maks %.6f m   eslesen %d   eslesmeyen %d\n", ate.rmse_m,
                ate.max_error_m, ate.matched, ate.unmatched);
  }
  std::printf("yorunge: %s\n", cikti_yolu.c_str());
  return 0;
}
