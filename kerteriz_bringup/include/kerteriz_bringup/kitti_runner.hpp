#pragma once

/// \file
/// KITTI veri setini MEVCUT uretim zincirinden gecirir — F1.8 Checkpoint B.
///
/// ============================ ZORUNLU ZINCIR ================================
///
///   KITTI adaptoru -> DatasetEvent -> MeasurementBuffer -> Estimator
///                  -> EskfBackend  -> yorunge
///
/// Alternatif bir EKF, veri setine ozel guncelleme denklemi, baslatmadan sonra
/// dogrudan durum yazma veya cevrimdisi kestirme yolu YOKTUR. Olcumler mevcut
/// `GnssPosition` / `GnssVelocity` kuruculariyla uretilir.
///
/// ====================== VERI SETINE OZEL BASLATMA ===========================
///
/// Baslatma veri setine OZELDIR ve bu yuzden burada ACIKCA yazilir:
///
///   * zaman     : ilk ARA-DEGERLENMEMIS OXTS kaydinin damgasi
///   * yonelim   : ayni kaydin referans pozundan
///   * konum     : ayni kaydin ENU konumu (ilk kayit orijinse [0,0,0])
///   * hiz       : ayni damgadaki GNSS hiz olayindan, yoksa sifir
///   * bias'lar  : sifir
///
/// BILIMSEL DURUSTLUK: KITTI'de referans poz OXTS'in KENDI INS cozumudur.
/// Baslangic yonelimi oradan alindigi icin baslangic durumu ile referans
/// BAGIMSIZ DEGILDIR. Bu bir entegrasyon/MVP dogrulamasidir.
///
/// Baslatma karesindeki olcumler filtreye VERILMEZ (damgasi <= t0 olan olaylar
/// atlanir): ayni cozumu hem baslangic hem olcum olarak kullanmak bilgiyi iki
/// kez saymak olurdu.
///
/// ====================== ARA-DEGERLEME POLITIKASI ============================
///
/// KITTI kisa OXTS kesintilerinde tum degerleri dogrusal ara-degerler ve son uc
/// kipi -1 yapar (Checkpoint A bunu `source_interpolated` olarak tasir).
/// Checkpoint B'nin ACIK politikasi:
///
///   GNSS konum   : ara-degerlenmisse OLCUM OLARAK KULLANILMAZ
///   GNSS hiz     : ara-degerlenmisse OLCUM OLARAK KULLANILMAZ
///   Referans poz : ara-degerlenmisse ATE'ye GIRMEZ
///   IMU          : DUSURULMEZ — yayilim zaman surekliligi gerektirir; bir
///                  kareyi atmak dt'yi iki katina cikarir ve kesinti boyunca
///                  ivme/acisal hizi tamamen bilinmez yapar. AYRI SAYILIR ve
///                  raporda gercek gozlem gibi sunulmaz.

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/buffer/estimator.hpp"
#include "kerteriz/buffer/measurement_buffer.hpp"
#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz/measurements/gnss_velocity.hpp"
#include "kerteriz_bringup/ate.hpp"
#include "kerteriz_bringup/dataset_event.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace kerteriz_bringup {

using kerteriz::TimeNs;

/// Kosucu yapilandirmasi. SIHIRLI SAYI YOKTUR: baslangic belirsizligi, surec
/// gurultusu ve kapi guveni burada aciktir ve ayar yuzeyi budur.
struct KittiRunnerConfig {
  // --- baslangic belirsizligi (1-sigma) ---
  Scalar sigma_orientation_rad = 0.02; ///< ~1.1 derece
  Scalar sigma_velocity_mps = 0.5;
  Scalar sigma_position_m = 1.0;
  Scalar sigma_gyro_bias = 0.01;  ///< rad/s
  Scalar sigma_accel_bias = 0.10; ///< m/s^2

  /// IMU surec gurultusu. Veri setinden TURETILMEMISTIR; kosucu ayaridir.
  kerteriz::ImuNoiseParams imu{1e-3, 1e-5, 1e-2, 1e-4};

  Scalar chi2_confidence = 0.997;
  TimeNs buffer_window_ns = 200000000; ///< 200 ms (ADR-5 varsayilani)

  /// GNSS anteninin govdeye gore konumu. KITTI'de OXTS birimi govde
  /// cercevesinin KENDISIDIR (devkit: GPS/IMU ekseni = arac ekseni), bu yuzden
  /// varsayilan SIFIRDIR. Alan yine de kuruluya gecirilir; baska bir platform
  /// icin tek degistirilecek yer burasidir.
  Vec3 gnss_lever_arm_b = Vec3::Zero();

  bool use_gnss_position = true;
  bool use_gnss_velocity = true;
};

/// Yorunge satiri. ATE yalnizca konum ister; CSV hiz da tasir.
struct TrajectoryRow {
  TimeNs stamp_ns = 0;
  Vec3 position_w = Vec3::Zero();
  Vec3 velocity_w = Vec3::Zero();
};

/// Sensor basina ardisik duzen sayaclari.
struct SensorCounts {
  int total = 0;                ///< veri setindeki olay sayisi
  int skipped_interpolated = 0; ///< politika geregi filtreye verilmedi
  int submitted = 0;            ///< tampona verildi
  int accepted = 0;             ///< RejectReason::kNone
  int chi_square_gated = 0;     ///< kChiSquareGate
  int numerical_failure = 0;    ///< kNumericalFailure
  int buffer_rejected = 0;      ///< kTooOld veya kOutOfOrderDrop
};

struct RunStats {
  int input_frames = 0;
  int imu_events = 0;
  int imu_interpolated = 0; ///< veri setinin KENDI ara-degerledigi kareler
  SensorCounts gnss_position;
  SensorCounts gnss_velocity;
  int reference_total = 0;
  int reference_used = 0; ///< ara-degerlenmemis, ATE'ye giren

  TimeNs init_stamp_ns = 0;
  TimeNs last_stamp_ns = 0;

  bool final_state_finite = false;
  bool final_covariance_finite = false;
  Scalar final_covariance_symmetry_error = 0;
  Scalar final_covariance_min_diagonal = 0;
};

/// Baslatma durumu. Hem Kerteriz'i kurmak hem de HARICI taban cizgisine AYNI
/// baslangic bilgisini vermek icin kullanilir — iki tarafi tek kod uretir,
/// boylece "ayni bilgi kumesi" iddiasi elle senkronize edilen iki dosyaya
/// dayanmaz.
struct InitialState {
  TimeNs stamp_ns = 0;
  Vec3 position_w = Vec3::Zero();
  Eigen::Quaternion<Scalar> orientation_wb = Eigen::Quaternion<Scalar>::Identity();
  Vec3 velocity_w = Vec3::Zero();
  /// Govde cercevesinde hiz: robot_localization'in durum vektorundeki dogrusal
  /// hizlar GOVDE cercevesindedir (dunya degil).
  Vec3 velocity_b = Vec3::Zero();
  Scalar roll = 0, pitch = 0, yaw = 0;
};

struct RunResult {
  bool ok = false;
  std::string message;
  RunStats stats;
  InitialState init;
  std::vector<TrajectoryRow> trajectory;
  std::vector<TrajectorySample> reference; ///< ara-degerlenmemis referans
};

namespace detail {

/// Ayni damgaya birden fazla satir dustuyse SONUNCUSU gecerlidir (geri sarma
/// sonrasi yeniden yayilim boyle bir durum uretebilir).
inline void tekillestir(std::vector<TrajectoryRow>& satirlar) {
  std::stable_sort(
      satirlar.begin(), satirlar.end(),
      [](const TrajectoryRow& a, const TrajectoryRow& b) { return a.stamp_ns < b.stamp_ns; });
  std::vector<TrajectoryRow> temiz;
  temiz.reserve(satirlar.size());
  for (const auto& s : satirlar) {
    if (!temiz.empty() && temiz.back().stamp_ns == s.stamp_ns) {
      temiz.back() = s;
    } else {
      temiz.push_back(s);
    }
  }
  satirlar = std::move(temiz);
}

inline void say(SensorCounts& c, kerteriz::RejectReason sebep) {
  switch (sebep) {
  case kerteriz::RejectReason::kNone:
    ++c.accepted;
    break;
  case kerteriz::RejectReason::kChiSquareGate:
    ++c.chi_square_gated;
    break;
  case kerteriz::RejectReason::kNumericalFailure:
    ++c.numerical_failure;
    break;
  default:
    ++c.buffer_rejected;
    break;
  }
}

} // namespace detail

/// Kanonik olaylari uretim zincirinden gecirir.
///
/// Girdi damga sirasinda olmalidir (adaptor bunu saglar).
inline RunResult run_kitti(const std::vector<DatasetEvent>& events, const KittiRunnerConfig& cfg) {
  RunResult r;

  // --- 1. Baslatma kaydini bul -------------------------------------------
  const DatasetEvent* ilk_ref = nullptr;
  for (const auto& e : events) {
    if (e.kind == DatasetEventKind::kReferencePose && !e.source_interpolated) {
      ilk_ref = &e;
      break;
    }
  }
  if (ilk_ref == nullptr) {
    r.message = "ara-degerlenmemis referans poz bulunamadi: baslatilamaz";
    return r;
  }
  const TimeNs t0 = ilk_ref->stamp_ns;

  Vec3 v0 = Vec3::Zero();
  for (const auto& e : events) {
    if (e.stamp_ns == t0 && e.kind == DatasetEventKind::kGnssVelocity && !e.source_interpolated) {
      v0 = e.velocity_w;
      break;
    }
  }

  r.init.stamp_ns = t0;
  r.init.position_w = ilk_ref->position_w;
  r.init.orientation_wb = ilk_ref->orientation_wb;
  r.init.velocity_w = v0;
  {
    const Eigen::Matrix<Scalar, 3, 3> r_wb = ilk_ref->orientation_wb.toRotationMatrix();
    r.init.velocity_b = r_wb.transpose() * v0;
    // ZYX (yaw-pitch-roll) ayristirmasi — KITTI'nin Rz*Ry*Rx sirasiyla ayni.
    const Vec3 ypr = r_wb.eulerAngles(2, 1, 0);
    r.init.yaw = ypr[0];
    r.init.pitch = ypr[1];
    r.init.roll = ypr[2];
  }

  kerteriz::NavState x0;
  x0.extended_pose() = kerteriz::SE23(ilk_ref->position_w, ilk_ref->orientation_wb, v0);
  // Bias'lar sifir: veri seti bir bias kestirimi vermiyor, uydurulmaz.

  kerteriz::NavCovariance p0 = kerteriz::NavCovariance::Zero();
  const auto kare = [](Scalar s) { return s * s; };
  p0.block<3, 3>(kerteriz::kIdxTheta, kerteriz::kIdxTheta)
      .diagonal()
      .setConstant(kare(cfg.sigma_orientation_rad));
  p0.block<3, 3>(kerteriz::kIdxVel, kerteriz::kIdxVel)
      .diagonal()
      .setConstant(kare(cfg.sigma_velocity_mps));
  p0.block<3, 3>(kerteriz::kIdxPos, kerteriz::kIdxPos)
      .diagonal()
      .setConstant(kare(cfg.sigma_position_m));
  p0.block<3, 3>(9, 9).diagonal().setConstant(kare(cfg.sigma_gyro_bias));
  p0.block<3, 3>(12, 12).diagonal().setConstant(kare(cfg.sigma_accel_bias));

  kerteriz::Estimator est(std::make_unique<kerteriz::EskfBackend>(
                              kerteriz::EskfConfig{x0, p0, t0, cfg.imu, cfg.chi2_confidence}),
                          kerteriz::NisMonitor{}, kerteriz::FaultDetector{});
  kerteriz::MeasurementBuffer tampon(cfg.buffer_window_ns);

  r.stats.init_stamp_ns = t0;

  // --- 2. Olaylari isle ---------------------------------------------------
  const auto sonuclari_topla = [&]() {
    for (const auto& s : tampon.process(est)) {
      if (s.sensor == "gnss_position") {
        detail::say(r.stats.gnss_position, s.reason);
      } else if (s.sensor == "gnss_velocity") {
        detail::say(r.stats.gnss_velocity, s.reason);
      }
    }
    r.trajectory.push_back(TrajectoryRow{est.backend().stamp_ns(),
                                         est.backend().state().extended_pose().translation(),
                                         est.backend().state().extended_pose().linearVelocity()});
  };

  for (const auto& e : events) {
    switch (e.kind) {
    case DatasetEventKind::kImu:
      ++r.stats.imu_events;
      r.stats.imu_interpolated += e.source_interpolated ? 1 : 0;
      break;
    case DatasetEventKind::kGnssPosition:
      ++r.stats.gnss_position.total;
      break;
    case DatasetEventKind::kGnssVelocity:
      ++r.stats.gnss_velocity.total;
      break;
    case DatasetEventKind::kReferencePose:
      ++r.stats.reference_total;
      if (!e.source_interpolated) {
        ++r.stats.reference_used;
        r.reference.push_back(TrajectorySample{e.stamp_ns, e.position_w});
      }
      break;
    }

    // Baslatma karesi ve oncesi filtreye VERILMEZ.
    if (e.stamp_ns <= t0) {
      continue;
    }
    r.stats.last_stamp_ns = std::max(r.stats.last_stamp_ns, e.stamp_ns);

    switch (e.kind) {
    case DatasetEventKind::kImu:
      // Ara-degerlenmis IMU DUSURULMEZ: zaman surekliligi gerekir.
      tampon.add_imu(e.imu);
      sonuclari_topla();
      break;

    case DatasetEventKind::kGnssPosition:
      if (!cfg.use_gnss_position) {
        break;
      }
      if (e.source_interpolated) {
        ++r.stats.gnss_position.skipped_interpolated;
        break;
      }
      ++r.stats.gnss_position.submitted;
      if (!tampon.add_measurement(std::make_unique<kerteriz::GnssPosition>(
              e.stamp_ns, "gnss_position", e.position_w, e.position_cov_w, cfg.gnss_lever_arm_b))) {
        r.message = "tampon GNSS konum olcumunu kabul etmedi (kapasite)";
        return r;
      }
      sonuclari_topla();
      break;

    case DatasetEventKind::kGnssVelocity:
      if (!cfg.use_gnss_velocity) {
        break;
      }
      if (e.source_interpolated) {
        ++r.stats.gnss_velocity.skipped_interpolated;
        break;
      }
      ++r.stats.gnss_velocity.submitted;
      if (!tampon.add_measurement(std::make_unique<kerteriz::GnssVelocity>(
              e.stamp_ns, "gnss_velocity", e.velocity_w, e.velocity_cov_w))) {
        r.message = "tampon GNSS hiz olcumunu kabul etmedi (kapasite)";
        return r;
      }
      sonuclari_topla();
      break;

    case DatasetEventKind::kReferencePose:
      break; // referans filtreye GIRMEZ
    }
  }
  sonuclari_topla(); // bekleyen olcumleri bosalt

  detail::tekillestir(r.trajectory);
  r.stats.input_frames = r.stats.reference_total;

  // --- 3. Son durum saglik kontrolu --------------------------------------
  const auto& son_x = est.backend().state();
  const auto& son_p = est.backend().covariance();
  const int n = son_x.active_dof();

  r.stats.final_state_finite = son_x.extended_pose().coeffs().allFinite() &&
                               son_x.gyro_bias().allFinite() && son_x.accel_bias().allFinite();
  const auto aktif = son_p.topLeftCorner(n, n);
  r.stats.final_covariance_finite = aktif.allFinite();
  r.stats.final_covariance_symmetry_error = (aktif - aktif.transpose()).cwiseAbs().maxCoeff();
  r.stats.final_covariance_min_diagonal = aktif.diagonal().minCoeff();

  r.ok = true;
  return r;
}

} // namespace kerteriz_bringup
