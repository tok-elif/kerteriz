#pragma once

/// \file
/// Veri setinden bagimsiz KANONIK olay gosterimi — CLAUDE.md §4.
///
/// ============================ NEDEN KANONIK =================================
///
/// KITTI ve NCLT parser'larinin filtreye AYRI YOLLAR acmasini istemiyoruz.
/// Her adaptor kendi ham bicimini okur, cerceve/birim donusumunu KENDI icinde
/// yapar ve BU tipi uretir. Filtre matematigi veri seti adini hic gormez:
///
///   ham veri seti -> cerceve/birim donusumu -> DatasetEvent -> Estimator
///
/// Veri setine ozel hicbir sinif olcum modellerine veya backend'e girmez.
///
/// ============================== ZAMAN =======================================
///
/// `stamp_ns` mutlak zamandir ve HER ZAMAN `TimeNs` (int64 ns). Adaptorler
/// tamsayi olarak ayristirir; kayan noktaya cevrilip geri donmez
/// (CONVENTIONS §6). `dt` yalnizca iki damga farkindan uretilir ve bunu
/// MeasurementBuffer yapar.
///
/// ============================== KAPSAM ======================================
///
/// Deger tipidir, sahiplik tasimaz, sabit boyutludur. Olcum nesneleri buradan
/// MEVCUT Faz 1 kurucularila uretilir; yeni bir olcum modeli TANIMLANMAZ.

#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kerteriz_bringup {

using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::Vec3;

enum class DatasetEventKind {
  kImu,           ///< `imu` gecerli
  kGnssPosition,  ///< `position_w`, `position_cov_w` gecerli
  kGnssVelocity,  ///< `velocity_w`, `velocity_cov_w` gecerli
  kWheelVelocity, ///< `forward_velocity_b`, `forward_variance` gecerli
  kReferencePose  ///< `position_w`, `orientation_wb` gecerli — GIRDI DEGIL, kiyas icin
};

/// Tek bir veri seti olayi.
///
/// `kReferencePose` KESINLIKLE filtreye verilmez; yalnizca degerlendirme
/// tarafinda kullanilir. Ayni tipte tasinmasinin sebebi zaman sirasinin tek
/// bir akista korunmasidir; ayirma tuketicinin isidir.
struct DatasetEvent {
  TimeNs stamp_ns = 0;
  DatasetEventKind kind = DatasetEventKind::kImu;

  kerteriz::ImuSample imu{};

  Vec3 position_w = Vec3::Zero();
  Eigen::Matrix<Scalar, 3, 3> position_cov_w = Eigen::Matrix<Scalar, 3, 3>::Identity();

  Vec3 velocity_w = Vec3::Zero();
  Eigen::Matrix<Scalar, 3, 3> velocity_cov_w = Eigen::Matrix<Scalar, 3, 3>::Identity();

  Scalar forward_velocity_b = 0;
  Scalar forward_variance = 1;

  Eigen::Quaternion<Scalar> orientation_wb = Eigen::Quaternion<Scalar>::Identity();

  /// Kaynak kayit, veri setinin KENDI isaretine gore eksik/ara-degerlenmis mi.
  ///
  /// KITTI kisa OXTS kesintilerinde tum degerleri dogrusal olarak ara-degerler
  /// ve bunu son uc kipi (-1) yaparak isaretler. Boyle bir kareden uretilen
  /// TUM olaylarda bu bayrak true'dur.
  ///
  /// FAZ 1 CHECKPOINT A BU BAYRAGA GORE HICBIR SEY ATMAZ. Adaptorun isi veri
  /// seti gercegini KAYIPSIZ tasimaktir; ara-degerlenmis bir olcumun filtreye
  /// verilip verilmeyecegi bir POLITIKA kararidir ve kosucuda acikca
  /// yazilacaktir. Bayrak tasinmasaydi o karar sessizce "hepsini kullan"
  /// olurdu.
  bool source_interpolated = false;
};

/// Damgaya gore siralama olcutu. Ayni damgada IMU once gelir — tampon da ayni
/// kurali uygular (MeasurementBuffer), boylece adaptor ciktisi ile tamponun
/// ic sirasi AYNI konvansiyondadir.
inline bool event_before(const DatasetEvent& a, const DatasetEvent& b) {
  if (a.stamp_ns != b.stamp_ns) {
    return a.stamp_ns < b.stamp_ns;
  }
  const int a_imu = a.kind == DatasetEventKind::kImu ? 0 : 1;
  const int b_imu = b.kind == DatasetEventKind::kImu ? 0 : 1;
  return a_imu < b_imu;
}

} // namespace kerteriz_bringup
