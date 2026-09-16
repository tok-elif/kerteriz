#pragma once

/// \file
/// WGS84 LLH -> ECEF -> yerel ENU — veri seti adaptor katmani.
///
/// ============================== NEDEN BURADA ================================
///
/// `GnssPosition` cekirdekte ZATEN yerel ENU metre ister (CONVENTIONS §2
/// `origin_policy`). Cografi donusum cekirdege SOKULMAZ: kerteriz_core ROS'suz
/// VE cografi kutuphane bagimsizdir (R1, ADR-1). Bu dosya adaptor katmanindadir.
///
/// Harici bir cografi kutuphane de eklenmedi: donusum kucuk, kapali formlu ve
/// test edilebilir. Yeni bir bagimlilik, kazandirdigindan fazlasini maliyet
/// olarak getirirdi.
///
/// ============================== HASSASIYET ==================================
///
/// Tam elipsoid modeli kullanilir (kuresel yaklasim DEGIL). Faz 1 hedefi
/// metre-alti akil sagligidir; ECEF farki uzerinden kurulan yerel teget duzlem
/// birkac kilometrelik calisma alaninda bunun cok otesinde dogruluk verir.
///
/// Orijin politikasi CAGIRANINDIR: ilk gecerli fix veya yapilandirmada verilen
/// acik orijin. Bu dosya orijin SECMEZ.

#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <cmath>

namespace kerteriz_bringup {

using kerteriz::Scalar;
using kerteriz::Vec3;

/// WGS84 tanimlayici sabitleri.
inline constexpr Scalar kWgs84SemiMajorM = 6378137.0;
inline constexpr Scalar kWgs84Flattening = 1.0 / 298.257223563;

/// Birinci eksantriklik karesi: e^2 = f (2 - f).
inline constexpr Scalar kWgs84Ecc2 = kWgs84Flattening * (2.0 - kWgs84Flattening);

/// Cografi konum. Aci birimi RADYANDIR — dosya bicimleri farkli birim
/// kullanabilir (KITTI derece, NCLT radyan), donusum PARSER'da yapilir ki
/// birim hatasi tek bir yerde gorunur olsun.
struct Llh {
  Scalar lat_rad = 0;
  Scalar lon_rad = 0;
  Scalar alt_m = 0;
};

/// Geodetik -> ECEF (metre).
inline Vec3 llh_to_ecef(const Llh& p) {
  const Scalar sin_lat = std::sin(p.lat_rad);
  const Scalar cos_lat = std::cos(p.lat_rad);
  const Scalar sin_lon = std::sin(p.lon_rad);
  const Scalar cos_lon = std::cos(p.lon_rad);

  // Asal dikey egrilik yaricapi.
  const Scalar n = kWgs84SemiMajorM / std::sqrt(1.0 - kWgs84Ecc2 * sin_lat * sin_lat);

  return Vec3((n + p.alt_m) * cos_lat * cos_lon, (n + p.alt_m) * cos_lat * sin_lon,
              (n * (1.0 - kWgs84Ecc2) + p.alt_m) * sin_lat);
}

/// Orijine gore ENU donusum matrisi: enu = R_en * (ecef - ecef_orijin).
///
/// Satirlar sirasiyla Dogu, Kuzey, Yukari birim vektorleridir. Acik ters alma
/// yasaktir (ADR-17); bu matris ortonormaldir ve tersi transpozudur.
inline Eigen::Matrix<Scalar, 3, 3> ecef_to_enu_rotation(const Llh& origin) {
  const Scalar sin_lat = std::sin(origin.lat_rad);
  const Scalar cos_lat = std::cos(origin.lat_rad);
  const Scalar sin_lon = std::sin(origin.lon_rad);
  const Scalar cos_lon = std::cos(origin.lon_rad);

  Eigen::Matrix<Scalar, 3, 3> r;
  r << -sin_lon, cos_lon, 0.0,                         // Dogu
      -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat, // Kuzey
      cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;   // Yukari
  return r;
}

/// Yerel teget duzlem donusumu. Orijin her zaman tam olarak [0, 0, 0] verir.
class EnuProjector {
 public:
  explicit EnuProjector(const Llh& origin)
      : origin_(origin), origin_ecef_(llh_to_ecef(origin)), r_en_(ecef_to_enu_rotation(origin)) {}

  const Llh& origin() const { return origin_; }

  Vec3 to_enu(const Llh& p) const { return r_en_ * (llh_to_ecef(p) - origin_ecef_); }

 private:
  Llh origin_;
  Vec3 origin_ecef_;
  Eigen::Matrix<Scalar, 3, 3> r_en_;
};

} // namespace kerteriz_bringup
