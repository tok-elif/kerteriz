#pragma once

/// \file
/// NCLT (Michigan) adaptoru — CLAUDE.md §4.
///
/// ===================== DOGRULANMIS BICIM SOZLESMESI =========================
///
/// Asagidakiler veri setinin RESMI teknik makalesinden (NCLT dataset paper,
/// Tablo 3, 4, 7, 8 ve §7.2) DOGRULANMISTIR; tahmin edilmemistir.
///
///   * UTIME: UNIX epoch'undan MIKROSANIYE, 16 haneli tamsayi.
///   * `ms25.csv`      : utime, manyetik alan x/y/z [Gauss],
///                       ivme x/y/z [m/s^2], acisal hiz roll/pitch/heading [rad/s]
///   * `gps.csv` ve `gps_rtk.csv`:
///                       utime, fix modu, uydu sayisi, enlem [rad], boylam [rad],
///                       yukseklik [m], track, hiz [m/s]
///                       fix modu: 0 gorulmedi, 1 fix yok, 2 enlem/boylam iyi,
///                       3 yukseklik de iyi
///
/// ENLEM/BOYLAM RADYANDIR. KITTI derece verir; birim donusumu her iki
/// adaptorde de KENDI icinde yapilir ki hata tek bir yerde gorunur olsun.
///
/// ============================== CERCEVE =====================================
///
/// Resmi tanim: govde cercevesi Segway'in teker akslarinin ortasinda,
/// "x pointing forward, y to the right, and z down". REP-103 ise x ileri,
/// y SOL, z YUKARI'dir (CONVENTIONS §1). Donusum bu yuzden
///
///   R_rep103_nclt = diag(1, -1, -1)
///
/// olur ve `nclt_body_to_rep103()` icinde ADI KONULARAK uygulanir. Cagri
/// yerlerinde cikplak isaret takasi YOKTUR.
///
/// Tablo 4'e gore Microstrain IMU'nun govdeye gore ACISAL ofseti SIFIRDIR
/// (phi = theta = psi = 0), yalnizca oteleme vardir. Dolayisiyla IMU ile govde
/// arasinda ek bir rotasyon uygulanmaz. Oteleme farkinin yarattigi merkezkac/
/// Euler terimleri Faz 1'de TELAFI EDILMEZ; bunun icin acisal ivme gerekir ve
/// o NavState'in parcasi degildir. Bu bir yaklasimdir, sessiz degildir.
///
/// ======================= FAZ 1'DE ERTELENEN KANALLAR ========================
///
/// 1. TEKER ILERI HIZI — ERTELENDI.
///    NCLT `odometry_mu_100hz.csv` ANLIK hiz vermez; kosu basindan itibaren
///    ENTEGRE 6-DOF pozu verir. Oradan anlik ileri hiz cikarmak turev almayi
///    gerektirir. Dahasi resmi makale §4'e gore bu odometri bir EKF ciktisidir
///    ve "measurement updates are derived from a commodity IMU" — yani IMU
///    bilgisini ZATEN icerir. Turevini bagimsiz bir teker olcumu gibi filtreye
///    vermek ayni bilgiyi iki kez saymak olurdu. Ek varsayim uydurmak yerine
///    kanal desteklenmiyor olarak birakildi (ADR-9'daki RelativePose Faz 3'tur).
///
/// 2. YER GERCEGI (ground truth) — ERTELENDI.
///    Resmi makalenin §7 dizin/bicim bolumu images, sensor_data, velodyne,
///    hokuyo, ROSbag, ladybug kalibrasyon ve CAD dizinlerini tanimlar; bir
///    ground-truth CSV'sinin SUTUN BICIMI tanimlanmamistir. Tek anlamli bir
///    kaynak olmadan sutun sirasi/cerceve tahmin EDILMEZ.
///
/// 3. GNSS HIZI — ERTELENDI.
///    Tablo 7'de `track` alaninin birimi metre olarak listelenmistir; bu bir
///    yon buyuklugu icin tutarsizdir ve `speed` ile birlikte bir hiz VEKTORU
///    kurmak icin gereken yon bilgisi tek anlamli degildir.
///
/// ============================== KAPSAM ======================================
///
/// Kamera, Velodyne ve Hokuyo AYRISTIRILMAZ.

#include "kerteriz_bringup/dataset_event.hpp"
#include "kerteriz_bringup/geodetic.hpp"
#include "kerteriz_bringup/kitti_oxts.hpp" // ParseStatus

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace kerteriz_bringup {

/// NCLT govde ekseni -> REP-103 govde ekseni.
/// (x ileri, y sag, z asagi) -> (x ileri, y sol, z yukari)
inline Vec3 nclt_body_to_rep103(const Vec3& v) { return Vec3(v.x(), -v.y(), -v.z()); }

/// Tablo 4'ten okunan, govde cercevesindeki sensor otelemeleri (NCLT ekseni).
/// Bunlar VARSAYILANDIR; yapilandirma bunlari ezebilir (CLAUDE.md §4:
/// platforma ozel sabitler koda gomulmez).
inline const Vec3 kNcltImuOffsetBodyNclt(-0.11, -0.18, -0.71);
inline const Vec3 kNcltRtkOffsetBodyNclt(-0.24, 0.0, -1.24);

/// UTIME (mikrosaniye) -> TimeNs. Tasma denetlenir.
inline bool nclt_utime_to_ns(std::int64_t utime_us, kerteriz::TimeNs& out) {
  constexpr std::int64_t kSinir = std::numeric_limits<std::int64_t>::max() / 1000;
  if (utime_us < 0 || utime_us > kSinir) {
    return false;
  }
  out = utime_us * 1000;
  return true;
}

/// Virgulle ayrilmis bir satiri TAM `beklenen` alan olarak ayristirir.
/// Eksik, fazla veya sayisal olmayan alan HATADIR — sessiz kabul yoktur.
inline bool parse_csv_row(const std::string& satir, int beklenen, std::vector<Scalar>& out) {
  out.clear();
  out.reserve(static_cast<std::size_t>(beklenen));

  std::size_t bas = 0;
  while (true) {
    const std::size_t virgul = satir.find(',', bas);
    const std::string alan =
        satir.substr(bas, virgul == std::string::npos ? std::string::npos : virgul - bas);

    std::size_t tuketilen = 0;
    Scalar deger = 0;
    try {
      deger = std::stod(alan, &tuketilen);
    } catch (...) {
      return false;
    }
    // Alanin GERI KALANI yalnizca bosluk olabilir; "1.5abc" kabul edilmez.
    for (std::size_t i = tuketilen; i < alan.size(); ++i) {
      if (std::isspace(static_cast<unsigned char>(alan[i])) == 0) {
        return false;
      }
    }
    out.push_back(deger);
    if (out.size() > static_cast<std::size_t>(beklenen)) {
      return false;
    }
    if (virgul == std::string::npos) {
      break;
    }
    bas = virgul + 1;
  }
  return out.size() == static_cast<std::size_t>(beklenen);
}

inline constexpr int kNcltMs25FieldCount = 10;
inline constexpr int kNcltGpsFieldCount = 8;

/// `ms25.csv` tek satiri -> IMU olayi (REP-103 govde ekseninde).
inline bool parse_nclt_ms25_line(const std::string& satir, DatasetEvent& out) {
  std::vector<Scalar> v;
  if (!parse_csv_row(satir, kNcltMs25FieldCount, v)) {
    return false;
  }
  kerteriz::TimeNs t = 0;
  if (!nclt_utime_to_ns(static_cast<std::int64_t>(v[0]), t)) {
    return false;
  }

  const Vec3 ivme_nclt(v[4], v[5], v[6]);
  const Vec3 acisal_nclt(v[7], v[8], v[9]);

  out = DatasetEvent{};
  out.stamp_ns = t;
  out.kind = DatasetEventKind::kImu;
  out.imu =
      kerteriz::ImuSample{t, nclt_body_to_rep103(acisal_nclt), nclt_body_to_rep103(ivme_nclt)};
  return true;
}

/// `gps.csv` / `gps_rtk.csv` tek satiri. Fix modu 2'den kucukse olay
/// URETILMEZ (`gecerli = false`); bu bir ayristirma hatasi DEGILDIR.
inline bool parse_nclt_gps_line(const std::string& satir, const EnuProjector& izdusum,
                                Scalar sigma_m, const Vec3& lever_arm_b, DatasetEvent& out,
                                bool& gecerli) {
  std::vector<Scalar> v;
  if (!parse_csv_row(satir, kNcltGpsFieldCount, v)) {
    return false;
  }
  kerteriz::TimeNs t = 0;
  if (!nclt_utime_to_ns(static_cast<std::int64_t>(v[0]), t)) {
    return false;
  }

  const int fix = static_cast<int>(v[1]);
  if (fix < 2) {
    gecerli = false;
    return true; // gecerli satir, kullanilabilir fix yok
  }

  out = DatasetEvent{};
  out.stamp_ns = t;
  out.kind = DatasetEventKind::kGnssPosition;
  out.position_w = izdusum.to_enu(Llh{v[3], v[4], v[5]}); // enlem/boylam RADYAN
  out.position_cov_w = Eigen::Matrix<Scalar, 3, 3>::Identity() * (sigma_m * sigma_m);
  (void)lever_arm_b; // kol, olcum nesnesi kurulurken GnssPosition'a verilir
  gecerli = true;
  return true;
}

struct NcltConfig {
  std::string ms25_csv; ///< bos ise IMU okunmaz
  std::string gps_csv;  ///< bos ise GNSS okunmaz
  Scalar gps_sigma_m = 3.0;
  /// GNSS anteninin IMU'ya gore konumu, REP-103 govde ekseninde.
  /// Varsayilan Tablo 4'ten turetilir: x_imu,rtk = x_body,rtk - x_body,imu.
  Vec3 gnss_lever_arm_b =
      nclt_body_to_rep103(Vec3(kNcltRtkOffsetBodyNclt - kNcltImuOffsetBodyNclt));
  bool explicit_origin = false;
  Llh origin{};
};

/// NCLT navigasyon kanallarini kanonik olaylara cevirir.
inline ParseStatus load_nclt(const NcltConfig& cfg, std::vector<DatasetEvent>& out) {
  out.clear();
  std::vector<DatasetEvent> olaylar;

  if (!cfg.ms25_csv.empty()) {
    std::ifstream f(cfg.ms25_csv);
    if (!f) {
      return ParseStatus::failure("ms25 acilamadi: " + cfg.ms25_csv);
    }
    std::string satir;
    int no = 0;
    while (std::getline(f, satir)) {
      ++no;
      if (satir.empty()) {
        continue;
      }
      DatasetEvent e;
      if (!parse_nclt_ms25_line(satir, e)) {
        return ParseStatus::failure(cfg.ms25_csv + ":" + std::to_string(no) + " bozuk ms25 satiri");
      }
      olaylar.push_back(e);
    }
  }

  if (!cfg.gps_csv.empty()) {
    std::ifstream f(cfg.gps_csv);
    if (!f) {
      return ParseStatus::failure("gps acilamadi: " + cfg.gps_csv);
    }
    // Orijin: acik verilmediyse ILK kullanilabilir fix.
    bool orijin_kuruldu = cfg.explicit_origin;
    Llh orijin = cfg.origin;

    std::string satir;
    int no = 0;
    while (std::getline(f, satir)) {
      ++no;
      if (satir.empty()) {
        continue;
      }
      if (!orijin_kuruldu) {
        std::vector<Scalar> v;
        if (!parse_csv_row(satir, kNcltGpsFieldCount, v)) {
          return ParseStatus::failure(cfg.gps_csv + ":" + std::to_string(no) + " bozuk gps satiri");
        }
        if (static_cast<int>(v[1]) < 2) {
          continue;
        }
        orijin = Llh{v[3], v[4], v[5]};
        orijin_kuruldu = true;
      }
      const EnuProjector izdusum(orijin);
      DatasetEvent e;
      bool gecerli = false;
      if (!parse_nclt_gps_line(satir, izdusum, cfg.gps_sigma_m, cfg.gnss_lever_arm_b, e, gecerli)) {
        return ParseStatus::failure(cfg.gps_csv + ":" + std::to_string(no) + " bozuk gps satiri");
      }
      if (gecerli) {
        olaylar.push_back(e);
      }
    }
  }

  std::stable_sort(olaylar.begin(), olaylar.end(), event_before);
  out = std::move(olaylar);
  return ParseStatus::success();
}

} // namespace kerteriz_bringup
