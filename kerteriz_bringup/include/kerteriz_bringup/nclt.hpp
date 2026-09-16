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

/// Virgulle ayrilmis satiri TAM `beklenen` parcaya boler. Alanlar HAM METIN
/// olarak doner; sayisal yorum cagirana aittir. Bu ayrim kasitlidir: UTIME
/// alani tamsayi, digerleri kayan noktadir ve ayni fonksiyon ikisini de
/// sayiya cevirseydi UTIME zorunlu olarak kayan noktadan gecerdi.
inline bool split_csv(const std::string& satir, int beklenen, std::vector<std::string>& out) {
  out.clear();
  out.reserve(static_cast<std::size_t>(beklenen));

  std::size_t bas = 0;
  while (true) {
    const std::size_t virgul = satir.find(',', bas);
    out.push_back(
        satir.substr(bas, virgul == std::string::npos ? std::string::npos : virgul - bas));
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

/// TAM ONDALIK TAMSAYI ayristirici — mutlak zaman icin.
///
/// `double` ARA ADIMI YOKTUR (CONVENTIONS §6). NCLT UTIME 16 hanelidir;
/// `double`'in 53 bitlik mantisi ~9.0e15'e kadar tamsayilari tam tasir,
/// yani bugunku degerler sinira YAKINDIR ve ileri tarihli oturumlarda sessizce
/// yuvarlanabilirdi. Ondalik nokta, ussel gosterim veya artik simge iceren
/// bir alan tamsayi DEGILDIR ve reddedilir.
inline bool parse_int64_token(const std::string& token, std::int64_t& out) {
  std::size_t i = 0;
  while (i < token.size() && std::isspace(static_cast<unsigned char>(token[i])) != 0) {
    ++i;
  }
  bool eksi = false;
  if (i < token.size() && (token[i] == '+' || token[i] == '-')) {
    eksi = token[i] == '-';
    ++i;
  }
  const std::size_t rakam_bas = i;
  std::int64_t deger = 0;
  constexpr std::int64_t kMaks = std::numeric_limits<std::int64_t>::max();
  for (; i < token.size(); ++i) {
    const unsigned char c = static_cast<unsigned char>(token[i]);
    if (std::isdigit(c) == 0) {
      break;
    }
    const int basamak = token[i] - '0';
    if (deger > (kMaks - basamak) / 10) {
      return false; // tasma
    }
    deger = deger * 10 + basamak;
  }
  if (i == rakam_bas) {
    return false; // hic rakam yok
  }
  // Geri kalan YALNIZCA bosluk olabilir: "...0.5", "1.3e15", "...abc" reddedilir.
  for (std::size_t k = i; k < token.size(); ++k) {
    if (std::isspace(static_cast<unsigned char>(token[k])) == 0) {
      return false;
    }
  }
  out = eksi ? -deger : deger;
  return true;
}

/// Tek bir alani kayan noktaya cevirir. "1.5abc" gibi kismi sayilar reddedilir.
inline bool parse_scalar_token(const std::string& token, Scalar& out) {
  std::size_t tuketilen = 0;
  try {
    out = std::stod(token, &tuketilen);
  } catch (...) {
    return false;
  }
  for (std::size_t i = tuketilen; i < token.size(); ++i) {
    if (std::isspace(static_cast<unsigned char>(token[i])) == 0) {
      return false;
    }
  }
  return true;
}

/// Virgulle ayrilmis bir satiri TAM `beklenen` SAYISAL alan olarak ayristirir.
/// Eksik, fazla veya sayisal olmayan alan HATADIR — sessiz kabul yoktur.
///
/// UTIME icin KULLANILMAZ; o alan `parse_int64_token` ile okunur.
inline bool parse_csv_row(const std::string& satir, int beklenen, std::vector<Scalar>& out) {
  std::vector<std::string> parcalar;
  if (!split_csv(satir, beklenen, parcalar)) {
    return false;
  }
  out.clear();
  out.reserve(parcalar.size());
  for (const auto& t : parcalar) {
    Scalar deger = 0;
    if (!parse_scalar_token(t, deger)) {
      return false;
    }
    out.push_back(deger);
  }
  return true;
}

inline constexpr int kNcltMs25FieldCount = 10;
inline constexpr int kNcltGpsFieldCount = 8;

/// `ms25.csv` tek satiri -> IMU olayi (REP-103 govde ekseninde).
inline bool parse_nclt_ms25_line(const std::string& satir, DatasetEvent& out) {
  std::vector<std::string> parcalar;
  if (!split_csv(satir, kNcltMs25FieldCount, parcalar)) {
    return false;
  }
  std::int64_t utime_us = 0;
  if (!parse_int64_token(parcalar[0], utime_us)) {
    return false; // UTIME tamsayidir; kayan noktadan GECMEZ
  }
  kerteriz::TimeNs t = 0;
  if (!nclt_utime_to_ns(utime_us, t)) {
    return false;
  }
  Scalar v[kNcltMs25FieldCount] = {};
  for (int i = 1; i < kNcltMs25FieldCount; ++i) {
    if (!parse_scalar_token(parcalar[static_cast<std::size_t>(i)], v[i])) {
      return false;
    }
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

/// Faz 1 `GnssPosition` UC BOYUTLUDUR, dolayisiyla yukseklik de gecerli
/// olmalidir. Resmi sozlesme (Tablo 7): 2 = enlem/boylam iyi, 3 = yukseklik
/// de iyi. Bu yuzden esik 3'tur.
inline constexpr int kNcltMinFixMode3d = 3;

/// `gps.csv` / `gps_rtk.csv` tek satiri. Fix modu 3D icin yetersizse olay
/// URETILMEZ (`gecerli = false`); bu bir ayristirma hatasi DEGILDIR.
///
/// Mod 2 kaydini buyuk bir dusey kovaryansla 3D'ye ZORLAMIYORUZ: veri seti o
/// kayit icin yukseklik hakkinda "gecersiz" diyor, "belirsiz" degil. Uydurma
/// bir sigma ile gecersiz veriyi olcume cevirmek sessiz bir sapma kaynagi
/// olurdu. Iki boyutlu bir olcum modeli de EKLENMEDI (kapsam disi).
inline bool parse_nclt_gps_line(const std::string& satir, const EnuProjector& izdusum,
                                Scalar sigma_m, const Vec3& lever_arm_b, DatasetEvent& out,
                                bool& gecerli) {
  std::vector<std::string> parcalar;
  if (!split_csv(satir, kNcltGpsFieldCount, parcalar)) {
    return false;
  }
  std::int64_t utime_us = 0;
  if (!parse_int64_token(parcalar[0], utime_us)) {
    return false; // UTIME tamsayidir; kayan noktadan GECMEZ
  }
  kerteriz::TimeNs t = 0;
  if (!nclt_utime_to_ns(utime_us, t)) {
    return false;
  }
  Scalar v[kNcltGpsFieldCount] = {};
  for (int i = 1; i < kNcltGpsFieldCount; ++i) {
    if (!parse_scalar_token(parcalar[static_cast<std::size_t>(i)], v[i])) {
      return false;
    }
  }

  const int fix = static_cast<int>(v[1]);
  if (fix < kNcltMinFixMode3d) {
    gecerli = false;
    return true; // gecerli satir, 3D icin yeterli fix yok
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
  ///
  /// SU AN YALNIZCA YAPILANDIRMA VERISIDIR. Checkpoint B'deki veri seti
  /// kosucusu bunu `GnssPosition`'in `p_BS` parametresine GERCEKTEN gececek;
  /// olcum modeli kol duzeltmesini zaten destekliyor (F1.4).
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
        // Orijin de ancak TAM 3D bir fix'ten kurulabilir: yuksekligi gecersiz
        // bir kayitla orijin secmek tum yorungeye sabit bir dusey sapma
        // enjekte ederdi.
        std::vector<std::string> parcalar;
        if (!split_csv(satir, kNcltGpsFieldCount, parcalar)) {
          return ParseStatus::failure(cfg.gps_csv + ":" + std::to_string(no) + " bozuk gps satiri");
        }
        std::int64_t utime_us = 0;
        if (!parse_int64_token(parcalar[0], utime_us)) {
          return ParseStatus::failure(cfg.gps_csv + ":" + std::to_string(no) + " bozuk UTIME");
        }
        Scalar v[kNcltGpsFieldCount] = {};
        for (int i = 1; i < kNcltGpsFieldCount; ++i) {
          if (!parse_scalar_token(parcalar[static_cast<std::size_t>(i)], v[i])) {
            return ParseStatus::failure(cfg.gps_csv + ":" + std::to_string(no) +
                                        " bozuk gps satiri");
          }
        }
        if (static_cast<int>(v[1]) < kNcltMinFixMode3d) {
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
