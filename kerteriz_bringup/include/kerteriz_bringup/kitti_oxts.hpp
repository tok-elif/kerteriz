#pragma once

/// \file
/// KITTI raw OXTS adaptoru — CLAUDE.md §4 (veri seti adaptorleri bringup'tadir).
///
/// ===================== DOGRULANMIS BICIM SOZLESMESI =========================
///
/// Asagidakiler resmi KITTI raw geliştirme kitindeki `dataformat.txt` ve
/// `convertOxtsToPose.m` uzerinden DOGRULANMISTIR; tahmin edilmemistir.
///
///   * `oxts/data/NNNNNNNNNN.txt` — kare basina 30 alan, bosluk ayrili
///   * `oxts/timestamps.txt`      — satir sirasi = kare sirasi (0 tabanli),
///                                  bicim "YYYY-MM-DD HH:MM:SS.fffffffff"
///
///   Alan sirasi (0 tabanli):
///     0 lat[deg] 1 lon[deg] 2 alt[m] 3 roll 4 pitch 5 yaw
///     6 vn 7 ve 8 vf 9 vl 10 vu          [m/s]
///     11 ax 12 ay 13 az 14 af 15 al 16 au [m/s^2]
///     17 wx 18 wy 19 wz 20 wf 21 wl 22 wu [rad/s]
///     23 pos_accuracy[m] 24 vel_accuracy[m/s]
///     25 navstat 26 numsats 27 posmode 28 velmode 29 orimode
///
/// ======================= CERCEVE — DONUSUM GEREKMIYOR =======================
///
/// Resmi tanim: GPS/IMU eksenleri "x: forward, y: left, z: up". Bu REP-103
/// govde cercevesinin TA KENDISIDIR (CONVENTIONS §1), dolayisiyla
/// (ax, ay, az) ve (wx, wy, wz) icin eksen permutasyonu YAPILMAZ. Sihirli
/// isaret takasi yok; donusum yoklugunun sebebi burada yazilidir.
///
/// `vf/vl/vu` ve `af/al/au` ise yer yuzeyine tegetlenmis cerceveye aittir ve
/// KULLANILMAZ — govde olcumu isteniyor, o da x/y/z alanlaridir.
///
/// Dunya cercevesi: `yaw` icin 0 = dogu, pozitif = saat yonunun tersi; `vn`
/// kuzey, `ve` dogu, `vu` yukari. Bu dogrudan ENU'dur (CONVENTIONS §1).
/// Dolayisiyla:
///
///   v_W       = (ve, vn, vu)
///   R_WB      = Rz(yaw) Ry(pitch) Rx(roll)      (devkit ile ayni sira)
///
/// ============================ ZAMAN DILIMI ==================================
///
/// Veri seti belgeleri damgalarin saat dilimini TEK ANLAMLI BELIRTMEZ. Burada
/// UTC olarak ayristirilir. Bu Faz 1 icin zararsizdir cunku tum kanallar AYNI
/// saatten gelir: sabit bir ofset `dt`'yi ve olay sirasini DEGISTIRMEZ. Mutlak
/// epoch'un baska bir veri setiyle hizalanmasi gerekirse bu varsayim yeniden
/// ele alinmalidir.
///
/// ====================== BILIMSEL DURUSTLUK UYARISI ==========================
///
/// KITTI'de "ground truth" OXTS'in KENDI RTK/INS cozumudur. Yani referans poz
/// ile filtreye verilen GNSS olcumu AYNI KAYNAKTAN gelir ve istatistiksel
/// olarak BAGIMSIZ DEGILDIR. Buradan cikan ATE bir entegrasyon/MVP
/// dogrulamasidir; bagimsiz bir dogruluk deneyi (SPEC §8 E2) degildir.
///
/// ===================== EKSIK BILGI ISARETI TASINIR ==========================
///
/// Resmi sozlesmeye gore KITTI, kisa OXTS iletisim kesintilerinde kaydin TUM
/// degerlerini dogrusal olarak ara-degerler ve bunu son uc kipi
/// (`posmode`, `velmode`, `orimode`) -1 yaparak isaretler. Bu isaret
/// `OxtsRecord::interpolated_missing` ve oradan `DatasetEvent::source_interpolated`
/// olarak TASINIR.
///
/// CHECKPOINT A BU BAYRAGA GORE HICBIR SEY ATMAZ. Adaptorun isi veri seti
/// gercegini KAYIPSIZ tasimaktir; ara-degerlenmis bir GNSS'in olcum olarak
/// kullanilip kullanilmayacagi, IMU'nun ne yapilacagi ve referansin ATE'ye
/// girip girmeyecegi POLITIKA kararlaridir ve kosucuda (Checkpoint B) ACIKCA
/// yazilacaktir. Bayrak tasinmasaydi o karar sessizce "hepsini kullan" olurdu.
///
/// ============================== KAPSAM ======================================
///
/// Goruntu ve LiDAR AYRISTIRILMAZ. Yalnizca kestirime giren navigasyon verisi.

#include "kerteriz_bringup/dataset_event.hpp"
#include "kerteriz_bringup/geodetic.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace kerteriz_bringup {

/// Ayristirma sonucu. Sessiz kabul YOKTUR: bozuk tek bir satir bile
/// `ok = false` ve okunabilir bir mesaj uretir.
struct ParseStatus {
  bool ok = false;
  std::string message;

  static ParseStatus success() { return ParseStatus{true, {}}; }
  static ParseStatus failure(std::string m) { return ParseStatus{false, std::move(m)}; }
};

/// OXTS kaydinin bu fazda kullanilan alanlari.
struct OxtsRecord {
  Scalar lat_deg = 0, lon_deg = 0, alt_m = 0;
  Scalar roll = 0, pitch = 0, yaw = 0;
  Scalar vn = 0, ve = 0, vu = 0;
  Scalar ax = 0, ay = 0, az = 0;
  Scalar wx = 0, wy = 0, wz = 0;
  Scalar pos_accuracy = 0, vel_accuracy = 0;

  // Kategorik alanlar TAM TAMSAYIDIR. Kayan noktaya cevirip kirpmak "3.7"yi
  // sessizce 3 yapardi; bunlar sayisal buyukluk degil, kod degerleridir.
  int navstat = 0;
  int numsats = 0;
  int posmode = 0;
  int velmode = 0;
  int orimode = 0;

  /// Veri setinin KENDI eksik-bilgi isareti. KITTI kisa iletisim
  /// kesintilerinde tum alanlari dogrusal ara-degerler ve son uc kipi -1
  /// yapar. Burada yalnizca TASINIR; enum/aralik semantigi UYDURULMAZ.
  bool interpolated_missing = false;
};

inline constexpr int kOxtsFieldCount = 30;

/// Kategorik alanlarin ilk indeksi: navstat, numsats, posmode, velmode, orimode.
inline constexpr int kOxtsFirstCategoricalField = 25;

/// Eksik bilgi isaretcisi (KITTI sozlesmesi).
inline constexpr int kOxtsMissingMode = -1;

namespace detail {

/// Gunes takvimi -> UNIX epoch'undan gun sayisi. TAMSAYI aritmetigi;
/// hicbir yerde kayan nokta yoktur (Howard Hinnant, `days_from_civil`).
constexpr std::int64_t days_from_civil(std::int64_t y, unsigned m, unsigned d) {
  y -= m <= 2;
  const std::int64_t era = (y >= 0 ? y : y - 399) / 400;
  const auto yoe = static_cast<unsigned>(y - era * 400);
  const unsigned doy = (153U * (m + (m > 2 ? -3U : 9U)) + 2U) / 5U + d - 1U;
  const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;
  return era * 146097LL + static_cast<std::int64_t>(doe) - 719468LL;
}

/// TAM ONDALIK ISARETLI TAMSAYI. Ondalik nokta, ussel gosterim veya artik
/// simge iceren bir alan tamsayi DEGILDIR ve reddedilir.
///
/// NOT: NCLT tarafinda ayni sozlesmeyi zorlayan ayri bir ayristirici vardir.
/// Ortak bir yardimciya tasimak ayri bir temizlik isidir; bu adimin kapsami
/// disindadir.
inline bool tam_isaretli_sayi(const std::string& token, int& out) {
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
  for (; i < token.size(); ++i) {
    const auto c = static_cast<unsigned char>(token[i]);
    if (std::isdigit(c) == 0) {
      break;
    }
    deger = deger * 10 + (token[i] - '0');
    if (deger > 2147483647LL) {
      return false; // int araligini asiyor
    }
  }
  if (i == rakam_bas) {
    return false;
  }
  for (std::size_t k = i; k < token.size(); ++k) {
    if (std::isspace(static_cast<unsigned char>(token[k])) == 0) {
      return false;
    }
  }
  out = static_cast<int>(eksi ? -deger : deger);
  return true;
}

inline bool tam_sayi(const std::string& s, std::int64_t& out) {
  if (s.empty()) {
    return false;
  }
  for (const char c : s) {
    if (std::isdigit(static_cast<unsigned char>(c)) == 0) {
      return false;
    }
  }
  try {
    out = std::stoll(s);
  } catch (...) {
    return false;
  }
  return true;
}

} // namespace detail

/// "YYYY-MM-DD HH:MM:SS.fffffffff" -> TimeNs (UTC).
///
/// Kesir basamaklari 9'a TAMAMLANIR veya kirpilir; hicbir asamada `double`
/// saniye uretilmez (CONVENTIONS §6).
inline bool parse_kitti_timestamp(const std::string& satir, kerteriz::TimeNs& out) {
  // "2011-09-26 13:02:25.964389445"
  if (satir.size() < 19 || satir[4] != '-' || satir[7] != '-' || satir[10] != ' ' ||
      satir[13] != ':' || satir[16] != ':') {
    return false;
  }
  std::int64_t yil = 0, ay = 0, gun = 0, saat = 0, dakika = 0, saniye = 0;
  if (!detail::tam_sayi(satir.substr(0, 4), yil) || !detail::tam_sayi(satir.substr(5, 2), ay) ||
      !detail::tam_sayi(satir.substr(8, 2), gun) || !detail::tam_sayi(satir.substr(11, 2), saat) ||
      !detail::tam_sayi(satir.substr(14, 2), dakika) ||
      !detail::tam_sayi(satir.substr(17, 2), saniye)) {
    return false;
  }
  if (ay < 1 || ay > 12 || gun < 1 || gun > 31 || saat > 23 || dakika > 59 || saniye > 60) {
    return false;
  }

  std::int64_t ns = 0;
  if (satir.size() > 19) {
    if (satir[19] != '.') {
      return false;
    }
    std::string kesir = satir.substr(20);
    while (!kesir.empty() && std::isspace(static_cast<unsigned char>(kesir.back())) != 0) {
      kesir.pop_back();
    }
    if (kesir.empty() || kesir.size() > 9) {
      return false;
    }
    std::int64_t deger = 0;
    if (!detail::tam_sayi(kesir, deger)) {
      return false;
    }
    for (std::size_t i = kesir.size(); i < 9; ++i) {
      deger *= 10; // eksik basamaklar sifirla TAMAMLANIR
    }
    ns = deger;
  }

  const std::int64_t gunler =
      detail::days_from_civil(yil, static_cast<unsigned>(ay), static_cast<unsigned>(gun));
  out = ((gunler * 86400LL + saat * 3600LL + dakika * 60LL + saniye) * 1000000000LL) + ns;
  return true;
}

/// Tek bir OXTS satiri. TAM 30 alan bekler; eksik veya fazla alan HATADIR.
///
/// Alanlar once HAM SIMGE olarak bolunur: ilk 25'i sayisal buyukluktur, son
/// 5'i KATEGORIKTIR ve tam tamsayi olarak okunur. Hepsini kayan noktaya
/// cevirip kirpmak, "3.7" gibi bozuk bir kip degerini sessizce 3 yapardi.
inline bool parse_oxts_line(const std::string& satir, OxtsRecord& out) {
  std::istringstream akis(satir);
  std::vector<std::string> parcalar;
  parcalar.reserve(kOxtsFieldCount);
  std::string simge;
  while (akis >> simge) {
    parcalar.push_back(simge);
    if (parcalar.size() > static_cast<std::size_t>(kOxtsFieldCount)) {
      return false; // fazla alan
    }
  }
  if (parcalar.size() != static_cast<std::size_t>(kOxtsFieldCount)) {
    return false;
  }

  std::vector<Scalar> v(static_cast<std::size_t>(kOxtsFirstCategoricalField), Scalar(0));
  for (int i = 0; i < kOxtsFirstCategoricalField; ++i) {
    const auto& t = parcalar[static_cast<std::size_t>(i)];
    std::size_t tuketilen = 0;
    try {
      v[static_cast<std::size_t>(i)] = std::stod(t, &tuketilen);
    } catch (...) {
      return false;
    }
    if (tuketilen != t.size()) {
      return false; // "1.5abc" gibi kismi sayi
    }
  }

  int kategorik[5] = {};
  for (int i = 0; i < 5; ++i) {
    if (!detail::tam_isaretli_sayi(
            parcalar[static_cast<std::size_t>(kOxtsFirstCategoricalField + i)], kategorik[i])) {
      return false;
    }
  }

  out.lat_deg = v[0];
  out.lon_deg = v[1];
  out.alt_m = v[2];
  out.roll = v[3];
  out.pitch = v[4];
  out.yaw = v[5];
  out.vn = v[6];
  out.ve = v[7];
  out.vu = v[10];
  out.ax = v[11];
  out.ay = v[12];
  out.az = v[13];
  out.wx = v[17];
  out.wy = v[18];
  out.wz = v[19];
  out.pos_accuracy = v[23];
  out.vel_accuracy = v[24];
  out.navstat = kategorik[0];
  out.numsats = kategorik[1];
  out.posmode = kategorik[2];
  out.velmode = kategorik[3];
  out.orimode = kategorik[4];

  // KITTI sozlesmesi: kisa kesintide tum degerler dogrusal ara-degerlenir ve
  // son uc kip -1 yapilir.
  out.interpolated_missing = out.posmode == kOxtsMissingMode || out.velmode == kOxtsMissingMode ||
                             out.orimode == kOxtsMissingMode;
  return true;
}

/// R_WB = Rz(yaw) Ry(pitch) Rx(roll) — resmi devkit ile AYNI sira.
inline Eigen::Quaternion<Scalar> oxts_orientation(const OxtsRecord& r) {
  return Eigen::Quaternion<Scalar>(Eigen::AngleAxis<Scalar>(r.yaw, Vec3::UnitZ()) *
                                   Eigen::AngleAxis<Scalar>(r.pitch, Vec3::UnitY()) *
                                   Eigen::AngleAxis<Scalar>(r.roll, Vec3::UnitX()));
}

inline Llh oxts_llh(const OxtsRecord& r) {
  constexpr Scalar kDegToRad = 3.14159265358979323846 / 180.0;
  return Llh{r.lat_deg * kDegToRad, r.lon_deg * kDegToRad, r.alt_m};
}

/// Adaptor yapilandirmasi. Platforma ozel sayilar KODA GOMULMEZ.
struct KittiConfig {
  std::string dataset_dir; ///< `oxts/` iceren dizin
  bool emit_gnss_position = true;
  bool emit_gnss_velocity = true;
  bool emit_reference_pose = true;
  /// `pos_accuracy`/`vel_accuracy` alanlarina uygulanan olcek. Kovaryans
  /// sigma^2 I olarak kurulur; DUSEY icin ayri bir carpan UYDURULMAZ, veri
  /// seti boyle bir ayrim vermiyor.
  Scalar position_sigma_scale = 1.0;
  Scalar velocity_sigma_scale = 1.0;
  /// `pos_accuracy` sifir/negatif geldiginde kullanilacak taban sigma.
  Scalar min_position_sigma_m = 0.05;
  Scalar min_velocity_sigma_mps = 0.05;
  /// Orijin: bos ise ILK gecerli kayit orijin olur.
  bool explicit_origin = false;
  Llh origin{};
};

/// KITTI raw OXTS dizinini kanonik olaylara cevirir.
///
/// Olaylar damga sirasindadir. Bir kare icin IMU, GNSS ve referans AYNI
/// damgayi tasir — OXTS tek bir birlesik cozum yayinlar.
inline ParseStatus load_kitti_oxts(const KittiConfig& cfg, std::vector<DatasetEvent>& out) {
  out.clear();

  const std::string ts_yolu = cfg.dataset_dir + "/oxts/timestamps.txt";
  std::ifstream ts(ts_yolu);
  if (!ts) {
    return ParseStatus::failure("timestamps acilamadi: " + ts_yolu);
  }

  std::vector<kerteriz::TimeNs> damgalar;
  std::string satir;
  int no = 0;
  while (std::getline(ts, satir)) {
    ++no;
    if (satir.empty()) {
      continue;
    }
    kerteriz::TimeNs t = 0;
    if (!parse_kitti_timestamp(satir, t)) {
      return ParseStatus::failure(ts_yolu + ":" + std::to_string(no) + " bozuk zaman damgasi");
    }
    damgalar.push_back(t);
  }
  if (damgalar.empty()) {
    return ParseStatus::failure("timestamps bos: " + ts_yolu);
  }

  bool orijin_kuruldu = cfg.explicit_origin;
  Llh orijin = cfg.origin;

  std::vector<DatasetEvent> olaylar;
  olaylar.reserve(damgalar.size() * 3);

  for (std::size_t k = 0; k < damgalar.size(); ++k) {
    char ad[64];
    std::snprintf(ad, sizeof(ad), "/oxts/data/%010zu.txt", k);
    const std::string yol = cfg.dataset_dir + ad;

    std::ifstream f(yol);
    if (!f) {
      return ParseStatus::failure("OXTS karesi acilamadi: " + yol);
    }
    std::string icerik;
    if (!std::getline(f, icerik)) {
      return ParseStatus::failure("OXTS karesi bos: " + yol);
    }

    OxtsRecord r;
    if (!parse_oxts_line(icerik, r)) {
      return ParseStatus::failure(yol + ": OXTS satiri " + std::to_string(kOxtsFieldCount) +
                                  " alan icermiyor veya sayisal degil");
    }

    if (!orijin_kuruldu) {
      orijin = oxts_llh(r);
      orijin_kuruldu = true;
    }
    const EnuProjector izdusum(orijin);
    const Vec3 p_enu = izdusum.to_enu(oxts_llh(r));
    const kerteriz::TimeNs t = damgalar[k];

    { // IMU — govde cercevesi zaten REP-103
      DatasetEvent e;
      e.stamp_ns = t;
      e.kind = DatasetEventKind::kImu;
      e.imu = kerteriz::ImuSample{t, Vec3(r.wx, r.wy, r.wz), Vec3(r.ax, r.ay, r.az)};
      e.source_interpolated = r.interpolated_missing;
      olaylar.push_back(e);
    }
    if (cfg.emit_gnss_position) {
      DatasetEvent e;
      e.stamp_ns = t;
      e.kind = DatasetEventKind::kGnssPosition;
      e.position_w = p_enu;
      e.source_interpolated = r.interpolated_missing;
      const Scalar s =
          std::max(cfg.min_position_sigma_m, cfg.position_sigma_scale * r.pos_accuracy);
      e.position_cov_w = Eigen::Matrix<Scalar, 3, 3>::Identity() * (s * s);
      olaylar.push_back(e);
    }
    if (cfg.emit_gnss_velocity) {
      DatasetEvent e;
      e.stamp_ns = t;
      e.kind = DatasetEventKind::kGnssVelocity;
      e.velocity_w = Vec3(r.ve, r.vn, r.vu); // ENU
      e.source_interpolated = r.interpolated_missing;
      const Scalar s =
          std::max(cfg.min_velocity_sigma_mps, cfg.velocity_sigma_scale * r.vel_accuracy);
      e.velocity_cov_w = Eigen::Matrix<Scalar, 3, 3>::Identity() * (s * s);
      olaylar.push_back(e);
    }
    if (cfg.emit_reference_pose) {
      DatasetEvent e;
      e.stamp_ns = t;
      e.kind = DatasetEventKind::kReferencePose;
      e.position_w = p_enu;
      e.source_interpolated = r.interpolated_missing;
      e.orientation_wb = oxts_orientation(r);
      olaylar.push_back(e);
    }
  }

  std::stable_sort(olaylar.begin(), olaylar.end(), event_before);
  out = std::move(olaylar);
  return ParseStatus::success();
}

} // namespace kerteriz_bringup
