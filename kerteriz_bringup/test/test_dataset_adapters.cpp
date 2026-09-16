/// \file
/// F1.8 — KITTI OXTS ve NCLT adaptorleri.
///
/// SENTETIK FIXTURE kullanir (bkz. test/fixtures/OKUBENI.md). Fixture'lar dosya
/// BICIMINI temsil eder; gercek veri seti icerigi veya benchmark sonucu
/// DEGILDIR. Buradan hicbir dogruluk iddiasi cikarilmaz — sinanan sey
/// ayristirici mekanigidir: damga aritmetigi, alan sayisi, birim/cerceve
/// donusumu, olay siralamasi ve BOZUK SATIR REDDI.

#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz/measurements/gnss_velocity.hpp"
#include "kerteriz_bringup/kitti_oxts.hpp"
#include "kerteriz_bringup/nclt.hpp"

#include <cmath>
#include <cstdint>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::Vec3;
using kerteriz_bringup::DatasetEvent;
using kerteriz_bringup::DatasetEventKind;
using kerteriz_bringup::EnuProjector;
using kerteriz_bringup::KittiConfig;
using kerteriz_bringup::Llh;
using kerteriz_bringup::NcltConfig;
using kerteriz_bringup::OxtsRecord;
using kerteriz_bringup::parse_csv_row;
using kerteriz_bringup::parse_kitti_timestamp;
using kerteriz_bringup::parse_nclt_gps_line;
using kerteriz_bringup::parse_nclt_ms25_line;
using kerteriz_bringup::parse_oxts_line;

std::string fixture(const std::string& alt) {
  return std::string(KERTERIZ_FIXTURE_DIR) + "/" + alt;
}

int say(const std::vector<DatasetEvent>& v, DatasetEventKind k) {
  int n = 0;
  for (const auto& e : v) {
    n += (e.kind == k) ? 1 : 0;
  }
  return n;
}

// =============================================================================
// KITTI — zaman damgasi
// =============================================================================

TEST(KittiTimestamp, ParsesToExactNanoseconds) {
  TimeNs t = 0;
  ASSERT_TRUE(parse_kitti_timestamp("1970-01-01 00:00:00.000000000", t));
  EXPECT_EQ(t, 0) << "UNIX epoch tam sifir olmali";

  ASSERT_TRUE(parse_kitti_timestamp("1970-01-01 00:00:01.000000001", t));
  EXPECT_EQ(t, 1000000001LL);

  ASSERT_TRUE(parse_kitti_timestamp("2011-09-26 13:02:25.964389445", t));
  // 2011-09-26 13:02:25 UTC = 1317042145 s.
  EXPECT_EQ(t, 1317042145LL * 1000000000LL + 964389445LL);
}

TEST(KittiTimestamp, DifferenceIsExactAndIntegral) {
  // dt DAIMA iki TimeNs farkindan gelir; kayan noktaya ugramaz.
  TimeNs a = 0;
  TimeNs b = 0;
  ASSERT_TRUE(parse_kitti_timestamp("2011-09-26 13:02:25.964389445", a));
  ASSERT_TRUE(parse_kitti_timestamp("2011-09-26 13:02:26.068765183", b));
  EXPECT_EQ(b - a, 104375738LL);
}

TEST(KittiTimestamp, ShortFractionIsZeroPaddedNotTruncated) {
  TimeNs t = 0;
  ASSERT_TRUE(parse_kitti_timestamp("1970-01-01 00:00:00.5", t));
  EXPECT_EQ(t, 500000000LL) << ".5 saniye 500 ms olmali";
}

TEST(KittiTimestamp, RejectsMalformed) {
  TimeNs t = 0;
  EXPECT_FALSE(parse_kitti_timestamp("", t));
  EXPECT_FALSE(parse_kitti_timestamp("2011-09-26", t));
  EXPECT_FALSE(parse_kitti_timestamp("2011/09/26 13:02:25.0", t));
  EXPECT_FALSE(parse_kitti_timestamp("2011-09-26T13:02:25.0", t));
  EXPECT_FALSE(parse_kitti_timestamp("2011-13-26 13:02:25.0", t)) << "13. ay";
  EXPECT_FALSE(parse_kitti_timestamp("2011-09-26 25:02:25.0", t)) << "25. saat";
  EXPECT_FALSE(parse_kitti_timestamp("2011-09-26 13:02:25.9643894451", t)) << "10 basamak kesir";
  EXPECT_FALSE(parse_kitti_timestamp("2011-09-26 13:02:25.abc", t));
  EXPECT_FALSE(parse_kitti_timestamp("20x1-09-26 13:02:25.0", t));
}

// =============================================================================
// KITTI — OXTS satiri
// =============================================================================

TEST(KittiOxts, RequiresExactlyThirtyFields) {
  OxtsRecord r;
  std::string tam;
  for (int i = 0; i < 30; ++i) {
    tam += std::to_string(i) + " ";
  }
  EXPECT_TRUE(parse_oxts_line(tam, r));

  std::string eksik;
  for (int i = 0; i < 29; ++i) {
    eksik += std::to_string(i) + " ";
  }
  EXPECT_FALSE(parse_oxts_line(eksik, r)) << "29 alan kabul edildi";

  EXPECT_FALSE(parse_oxts_line(tam + " 30", r)) << "31 alan kabul edildi";
  EXPECT_FALSE(parse_oxts_line("", r));
  EXPECT_FALSE(parse_oxts_line(tam.substr(0, tam.size() - 2) + " abc", r)) << "sayisal olmayan";
}

TEST(KittiOxts, MapsFieldsByDocumentedIndex) {
  // Her alani kendi indeksine esit yaparak indeks haritasini dogrudan sinar.
  OxtsRecord r;
  std::string s;
  for (int i = 0; i < 30; ++i) {
    s += std::to_string(i) + " ";
  }
  ASSERT_TRUE(parse_oxts_line(s, r));

  EXPECT_EQ(r.lat_deg, 0);
  EXPECT_EQ(r.lon_deg, 1);
  EXPECT_EQ(r.alt_m, 2);
  EXPECT_EQ(r.roll, 3);
  EXPECT_EQ(r.pitch, 4);
  EXPECT_EQ(r.yaw, 5);
  EXPECT_EQ(r.vn, 6);
  EXPECT_EQ(r.ve, 7);
  EXPECT_EQ(r.vu, 10);
  EXPECT_EQ(r.ax, 11);
  EXPECT_EQ(r.ay, 12);
  EXPECT_EQ(r.az, 13);
  EXPECT_EQ(r.wx, 17);
  EXPECT_EQ(r.wy, 18);
  EXPECT_EQ(r.wz, 19);
  EXPECT_EQ(r.pos_accuracy, 23);
  EXPECT_EQ(r.vel_accuracy, 24);
  EXPECT_EQ(r.navstat, 25);
  EXPECT_EQ(r.numsats, 26);
}

/// 30 alanli bir OXTS satiri kurar; istenen indeksler degistirilebilir.
std::string oxts_satiri(const std::map<int, std::string>& degisiklik) {
  std::string s;
  for (int i = 0; i < 30; ++i) {
    const auto it = degisiklik.find(i);
    s += (it == degisiklik.end() ? std::to_string(i) : it->second);
    s += " ";
  }
  return s;
}

TEST(KittiOxts, CategoricalFieldsAreExactIntegers) {
  // navstat, numsats, posmode, velmode, orimode SAYISAL BUYUKLUK DEGIL, kod
  // degerleridir. Kayan noktaya cevirip kirpmak "3.7"yi sessizce 3 yapardi.
  OxtsRecord r;
  ASSERT_TRUE(parse_oxts_line(oxts_satiri({}), r));
  EXPECT_EQ(r.navstat, 25);
  EXPECT_EQ(r.numsats, 26);
  EXPECT_EQ(r.posmode, 27);
  EXPECT_EQ(r.velmode, 28);
  EXPECT_EQ(r.orimode, 29);
  EXPECT_FALSE(r.interpolated_missing);

  for (int alan = 25; alan < 30; ++alan) {
    for (const char* bozuk : {"3.7", "2e0", "abc", "", "5.0"}) {
      EXPECT_FALSE(parse_oxts_line(oxts_satiri({{alan, bozuk}}), r))
          << "alan " << alan << " kabul etti: '" << bozuk << "'";
    }
  }
}

TEST(KittiOxts, MinusOneIsAValidMissingMarkerNotAParseError) {
  OxtsRecord r;
  ASSERT_TRUE(parse_oxts_line(oxts_satiri({{27, "-1"}, {28, "-1"}, {29, "-1"}}), r));
  EXPECT_EQ(r.posmode, -1);
  EXPECT_EQ(r.velmode, -1);
  EXPECT_EQ(r.orimode, -1);
  EXPECT_TRUE(r.interpolated_missing);

  // Tek bir -1 de yeterlidir.
  ASSERT_TRUE(parse_oxts_line(oxts_satiri({{28, "-1"}}), r));
  EXPECT_TRUE(r.interpolated_missing);
  ASSERT_TRUE(parse_oxts_line(oxts_satiri({{25, "-1"}}), r));
  EXPECT_FALSE(r.interpolated_missing) << "navstat isaretci alanlarindan biri DEGIL";
}

TEST(KittiOxts, OrientationFollowsDevkitZyxOrder) {
  OxtsRecord r;
  r.roll = 0.0;
  r.pitch = 0.0;
  r.yaw = 1.5707963267948966; // +90 derece
  const auto q = kerteriz_bringup::oxts_orientation(r);
  const Eigen::Matrix<Scalar, 3, 3> m = q.toRotationMatrix();

  // yaw 0 = DOGU, pozitif saat yonunun tersi. +90 derece donunce govde x
  // (ileri) dunya +y'yi (kuzey) gostermeli.
  const Vec3 ileri = m * Vec3::UnitX();
  EXPECT_NEAR(ileri.x(), 0.0, 1e-12);
  EXPECT_NEAR(ileri.y(), 1.0, 1e-12);
  EXPECT_NEAR(ileri.z(), 0.0, 1e-12);

  // Sira Rz*Ry*Rx: yalniz roll varken ileri ekseni DEGISMEZ.
  OxtsRecord s;
  s.roll = 0.4;
  const Vec3 ileri_roll = kerteriz_bringup::oxts_orientation(s).toRotationMatrix() * Vec3::UnitX();
  EXPECT_NEAR(ileri_roll.x(), 1.0, 1e-12);
}

// =============================================================================
// KITTI — dizin yukleme
// =============================================================================

TEST(KittiLoader, LoadsFixtureIntoOrderedCanonicalEvents) {
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_sentetik");
  std::vector<DatasetEvent> olaylar;

  const auto d = kerteriz_bringup::load_kitti_oxts(cfg, olaylar);
  ASSERT_TRUE(d.ok) << d.message;

  EXPECT_EQ(say(olaylar, DatasetEventKind::kImu), 3);
  EXPECT_EQ(say(olaylar, DatasetEventKind::kGnssPosition), 3);
  EXPECT_EQ(say(olaylar, DatasetEventKind::kGnssVelocity), 3);
  EXPECT_EQ(say(olaylar, DatasetEventKind::kReferencePose), 3);
  EXPECT_EQ(olaylar.size(), 12U);

  for (std::size_t i = 1; i < olaylar.size(); ++i) {
    EXPECT_LE(olaylar[i - 1].stamp_ns, olaylar[i].stamp_ns) << "olaylar damga sirasinda degil";
  }
  // Ayni damgada IMU once gelmeli.
  EXPECT_EQ(olaylar[0].kind, DatasetEventKind::kImu);
}

TEST(KittiLoader, FirstFixBecomesOriginAndEnuSignsAreCorrect) {
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_sentetik");
  std::vector<DatasetEvent> olaylar;
  ASSERT_TRUE(kerteriz_bringup::load_kitti_oxts(cfg, olaylar).ok);

  std::vector<const DatasetEvent*> gnss;
  for (const auto& e : olaylar) {
    if (e.kind == DatasetEventKind::kGnssPosition) {
      gnss.push_back(&e);
    }
  }
  ASSERT_EQ(gnss.size(), 3U);

  EXPECT_LT(gnss[0]->position_w.norm(), 1e-9) << "ilk fix orijin olmali";
  // Fixture kuzeye ve doguya ilerliyor, ucuncu karede yukselti artiyor.
  EXPECT_GT(gnss[1]->position_w.x(), 0.0) << "dogu";
  EXPECT_GT(gnss[1]->position_w.y(), 0.0) << "kuzey";
  EXPECT_GT(gnss[2]->position_w.x(), gnss[1]->position_w.x());
  EXPECT_GT(gnss[2]->position_w.z(), 0.4) << "yukselti artisi yukari olmali";
}

TEST(KittiLoader, ImuUsesBodyAxesAndVelocityIsEnu) {
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_sentetik");
  std::vector<DatasetEvent> olaylar;
  ASSERT_TRUE(kerteriz_bringup::load_kitti_oxts(cfg, olaylar).ok);

  const DatasetEvent* imu = nullptr;
  const DatasetEvent* hiz = nullptr;
  for (const auto& e : olaylar) {
    if (imu == nullptr && e.kind == DatasetEventKind::kImu) {
      imu = &e;
    }
    if (hiz == nullptr && e.kind == DatasetEventKind::kGnssVelocity) {
      hiz = &e;
    }
  }
  ASSERT_NE(imu, nullptr);
  ASSERT_NE(hiz, nullptr);

  // ax/ay/az ve wx/wy/wz dogrudan: KITTI govde ekseni zaten REP-103.
  EXPECT_NEAR(imu->imu.accel.x(), 0.3, 1e-12);
  EXPECT_NEAR(imu->imu.accel.y(), -0.2, 1e-12);
  EXPECT_NEAR(imu->imu.accel.z(), 9.79, 1e-12);
  EXPECT_NEAR(imu->imu.gyro.x(), 0.01, 1e-12);
  EXPECT_NEAR(imu->imu.gyro.z(), 0.03, 1e-12);
  EXPECT_EQ(imu->imu.stamp_ns, imu->stamp_ns);

  // ENU = (ve, vn, vu); fixture'da ve=2.0, vn=1.0, vu=0.05.
  EXPECT_NEAR(hiz->velocity_w.x(), 2.0, 1e-12);
  EXPECT_NEAR(hiz->velocity_w.y(), 1.0, 1e-12);
  EXPECT_NEAR(hiz->velocity_w.z(), 0.05, 1e-12);
}

TEST(KittiLoader, CovarianceUsesReportedAccuracyWithFloor) {
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_sentetik");
  cfg.min_position_sigma_m = 0.01; // taban, bildirilen 0.04'un ALTINDA olmali
  std::vector<DatasetEvent> olaylar;
  ASSERT_TRUE(kerteriz_bringup::load_kitti_oxts(cfg, olaylar).ok);

  std::vector<const DatasetEvent*> gnss;
  for (const auto& e : olaylar) {
    if (e.kind == DatasetEventKind::kGnssPosition) {
      gnss.push_back(&e);
    }
  }
  ASSERT_EQ(gnss.size(), 3U);
  EXPECT_NEAR(gnss[1]->position_cov_w(0, 0), 0.04 * 0.04, 1e-15) << "pos_accuracy kullanilmadi";
  // Ucuncu karede pos_accuracy = 0; taban sigma devreye girmeli.
  EXPECT_NEAR(gnss[2]->position_cov_w(0, 0), 0.01 * 0.01, 1e-15) << "taban sigma uygulanmadi";
}

TEST(KittiLoader, RejectsMalformedFrameInsteadOfSkipping) {
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_bozuk");
  std::vector<DatasetEvent> olaylar;

  const auto d = kerteriz_bringup::load_kitti_oxts(cfg, olaylar);
  EXPECT_FALSE(d.ok) << "bozuk kare sessizce atlandi";
  EXPECT_NE(d.message.find("0000000001"), std::string::npos) << d.message;
}

TEST(KittiLoader, OutageFrameMarksEveryDerivedEventAsInterpolated) {
  // Fixture'in 3. karesi (indeks 2) bilerek posmode/velmode/orimode = -1'dir.
  // KITTI o karenin TUM degerlerini dogrusal ara-degerlemistir; bayrak o
  // kareden uretilen HER olayda tasinmalidir.
  //
  // Checkpoint A hicbir seyi ATMAZ: dusurulseydi kayip bilgi sessizce yok
  // olur, politika karari da kosucuda gorunmez hale gelirdi.
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_sentetik");
  std::vector<DatasetEvent> olaylar;
  ASSERT_TRUE(kerteriz_bringup::load_kitti_oxts(cfg, olaylar).ok);
  ASSERT_EQ(olaylar.size(), 12U) << "kesinti karesi dusurulmus";

  const TimeNs kesinti_ns = olaylar.back().stamp_ns;
  int isaretli = 0;
  int temiz = 0;
  for (const auto& e : olaylar) {
    if (e.stamp_ns == kesinti_ns) {
      EXPECT_TRUE(e.source_interpolated) << "kesinti karesinden uretilen olay isaretlenmemis";
      ++isaretli;
    } else {
      EXPECT_FALSE(e.source_interpolated) << "saglam kare yanlislikla isaretlenmis";
      ++temiz;
    }
  }
  EXPECT_EQ(isaretli, 4) << "IMU + GNSS konum + GNSS hiz + referans";
  EXPECT_EQ(temiz, 8);
}

TEST(KittiLoader, MissingDirectoryIsAnError) {
  KittiConfig cfg;
  cfg.dataset_dir = fixture("boyle_bir_dizin_yok");
  std::vector<DatasetEvent> olaylar;
  EXPECT_FALSE(kerteriz_bringup::load_kitti_oxts(cfg, olaylar).ok);
}

// =============================================================================
// NCLT
// =============================================================================

TEST(NcltTime, UtimeIsParsedAsExactIntegerWithoutFloatingPoint) {
  // Mutlak zaman hicbir asamada kayan noktadan GECMEZ (CONVENTIONS §6).
  // 16 haneli UTIME, double'in tam tamsayi araliginin (~9.0e15) sinirindadir;
  // ondalik/ussel gosterim veya artik simge tamsayi degildir ve REDDEDILIR.
  std::int64_t v = 0;
  ASSERT_TRUE(kerteriz_bringup::parse_int64_token("1326036000000000", v));
  EXPECT_EQ(v, 1326036000000000LL);

  EXPECT_FALSE(kerteriz_bringup::parse_int64_token("1326036000000000.5", v)) << "ondalik";
  EXPECT_FALSE(kerteriz_bringup::parse_int64_token("1.326036e15", v)) << "ussel gosterim";
  EXPECT_FALSE(kerteriz_bringup::parse_int64_token("1326036000000000abc", v)) << "artik simge";
  EXPECT_FALSE(kerteriz_bringup::parse_int64_token("", v));
  EXPECT_FALSE(kerteriz_bringup::parse_int64_token("abc", v));
  EXPECT_FALSE(kerteriz_bringup::parse_int64_token("99999999999999999999", v)) << "tasma";

  // Isaret ayristirilir; negatif deger UTIME olarak sonra reddedilir.
  ASSERT_TRUE(kerteriz_bringup::parse_int64_token("-1", v));
  EXPECT_EQ(v, -1);
  kerteriz::TimeNs t = 0;
  EXPECT_FALSE(kerteriz_bringup::nclt_utime_to_ns(v, t)) << "negatif utime kabul edildi";
}

TEST(NcltTime, RowParsersRejectNonIntegerUtime) {
  // Hem ms25 hem GPS yolu AYNI tam ayristiriciyi kullanmali.
  DatasetEvent e;
  EXPECT_FALSE(
      parse_nclt_ms25_line("1326036000000000.5,0.1,0.2,0.3,1.0,2.0,-9.8,0.01,0.02,0.03", e));
  EXPECT_FALSE(parse_nclt_ms25_line("1.326036e15,0.1,0.2,0.3,1.0,2.0,-9.8,0.01,0.02,0.03", e));
  EXPECT_FALSE(parse_nclt_ms25_line("-1,0.1,0.2,0.3,1.0,2.0,-9.8,0.01,0.02,0.03", e));

  const EnuProjector izdusum(Llh{0.738168521, -1.461022891, 270.0});
  bool gecerli = false;
  EXPECT_FALSE(parse_nclt_gps_line("1326036000000000.5,3,9,0.7381,-1.4610,270.0,0,1.4", izdusum,
                                   3.0, Vec3::Zero(), e, gecerli));
  EXPECT_FALSE(parse_nclt_gps_line("1.326036e15,3,9,0.7381,-1.4610,270.0,0,1.4", izdusum, 3.0,
                                   Vec3::Zero(), e, gecerli));
  EXPECT_FALSE(parse_nclt_gps_line("-1,3,9,0.7381,-1.4610,270.0,0,1.4", izdusum, 3.0, Vec3::Zero(),
                                   e, gecerli));
}

TEST(NcltTime, UtimeMicrosecondsBecomeNanoseconds) {
  TimeNs t = 0;
  ASSERT_TRUE(kerteriz_bringup::nclt_utime_to_ns(1326036000000000LL, t));
  EXPECT_EQ(t, 1326036000000000LL * 1000LL);

  ASSERT_TRUE(kerteriz_bringup::nclt_utime_to_ns(0, t));
  EXPECT_EQ(t, 0);

  EXPECT_FALSE(kerteriz_bringup::nclt_utime_to_ns(-1, t)) << "negatif utime";
  EXPECT_FALSE(kerteriz_bringup::nclt_utime_to_ns(std::numeric_limits<std::int64_t>::max(), t))
      << "tasma denetlenmedi";
}

TEST(NcltCsv, RequiresExactFieldCountAndRejectsGarbage) {
  std::vector<Scalar> v;
  EXPECT_TRUE(parse_csv_row("1,2,3", 3, v));
  EXPECT_EQ(v.size(), 3U);
  EXPECT_FALSE(parse_csv_row("1,2", 3, v)) << "eksik alan";
  EXPECT_FALSE(parse_csv_row("1,2,3,4", 3, v)) << "fazla alan";
  EXPECT_FALSE(parse_csv_row("1,abc,3", 3, v)) << "sayisal olmayan";
  EXPECT_FALSE(parse_csv_row("1,2.5abc,3", 3, v)) << "kismi sayi kabul edildi";
  EXPECT_FALSE(parse_csv_row("1,,3", 3, v)) << "bos alan";
}

TEST(NcltMs25, ConvertsBodyAxesToRep103) {
  // NCLT: x ileri, y SAG, z ASAGI  ->  REP-103: x ileri, y SOL, z YUKARI.
  DatasetEvent e;
  ASSERT_TRUE(parse_nclt_ms25_line("1326036000000000,0.1,0.2,0.3,1.0,2.0,-9.8,0.01,0.02,0.03", e));

  EXPECT_EQ(e.kind, DatasetEventKind::kImu);
  EXPECT_EQ(e.stamp_ns, 1326036000000000LL * 1000LL);
  EXPECT_NEAR(e.imu.accel.x(), 1.0, 1e-15);
  EXPECT_NEAR(e.imu.accel.y(), -2.0, 1e-15) << "y isareti cevrilmedi";
  EXPECT_NEAR(e.imu.accel.z(), 9.8, 1e-15) << "z isareti cevrilmedi";
  EXPECT_NEAR(e.imu.gyro.x(), 0.01, 1e-15);
  EXPECT_NEAR(e.imu.gyro.y(), -0.02, 1e-15);
  EXPECT_NEAR(e.imu.gyro.z(), -0.03, 1e-15);
}

TEST(NcltMs25, RejectsWrongFieldCount) {
  DatasetEvent e;
  EXPECT_FALSE(parse_nclt_ms25_line("1326036000000000,0.1,0.2,0.3,1.0,2.0", e));
}

TEST(NcltGps, FixModeMustBeAnExactIntegerInTheDocumentedRange) {
  // Kayan noktaya cevirip kirpmak "3.7"yi sessizce 3 yapardi: gecersiz bir
  // kayit tam 3D fix gibi gorunurdu.
  int m = -99;
  for (const char* gecerli : {"0", "1", "2", "3"}) {
    EXPECT_TRUE(kerteriz_bringup::parse_nclt_fix_mode(gecerli, m)) << gecerli;
  }
  for (const char* gecersiz : {"-1", "4", "2.5", "3.7", "3e0", "abc", "", " "}) {
    EXPECT_FALSE(kerteriz_bringup::parse_nclt_fix_mode(gecersiz, m))
        << "kabul edildi: " << gecersiz;
  }

  // Satir yolundan da gecerli: bozuk fix modu AYRISTIRMA HATASIDIR.
  const EnuProjector izdusum(Llh{0.738168521, -1.461022891, 270.0});
  DatasetEvent e;
  bool gecerli = false;
  EXPECT_FALSE(parse_nclt_gps_line("1326036000000000,3.7,9,0.7381,-1.4610,270.0,0,1.4", izdusum,
                                   3.0, Vec3::Zero(), e, gecerli));
  EXPECT_FALSE(parse_nclt_gps_line("1326036000000000,4,9,0.7381,-1.4610,270.0,0,1.4", izdusum, 3.0,
                                   Vec3::Zero(), e, gecerli));
}

TEST(NcltParsing, NonFiniteFieldsAreRejected) {
  // stod "nan"/"inf" metinlerini kabul eder; boyle bir deger durumu ve
  // kovaryansi tek adimda zehirlerdi. Bozuk veri ayristiricida durur.
  Scalar x = 0;
  EXPECT_FALSE(kerteriz_bringup::parse_scalar_token("nan", x));
  EXPECT_FALSE(kerteriz_bringup::parse_scalar_token("inf", x));
  EXPECT_FALSE(kerteriz_bringup::parse_scalar_token("-inf", x));
  EXPECT_TRUE(kerteriz_bringup::parse_scalar_token("-1.25", x));

  DatasetEvent e;
  EXPECT_FALSE(parse_nclt_ms25_line("1326036000000000,0.1,0.2,0.3,nan,2.0,-9.8,0.01,0.02,0.03", e))
      << "NaN ivme";
  EXPECT_FALSE(parse_nclt_ms25_line("1326036000000000,0.1,0.2,0.3,1.0,2.0,-9.8,inf,0.02,0.03", e))
      << "Inf jiro";

  const EnuProjector izdusum(Llh{0.738168521, -1.461022891, 270.0});
  bool gecerli = false;
  EXPECT_FALSE(parse_nclt_gps_line("1326036000000000,3,9,nan,-1.4610,270.0,0,1.4", izdusum, 3.0,
                                   Vec3::Zero(), e, gecerli))
      << "NaN enlem";
  EXPECT_FALSE(parse_nclt_gps_line("1326036000000000,3,9,0.7381,-1.4610,inf,0,1.4", izdusum, 3.0,
                                   Vec3::Zero(), e, gecerli))
      << "Inf yukseklik";
}

TEST(NcltGps, ThreeDimensionalEventNeedsFixModeThree) {
  // Tablo 7: 2 = enlem/boylam iyi, 3 = YUKSEKLIK DE iyi. Faz 1 GnssPosition
  // uc boyutlu oldugu icin esik 3'tur. Mod 2, buyuk bir dusey sigma ile
  // 3D'ye ZORLANMAZ ve iki boyutlu bir olcum modeli de eklenmez.
  const EnuProjector izdusum(Llh{0.738168521, -1.461022891, 270.0});
  DatasetEvent e;
  bool gecerli = true;

  ASSERT_TRUE(parse_nclt_gps_line("1326036001000000,1,2,0,0,0,0,0", izdusum, 3.0, Vec3::Zero(), e,
                                  gecerli));
  EXPECT_FALSE(gecerli) << "fix modu 1 kullanilabilir sayildi";

  gecerli = true;
  ASSERT_TRUE(parse_nclt_gps_line("1326036001200000,2,7,0.738168696,-1.461022891,0.0,0,1.4",
                                  izdusum, 3.0, Vec3::Zero(), e, gecerli));
  EXPECT_FALSE(gecerli) << "fix modu 2 (yukseklik GECERSIZ) 3D olcume cevrildi";

  ASSERT_TRUE(parse_nclt_gps_line("1326036000000000,3,9,0.738168521,-1.461022891,270.0,0,1.4",
                                  izdusum, 3.0, Vec3::Zero(), e, gecerli));
  EXPECT_TRUE(gecerli) << "fix modu 3 reddedildi";
  EXPECT_LT(e.position_w.norm(), 1e-9) << "orijin fix'i [0,0,0] vermeli";
  EXPECT_NEAR(e.position_cov_w(1, 1), 9.0, 1e-12);
}

TEST(NcltLoader, OriginSkipsEarlierInvalidFixes) {
  // Fixture'in ILK kaydi bilerek mod 2'dir ve LLH'si mod 3 kayitlarindan
  // FARKLIDIR. Orijin ondan kurulsaydi ilk 3D konum [0,0,0] CIKMAZDI —
  // yani bu test gercekten atlamanin olup olmadigini ayirt eder.
  NcltConfig cfg;
  cfg.gps_csv = fixture("nclt_sentetik/gps_rtk.csv");
  std::vector<DatasetEvent> olaylar;
  ASSERT_TRUE(kerteriz_bringup::load_nclt(cfg, olaylar).ok);

  ASSERT_EQ(olaylar.size(), 3U) << "mod 1 ve mod 2 kayitlari olay uretmemeli";
  EXPECT_EQ(olaylar.front().stamp_ns, 1326036000200000LL * 1000LL)
      << "ilk olay, ilk MOD 3 kaydinin damgasini tasimali";
  EXPECT_LT(olaylar.front().position_w.norm(), 1e-9)
      << "orijin ilk 3D fix'ten kurulmadi: " << olaylar.front().position_w.transpose();
}

TEST(NcltGps, LatLonAreRadiansNotDegrees) {
  // Ayni sayisal degeri derece sansaydik konum Dunya'nin disina duserdi.
  const EnuProjector izdusum(Llh{0.738168521, -1.461022891, 270.0});
  DatasetEvent e;
  bool gecerli = false;
  ASSERT_TRUE(parse_nclt_gps_line("1326036000500000,3,9,0.738168696,-1.461022891,270.0,0,1.4",
                                  izdusum, 3.0, Vec3::Zero(), e, gecerli));
  ASSERT_TRUE(gecerli);
  // 1.75e-7 rad ~ 1.1 m kuzey.
  EXPECT_GT(e.position_w.y(), 0.5);
  EXPECT_LT(e.position_w.y(), 2.0);
  EXPECT_LT(std::abs(e.position_w.x()), 1e-3);
}

TEST(NcltLoader, LoadsFixtureAndOrdersEvents) {
  NcltConfig cfg;
  cfg.ms25_csv = fixture("nclt_sentetik/ms25.csv");
  cfg.gps_csv = fixture("nclt_sentetik/gps_rtk.csv");
  std::vector<DatasetEvent> olaylar;

  const auto d = kerteriz_bringup::load_nclt(cfg, olaylar);
  ASSERT_TRUE(d.ok) << d.message;

  EXPECT_EQ(say(olaylar, DatasetEventKind::kImu), 3);
  EXPECT_EQ(say(olaylar, DatasetEventKind::kGnssPosition), 3)
      << "fix modu 1 ve 2 olan satirlar atlanmali (3D icin mod 3 gerekir)";
  EXPECT_EQ(say(olaylar, DatasetEventKind::kWheelVelocity), 0) << "teker kanali Faz 1'de ERTELENDI";
  EXPECT_EQ(say(olaylar, DatasetEventKind::kReferencePose), 0) << "yer gercegi ERTELENDI";

  for (std::size_t i = 1; i < olaylar.size(); ++i) {
    EXPECT_LE(olaylar[i - 1].stamp_ns, olaylar[i].stamp_ns);
  }
}

TEST(NcltLoader, RejectsMalformedRowsInsteadOfSkipping) {
  {
    NcltConfig cfg;
    cfg.ms25_csv = fixture("nclt_sentetik/ms25_bozuk.csv");
    std::vector<DatasetEvent> olaylar;
    const auto d = kerteriz_bringup::load_nclt(cfg, olaylar);
    EXPECT_FALSE(d.ok) << "bozuk ms25 satiri sessizce atlandi";
  }
  {
    NcltConfig cfg;
    cfg.gps_csv = fixture("nclt_sentetik/gps_bozuk.csv");
    std::vector<DatasetEvent> olaylar;
    const auto d = kerteriz_bringup::load_nclt(cfg, olaylar);
    EXPECT_FALSE(d.ok) << "bozuk gps satiri sessizce atlandi";
  }
}

TEST(NcltLoader, DefaultLeverArmComesFromDocumentedSensorTable) {
  // Tablo 4: x_body,rtk - x_body,imu, sonra REP-103'e cevrilir.
  const NcltConfig cfg;
  const Vec3 beklenen = kerteriz_bringup::nclt_body_to_rep103(
      Vec3(kerteriz_bringup::kNcltRtkOffsetBodyNclt - kerteriz_bringup::kNcltImuOffsetBodyNclt));
  EXPECT_LT((cfg.gnss_lever_arm_b - beklenen).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_NEAR(cfg.gnss_lever_arm_b.x(), -0.13, 1e-12);
  EXPECT_NEAR(cfg.gnss_lever_arm_b.y(), -0.18, 1e-12);
  EXPECT_NEAR(cfg.gnss_lever_arm_b.z(), 0.53, 1e-12);
}

// =============================================================================
// Kanonik olay sozlesmesi — filtreye veri seti sizmiyor
// =============================================================================

TEST(CanonicalEvents, FeedExistingPhase1MeasurementConstructors) {
  // Veri seti olaylarindan MEVCUT olcum siniflari kuruluyor; yeni bir olcum
  // modeli tanimlanmiyor. Bu test o sozlesmenin derleme zamani kanitidir.
  KittiConfig cfg;
  cfg.dataset_dir = fixture("kitti_sentetik");
  std::vector<DatasetEvent> olaylar;
  ASSERT_TRUE(kerteriz_bringup::load_kitti_oxts(cfg, olaylar).ok);

  int kurulan = 0;
  for (const auto& e : olaylar) {
    if (e.kind == DatasetEventKind::kGnssPosition) {
      const kerteriz::GnssPosition z(e.stamp_ns, "gnss_position", e.position_w, e.position_cov_w);
      EXPECT_EQ(z.residual_dim(), 3);
      ++kurulan;
    } else if (e.kind == DatasetEventKind::kGnssVelocity) {
      const kerteriz::GnssVelocity z(e.stamp_ns, "gnss_velocity", e.velocity_w, e.velocity_cov_w);
      EXPECT_EQ(z.residual_dim(), 3);
      ++kurulan;
    }
  }
  EXPECT_EQ(kurulan, 6);
}

} // namespace
