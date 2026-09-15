/// \file
/// S10 — olcum registry'si ve CLAUDE.md §6.1 Denetim (4).
///
/// Iki ayri sey sinanir:
///   1. MEKANIZMA: yerel `Registry` ornekleri uzerinde denetim fonksiyonlari
///      ihlali gercekten yakaliyor mu. Ihlalli durum kalici olarak test
///      paketindedir, elle bozup geri almaya gerek yoktur.
///   2. DENETIM: kuresel defterin kendisi temiz mi. Testsiz bir olcum
///      kaydedilirse bu testler duser.
///
/// F1.6'dan itibaren kuresel defter SAHTE TIPLERDEN IBARET DEGILDIR: bes
/// GERCEK Faz 1 olcumu (GnssPosition, GnssVelocity, WheelVelocity,
/// NonHolonomic, ZeroVelocity) kayitlidir ve her birinin Jacobian test govdesi
/// gercek analitik/sayisal karsilastirmadir. Onceki hâlde denetim yalnizca
/// sahte tipleri goruyordu; uretimde GnssPosition varken kapsam disindaydi.

#include "fake_measurements.hpp"
#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz/measurements/gnss_velocity.hpp"
#include "kerteriz/measurements/non_holonomic.hpp"
#include "kerteriz/measurements/registry.hpp"
#include "kerteriz/measurements/wheel_velocity.hpp"
#include "kerteriz/measurements/zero_velocity.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/util/numeric_residual_jacobian.hpp"
#include "measurement_jacobian_check.hpp"

#include <algorithm>
#include <gtest/gtest.h>
#include <string>
#include <string_view>
#include <type_traits>

namespace {

using kerteriz::exp_map;
using kerteriz::JacMat;
using kerteriz::numeric_residual_jacobian;
using kerteriz::SE23;
using kerteriz::TangentVec;
using kerteriz::measurements::duplicate_config_names;
using kerteriz::measurements::global_registry;
using kerteriz::measurements::jacobian_tests_without_measurement;
using kerteriz::measurements::measurements_without_jacobian_test;
using kerteriz::measurements::Registry;
using kerteriz::measurements::type_id;
using kerteriz::test_fakes::SahteGnss;
using kerteriz::test_fakes::SahteTeker;
using kerteriz::test_support::jacobian_hatasi;
using kerteriz::test_support::ornek_durum_nav;

/// Yerel defter testleri icin tip kimligi tasiyicilari. Icerikleri onemsiz;
/// onemli olan FARKLI tipler olmalari.
struct TipA {};
struct TipB {};

/// Ayni yazimli, farkli ad alanlarinda iki GERCEK tip. Metin uzerinden
/// eslestirme bunlari birbirine karistirirdi.
namespace ad_alani_bir {
struct AyniIsim {};
} // namespace ad_alani_bir
namespace ad_alani_iki {
struct AyniIsim {};
} // namespace ad_alani_iki

SE23 ornek_durum() {
  TangentVec d;
  d << 0.10, -0.20, 0.30, 1.10, -0.40, 0.70, -2.30, 0.90, 1.70;
  return exp_map(d);
}

/// Faz 1 test govdelerinin sablonu: analitik Jacobian, sayisal olanla
/// karsilastirilir. Isaret ve pertürbasyon konvansiyonu (CONVENTIONS §3)
/// sayisal tarafla ortak oldugu icin sessiz uyusmazlik mumkun degildir.
template <typename Olcum>
void jacobian_karsilastir() {
  const SE23 x = ornek_durum();
  const JacMat sayisal =
      numeric_residual_jacobian(x, [](const SE23& p) { return Olcum::residual(p); }, Olcum::kDim);
  const auto analitik = Olcum::analytic_jacobian(x);

  for (int i = 0; i < Olcum::kDim; ++i) {
    for (int k = 0; k < SE23::DoF; ++k) {
      EXPECT_NEAR(sayisal(i, k), analitik(i, k), 1e-6) << "satir " << i << " sutun " << k;
    }
  }
}

bool kayitli_tip(const Registry& reg, std::string_view type_name) {
  const auto& m = reg.measurements();
  return std::any_of(m.begin(), m.end(),
                     [&](const auto& kayit) { return kayit.type_name == type_name; });
}

bool kayitli_config(const Registry& reg, std::string_view config_name) {
  const auto& m = reg.measurements();
  return std::any_of(m.begin(), m.end(),
                     [&](const auto& kayit) { return kayit.config_name == config_name; });
}

std::string listele(const std::vector<std::string_view>& adlar) {
  std::string s;
  for (const auto& ad : adlar) {
    s += " ";
    s.append(ad);
  }
  return s;
}

} // namespace

// -----------------------------------------------------------------------------
// Jacobian test kayitlari.
//
// Makro govde bekler; kayit ile testin kendisi TEK yapidir. Govdesiz bir kayit
// derlenmez, dolayisiyla "testi olmayan test kaydi" mumkun degildir.
// -----------------------------------------------------------------------------
KERTERIZ_REGISTER_JACOBIAN_TEST(SahteGnss) { jacobian_karsilastir<SahteGnss>(); }

KERTERIZ_REGISTER_JACOBIAN_TEST(SahteTeker) { jacobian_karsilastir<SahteTeker>(); }

// --- GERCEK Faz 1 olcumleri -------------------------------------------------
//
// Govdeler sayisal Jacobian'i NavState::plus uzerinden alir (ortak arac,
// measurement_jacobian_check.hpp) ve olcumun KENDI doldurdugu J_res ile
// karsilastirir. Tolerans model testleriyle AYNI siniftadir; denetim govdesi
// gevsetilmis bir kopya degildir.
//
// Durum kasten birim-olmayan yonelimli ve uc bileseni de sifirdan farkli govde
// hizina sahiptir: -R ile -I ayrimi ve [u]x capraz terimi boylece gercekten
// sinanir. Augmentation da eklenir — olcumler o sutunlara dokunmamalidir.

namespace {

kerteriz::NavState denetim_durumu() {
  kerteriz::NavState x = ornek_durum_nav();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  return x;
}

} // namespace

KERTERIZ_REGISTER_JACOBIAN_TEST(kerteriz::GnssPosition) {
  const kerteriz::GnssPosition z(1, "gnss_position", kerteriz::Vec3(20.0, -3.0, 5.0),
                                 Eigen::Matrix3d::Identity() * 2.25,
                                 kerteriz::Vec3(0.10, -0.25, 0.35));
  EXPECT_LT(jacobian_hatasi(z, denetim_durumu()), 1e-6);
}

KERTERIZ_REGISTER_JACOBIAN_TEST(kerteriz::GnssVelocity) {
  const kerteriz::GnssVelocity z(1, "gnss_velocity", kerteriz::Vec3(1.4, -0.7, 0.3),
                                 Eigen::Matrix3d::Identity() * 0.09);
  EXPECT_LT(jacobian_hatasi(z, denetim_durumu()), 1e-6);
}

KERTERIZ_REGISTER_JACOBIAN_TEST(kerteriz::WheelVelocity) {
  const kerteriz::WheelVelocity z(1, "wheel_velocity", 2.1, 0.04);
  EXPECT_LT(jacobian_hatasi(z, denetim_durumu()), 1e-6);
}

KERTERIZ_REGISTER_JACOBIAN_TEST(kerteriz::NonHolonomic) {
  Eigen::Matrix<kerteriz::Scalar, 2, 2> c;
  c << 0.04, 0.005, 0.005, 0.09;
  const kerteriz::NonHolonomic z(1, "non_holonomic", c);
  EXPECT_LT(jacobian_hatasi(z, denetim_durumu()), 1e-6);
}

KERTERIZ_REGISTER_JACOBIAN_TEST(kerteriz::ZeroVelocity) {
  const kerteriz::ZeroVelocity z(1, "zero_velocity", Eigen::Matrix3d::Identity() * 0.01);
  EXPECT_LT(jacobian_hatasi(z, denetim_durumu()), 1e-6);
}

// -----------------------------------------------------------------------------
// 1. Mekanizma — yerel defterler uzerinde
// -----------------------------------------------------------------------------

TEST(MeasurementRegistry, DenetimTestsizOlcumuYakalar) {
  // S10 DoD: sahte bir olcum tipiyle mekanizma dogrulanir.
  Registry yerel;
  yerel.add_measurement({type_id<TipA>(), "SahteTestsiz", "sahte_testsiz", __FILE__, __LINE__});

  const auto eksik = measurements_without_jacobian_test(yerel);
  ASSERT_EQ(eksik.size(), 1U) << "testsiz olcum yakalanmadi";
  EXPECT_EQ(eksik[0], "SahteTestsiz");
}

TEST(MeasurementRegistry, TestKaydedilinceDenetimTemizlenir) {
  Registry yerel;
  yerel.add_measurement({type_id<TipA>(), "SahteTestli", "sahte_testli", __FILE__, __LINE__});
  ASSERT_EQ(measurements_without_jacobian_test(yerel).size(), 1U);

  yerel.add_jacobian_test({type_id<TipA>(), "SahteTestli", nullptr, __FILE__, __LINE__});
  EXPECT_TRUE(measurements_without_jacobian_test(yerel).empty())
      << "test kaydedildi ama denetim hala ihlal bildiriyor";
}

TEST(MeasurementRegistry, BaskaTipeKaydedilmisTestYakalanir) {
  // Iki liste de TEK BASINA bakildiginda doludur: bir olcum var, bir test var.
  // Esleme ise tutmaz. Denetim yalnizca sayilara baksaydi bunu kacirirdi.
  Registry yerel;
  yerel.add_measurement({type_id<TipA>(), "GnssPosition", "gnss_position", __FILE__, __LINE__});
  yerel.add_jacobian_test({type_id<TipB>(), "GnssVelocity", nullptr, __FILE__, __LINE__});

  ASSERT_FALSE(yerel.measurements().empty());
  ASSERT_FALSE(yerel.jacobian_tests().empty());

  const auto eksik = measurements_without_jacobian_test(yerel);
  ASSERT_EQ(eksik.size(), 1U);
  EXPECT_EQ(eksik[0], "GnssPosition");

  const auto sahipsiz = jacobian_tests_without_measurement(yerel);
  ASSERT_EQ(sahipsiz.size(), 1U);
  EXPECT_EQ(sahipsiz[0], "GnssVelocity");
}

TEST(MeasurementRegistry, AyniYazimFarkliAdAlaniESLESMEZ) {
  // Metin uzerinden eslestirme bunu YANLIS gecirirdi: iki kayit da "AyniIsim"
  // etiketini tasiyor, ama tipler farkli. Denetim ikisini de ihlal saymali.
  Registry yerel;
  yerel.add_measurement(
      {type_id<ad_alani_bir::AyniIsim>(), "AyniIsim", "birinci", __FILE__, __LINE__});
  yerel.add_jacobian_test(
      {type_id<ad_alani_iki::AyniIsim>(), "AyniIsim", nullptr, __FILE__, __LINE__});

  ASSERT_NE(type_id<ad_alani_bir::AyniIsim>(), type_id<ad_alani_iki::AyniIsim>())
      << "tip kimligi ayirt edemiyor";

  EXPECT_EQ(measurements_without_jacobian_test(yerel).size(), 1U)
      << "ayni yazimli farkli tip yanlislikla eslesti";
  EXPECT_EQ(jacobian_tests_without_measurement(yerel).size(), 1U)
      << "ayni yazimli farkli tip yanlislikla eslesti";
}

TEST(MeasurementRegistry, AyniTipFarkliYazimlarlaESLESIR) {
  // Metin uzerinden eslestirme bunu YANLIS DUSURURDU: ayni gercek tip, biri
  // nitelikli biri niteliksiz yazilmis. Anahtar tipin kendisi oldugu icin
  // yazim farki onemsizdir.
  Registry yerel;
  yerel.add_measurement(
      {type_id<ad_alani_bir::AyniIsim>(), "ad_alani_bir::AyniIsim", "tek", __FILE__, __LINE__});
  yerel.add_jacobian_test(
      {type_id<ad_alani_bir::AyniIsim>(), "AyniIsim", nullptr, __FILE__, __LINE__});

  EXPECT_TRUE(measurements_without_jacobian_test(yerel).empty())
      << "ayni tip farkli yazildi diye eslesemedi";
  EXPECT_TRUE(jacobian_tests_without_measurement(yerel).empty())
      << "ayni tip farkli yazildi diye eslesemedi";
}

TEST(MeasurementRegistry, AyniYamlAdiniIkiTipTalepEdemez) {
  Registry yerel;
  yerel.add_measurement({type_id<TipA>(), "Birinci", "gnss_position", __FILE__, __LINE__});
  yerel.add_measurement({type_id<TipB>(), "Ikinci", "gnss_position", __FILE__, __LINE__});

  const auto cift = duplicate_config_names(yerel);
  ASSERT_EQ(cift.size(), 1U) << "fabrika anahtari cakismasi yakalanmadi";
  EXPECT_EQ(cift[0], "gnss_position");
}

TEST(MeasurementRegistry, BosDefterTemizdir) {
  const Registry yerel;
  EXPECT_TRUE(measurements_without_jacobian_test(yerel).empty());
  EXPECT_TRUE(jacobian_tests_without_measurement(yerel).empty());
  EXPECT_TRUE(duplicate_config_names(yerel).empty());
}

// -----------------------------------------------------------------------------
// 2. Denetim (4) — kuresel defter
// -----------------------------------------------------------------------------

TEST(Denetim4, KayitlarBaskaCeviriBirimindenGorulur) {
  // Kayitlar fake_measurements.cpp'den, Jacobian testleri BU dosyadan gelir.
  // Bos bir defter uzerinde denetim bos doner ve YESIL gorunurdu; bu proje o
  // sinif hatayi (kerteriz_eval'in 0 testle gecmesi, BUILD_TESTING sizintisi)
  // birkac kez yasadi. O yuzden once defterin DOLU oldugu dogrulanir.
  const auto& defter = global_registry();
  ASSERT_EQ(defter.measurements().size(), 7U)
      << "kuresel defter beklenen kayitlari icermiyor — statik kayitlar "
         "baglayici tarafindan dusurulmus olabilir";
  EXPECT_EQ(defter.jacobian_tests().size(), 7U);

  for (const auto beklenen : {"SahteGnss", "SahteTeker"}) {
    EXPECT_TRUE(kayitli_tip(defter, beklenen)) << "mekanizma kaydi kayip: " << beklenen;
  }
}

TEST(Denetim4, GercekFaz1OlcumleriKapsamda) {
  // Denetim (4) bir donem YALNIZCA sahte tipleri goruyordu; uretimde
  // GnssPosition varken kapsam disindaydi. Bu test o bosluga karsi bekcidir:
  // bes gercek Faz 1 olcumu kanonik config adlariyla kayitli olmali.
  const auto& defter = global_registry();

  for (const auto beklenen :
       {"gnss_position", "gnss_velocity", "wheel_velocity", "non_holonomic", "zero_velocity"}) {
    EXPECT_TRUE(kayitli_config(defter, beklenen)) << "gercek olcum kaydi kayip: " << beklenen;
  }

  // Ve her biri GERCEKTEN Measurement turevidir — kayit makrosu bunu tek
  // basina zorlamaz, tam tip olmasi yeterlidir.
  static_assert(std::is_base_of<kerteriz::Measurement, kerteriz::GnssPosition>::value, "");
  static_assert(std::is_base_of<kerteriz::Measurement, kerteriz::GnssVelocity>::value, "");
  static_assert(std::is_base_of<kerteriz::Measurement, kerteriz::WheelVelocity>::value, "");
  static_assert(std::is_base_of<kerteriz::Measurement, kerteriz::NonHolonomic>::value, "");
  static_assert(std::is_base_of<kerteriz::Measurement, kerteriz::ZeroVelocity>::value, "");
}

TEST(Denetim4, HerKayitliOlcumunJacobianTestiVar) {
  const auto& defter = global_registry();
  const auto eksik = measurements_without_jacobian_test(defter);
  EXPECT_TRUE(eksik.empty()) << "Jacobian testi olmayan olcum tipleri:" << listele(eksik);
}

TEST(Denetim4, SahipsizJacobianTestiYok) {
  const auto& defter = global_registry();
  const auto sahipsiz = jacobian_tests_without_measurement(defter);
  EXPECT_TRUE(sahipsiz.empty()) << "Karsiligi olmayan Jacobian testleri:" << listele(sahipsiz);
}

TEST(Denetim4, YamlAdlariBenzersiz) {
  const auto& defter = global_registry();
  const auto cift = duplicate_config_names(defter);
  EXPECT_TRUE(cift.empty()) << "Cakisan YAML adlari:" << listele(cift);
}

TEST(Denetim4, KayitliJacobianTestleriGercektenKosar) {
  // Testlerin VARLIGINI saymak yetmez; kaydi bos bir beyana cevirirdi.
  const auto& defter = global_registry();
  ASSERT_FALSE(defter.jacobian_tests().empty());

  for (const auto& test : defter.jacobian_tests()) {
    SCOPED_TRACE(std::string(test.type_name));
    ASSERT_NE(test.run, nullptr);
    test.run();
  }
}
