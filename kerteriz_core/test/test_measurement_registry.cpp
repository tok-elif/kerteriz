/// \file
/// S10 — olcum registry'si ve CLAUDE.md §6.1 Denetim (4).
///
/// Iki ayri sey sinanir:
///   1. MEKANIZMA: yerel `Registry` ornekleri uzerinde denetim fonksiyonlari
///      ihlali gercekten yakaliyor mu. Ihlalli durum kalici olarak test
///      paketindedir, elle bozup geri almaya gerek yoktur.
///   2. DENETIM: kuresel defterin kendisi temiz mi. Faz 1'de testsiz bir
///      olcum kaydedilirse bu testler duser.

#include "fake_measurements.hpp"
#include "kerteriz/measurements/registry.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/util/numeric_residual_jacobian.hpp"

#include <gtest/gtest.h>
#include <string>
#include <string_view>

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
using kerteriz::test_fakes::SahteGnss;
using kerteriz::test_fakes::SahteTeker;

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

// -----------------------------------------------------------------------------
// 1. Mekanizma — yerel defterler uzerinde
// -----------------------------------------------------------------------------

TEST(MeasurementRegistry, DenetimTestsizOlcumuYakalar) {
  // S10 DoD: sahte bir olcum tipiyle mekanizma dogrulanir.
  Registry yerel;
  yerel.add_measurement({"SahteTestsiz", "sahte_testsiz", __FILE__, __LINE__});

  const auto eksik = measurements_without_jacobian_test(yerel);
  ASSERT_EQ(eksik.size(), 1U) << "testsiz olcum yakalanmadi";
  EXPECT_EQ(eksik[0], "SahteTestsiz");
}

TEST(MeasurementRegistry, TestKaydedilinceDenetimTemizlenir) {
  Registry yerel;
  yerel.add_measurement({"SahteTestli", "sahte_testli", __FILE__, __LINE__});
  ASSERT_EQ(measurements_without_jacobian_test(yerel).size(), 1U);

  yerel.add_jacobian_test({"SahteTestli", nullptr, __FILE__, __LINE__});
  EXPECT_TRUE(measurements_without_jacobian_test(yerel).empty())
      << "test kaydedildi ama denetim hala ihlal bildiriyor";
}

TEST(MeasurementRegistry, YazimHatasiOlanTestAdiYakalanir) {
  // Iki liste de TEK BASINA bakildiginda doludur: bir olcum var, bir test var.
  // Esleme ise tutmaz. Denetim yalnizca sayilara baksaydi bunu kacirirdi.
  Registry yerel;
  yerel.add_measurement({"GnssPosition", "gnss_position", __FILE__, __LINE__});
  yerel.add_jacobian_test({"GnssPositon", nullptr, __FILE__, __LINE__});

  ASSERT_FALSE(yerel.measurements().empty());
  ASSERT_FALSE(yerel.jacobian_tests().empty());

  const auto eksik = measurements_without_jacobian_test(yerel);
  ASSERT_EQ(eksik.size(), 1U);
  EXPECT_EQ(eksik[0], "GnssPosition");

  const auto sahipsiz = jacobian_tests_without_measurement(yerel);
  ASSERT_EQ(sahipsiz.size(), 1U);
  EXPECT_EQ(sahipsiz[0], "GnssPositon");
}

TEST(MeasurementRegistry, AyniYamlAdiniIkiTipTalepEdemez) {
  Registry yerel;
  yerel.add_measurement({"Birinci", "gnss_position", __FILE__, __LINE__});
  yerel.add_measurement({"Ikinci", "gnss_position", __FILE__, __LINE__});

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
  ASSERT_EQ(defter.measurements().size(), 2U)
      << "kuresel defter beklenen kayitlari icermiyor — statik kayitlar "
         "baglayici tarafindan dusurulmus olabilir";

  bool gnss_var = false;
  bool teker_var = false;
  for (const auto& olcum : defter.measurements()) {
    gnss_var = gnss_var || olcum.type_name == "SahteGnss";
    teker_var = teker_var || olcum.type_name == "SahteTeker";
  }
  EXPECT_TRUE(gnss_var);
  EXPECT_TRUE(teker_var);
  EXPECT_EQ(defter.jacobian_tests().size(), 2U);
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
