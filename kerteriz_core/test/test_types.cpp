/// \file
/// S5 — INTERFACES §0 tipleri ve ArrayView birim testleri.

#include <array>
#include <type_traits>

#include <gtest/gtest.h>

#include "kerteriz/types.hpp"

namespace {

using kerteriz::ArrayView;

// --- Denetim (2): durum kapasitesi -----------------------------------------

TEST(Types, DurumKapasitesiTutarli) {
  // static_assert derleme zamaninda zaten dustu; burada degerler belgelenir.
  EXPECT_EQ(kerteriz::kCoreDof, 15);
  EXPECT_EQ(kerteriz::kMaxAugmentDof, 48);
  EXPECT_EQ(kerteriz::kMaxStateDof, kerteriz::kCoreDof + kerteriz::kMaxAugmentDof);
  EXPECT_EQ(kerteriz::kMaxStateDof, 63);
  EXPECT_GT(kerteriz::kMaxResidualDim, 0);
  EXPECT_LE(kerteriz::kMaxResidualDim, kerteriz::kMaxStateDof);
}

TEST(Types, MatrisBoyutlariSabitVeDogru) {
  // CONVENTIONS §8.1 / ADR-16: hepsi derleme zamani sabit boyutlu olmalidir.
  EXPECT_EQ(kerteriz::StateVec::RowsAtCompileTime, kerteriz::kMaxStateDof);
  EXPECT_EQ(kerteriz::StateMat::RowsAtCompileTime, kerteriz::kMaxStateDof);
  EXPECT_EQ(kerteriz::StateMat::ColsAtCompileTime, kerteriz::kMaxStateDof);
  EXPECT_EQ(kerteriz::ResVec::RowsAtCompileTime, kerteriz::kMaxResidualDim);
  EXPECT_EQ(kerteriz::ResMat::ColsAtCompileTime, kerteriz::kMaxResidualDim);
  EXPECT_EQ(kerteriz::JacMat::RowsAtCompileTime, kerteriz::kMaxResidualDim);
  EXPECT_EQ(kerteriz::JacMat::ColsAtCompileTime, kerteriz::kMaxStateDof);

  // Dinamik boyut (-1) hicbirinde olmamali.
  EXPECT_NE(kerteriz::StateMat::RowsAtCompileTime, Eigen::Dynamic);
  EXPECT_NE(kerteriz::JacMat::ColsAtCompileTime, Eigen::Dynamic);
}

TEST(Types, ZamanTamsayidirDoubleDegildir) {
  // CONVENTIONS §6: timestamp asla double saniye degildir.
  EXPECT_TRUE(std::is_integral<kerteriz::TimeNs>::value);
  EXPECT_EQ(sizeof(kerteriz::TimeNs), 8U);
}

// --- ArrayView (DoD: bos, tek eleman, aralik tabanli for) -------------------

TEST(ArrayViewTest, VarsayilanBostur) {
  ArrayView<int> v;
  EXPECT_TRUE(v.empty());
  EXPECT_EQ(v.size(), 0U);
  EXPECT_EQ(v.data(), nullptr);
  EXPECT_EQ(v.begin(), v.end());
}

TEST(ArrayViewTest, BosAralikTabanliForHicDonmez) {
  ArrayView<int> v;
  int tur = 0;
  for (int x : v) {
    (void)x;
    ++tur;
  }
  EXPECT_EQ(tur, 0);
}

TEST(ArrayViewTest, TekEleman) {
  const std::array<int, 1> kaynak{42};
  ArrayView<int> v(kaynak.data(), kaynak.size());

  EXPECT_FALSE(v.empty());
  EXPECT_EQ(v.size(), 1U);
  EXPECT_EQ(v[0], 42);
  EXPECT_EQ(v.data(), kaynak.data());
  EXPECT_EQ(v.end() - v.begin(), 1);
}

TEST(ArrayViewTest, AralikTabanliForSirayiKorur) {
  const std::array<int, 4> kaynak{3, 1, 4, 1};
  ArrayView<int> v(kaynak.data(), kaynak.size());

  std::array<int, 4> gorulen{};
  std::size_t i = 0;
  for (int x : v) {
    gorulen[i++] = x;
  }

  EXPECT_EQ(i, 4U);
  EXPECT_EQ(gorulen, kaynak);
}

TEST(ArrayViewTest, KopyalamazSadeceGosterir) {
  // Tahsissiz gorunum: ayni bellegi gosterir, sahiplenmez.
  std::array<int, 2> kaynak{7, 8};
  ArrayView<int> v(kaynak.data(), kaynak.size());

  kaynak[1] = 99;
  EXPECT_EQ(v[1], 99) << "ArrayView kopya tutuyor — tahsissiz gorunum degil";
}

TEST(ArrayViewTest, DerlemeZamaninda) {
  // constexpr sozlesmesi: bos gorunum derleme zamaninda degerlendirilebilmeli.
  constexpr ArrayView<int> v;
  static_assert(v.empty(), "ArrayView::empty constexpr degil");
  static_assert(v.size() == 0, "ArrayView::size constexpr degil");
  SUCCEED();
}

TEST(ArrayViewTest, KullaniciTipiyleCalisir) {
  const std::array<kerteriz::WeakDirection, 2> yonler{
      kerteriz::WeakDirection{"yaw", 0.5}, kerteriz::WeakDirection{"wheel_scale", 0.1}};
  ArrayView<kerteriz::WeakDirection> v(yonler.data(), yonler.size());

  ASSERT_EQ(v.size(), 2U);
  EXPECT_EQ(v[0].label, "yaw");
  EXPECT_DOUBLE_EQ(v[1].sigma, 0.1);
}

// --- Kucuk tipler -----------------------------------------------------------

TEST(Types, CloneIdGuclutiptirVeGecersizDegerAyridir) {
  EXPECT_FALSE((std::is_convertible<kerteriz::CloneId, int>::value))
      << "CloneId ortuk olarak int'e donusuyor — guclu tip degil";
  EXPECT_EQ(static_cast<std::int32_t>(kerteriz::kInvalidClone), -1);
  EXPECT_NE(kerteriz::kInvalidClone, kerteriz::CloneId{0});
}

TEST(Types, ImuSampleAlanlari) {
  const kerteriz::ImuSample s{123456789LL, kerteriz::Vec3::Zero(), kerteriz::Vec3(0, 0, 9.81)};
  EXPECT_EQ(s.stamp_ns, 123456789LL);
  EXPECT_TRUE(s.gyro.isZero());
  EXPECT_DOUBLE_EQ(s.accel.z(), 9.81);
}

TEST(Types, EstimatorModeBesDurum) {
  // ADR-18: sistem seviyesi mod. SensorHealth ile karistirilmaz.
  EXPECT_NE(kerteriz::EstimatorMode::kNominal, kerteriz::EstimatorMode::kDegraded);
  EXPECT_NE(kerteriz::EstimatorMode::kUninitialized, kerteriz::EstimatorMode::kInitializing);
  EXPECT_NE(kerteriz::EstimatorMode::kDegraded, kerteriz::EstimatorMode::kFaulted);
}

} // namespace
