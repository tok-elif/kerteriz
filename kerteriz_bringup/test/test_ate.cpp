/// \file
/// F1.8 — asgari oteleme ATE olcutu.

#include "kerteriz_bringup/ate.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

namespace {

using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::Vec3;
using kerteriz_bringup::TrajectorySample;
using kerteriz_bringup::translational_ate;

constexpr TimeNs kMs = 1000000;

TEST(Ate, ZeroWhenTrajectoriesAreIdentical) {
  std::vector<TrajectorySample> a;
  for (int i = 0; i < 10; ++i) {
    a.push_back({i * 100 * kMs, Vec3(i * 1.0, i * -0.5, 0.25)});
  }
  const auto r = translational_ate(a, a, kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.matched, 10);
  EXPECT_EQ(r.unmatched, 0);
  EXPECT_LT(r.rmse_m, 1e-15);
  EXPECT_LT(r.max_error_m, 1e-15);
}

TEST(Ate, ConstantOffsetGivesThatOffset) {
  std::vector<TrajectorySample> ref;
  std::vector<TrajectorySample> est;
  const Vec3 ofset(3.0, 4.0, 0.0); // normu tam 5
  for (int i = 0; i < 6; ++i) {
    ref.push_back({i * 100 * kMs, Vec3(i * 2.0, 0.0, 0.0)});
    est.push_back({i * 100 * kMs, Vec3(i * 2.0, 0.0, 0.0) + ofset});
  }
  const auto r = translational_ate(ref, est, kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.rmse_m, 5.0, 1e-12);
  EXPECT_NEAR(r.max_error_m, 5.0, 1e-12);
}

TEST(Ate, RmseIsQuadraticMeanNotArithmeticMean) {
  // Hatalar 0 ve 10: aritmetik ortalama 5, RMSE sqrt(50) ~ 7.071.
  const std::vector<TrajectorySample> ref = {{0, Vec3::Zero()}, {100 * kMs, Vec3::Zero()}};
  const std::vector<TrajectorySample> est = {{0, Vec3::Zero()}, {100 * kMs, Vec3(10.0, 0, 0)}};
  const auto r = translational_ate(ref, est, kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.rmse_m, std::sqrt(50.0), 1e-12);
  EXPECT_NEAR(r.max_error_m, 10.0, 1e-12);
}

TEST(Ate, MatchesNearestStampWithinTolerance) {
  const std::vector<TrajectorySample> ref = {{100 * kMs, Vec3::Zero()}};
  // 99 ms ve 104 ms adaylari: 99 ms daha yakin.
  const std::vector<TrajectorySample> est = {{99 * kMs, Vec3(1.0, 0, 0)},
                                             {104 * kMs, Vec3(7.0, 0, 0)}};
  const auto r = translational_ate(ref, est, 10 * kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.matched, 1);
  EXPECT_NEAR(r.rmse_m, 1.0, 1e-12) << "en yakin damga secilmedi";
}

TEST(Ate, SamplesOutsideToleranceAreExcludedNotSilentlyMatched) {
  const std::vector<TrajectorySample> ref = {{100 * kMs, Vec3::Zero()}, {900 * kMs, Vec3::Zero()}};
  const std::vector<TrajectorySample> est = {{100 * kMs, Vec3(2.0, 0, 0)}};
  const auto r = translational_ate(ref, est, 10 * kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.matched, 1);
  EXPECT_EQ(r.unmatched, 1) << "toleransin disindaki ornek sessizce eslendi";
  EXPECT_NEAR(r.rmse_m, 2.0, 1e-12);
}

TEST(Ate, TieGoesToTheEarlierStampDeterministically) {
  const std::vector<TrajectorySample> ref = {{100 * kMs, Vec3::Zero()}};
  const std::vector<TrajectorySample> est = {{95 * kMs, Vec3(1.0, 0, 0)},
                                             {105 * kMs, Vec3(9.0, 0, 0)}};
  const auto r = translational_ate(ref, est, 10 * kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.rmse_m, 1.0, 1e-12) << "esitlikte erken damga secilmedi";
}

TEST(Ate, ReportsNotOkWhenNothingMatches) {
  const std::vector<TrajectorySample> ref = {{0, Vec3::Zero()}};
  const std::vector<TrajectorySample> est = {{10000 * kMs, Vec3::Zero()}};
  const auto r = translational_ate(ref, est, kMs);
  EXPECT_FALSE(r.ok) << "hicbir esleme yokken basarili raporlandi";
  EXPECT_EQ(r.matched, 0);
  EXPECT_EQ(r.unmatched, 1);

  EXPECT_FALSE(translational_ate({}, est, kMs).ok);
  EXPECT_FALSE(translational_ate(ref, {}, kMs).ok);
}

TEST(Ate, NoAlignmentIsApplied) {
  // Sabit bir oteleme ATE'yi DUSURMEZ. Rijit/Sim(3) hizalama uygulansaydi
  // bu test sifira duserdi; hizalama yoklugu boylece kanitlanir.
  std::vector<TrajectorySample> ref;
  std::vector<TrajectorySample> est;
  for (int i = 0; i < 20; ++i) {
    const Vec3 p(i * 0.5, std::sin(i * 0.3), 0.0);
    ref.push_back({i * 50 * kMs, p});
    est.push_back({i * 50 * kMs, p + Vec3(2.0, 0.0, 0.0)});
  }
  const auto r = translational_ate(ref, est, kMs);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.rmse_m, 2.0, 1e-12);
}

} // namespace
