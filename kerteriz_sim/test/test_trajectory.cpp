/// \file
/// S9 — analitik yorunge. Kapali formun kendi icinde tutarli oldugunu
/// dogrular: hiz konumun turevi, ivme hizin turevi, w_B yonelimin turevi.

#include "kerteriz_sim/trajectory.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace {

using kerteriz::TimeNs;
using kerteriz_sim::Scalar;
using kerteriz_sim::TrajectoryGenerator;
using kerteriz_sim::TrajectoryKind;
using kerteriz_sim::TrajectoryParams;

constexpr TimeNs kSaniye = 1000000000LL;

TrajectoryGenerator uretec(TrajectoryKind kind) {
  TrajectoryParams p;
  p.kind = kind;
  p.radius = 8.0;
  p.angular_rate = 0.35;
  p.height = 2.0;
  p.linear_velocity = kerteriz_sim::Vec3(1.5, -0.5, 0.25);
  return TrajectoryGenerator(p);
}

/// Merkezi farkla sayisal turev.
template <typename Fn>
kerteriz_sim::Vec3 turev(Fn&& f, TimeNs t, TimeNs h) {
  return (f(t + h) - f(t - h)) / (Scalar(2) * static_cast<Scalar>(h) * Scalar(1e-9));
}

class TrajectoryKindTest : public ::testing::TestWithParam<TrajectoryKind> {};

TEST_P(TrajectoryKindTest, VelocityIsDerivativeOfPosition) {
  const auto g = uretec(GetParam());
  const TimeNs h = kSaniye / 10000;
  for (TimeNs t = kSaniye; t < 6 * kSaniye; t += kSaniye) {
    const auto sayisal = turev([&](TimeNs s) { return g.at(s).position; }, t, h);
    EXPECT_LT((sayisal - g.at(t).velocity).norm(), 1e-6) << "t = " << t;
  }
}

TEST_P(TrajectoryKindTest, AccelerationIsDerivativeOfVelocity) {
  const auto g = uretec(GetParam());
  const TimeNs h = kSaniye / 10000;
  for (TimeNs t = kSaniye; t < 6 * kSaniye; t += kSaniye) {
    const auto sayisal = turev([&](TimeNs s) { return g.at(s).velocity; }, t, h);
    EXPECT_LT((sayisal - g.at(t).acceleration).norm(), 1e-6) << "t = " << t;
  }
}

TEST_P(TrajectoryKindTest, AngularVelocityMatchesOrientationDerivative) {
  // R_dot = R * skew(w_B)  =>  skew(w_B) = R^T R_dot
  const auto g = uretec(GetParam());
  const TimeNs h = kSaniye / 10000;
  const Scalar dt = Scalar(2) * static_cast<Scalar>(h) * Scalar(1e-9);

  for (TimeNs t = kSaniye; t < 6 * kSaniye; t += kSaniye) {
    const Eigen::Matrix3d r = g.at(t).rotation;
    const Eigen::Matrix3d r_nokta = (g.at(t + h).rotation - g.at(t - h).rotation) / dt;
    const Eigen::Matrix3d egik = r.transpose() * r_nokta;

    const kerteriz_sim::Vec3 w_sayisal(egik(2, 1), egik(0, 2), egik(1, 0));
    EXPECT_LT((w_sayisal - g.at(t).angular_velocity).norm(), 1e-5) << "t = " << t;
  }
}

TEST_P(TrajectoryKindTest, RotationIsProperOrthonormal) {
  const auto g = uretec(GetParam());
  for (TimeNs t = 0; t < 5 * kSaniye; t += kSaniye / 2) {
    const Eigen::Matrix3d r = g.at(t).rotation;
    EXPECT_LT((r * r.transpose() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
  }
}

TEST_P(TrajectoryKindTest, BodyXAxisFollowsVelocity) {
  // REP-103: x_B ileridir. Duz ucusta hiz yonu ile hizali olmali.
  const auto g = uretec(GetParam());
  for (TimeNs t = kSaniye; t < 5 * kSaniye; t += kSaniye) {
    const auto s = g.at(t);
    const kerteriz_sim::Vec3 yatay(s.velocity.x(), s.velocity.y(), 0.0);
    if (yatay.norm() < 1e-6) {
      continue;
    }
    const kerteriz_sim::Vec3 x_b = s.rotation.col(0);
    EXPECT_NEAR(x_b.dot(yatay.normalized()), 1.0, 1e-9) << "t = " << t;
  }
}

INSTANTIATE_TEST_SUITE_P(AllKinds, TrajectoryKindTest,
                         ::testing::Values(TrajectoryKind::kConstantVelocity,
                                           TrajectoryKind::kCircle, TrajectoryKind::kFigureEight));

TEST(Trajectory, CircleHasConstantRadiusAndSpeed) {
  const auto g = uretec(TrajectoryKind::kCircle);
  for (TimeNs t = 0; t < 5 * kSaniye; t += kSaniye / 4) {
    const auto s = g.at(t);
    EXPECT_NEAR(kerteriz_sim::Vec3(s.position.x(), s.position.y(), 0.0).norm(), 8.0, 1e-12);
    EXPECT_NEAR(s.velocity.norm(), 8.0 * 0.35, 1e-12);
  }
}

TEST(Trajectory, ConstantVelocityHasNoRotationRate) {
  const auto g = uretec(TrajectoryKind::kConstantVelocity);
  for (TimeNs t = 0; t < 5 * kSaniye; t += kSaniye) {
    EXPECT_LT(g.at(t).angular_velocity.norm(), 1e-15);
    EXPECT_LT(g.at(t).acceleration.norm(), 1e-15);
  }
}

} // namespace
