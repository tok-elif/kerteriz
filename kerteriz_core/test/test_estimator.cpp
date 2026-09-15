/// \file
/// F1.7 — Estimator orkestratoru. INTERFACES §5.1 · ADR-15, ADR-19, ADR-20.
///
/// Sinanan: predict delegasyonu, UpdateStatus -> RejectReason eslemesi ve tam
/// snapshot round-trip'i. Kapilama/dislama Faz 4'tedir ve BURADA YOKTUR.

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/buffer/estimator.hpp"
#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz/measurements/zero_velocity.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <memory>
#include <string_view>
#include <utility>

namespace {

using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::Estimator;
using kerteriz::FaultDetector;
using kerteriz::FaultState;
using kerteriz::GnssPosition;
using kerteriz::ImuNoiseParams;
using kerteriz::ImuSample;
using kerteriz::Measurement;
using kerteriz::MeasurementWorkspace;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::NisMonitor;
using kerteriz::NisState;
using kerteriz::PipelineSnapshot;
using kerteriz::ProcessingResult;
using kerteriz::RejectReason;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::StateBundle;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;
using kerteriz::ZeroVelocity;

constexpr Scalar kGuven = 0.997;
constexpr TimeNs kT0 = 1'000'000'000;

NavState ornek_durum() {
  NavState x;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.41, Vec3(0.2, -0.7, 0.68).normalized()));
  x.extended_pose() = SE23(Vec3(4.0, -2.0, 1.5), q, Vec3(1.3, 0.6, -0.2));
  x.gyro_bias() = Vec3(0.004, -0.002, 0.001);
  x.accel_bias() = Vec3(-0.02, 0.03, 0.01);
  return x;
}

EskfConfig config(const NavState& x, Scalar p_diag = 1.0) {
  NavCovariance p = NavCovariance::Zero();
  const int n = x.active_dof();
  p.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * p_diag;
  return EskfConfig{x, p, kT0, ImuNoiseParams{1e-3, 1e-5, 2e-2, 3e-4}, kGuven};
}

Estimator kestirimci(const NavState& x, Scalar p_diag = 1.0) {
  return Estimator(std::make_unique<EskfBackend>(config(x, p_diag)), NisMonitor{}, FaultDetector{});
}

/// S = J P J^T + R pozitif tanimli COZULEMEYECEK bir olcum.
///
/// Neden test-yerel: gercek Faz 1 olcumlerinin hicbiri bunu uretemez — J
/// tam siralidir ve R pozitif tanimlidir. Yani kNumericalFailure eslemesi
/// gercek zincir uzerinden hic gozlemlenemezdi ve yanlis eslenmis olsa bile
/// test yesil kalirdi.
class DejenereOlcum final : public Measurement {
 public:
  TimeNs stamp_ns() const override { return 2000; }
  int residual_dim() const override { return 3; }
  std::string_view name() const override { return "dejenere"; }

  void evaluate(const StateBundle&, MeasurementWorkspace& w) const override {
    w.dim = 3;
    w.r.head<3>() = Vec3(1.0, 1.0, 1.0);
    // J = 0 ve R = 0  ->  S = 0, pozitif tanimli degil.
  }
};

// -----------------------------------------------------------------------------
// predict — yalnizca delegasyon
// -----------------------------------------------------------------------------

TEST(Estimator, PredictDelegatesToBackend) {
  const NavState x = ornek_durum();
  Estimator est = kestirimci(x);

  EskfBackend referans(config(x));
  const ImuSample u{kT0 + 10'000'000, Vec3(0.02, -0.01, 0.03), Vec3(0.1, -0.2, 9.9)};
  const Scalar dt = 0.01;

  est.predict(u, dt);
  referans.predict(u, dt);

  EXPECT_EQ(est.backend().stamp_ns(), u.stamp_ns) << "mutlak zaman IMU damgasindan gelmeli";
  EXPECT_LT(est.backend().state().minus(referans.state()).norm(), 1e-18);
  EXPECT_LT((est.backend().covariance() - referans.covariance()).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_EQ(est.mode(), referans.mode());
}

// -----------------------------------------------------------------------------
// UpdateStatus -> RejectReason
// -----------------------------------------------------------------------------

TEST(Estimator, MapsAcceptedToNone) {
  const NavState x = ornek_durum();
  Estimator est = kestirimci(x);

  const Vec3 z_w = x.extended_pose().translation() + Vec3(0.4, -0.2, 0.1);
  const GnssPosition z(2000, "gnss_position", z_w, Eigen::Matrix3d::Identity() * 2.25);

  const ProcessingResult r = est.apply(z);

  EXPECT_EQ(r.reason, RejectReason::kNone);
  EXPECT_EQ(r.sensor, "gnss_position");
  EXPECT_EQ(r.stamp_ns, 2000);
  ASSERT_TRUE(r.update.has_value());
  EXPECT_EQ(r.update->status, UpdateStatus::kAccepted);
  EXPECT_EQ(r.update->dof, 3);
}

TEST(Estimator, MapsChiSquareRejectedToChiSquareGate) {
  const NavState x = ornek_durum();
  Estimator est = kestirimci(x, 0.25);

  const Vec3 z_w = x.extended_pose().translation() + Vec3(500.0, -300.0, 100.0);
  const GnssPosition z(2000, "gnss_position", z_w, Eigen::Matrix3d::Identity() * 2.25);

  const ProcessingResult r = est.apply(z);

  EXPECT_EQ(r.reason, RejectReason::kChiSquareGate);
  ASSERT_TRUE(r.update.has_value());
  EXPECT_EQ(r.update->status, UpdateStatus::kChiSquareRejected);
  EXPECT_GT(*r.update->nis, r.update->threshold);
}

TEST(Estimator, MapsNumericalFailureToNumericalFailure) {
  Estimator est = kestirimci(ornek_durum());
  const ProcessingResult r = est.apply(DejenereOlcum{});

  EXPECT_EQ(r.reason, RejectReason::kNumericalFailure);
  ASSERT_TRUE(r.update.has_value());
  EXPECT_EQ(r.update->status, UpdateStatus::kNumericalFailure);
  EXPECT_FALSE(r.update->nis.has_value()) << "ADR-24: NIS tanimsiz, sentinel uretilmez";
}

TEST(Estimator, MappingIsTotalOverUpdateStatus) {
  // Esleme fonksiyonu tek yerdedir; ucunu de dogrudan sinar. Yeni bir
  // UpdateStatus eklenirse `default` olmadigi icin derleyici uyarir.
  EXPECT_EQ(kerteriz::reject_reason_of(UpdateStatus::kAccepted), RejectReason::kNone);
  EXPECT_EQ(kerteriz::reject_reason_of(UpdateStatus::kChiSquareRejected),
            RejectReason::kChiSquareGate);
  EXPECT_EQ(kerteriz::reject_reason_of(UpdateStatus::kNumericalFailure),
            RejectReason::kNumericalFailure);
}

// -----------------------------------------------------------------------------
// Snapshot round-trip — ADR-15 kapsam kurali
// -----------------------------------------------------------------------------

TEST(Estimator, SnapshotRoundTripRestoresStateCovarianceAndTime) {
  const NavState x0 = ornek_durum();
  Estimator est = kestirimci(x0);

  const ImuSample u1{kT0 + 10'000'000, Vec3(0.02, -0.01, 0.03), Vec3(0.1, -0.2, 9.9)};
  est.predict(u1, 0.01);
  const Vec3 z1 = est.backend().state().extended_pose().translation() + Vec3(0.3, -0.1, 0.2);
  ASSERT_EQ(
      est.apply(GnssPosition(u1.stamp_ns, "g", z1, Eigen::Matrix3d::Identity() * 2.25)).reason,
      RejectReason::kNone);

  const PipelineSnapshot kaydedilen = est.save_snapshot();
  const NavState durum_once = est.backend().state();
  const NavCovariance p_once = est.backend().covariance();
  const TimeNs t_once = est.backend().stamp_ns();

  // Durumu kasten ILERLET ve DEGISTIR.
  for (int i = 1; i <= 5; ++i) {
    const ImuSample u{u1.stamp_ns + static_cast<TimeNs>(i) * 10'000'000, Vec3(0.05, 0.04, -0.03),
                      Vec3(-0.3, 0.4, 10.1)};
    est.predict(u, 0.01);
  }
  ASSERT_EQ(
      est.apply(ZeroVelocity(u1.stamp_ns + 50'000'000, "z", Eigen::Matrix3d::Identity() * 0.01))
          .reason,
      RejectReason::kNone);
  ASSERT_GT(est.backend().stamp_ns(), t_once) << "test kurulumu: durum gercekten ilerlemeli";
  ASSERT_GT(est.backend().state().minus(durum_once).norm(), 1e-6);

  est.restore_snapshot(kaydedilen);

  EXPECT_LT(est.backend().state().minus(durum_once).norm(), 0.0 + 1e-18) << "durum geri gelmedi";
  EXPECT_LT((est.backend().covariance() - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "kovaryans geri gelmedi";
  EXPECT_EQ(est.backend().stamp_ns(), t_once) << "zaman snapshot'in parcasi (ADR-15)";
}

TEST(Estimator, SnapshotCarriesIntegrityScaffoldState) {
  // Faz 4 bilesenleri PASIFTIR; Faz 1 onlara hicbir sey yazmaz. Yine de
  // capture/restore yolunun degeri GERCEKTEN tasidigi bugun dogrulanabilir:
  // deger tipine disaridan yazilir, restore edilir, geri okunur.
  //
  // Bu test olmadan ADR-15'in "istisna yoktur" kurali Faz 4'te sessizce
  // kirilabilirdi: PipelineSnapshot'ta alan olur ama restore onu atlardi.
  Estimator est = kestirimci(ornek_durum());

  PipelineSnapshot s = est.save_snapshot();
  ASSERT_EQ(s.nis, NisState{}) << "Faz 1 iskeleti bos baslamali";
  ASSERT_EQ(s.fault, FaultState{});

  s.nis.yer_tutucu[0] = 12.5;
  s.nis.yer_tutucu[3] = -4.25;
  s.fault.yer_tutucu[1] = 7;
  est.restore_snapshot(s);

  const PipelineSnapshot geri = est.save_snapshot();
  EXPECT_EQ(geri.nis, s.nis) << "NisMonitor::restore degeri tasimadi";
  EXPECT_EQ(geri.fault, s.fault) << "FaultDetector::restore degeri tasimadi";
  EXPECT_NE(geri.nis, NisState{}) << "test kurulumu: deger gercekten degismis olmali";
}

TEST(Estimator, RestoreIsIdempotentAndRepeatable) {
  Estimator est = kestirimci(ornek_durum());
  const PipelineSnapshot s = est.save_snapshot();

  for (int tur = 0; tur < 3; ++tur) {
    const ImuSample u{kT0 + static_cast<TimeNs>(tur + 1) * 10'000'000, Vec3(0.03, 0.02, -0.01),
                      Vec3(0.2, 0.1, 9.7)};
    est.predict(u, 0.01);
    est.restore_snapshot(s);
    EXPECT_LT(est.backend().state().minus(s.backend.state).norm(), 1e-18) << "tur " << tur;
    EXPECT_EQ(est.backend().stamp_ns(), s.backend.stamp_ns);
  }
}

} // namespace
