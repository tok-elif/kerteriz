/// \file
/// F1.3 — EskfBackend. INTERFACES §3-4 · CONVENTIONS §3, §4, §6 · ADR-19, 20, 24.

#include "kerteriz/backends/eskf_backend.hpp"

#include <Eigen/Core>
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>
#include <memory>
#include <new>
#include <type_traits>

namespace {

using kerteriz::BackendSnapshot;
using kerteriz::CloneId;
using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::EstimatorMode;
using kerteriz::exp_map;
using kerteriz::ImuNoiseParams;
using kerteriz::ImuPropagator;
using kerteriz::ImuSample;
using kerteriz::innovation_nis;
using kerteriz::kCloneDof;
using kerteriz::kCoreDof;
using kerteriz::kInvalidClone;
using kerteriz::kMaxAugmentDof;
using kerteriz::kMaxStateDof;
using kerteriz::linear_update;
using kerteriz::Measurement;
using kerteriz::MeasurementWorkspace;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::StateBundle;
using kerteriz::StateMat;
using kerteriz::StateVec;
using kerteriz::TangentVec;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;

constexpr Scalar kG = 9.80665;
constexpr Scalar kGuven = 0.997;

// -----------------------------------------------------------------------------
// Test-only sahte olcum. URETIM AGACINDA DEGILDIR.
//
// Cekirdegin Oklidyen bilesenlerini gozler: jiro bias x (teget 9) ve istege
// bagli ivme bias x (teget 12). Boylece kapali form elle yazilabilir ve
// residual isareti dogrudan sinanabilir.
//
//   h(X) = X.gyro_bias().x()      r = z - h(X)      J_res = -e_9^T
// -----------------------------------------------------------------------------
class SahteBiasOlcumu : public Measurement {
 public:
  SahteBiasOlcumu(int dim, Scalar z0, Scalar z1, Scalar gurultu)
      : dim_(dim), z0_(z0), z1_(z1), gurultu_(gurultu) {}

  TimeNs stamp_ns() const override { return 42; }
  int residual_dim() const override { return dim_; }
  std::string_view name() const override { return "sahte_bias"; }

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    ++cagri_sayisi_;
    w.dim = dim_;
    w.r[0] = z0_ - states.current().gyro_bias().x();
    w.J_res(0, 9) = isaret_ * Scalar(-1);
    w.R(0, 0) = gurultu_;
    if (dim_ > 1) {
      w.r[1] = z1_ - states.current().accel_bias().x();
      w.J_res(1, 12) = isaret_ * Scalar(-1);
      w.R(1, 1) = gurultu_;
    }
  }

  int cagri_sayisi() const { return cagri_sayisi_; }
  void isareti_cevir() { isaret_ = -isaret_; }

 private:
  int dim_;
  Scalar z0_;
  Scalar z1_;
  Scalar gurultu_;
  Scalar isaret_ = Scalar(1);
  mutable int cagri_sayisi_ = 0;
};

/// S = 0 uretir: J_res sifir, R sifir, artik sifirdan farkli. ADR-24 yolu.
class TekilOlcum : public Measurement {
 public:
  TimeNs stamp_ns() const override { return 7; }
  int residual_dim() const override { return 1; }
  std::string_view name() const override { return "tekil"; }
  void evaluate(const StateBundle&, MeasurementWorkspace& w) const override {
    w.dim = 1;
    w.r[0] = Scalar(1); // J_res ve R sifir kalir
  }
};

NavState ornek_durum() {
  NavState x;
  TangentVec d;
  d << 0.20, -0.35, 0.55, 1.30, -0.70, 0.40, -2.10, 1.60, 0.90;
  x.extended_pose() = exp_map(d);
  x.gyro_bias() = Vec3(0.013, -0.021, 0.009);
  x.accel_bias() = Vec3(-0.07, 0.11, 0.04);
  return x;
}

ImuNoiseParams ornek_gurultu() { return ImuNoiseParams{1.0e-3, 1.0e-5, 2.0e-2, 3.0e-4}; }

NavCovariance kosegen_p(int n, Scalar deger) {
  NavCovariance p = NavCovariance::Zero();
  p.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * deger;
  return p;
}

EskfConfig ornek_config(NavState x = ornek_durum(), NavCovariance p = kosegen_p(kCoreDof, 0.1),
                        TimeNs t0 = 1000) {
  return EskfConfig{std::move(x), std::move(p), t0, ornek_gurultu(), kGuven};
}

ImuSample olcum(TimeNs t) { return ImuSample{t, Vec3(0.2, -0.1, 0.3), Vec3(0.4, 0.3, kG)}; }

// -----------------------------------------------------------------------------
// predict — ImuPropagator'a delegasyon
// -----------------------------------------------------------------------------

TEST(EskfBackend, PredictDelegatesExactlyToImuPropagator) {
  EskfBackend backend(ornek_config());

  NavState beklenen_x = ornek_durum();
  NavCovariance beklenen_p = kosegen_p(kCoreDof, 0.1);
  ImuPropagator prop(ornek_gurultu());

  const ImuSample u = olcum(2000);
  const Scalar dt = 0.01;

  backend.predict(u, dt);
  prop.propagate(beklenen_x, beklenen_p, u, dt);

  EXPECT_LT(backend.state().minus(beklenen_x).norm(), 1e-18) << "durum propagator'dan sapti";
  EXPECT_LT((backend.covariance() - beklenen_p).cwiseAbs().maxCoeff(), 1e-18)
      << "kovaryans propagator'dan sapti";
}

TEST(EskfBackend, TimestampComesFromSampleNotAccumulation) {
  EskfBackend backend(ornek_config());
  EXPECT_EQ(backend.stamp_ns(), 1000);

  // dt ile damga arasinda KASTEN tutarsizlik: damga ornekten gelmeli.
  backend.predict(olcum(123456789), 0.005);
  EXPECT_EQ(backend.stamp_ns(), 123456789) << "damga dt biriktirilerek uretilmis";

  backend.predict(olcum(500), 0.005);
  EXPECT_EQ(backend.stamp_ns(), 500) << "damga monoton varsayilmis";
}

TEST(EskfBackend, ConsecutivePredictsMatchPropagatorChain) {
  EskfBackend backend(ornek_config());
  NavState beklenen_x = ornek_durum();
  NavCovariance beklenen_p = kosegen_p(kCoreDof, 0.1);
  ImuPropagator prop(ornek_gurultu());

  for (int i = 1; i <= 25; ++i) {
    const ImuSample u = olcum(1000 + 10000000LL * i);
    backend.predict(u, 0.01);
    prop.propagate(beklenen_x, beklenen_p, u, 0.01);
  }
  EXPECT_LT(backend.state().minus(beklenen_x).norm(), 1e-18);
  EXPECT_LT((backend.covariance() - beklenen_p).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_EQ(backend.stamp_ns(), 1000 + 10000000LL * 25);
}

TEST(EskfBackend, ZeroDtOnlyAdvancesTimestamp) {
  EskfBackend backend(ornek_config());
  const NavState once = backend.state();
  const NavCovariance p_once = backend.covariance();

  backend.predict(olcum(9999), 0.0);

  EXPECT_LT(backend.state().minus(once).norm(), 1e-18);
  EXPECT_LT((backend.covariance() - p_once).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_EQ(backend.stamp_ns(), 9999) << "dt=0 damgayi yine de ilerletmeli";
}

TEST(EskfBackendDeathTest, NegativeDtPropagatesPropagatorPrecondition) {
  EXPECT_DEATH(
      {
        EskfBackend backend(ornek_config());
        backend.predict(olcum(2000), -0.01);
      },
      "");
}

TEST(EskfBackend, PredictPreservesAugmentationCrossCovariance) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  const int n = x.active_dof();

  NavCovariance p = kosegen_p(n, 0.1);
  p(3, kCoreDof) = 0.05;
  p(kCoreDof, 3) = 0.05;

  EskfBackend backend(ornek_config(x, p));
  backend.predict(olcum(2000), 0.02);

  EXPECT_GT(backend.covariance().block(0, kCoreDof, kCoreDof, 1).cwiseAbs().maxCoeff(), 1e-6)
      << "capraz kovaryans kayboldu";
  EXPECT_EQ(backend.state().calibration_offset("wheel_scale"), kCoreDof);
}

TEST(EskfBackend, ModeIsNominalThroughoutPhaseOne) {
  EskfBackend backend(ornek_config());
  EXPECT_EQ(backend.mode(), EstimatorMode::kNominal);
  backend.predict(olcum(2000), 0.01);
  SahteBiasOlcumu z(1, 0.02, 0.0, 1e-4);
  backend.update(z);
  EXPECT_EQ(backend.mode(), EstimatorMode::kNominal);
}

// -----------------------------------------------------------------------------
// update — kabul edilen yol, kapali form (wiring testi)
//
// Faz 0'in ClosedFormScalarGaussian testi BACKEND SEVIYESINDE tekrarlanir:
//   Measurement::evaluate -> EskfBackend::update -> NIS kapisi
//   -> linear_update -> NavState::plus
// Kopya degil, uctan uca. Wiring isaret hatasini boylece yakalar.
//
// Kapali form: h(X) = b_g.x,  r = z - b_g.x,  J_res = -e_9^T,  P kosegen
//   S       = P(9,9) + R
//   delta_9 = P(9,9) (z - b) / S        <- POZITIF
//   P'(9,9) = P(9,9) R / S
//   NIS     = r^2 / S
// -----------------------------------------------------------------------------

TEST(EskfBackend, AcceptedUpdateMatchesClosedFormScalarGaussian) {
  const Scalar p_bias = 4.0e-4;
  const Scalar r_gurultu = 1.0e-4;
  const Scalar z = 0.020;

  NavState x = ornek_durum();
  const Scalar b = x.gyro_bias().x();
  NavCovariance p = kosegen_p(kCoreDof, 0.1);
  p(9, 9) = p_bias;

  EskfBackend backend(ornek_config(x, p));
  SahteBiasOlcumu olcum_z(1, z, 0.0, r_gurultu);
  const auto sonuc = backend.update(olcum_z);

  const Scalar s = p_bias + r_gurultu;
  const Scalar beklenen_delta = p_bias * (z - b) / s;

  ASSERT_EQ(sonuc.status, UpdateStatus::kAccepted);
  ASSERT_TRUE(sonuc.nis.has_value());
  EXPECT_NEAR(*sonuc.nis, (z - b) * (z - b) / s, 1e-12);
  EXPECT_EQ(sonuc.dof, 1);

  EXPECT_GT(beklenen_delta, 0.0) << "test kurulumu: z > b bekleniyor";
  EXPECT_NEAR(backend.state().gyro_bias().x(), b + beklenen_delta, 1e-14)
      << "kapali form tutmadi (wiring veya isaret)";
  EXPECT_NEAR(backend.covariance()(9, 9), p_bias * r_gurultu / s, 1e-18);
  EXPECT_NEAR(backend.covariance()(3, 3), 0.1, 1e-18) << "ilgisiz kosegen degisti";
  EXPECT_EQ(olcum_z.cagri_sayisi(), 1) << "evaluate tam bir kez cagrilmali";
}

TEST(EskfBackend, ResidualSignDrivesStateInTheRightDirection) {
  // z < b ise duzeltme NEGATIF olmali. J_res isareti cevrilirse bu duser.
  NavState x = ornek_durum();
  const Scalar b = x.gyro_bias().x();
  NavCovariance p = kosegen_p(kCoreDof, 0.1);
  p(9, 9) = 4.0e-4;

  EskfBackend backend(ornek_config(x, p));
  SahteBiasOlcumu asagi(1, b - 0.01, 0.0, 1.0e-4);
  ASSERT_EQ(backend.update(asagi).status, UpdateStatus::kAccepted);
  EXPECT_LT(backend.state().gyro_bias().x(), b) << "artik isareti yanlis yone tasidi";
}

TEST(EskfBackend, JosephCovarianceMatchesLinearUpdateDirectly) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  const int n = x.active_dof();
  NavCovariance p = kosegen_p(n, 0.05);
  p(9, 9) = 3.0e-4;
  p(12, 12) = 5.0e-4;

  EskfBackend backend(ornek_config(x, p));
  SahteBiasOlcumu olcum_z(2, 0.02, -0.05, 1.0e-4);
  const auto sonuc = backend.update(olcum_z);
  ASSERT_EQ(sonuc.status, UpdateStatus::kAccepted);

  // Ayni girdileri DOGRUDAN linear_update'e ver.
  MeasurementWorkspace w;
  w.clear();
  const StateBundle bundle(x);
  olcum_z.evaluate(bundle, w);
  NavCovariance p_ref = p;
  StateVec delta = StateVec::Zero();
  const auto ref = linear_update(w.J_res, w.r, w.R, w.dim, n, p_ref, delta);
  ASSERT_TRUE(ref.spd_ok);

  EXPECT_LT(
      (backend.covariance().topLeftCorner(n, n) - p_ref.topLeftCorner(n, n)).cwiseAbs().maxCoeff(),
      1e-18)
      << "backend kovaryansi linear_update'ten sapti";
  EXPECT_LT(backend.state().minus(x.plus(delta)).norm(), 1e-18) << "durum sapti";
  ASSERT_TRUE(sonuc.nis.has_value());
  EXPECT_NEAR(*sonuc.nis, ref.nis, 1e-12) << "NIS-only ile linear_update NIS'i ayristi";
}

// -----------------------------------------------------------------------------
// update — reddedilen yol: KAPI MUTASYONDAN ONCE
// -----------------------------------------------------------------------------

TEST(EskfBackend, ChiSquareRejectionLeavesStateAndCovarianceUntouched) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  const int n = x.active_dof();
  NavCovariance p = kosegen_p(n, 0.02);
  p(9, 9) = 1.0e-4;

  EskfBackend backend(ornek_config(x, p));
  const NavState durum_once = backend.state();
  const NavCovariance p_once = backend.covariance();

  // Artik esigin acikca uzerinde: z, b'den cok uzak.
  SahteBiasOlcumu uzak(1, x.gyro_bias().x() + 10.0, 0.0, 1.0e-6);
  const auto sonuc = backend.update(uzak);

  ASSERT_EQ(sonuc.status, UpdateStatus::kChiSquareRejected);
  ASSERT_TRUE(sonuc.nis.has_value());
  EXPECT_GT(*sonuc.nis, sonuc.threshold) << "reddedildi ama NIS esigin altinda";
  EXPECT_EQ(sonuc.dof, 1);
  EXPECT_NEAR(sonuc.threshold, kerteriz::chi_square_quantile(kGuven, 1), 1e-12);

  EXPECT_LT(backend.state().minus(durum_once).norm(), 0.0 + 1e-18)
      << "reddedilen olcum durumu degistirdi";
  EXPECT_LT((backend.covariance() - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "reddedilen olcum kovaryansi degistirdi";
}

TEST(EskfBackend, AcceptedAndRejectedUseTheSameThresholdForSameDof) {
  EskfBackend backend(ornek_config());
  SahteBiasOlcumu yakin(1, ornek_durum().gyro_bias().x() + 1e-4, 0.0, 1.0e-4);
  SahteBiasOlcumu uzak(1, ornek_durum().gyro_bias().x() + 10.0, 0.0, 1.0e-6);

  const auto a = backend.update(yakin);
  const auto b = backend.update(uzak);
  ASSERT_EQ(a.status, UpdateStatus::kAccepted);
  ASSERT_EQ(b.status, UpdateStatus::kChiSquareRejected);
  EXPECT_EQ(a.threshold, b.threshold);
  EXPECT_EQ(a.dof, b.dof);
}

TEST(EskfBackend, ThresholdTracksResidualDimension) {
  EskfBackend backend(ornek_config());
  SahteBiasOlcumu bir(1, 0.02, 0.0, 1.0e-2);
  SahteBiasOlcumu iki(2, 0.02, -0.05, 1.0e-2);

  const auto a = backend.update(bir);
  const auto b = backend.update(iki);
  EXPECT_EQ(a.dof, 1);
  EXPECT_EQ(b.dof, 2);
  EXPECT_NEAR(a.threshold, kerteriz::chi_square_quantile(kGuven, 1), 1e-12);
  EXPECT_NEAR(b.threshold, kerteriz::chi_square_quantile(kGuven, 2), 1e-12);
  EXPECT_LT(a.threshold, b.threshold);
}

// -----------------------------------------------------------------------------
// update — ADR-24 sayisal basarisizlik
// -----------------------------------------------------------------------------

TEST(EskfBackend, NumericalFailureReportsEmptyNisAndChangesNothing) {
  NavState x = ornek_durum();
  NavCovariance p = NavCovariance::Zero(); // P = 0, olcum J = 0, R = 0 -> S = 0
  EskfBackend backend(ornek_config(x, p));

  const NavState durum_once = backend.state();
  const NavCovariance p_once = backend.covariance();

  TekilOlcum tekil;
  const auto sonuc = backend.update(tekil);

  EXPECT_EQ(sonuc.status, UpdateStatus::kNumericalFailure);
  EXPECT_FALSE(sonuc.nis.has_value()) << "ADR-24: NIS tanimsiz, sentinel uretilmez";
  EXPECT_EQ(sonuc.dof, 1) << "dof sayisal basarisizlikta da gecerli";
  EXPECT_NEAR(sonuc.threshold, kerteriz::chi_square_quantile(kGuven, 1), 1e-12)
      << "threshold sayisal basarisizlikta da gecerli";

  EXPECT_LT(backend.state().minus(durum_once).norm(), 0.0 + 1e-18);
  EXPECT_LT((backend.covariance() - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18);
}

TEST(EskfBackend, WorkspaceIsClearedBetweenUpdates) {
  // Once 2 boyutlu, sonra 1 boyutlu olcum. Ikinci update'te birinci olcumun
  // ikinci satiri artakalirsa NIS ve duzeltme bozulur.
  NavState x = ornek_durum();
  NavCovariance p = kosegen_p(kCoreDof, 0.1);
  p(9, 9) = 4.0e-4;
  p(12, 12) = 4.0e-4;

  EskfBackend a(ornek_config(x, p));
  SahteBiasOlcumu iki(2, 0.02, -0.05, 1.0e-4);
  SahteBiasOlcumu bir(1, 0.02, 0.0, 1.0e-4);
  a.update(iki);
  const auto ara = a.save_snapshot(); // birinci update SONRASI, ikinciden ONCE
  const auto sonra = a.update(bir);

  // Ayni ikinci update'i, ayni baslangictan, TEMIZ bir backend uzerinde tekrarla.
  EskfBackend b(ornek_config(ara.state, ara.covariance));
  SahteBiasOlcumu bir2(1, 0.02, 0.0, 1.0e-4);
  const auto temiz = b.update(bir2);

  ASSERT_TRUE(sonra.nis.has_value());
  ASSERT_TRUE(temiz.nis.has_value());
  EXPECT_NEAR(*sonra.nis, *temiz.nis, 1e-15) << "workspace artigi NIS'i bozdu";
}

// -----------------------------------------------------------------------------
// Snapshot — ADR-15, ADR-20
// -----------------------------------------------------------------------------

TEST(EskfBackend, SnapshotRestoresStateCovarianceAndTimestamp) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.register_calibration("time_offset", 2);
  const CloneId klon_a = x.push_clone();
  const CloneId klon_b = x.push_clone();
  const int n = x.active_dof();

  EskfBackend backend(ornek_config(x, kosegen_p(n, 0.03), 5000));
  backend.predict(olcum(6000), 0.01);

  const BackendSnapshot anlik = backend.save_snapshot();

  // Snapshot SONRASI islemeye devam et.
  for (int i = 1; i <= 10; ++i) {
    backend.predict(olcum(6000 + 1000000LL * i), 0.01);
  }
  SahteBiasOlcumu z(1, 0.02, 0.0, 1.0e-4);
  backend.update(z);
  ASSERT_NE(backend.stamp_ns(), anlik.stamp_ns);

  backend.restore_snapshot(anlik);

  EXPECT_EQ(backend.stamp_ns(), anlik.stamp_ns) << "zaman snapshot'in parcasidir (ADR-15)";
  EXPECT_LT(backend.state().minus(anlik.state).norm(), 0.0 + 1e-18);
  EXPECT_LT((backend.covariance() - anlik.covariance).cwiseAbs().maxCoeff(), 0.0 + 1e-18);

  // Duzen de birebir geri gelmeli.
  EXPECT_EQ(backend.state().active_dof(), n);
  EXPECT_EQ(backend.state().calibration_offset("wheel_scale"), kCoreDof);
  EXPECT_EQ(backend.state().calibration_offset("time_offset"), kCoreDof + 1);
  EXPECT_TRUE(backend.state().has_clone(klon_a));
  EXPECT_TRUE(backend.state().has_clone(klon_b));
  EXPECT_EQ(backend.state().clone_offset(klon_a), kCoreDof + 3);
  EXPECT_EQ(backend.state().clone_offset(klon_b), kCoreDof + 3 + kCloneDof);
}

TEST(EskfBackend, SnapshotCarriesOnlyBackendState) {
  // ADR-20: BackendSnapshot yalnizca backend'in KENDI durumudur; butunluk
  // katmanini icermez. Uc uyeyle toplu baslatilabilmesi bunun kontroludur.
  const BackendSnapshot uc{ornek_durum(), kosegen_p(kCoreDof, 0.1), 77};
  EXPECT_EQ(uc.stamp_ns, 77);

  const EskfBackend backend(ornek_config());
  const BackendSnapshot s = backend.save_snapshot();
  EXPECT_EQ(s.stamp_ns, 1000);
  EXPECT_EQ(s.state.active_dof(), kCoreDof);
}

// -----------------------------------------------------------------------------
// Klon kovaryans yasam dongusu
//
//   Jc  : klon tegetini [dtheta, dp] cekirdek indekslerine baglayan secici
//   P_xc = P Jc^T,  P_cc = Jc P Jc^T   -> klon guncel poza TAM KORELE baslar
// -----------------------------------------------------------------------------

TEST(EskfBackend, PushCloneStartsFullyCorrelatedWithCurrentPose) {
  NavState x = ornek_durum();
  const int n = x.active_dof();

  // Kosegen olmayan, gercekci bir P.
  StateMat a = StateMat::Zero();
  for (int i = 0; i < n; ++i) {
    for (int k = 0; k < n; ++k) {
      a(i, k) = 0.1 * std::sin(Scalar(3 * i + 7 * k));
    }
  }
  NavCovariance p = NavCovariance::Zero();
  p.topLeftCorner(n, n) = a.topLeftCorner(n, n) * a.topLeftCorner(n, n).transpose() +
                          Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * 0.05;
  const NavCovariance p_once = p;

  EskfBackend backend(ornek_config(x, p));
  const CloneId id = backend.push_clone();
  ASSERT_NE(id, kInvalidClone);
  const int ofs = backend.state().clone_offset(id);
  ASSERT_EQ(ofs, n);

  const int kaynak[kCloneDof] = {0, 1, 2, 6, 7, 8};
  for (int i = 0; i < kCloneDof; ++i) {
    // P_cc = Jc P Jc^T
    for (int k = 0; k < kCloneDof; ++k) {
      EXPECT_NEAR(backend.covariance()(ofs + i, ofs + k), p_once(kaynak[i], kaynak[k]), 1e-18)
          << "P_cc[" << i << "," << k << "]";
    }
    // P_xc = P Jc^T
    for (int rr = 0; rr < n; ++rr) {
      EXPECT_NEAR(backend.covariance()(rr, ofs + i), p_once(rr, kaynak[i]), 1e-18)
          << "P_xc[" << rr << "," << i << "]";
      EXPECT_NEAR(backend.covariance()(ofs + i, rr), p_once(kaynak[i], rr), 1e-18);
    }
  }

  // Eski blok DEGISMEDI.
  EXPECT_LT(
      (backend.covariance().topLeftCorner(n, n) - p_once.topLeftCorner(n, n)).cwiseAbs().maxCoeff(),
      1e-18);
}

TEST(EskfBackend, FailedPushCloneLeavesCovarianceUntouched) {
  NavState x = ornek_durum();
  ASSERT_TRUE(x.register_calibration("dolgu", kMaxAugmentDof).has_value());
  const int n = x.active_dof();
  ASSERT_EQ(n, kMaxStateDof);

  EskfBackend backend(ornek_config(x, kosegen_p(n, 0.07)));
  const NavCovariance p_once = backend.covariance();

  EXPECT_EQ(backend.push_clone(), kInvalidClone) << "kapasite yokken klon uretildi";
  EXPECT_LT((backend.covariance() - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "basarisiz push kovaryansi degistirdi";
}

TEST(EskfBackend, DropMiddleCloneCompactsCovarianceAndKeepsCalibrations) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 2);
  EskfBackend backend(ornek_config(x, kosegen_p(x.active_dof(), 0.01)));

  const CloneId a = backend.push_clone();
  const CloneId b = backend.push_clone();
  const CloneId c = backend.push_clone();
  ASSERT_NE(a, kInvalidClone);
  ASSERT_NE(c, kInvalidClone);

  // Ayirt edici isaretler: her klon blogunun kosegenini benzersiz yap.
  // (Kovaryans dogrudan yazilamadigi icin blok kimligini ofsetlerle izleriz.)
  const int ofs_a = backend.state().clone_offset(a);
  const int ofs_c = backend.state().clone_offset(c);
  const NavCovariance once = backend.covariance();
  const int n_once = backend.state().active_dof();

  backend.drop_clone(b);

  const int n = backend.state().active_dof();
  ASSERT_EQ(n, n_once - kCloneDof);

  // Kalibrasyon yerinden oynamamali.
  EXPECT_EQ(backend.state().calibration_offset("wheel_scale"), kCoreDof);
  // a yerinde, c asagi kaydi.
  EXPECT_EQ(backend.state().clone_offset(a), ofs_a);
  EXPECT_EQ(backend.state().clone_offset(c), ofs_c - kCloneDof);

  // a blogu degismedi; c blogu eski yerinden yeni yerine TASINDI.
  const int yeni_c = backend.state().clone_offset(c);
  for (int i = 0; i < kCloneDof; ++i) {
    for (int k = 0; k < kCloneDof; ++k) {
      EXPECT_NEAR(backend.covariance()(ofs_a + i, ofs_a + k), once(ofs_a + i, ofs_a + k), 1e-18);
      EXPECT_NEAR(backend.covariance()(yeni_c + i, yeni_c + k), once(ofs_c + i, ofs_c + k), 1e-18)
          << "hayatta kalan klonun kovaryansi tasinmadi";
    }
    // a <-> c capraz blogu da tasinmis olmali.
    for (int k = 0; k < kCloneDof; ++k) {
      EXPECT_NEAR(backend.covariance()(ofs_a + i, yeni_c + k), once(ofs_a + i, ofs_c + k), 1e-18);
    }
  }

  // Bosalan band deterministik sifir.
  EXPECT_LT(backend.covariance().block(n, 0, kCloneDof, n_once).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_LT(backend.covariance().block(0, n, n_once, kCloneDof).cwiseAbs().maxCoeff(), 1e-18);
}

TEST(EskfBackend, DroppedCloneCovarianceIsMarginalizedNotConditioned) {
  // Marjinalizasyon = satir/sutun KALDIRMA. Schur tumleyeni (kosullandirma)
  // kalan blogu DEGISTIRIRDI; burada degismemeli.
  NavState x = ornek_durum();
  EskfBackend backend(ornek_config(x, kosegen_p(kCoreDof, 0.2)));

  const CloneId a = backend.push_clone();
  ASSERT_NE(a, kInvalidClone);
  const NavCovariance klonlu = backend.covariance();

  backend.drop_clone(a);

  EXPECT_LT((backend.covariance().topLeftCorner(kCoreDof, kCoreDof) -
             klonlu.topLeftCorner(kCoreDof, kCoreDof))
                .cwiseAbs()
                .maxCoeff(),
            1e-18)
      << "kalan blok degisti: Schur tumleyeni uygulanmis olabilir";
}

} // namespace

// -----------------------------------------------------------------------------
// Tahsis yasagi — CONVENTIONS §8.1 / ADR-22
//
// operator new GLOBAL scope'ta OLMAK ZORUNDA; anonim ad alani icinde olsaydi
// globali degistirmez ve sayac hic artmazdi. Ayrica Eigen dinamik bellegi
// operator new ile DEGIL malloc ile alir, bu yuzden Eigen'in kendi kapisi da
// kullanilir. Iki kapinin da gercekten bagli oldugu ayrica dogrulanir.
// -----------------------------------------------------------------------------

namespace {
bool tahsis_sayimi_acik = false;
int tahsis_adedi = 0;

struct TahsisKapsami {
  TahsisKapsami() {
    tahsis_adedi = 0;
    tahsis_sayimi_acik = true;
  }
  ~TahsisKapsami() { tahsis_sayimi_acik = false; }
};
} // namespace

void* operator new(std::size_t n) {
  if (tahsis_sayimi_acik) {
    ++tahsis_adedi;
  }
  void* p = std::malloc(n);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {

TEST(EskfBackend, AllocationCounterIsActuallyWired) {
  int gorulen = 0;
  {
    const TahsisKapsami kapsam;
    void* ham = ::operator new(64); // dogrudan cagri: derleyici eleyemez
    gorulen = tahsis_adedi;
    ::operator delete(ham);
  }
  EXPECT_GT(gorulen, 0) << "operator new degistirilmemis";
}

TEST(EigenAllocGuardDeathTest, GuardIsWiredInThisBinary) {
  EXPECT_DEATH(
      {
        Eigen::internal::set_is_malloc_allowed(false);
        Eigen::MatrixXd m(64, 64);
        m.setZero();
      },
      "");
}

TEST(EskfBackend, PredictAndUpdateDoNotAllocate) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  EskfBackend backend(ornek_config(x, kosegen_p(x.active_dof() + kCloneDof, 0.01)));
  backend.push_clone();

  SahteBiasOlcumu yakin(1, x.gyro_bias().x() + 1e-4, 0.0, 1.0e-3);
  SahteBiasOlcumu uzak(1, x.gyro_bias().x() + 10.0, 0.0, 1.0e-6);
  SahteBiasOlcumu iki(2, 0.02, -0.05, 1.0e-3);
  TekilOlcum tekil;

  // Isinma.
  backend.predict(olcum(2000), 0.005);
  backend.update(yakin);

  {
    const TahsisKapsami kapsam;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < 50; ++i) {
      backend.predict(olcum(2000 + 1000000LL * i), 0.005);
      backend.update(yakin); // kabul
      backend.update(uzak);  // chi-kare reddi
      backend.update(iki);   // cok boyutlu
      backend.update(tekil); // sayisal basarisizlik
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_EQ(tahsis_adedi, 0) << "sicak yol operator new'e gitti";
}

TEST(EskfBackend, CloneLifecycleDoesNotAllocate) {
  EskfBackend backend(ornek_config());
  const CloneId isinma = backend.push_clone();
  backend.drop_clone(isinma);

  {
    const TahsisKapsami kapsam;
    Eigen::internal::set_is_malloc_allowed(false);
    const CloneId a = backend.push_clone();
    const CloneId b = backend.push_clone();
    const CloneId c = backend.push_clone();
    backend.drop_clone(b);
    backend.drop_clone(a);
    backend.drop_clone(c);
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_EQ(tahsis_adedi, 0) << "klon yasam dongusu heap'e gitti";
}

TEST(EskfBackend, EvaluateIsCalledExactlyOncePerUpdateOnEveryPath) {
  NavState x = ornek_durum();
  NavCovariance p = kosegen_p(kCoreDof, 0.1);
  p(9, 9) = 4.0e-4;
  EskfBackend backend(ornek_config(x, p));

  SahteBiasOlcumu kabul(1, x.gyro_bias().x() + 1e-4, 0.0, 1.0e-3);
  SahteBiasOlcumu red(1, x.gyro_bias().x() + 10.0, 0.0, 1.0e-8);

  ASSERT_EQ(backend.update(kabul).status, UpdateStatus::kAccepted);
  EXPECT_EQ(kabul.cagri_sayisi(), 1);
  ASSERT_EQ(backend.update(red).status, UpdateStatus::kChiSquareRejected);
  EXPECT_EQ(red.cagri_sayisi(), 1) << "reddedilen yolda evaluate tekrar cagrildi";
}

TEST(EskfBackend, JacobianColumnsBeyondActiveDofAreIgnored) {
  // J_res yalnizca aktif durum boyutunda kullanilmali. Aktif boyutun otesine
  // yazilan deger sonucu DEGISTIRMEMELI.
  class KuyrugaYazan : public Measurement {
   public:
    explicit KuyrugaYazan(Scalar z) : z_(z) {}
    TimeNs stamp_ns() const override { return 11; }
    int residual_dim() const override { return 1; }
    std::string_view name() const override { return "kuyruk"; }
    void evaluate(const StateBundle& s, MeasurementWorkspace& w) const override {
      w.dim = 1;
      w.r[0] = z_ - s.current().gyro_bias().x();
      w.J_res(0, 9) = Scalar(-1);
      w.R(0, 0) = Scalar(1e-4);
      if (kirlet_) {
        for (int c = s.current().active_dof(); c < kMaxStateDof; ++c) {
          w.J_res(0, c) = Scalar(1e3);
        }
      }
    }
    bool kirlet_ = false;

   private:
    Scalar z_;
  };

  NavState x = ornek_durum();
  NavCovariance p = kosegen_p(kCoreDof, 0.1);
  p(9, 9) = 4.0e-4;

  EskfBackend a(ornek_config(x, p));
  EskfBackend b(ornek_config(x, p));
  KuyrugaYazan temiz(0.02);
  KuyrugaYazan kirli(0.02);
  kirli.kirlet_ = true;

  const auto sa = a.update(temiz);
  const auto sb = b.update(kirli);

  ASSERT_EQ(sa.status, UpdateStatus::kAccepted);
  ASSERT_EQ(sb.status, UpdateStatus::kAccepted);
  ASSERT_TRUE(sa.nis.has_value() && sb.nis.has_value());
  EXPECT_NEAR(*sa.nis, *sb.nis, 1e-18) << "aktif boyut otesi sutunlar NIS'i etkiledi";
  EXPECT_LT(a.state().minus(b.state()).norm(), 1e-18);
  EXPECT_LT((a.covariance() - b.covariance()).cwiseAbs().maxCoeff(), 1e-18);
}

// -----------------------------------------------------------------------------
// INTERFACES §4 — soyut arayuz gercekten uygulaniyor
// -----------------------------------------------------------------------------

TEST(EskfBackend, ImplementsFilterBackendInterface) {
  static_assert(std::is_base_of<kerteriz::FilterBackend, EskfBackend>::value,
                "EskfBackend FilterBackend'den turemeli");
  static_assert(std::has_virtual_destructor<kerteriz::FilterBackend>::value,
                "polimorfik silme guvenli olmali");
}

TEST(EskfBackend, WorksThroughFilterBackendReference) {
  // Tum akis YALNIZCA soyut arayuz uzerinden kosar; Estimator ve
  // MeasurementBuffer backend'i boyle gorecek.
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  NavCovariance p = kosegen_p(x.active_dof(), 0.1);
  p(9, 9) = 4.0e-4;

  EskfBackend somut(ornek_config(x, p, 1000));
  kerteriz::FilterBackend& backend = somut;

  EXPECT_EQ(backend.mode(), EstimatorMode::kNominal);
  EXPECT_EQ(backend.stamp_ns(), 1000);

  backend.predict(olcum(2000), 0.01);
  EXPECT_EQ(backend.stamp_ns(), 2000);

  const CloneId id = backend.push_clone();
  ASSERT_NE(id, kInvalidClone);
  EXPECT_EQ(backend.state().clone_offset(id), kCoreDof + 1);

  const auto anlik = backend.save_snapshot();

  SahteBiasOlcumu z(1, 0.02, 0.0, 1.0e-4);
  const auto sonuc = backend.update(z);
  EXPECT_EQ(sonuc.status, UpdateStatus::kAccepted);
  ASSERT_TRUE(sonuc.nis.has_value());

  backend.drop_clone(id);
  EXPECT_FALSE(backend.state().has_clone(id));

  backend.restore_snapshot(anlik);
  EXPECT_TRUE(backend.state().has_clone(id)) << "snapshot soyut arayuzden geri gelmedi";
  EXPECT_EQ(backend.stamp_ns(), anlik.stamp_ns);
  EXPECT_TRUE(backend.weak_directions().empty());
}

TEST(EskfBackend, PolymorphicDeletionThroughBasePointer) {
  std::unique_ptr<kerteriz::FilterBackend> backend = std::make_unique<EskfBackend>(ornek_config());
  backend->predict(olcum(2000), 0.01);
  EXPECT_EQ(backend->stamp_ns(), 2000);
  EXPECT_EQ(backend->covariance().rows(), kMaxStateDof);
  backend.reset(); // sanal yikici yoksa buraya kadar UB
}

} // namespace
