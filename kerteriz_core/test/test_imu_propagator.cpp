/// \file
/// F1.2 — ImuPropagator. INTERFACES §2 · CONVENTIONS §1, §3.1, §5.1, §6, §7, §8.1.

#include "kerteriz/process/imu_propagator.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cstdlib>
#include <gtest/gtest.h>
#include <new>

namespace {

using kerteriz::CloneId;
using kerteriz::exp_map;
using kerteriz::ImuNoiseParams;
using kerteriz::ImuPropagator;
using kerteriz::ImuSample;
using kerteriz::kCloneDof;
using kerteriz::kCoreDof;
using kerteriz::kMaxStateDof;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::StateMat;
using kerteriz::StateVec;
using kerteriz::TangentVec;
using kerteriz::Vec3;

constexpr Scalar kG = 9.80665;

ImuSample olcum(const Vec3& gyro, const Vec3& accel) { return ImuSample{0, gyro, accel}; }

/// Gurultusuz propagator — nominal testler icin.
ImuPropagator sessiz() { return ImuPropagator(ImuNoiseParams{0.0, 0.0, 0.0, 0.0}); }

ImuNoiseParams ornek_gurultu() {
  ImuNoiseParams n;
  n.gyro_noise_density = 1.0e-3;  // rad/s/sqrt(Hz)
  n.gyro_random_walk = 1.0e-5;    // rad/s^2/sqrt(Hz)
  n.accel_noise_density = 2.0e-2; // m/s^2/sqrt(Hz)
  n.accel_random_walk = 3.0e-4;   // m/s^3/sqrt(Hz)
  return n;
}

NavState ornek_durum() {
  NavState x;
  TangentVec d;
  d << 0.20, -0.35, 0.55, 1.30, -0.70, 0.40, -2.10, 1.60, 0.90;
  x.extended_pose() = exp_map(d);
  x.gyro_bias() = Vec3(0.013, -0.021, 0.009);
  x.accel_bias() = Vec3(-0.07, 0.11, 0.04);
  return x;
}

// -----------------------------------------------------------------------------
// Nominal yayilim
// -----------------------------------------------------------------------------

TEST(ImuPropagator, StationaryLevelImuDoesNotMove) {
  // TURETME (ezberden degil):
  //   ENU'da hareketsiz, world ile hizali govde. Gercek dunya ivmesi a_W = 0.
  //   Olcum modeli:  a_m = R_WB^T (a_W - g_W) + b_a + n_a   (CONVENTIONS §7)
  //   R_WB = I, b_a = 0, n_a = 0, g_W = [0,0,-g]  =>  a_m = -g_W = [0,0,+g].
  // Yani duragan ve duz bir ivmeolcer +z'de +g okur.
  NavState x;
  NavCovariance p = NavCovariance::Zero();

  const ImuSample u = olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG));
  sessiz().propagate(x, p, u, 0.01);

  EXPECT_LT(x.extended_pose().linearVelocity().norm(), 1e-12) << "duragan IMU hiz uretti";
  EXPECT_LT(x.extended_pose().translation().norm(), 1e-12);
  EXPECT_LT(kerteriz::log_map(x.extended_pose()).head(3).norm(), 1e-15);
}

TEST(ImuPropagator, GravitySignPullsDownWhenAccelerometerReadsZero) {
  // Serbest dusus: ivmeolcer 0 okur, cisim -z yonunde hizlanir.
  NavState x;
  NavCovariance p = NavCovariance::Zero();

  sessiz().propagate(x, p, olcum(Vec3::Zero(), Vec3::Zero()), 1.0);

  EXPECT_NEAR(x.extended_pose().linearVelocity().z(), -kG, 1e-12) << "yer cekimi isareti ters";
  EXPECT_NEAR(x.extended_pose().translation().z(), -0.5 * kG, 1e-12);
}

TEST(ImuPropagator, ConstantAngularVelocityOnlyRotates) {
  NavState x;
  NavCovariance p = NavCovariance::Zero();

  const Vec3 w(0.0, 0.0, 0.5); // rad/s, z ekseni
  const Scalar dt = 0.4;
  sessiz().propagate(x, p, olcum(w, Vec3(0.0, 0.0, kG)), dt);

  const Eigen::Matrix3d beklenen =
      Eigen::AngleAxisd(w.norm() * dt, w.normalized()).toRotationMatrix();
  EXPECT_LT((x.extended_pose().rotation() - beklenen).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT(x.extended_pose().linearVelocity().norm(), 1e-12);
}

TEST(ImuPropagator, ConstantAccelerationMatchesClosedForm) {
  // Secilen ayriklastirma: v1 = v0 + a_W dt, p1 = p0 + v0 dt + 1/2 a_W dt^2.
  NavState x;
  NavCovariance p = NavCovariance::Zero();

  const Vec3 a_dunya(1.0, -2.0, 3.0);
  const Vec3 olculen = a_dunya - Vec3(0.0, 0.0, -kG); // a_m = a_W - g_W (R = I)
  const Scalar dt = 0.25;

  sessiz().propagate(x, p, olcum(Vec3::Zero(), olculen), dt);

  EXPECT_LT((x.extended_pose().linearVelocity() - a_dunya * dt).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT((x.extended_pose().translation() - 0.5 * a_dunya * dt * dt).cwiseAbs().maxCoeff(),
            1e-12);
}

TEST(ImuPropagator, BodyAccelerationIsRotatedIntoWorld) {
  // Yonelim birim DEGILKEN govde ivmesi dunyaya R_WB ile tasinmali.
  NavState x;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI / 2.0, Vec3::UnitZ()));
  x.extended_pose() = SE23(Vec3::Zero(), q, Vec3::Zero());
  NavCovariance p = NavCovariance::Zero();

  const Eigen::Matrix3d r = x.extended_pose().rotation();
  const Vec3 govde_ozgul(2.0, 0.0, 0.0); // govde +x
  const Vec3 olculen = govde_ozgul + r.transpose() * Vec3(0.0, 0.0, kG);
  const Scalar dt = 0.5;

  sessiz().propagate(x, p, olcum(Vec3::Zero(), olculen), dt);

  // 90 derece yaw: govde +x dunya +y'dir.
  const Vec3 beklenen = r * govde_ozgul * dt;
  EXPECT_LT((x.extended_pose().linearVelocity() - beklenen).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(x.extended_pose().linearVelocity().y(), 2.0 * dt, 1e-12);
}

TEST(ImuPropagator, GyroBiasIsSubtracted) {
  NavState x;
  const Vec3 bias(0.0, 0.0, 0.3);
  x.gyro_bias() = bias;
  NavCovariance p = NavCovariance::Zero();

  // Olcum = gercek + bias. Gercek sifirsa olcum tam bias kadardir; dolayisiyla
  // donme OLMAMALI. Isaret ters olsaydi iki kat donerdi.
  sessiz().propagate(x, p, olcum(bias, Vec3(0.0, 0.0, kG)), 0.5);

  EXPECT_LT(kerteriz::log_map(x.extended_pose()).head(3).norm(), 1e-14) << "jiro bias isareti ters";
}

TEST(ImuPropagator, AccelBiasIsSubtracted) {
  NavState x;
  const Vec3 bias(0.2, -0.3, 0.1);
  x.accel_bias() = bias;
  NavCovariance p = NavCovariance::Zero();

  sessiz().propagate(x, p, olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG) + bias), 0.5);

  EXPECT_LT(x.extended_pose().linearVelocity().norm(), 1e-12) << "ivme bias isareti ters";
}

TEST(ImuPropagator, BiasesAreConstantUnderNominalPropagation) {
  NavState x = ornek_durum();
  const Vec3 jiro = x.gyro_bias();
  const Vec3 ivme = x.accel_bias();
  NavCovariance p = NavCovariance::Zero();

  for (int i = 0; i < 50; ++i) {
    ImuPropagator(ornek_gurultu())
        .propagate(x, p, olcum(Vec3(0.1, -0.2, 0.3), Vec3(0.5, 0.4, kG)), 0.01);
  }
  EXPECT_LT((x.gyro_bias() - jiro).norm(), 1e-15) << "bias nominal yayilimda degisti";
  EXPECT_LT((x.accel_bias() - ivme).norm(), 1e-15);
}

TEST(ImuPropagator, ZeroDtIsNoOp) {
  // Sozlesme: dt = 0 hicbir sey yapmaz. F = I, Q = 0.
  NavState x = ornek_durum();
  const NavState once = x;
  NavCovariance p = NavCovariance::Identity() * 0.3;
  const NavCovariance p_once = p;

  StateMat f;
  StateMat q;
  sessiz().propagate(x, p, olcum(Vec3(1.0, 2.0, 3.0), Vec3(4.0, 5.0, 6.0)), 0.0, &f, &q);

  EXPECT_LT(x.minus(once).norm(), 1e-15) << "dt=0 durumu degistirdi";
  const int n = x.active_dof();
  EXPECT_LT((p.topLeftCorner(n, n) - p_once.topLeftCorner(n, n)).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT(
      (f.topLeftCorner(n, n) - StateMat::Identity().topLeftCorner(n, n)).cwiseAbs().maxCoeff(),
      1e-15);
  EXPECT_LT(q.topLeftCorner(n, n).cwiseAbs().maxCoeff(), 1e-18);
}

// -----------------------------------------------------------------------------
// Denetim (6) benzeri: analitik F, SAYISAL surec Jacobian'i ile dogrulanir.
//
// Formule guvenilmez. x (+) delta ayni IMU ve dt ile yayilir, nominal yayilmis
// duruma gore (-) farki olculur; merkezi farkla ayrik gecis Jacobian'i uretilip
// analitik F ile karsilastirilir. Cerceve veya bias isaret hatasi burada
// dogrudan gorunur.
// -----------------------------------------------------------------------------

StateMat sayisal_gecis(const ImuPropagator& prop, const NavState& x0, const ImuSample& u, Scalar dt,
                       int n, Scalar eps = 1e-6) {
  NavCovariance p_at = NavCovariance::Zero();
  NavState nominal = x0;
  prop.propagate(nominal, p_at, u, dt);

  StateMat j = StateMat::Zero();
  StateVec d = StateVec::Zero();

  for (int i = 0; i < n; ++i) {
    d.setZero();
    d[i] = eps;
    NavState arti = x0.plus(d);
    NavCovariance pa = NavCovariance::Zero();
    prop.propagate(arti, pa, u, dt);

    d[i] = -eps;
    NavState eksi = x0.plus(d);
    NavCovariance pe = NavCovariance::Zero();
    prop.propagate(eksi, pe, u, dt);

    const StateVec fark = arti.minus(nominal) - eksi.minus(nominal);
    j.col(i).head(n) = fark.head(n) / (2.0 * eps);
  }
  return j;
}

Scalar jacobian_hatasi(const NavState& x0, const ImuSample& u, Scalar dt) {
  const ImuPropagator prop = sessiz();
  const int n = x0.active_dof();

  NavState x = x0;
  NavCovariance p = NavCovariance::Zero();
  StateMat f;
  StateMat q;
  prop.propagate(x, p, u, dt, &f, &q);

  const StateMat sayisal = sayisal_gecis(prop, x0, u, dt, n);
  return (f.topLeftCorner(n, n) - sayisal.topLeftCorner(n, n)).cwiseAbs().maxCoeff();
}

TEST(ImuPropagator, AnalyticTransitionMatchesNumericJacobian) {
  const ImuSample u = olcum(Vec3(0.30, -0.45, 0.20), Vec3(0.8, -1.2, kG + 0.4));
  EXPECT_LT(jacobian_hatasi(ornek_durum(), u, 0.02), 1e-6);
}

TEST(ImuPropagator, NumericJacobianPerTangentBlock) {
  // Her blok AYRI perturbe edilir; boylece bir blogun hatasi digerlerinin
  // altinda gizlenmez.
  const ImuSample u = olcum(Vec3(0.30, -0.45, 0.20), Vec3(0.8, -1.2, kG + 0.4));
  const Scalar dt = 0.02;
  const NavState x0 = ornek_durum();
  const ImuPropagator prop = sessiz();
  const int n = x0.active_dof();

  NavState x = x0;
  NavCovariance p = NavCovariance::Zero();
  StateMat f;
  prop.propagate(x, p, u, dt, &f, nullptr);
  const StateMat sayisal = sayisal_gecis(prop, x0, u, dt, n);

  struct Blok {
    const char* ad;
    int bas;
  };
  const Blok bloklar[] = {{"dtheta", 0}, {"dv", 3}, {"dp", 6}, {"db_g", 9}, {"db_a", 12}};

  for (const auto& b : bloklar) {
    const Scalar hata =
        (f.block(0, b.bas, n, 3) - sayisal.block(0, b.bas, n, 3)).cwiseAbs().maxCoeff();
    EXPECT_LT(hata, 1e-6) << "blok " << b.ad;
  }
}

TEST(ImuPropagator, NumericJacobianWithAugmentation) {
  NavState x0 = ornek_durum();
  x0.register_calibration("wheel_scale", 1);
  x0.push_clone();
  ASSERT_EQ(x0.active_dof(), kCoreDof + 1 + kCloneDof);

  const ImuSample u = olcum(Vec3(0.2, 0.1, -0.3), Vec3(0.4, 0.6, kG - 0.2));
  EXPECT_LT(jacobian_hatasi(x0, u, 0.02), 1e-6);
}

TEST(ImuPropagator, AugmentationTransitionIsIdentityWithNoCoreCoupling) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 2);
  x.push_clone();
  const int n = x.active_dof();
  const int m = n - kCoreDof;

  NavCovariance p = NavCovariance::Zero();
  StateMat f;
  StateMat q;
  sessiz().propagate(x, p, olcum(Vec3(0.1, 0.2, 0.3), Vec3(0.0, 0.0, kG)), 0.02, &f, &q);

  EXPECT_LT((f.block(kCoreDof, kCoreDof, m, m) - Eigen::MatrixXd::Identity(m, m).cast<Scalar>())
                .cwiseAbs()
                .maxCoeff(),
            1e-15)
      << "augmentation gecisi birim degil";
  EXPECT_LT(f.block(0, kCoreDof, kCoreDof, m).cwiseAbs().maxCoeff(), 1e-15) << "core<-aug eslesme";
  EXPECT_LT(f.block(kCoreDof, 0, m, kCoreDof).cwiseAbs().maxCoeff(), 1e-15) << "aug<-core eslesme";
  EXPECT_LT(q.block(kCoreDof, kCoreDof, m, m).cwiseAbs().maxCoeff(), 1e-18)
      << "augmentation'a surec gurultusu enjekte edildi";
}

TEST(ImuPropagator, OutputPointersDoNotChangeResult) {
  // F/Q yalnizca teshis ciktisidir; algoritmanin davranisini degistiremez.
  const ImuSample u = olcum(Vec3(0.12, -0.33, 0.44), Vec3(1.1, -0.6, kG + 0.9));
  const ImuPropagator prop(ornek_gurultu());

  NavState a = ornek_durum();
  a.register_calibration("wheel_scale", 1);
  a.push_clone();
  NavState b = a;

  NavCovariance pa = NavCovariance::Identity() * 0.05;
  NavCovariance pb = pa;

  prop.propagate(a, pa, u, 0.02, nullptr, nullptr);
  StateMat f;
  StateMat q;
  prop.propagate(b, pb, u, 0.02, &f, &q);

  EXPECT_LT(a.minus(b).norm(), 1e-18) << "cikti isaretcileri durumu degistirdi";
  const int n = a.active_dof();
  EXPECT_LT((pa.topLeftCorner(n, n) - pb.topLeftCorner(n, n)).cwiseAbs().maxCoeff(), 1e-18)
      << "cikti isaretcileri kovaryansi degistirdi";
}

TEST(ImuPropagator, OutputTailIsDeterministicZero) {
  NavState x = ornek_durum();
  const int n = x.active_dof();

  StateMat f = StateMat::Constant(7.0);
  StateMat q = StateMat::Constant(7.0);
  NavCovariance p = NavCovariance::Zero();
  sessiz().propagate(x, p, olcum(Vec3(0.1, 0.0, 0.0), Vec3(0.0, 0.0, kG)), 0.01, &f, &q);

  EXPECT_LT(f.bottomRightCorner(kMaxStateDof - n, kMaxStateDof - n).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_LT(q.bottomRightCorner(kMaxStateDof - n, kMaxStateDof - n).cwiseAbs().maxCoeff(), 1e-18);
}

// -----------------------------------------------------------------------------
// Kovaryans ve surec gurultusu
//
// BIRIM NOTU: yogunluklar sigma/sqrt(Hz) cinsindendir, yani PSD = sigma^2.
// dt boyunca varyans katkisi sigma^2 * dt'dir (birim KARESI alinir).
// -----------------------------------------------------------------------------

TEST(ImuPropagator, SymmetryIsPreserved) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  const int n = x.active_dof();

  // Simetrik, pozitif tanimli baslangic.
  StateMat a = StateMat::Random();
  NavCovariance p = NavCovariance::Zero();
  p.topLeftCorner(n, n) = a.topLeftCorner(n, n) * a.topLeftCorner(n, n).transpose() +
                          Eigen::MatrixXd::Identity(n, n).cast<Scalar>();

  const ImuPropagator prop(ornek_gurultu());
  for (int i = 0; i < 20; ++i) {
    prop.propagate(x, p, olcum(Vec3(0.2, -0.1, 0.4), Vec3(0.3, 0.2, kG)), 0.005);
  }

  const auto blok = p.topLeftCorner(n, n);
  EXPECT_LT((blok - blok.transpose()).cwiseAbs().maxCoeff(), 1e-12) << "simetri kayboldu";
}

TEST(ImuPropagator, ProcessNoiseIsSymmetricAndPositiveSemidefinite) {
  NavState x = ornek_durum();
  NavCovariance p = NavCovariance::Zero();
  StateMat q;
  ImuPropagator(ornek_gurultu())
      .propagate(x, p, olcum(Vec3(0.1, 0.2, 0.3), Vec3(0.4, 0.5, kG)), 0.01, nullptr, &q);

  const int n = x.active_dof();
  const Eigen::MatrixXd blok = q.topLeftCorner(n, n);
  EXPECT_LT((blok - blok.transpose()).cwiseAbs().maxCoeff(), 1e-18) << "Q simetrik degil";

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> coz(blok);
  ASSERT_EQ(coz.info(), Eigen::Success);
  EXPECT_GT(coz.eigenvalues().minCoeff(), -1e-15) << "Q pozitif yari-tanimli degil";
}

TEST(ImuPropagator, ZeroNoiseParamsInjectNothing) {
  NavState x = ornek_durum();
  NavCovariance p = NavCovariance::Zero();
  StateMat q;
  sessiz().propagate(x, p, olcum(Vec3(0.1, 0.2, 0.3), Vec3(0.4, 0.5, kG)), 0.01, nullptr, &q);

  EXPECT_LT(q.cwiseAbs().maxCoeff(), 1e-18) << "gurultu sifirken Q sifir degil";
  EXPECT_LT(p.cwiseAbs().maxCoeff(), 1e-18) << "sifir P'ye gurultu eklendi";
}

TEST(ImuPropagator, NoiseCouplesOnlyToItsOwnSubspace) {
  const Scalar dt = 0.01;
  NavCovariance p = NavCovariance::Zero();
  StateMat q;

  auto blok_normu = [](const StateMat& m, int bas) {
    return m.block(bas, bas, 3, 3).cwiseAbs().maxCoeff();
  };

  // Yalnizca jiro beyaz gurultusu -> yalnizca dtheta.
  {
    NavState x = ornek_durum();
    ImuNoiseParams n{1.0e-3, 0.0, 0.0, 0.0};
    ImuPropagator(n).propagate(x, p, olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG)), dt, nullptr, &q);
    EXPECT_GT(blok_normu(q, 0), 0.0);
    EXPECT_LT(blok_normu(q, 3), 1e-18) << "jiro gurultusu hiza sizdi";
    EXPECT_LT(blok_normu(q, 9), 1e-18) << "jiro gurultusu bias'a sizdi";
  }
  // Yalnizca ivme beyaz gurultusu -> yalnizca dv.
  {
    NavState x = ornek_durum();
    ImuNoiseParams n{0.0, 0.0, 2.0e-2, 0.0};
    ImuPropagator(n).propagate(x, p, olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG)), dt, nullptr, &q);
    EXPECT_GT(blok_normu(q, 3), 0.0);
    EXPECT_LT(blok_normu(q, 0), 1e-18) << "ivme gurultusu aciya sizdi";
    EXPECT_LT(blok_normu(q, 12), 1e-18) << "ivme gurultusu bias'a sizdi";
  }
  // Yalnizca jiro rastgele yuruyusu -> yalnizca db_g.
  {
    NavState x = ornek_durum();
    ImuNoiseParams n{0.0, 1.0e-5, 0.0, 0.0};
    ImuPropagator(n).propagate(x, p, olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG)), dt, nullptr, &q);
    EXPECT_GT(blok_normu(q, 9), 0.0);
    EXPECT_LT(blok_normu(q, 0), 1e-18);
    EXPECT_LT(blok_normu(q, 12), 1e-18);
  }
  // Yalnizca ivme rastgele yuruyusu -> yalnizca db_a.
  {
    NavState x = ornek_durum();
    ImuNoiseParams n{0.0, 0.0, 0.0, 3.0e-4};
    ImuPropagator(n).propagate(x, p, olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG)), dt, nullptr, &q);
    EXPECT_GT(blok_normu(q, 12), 0.0);
    EXPECT_LT(blok_normu(q, 3), 1e-18);
    EXPECT_LT(blok_normu(q, 9), 1e-18);
  }
}

TEST(ImuPropagator, NoiseScalesLinearlyWithDtForFirstOrderDiscretization) {
  // Q = G Qc G^T dt: varyans dt ile DOGRUSAL buyur. sigma^2 dt.
  const ImuNoiseParams n = ornek_gurultu();
  NavCovariance p = NavCovariance::Zero();

  auto q_uret = [&](Scalar dt) {
    NavState x = ornek_durum();
    StateMat q;
    ImuPropagator(n).propagate(x, p, olcum(Vec3::Zero(), Vec3(0.0, 0.0, kG)), dt, nullptr, &q);
    return q;
  };

  const Scalar dt1 = 0.001;
  const Scalar dt2 = 0.004;
  const StateMat q1 = q_uret(dt1);
  const StateMat q2 = q_uret(dt2);

  EXPECT_NEAR(q2(0, 0) / q1(0, 0), dt2 / dt1, 1e-9) << "dtheta olceklemesi dogrusal degil";
  EXPECT_NEAR(q2(3, 3) / q1(3, 3), dt2 / dt1, 1e-9) << "dv olceklemesi dogrusal degil";

  // Mutlak deger: birim KARESI. sigma^2 * dt.
  EXPECT_NEAR(q1(0, 0), n.gyro_noise_density * n.gyro_noise_density * dt1, 1e-18);
  EXPECT_NEAR(q1(3, 3), n.accel_noise_density * n.accel_noise_density * dt1, 1e-18);
  EXPECT_NEAR(q1(9, 9), n.gyro_random_walk * n.gyro_random_walk * dt1, 1e-24);
  EXPECT_NEAR(q1(12, 12), n.accel_random_walk * n.accel_random_walk * dt1, 1e-24);
}

TEST(ImuPropagator, CrossCovarianceWithAugmentationIsPreservedAndTransformed) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  const int n = x.active_dof();
  const int aug = kCoreDof;

  NavCovariance p = NavCovariance::Zero();
  p.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * 0.1;
  p(3, aug) = 0.05; // dv <-> kalibrasyon capraz terimi
  p(aug, 3) = 0.05;
  const NavCovariance p_once = p;

  StateMat f;
  sessiz().propagate(x, p, olcum(Vec3(0.2, 0.1, -0.1), Vec3(0.3, 0.0, kG)), 0.02, &f, nullptr);

  EXPECT_GT(p.block(0, aug, kCoreDof, 1).cwiseAbs().maxCoeff(), 1e-6)
      << "capraz kovaryans kayboldu";

  // Dogru donusum: P_ca' = F_cc P_ca  (F_aa = I oldugu icin).
  const Eigen::MatrixXd beklenen =
      f.topLeftCorner(kCoreDof, kCoreDof) * p_once.block(0, aug, kCoreDof, 1);
  EXPECT_LT((p.block(0, aug, kCoreDof, 1) - beklenen).cwiseAbs().maxCoeff(), 1e-12)
      << "capraz kovaryans yanlis donusturuldu";

  EXPECT_NEAR(p(aug, aug), p_once(aug, aug), 1e-15) << "augmentation blogu degisti";
}

} // namespace

// -----------------------------------------------------------------------------
// Tahsis yasagi — CONVENTIONS §8.1 / ADR-22
//
// operator new GLOBAL scope'ta olmak ZORUNDA. Anonim ad alani icinde olsaydi
// global olani degistirmez, sayac hic artmaz ve test sahte bir 0 raporlardi.
// Bunu iddia etmek yerine dogruluyoruz: sayac once bilerek tahsis yapilarak
// sinaniyor.
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

TEST(ImuPropagator, AllocationCounterIsActuallyWired) {
  // Sayacin GERCEKTEN global operator new'i degistirdigini dogrular.
  // `new int(7)` derleyici tarafindan ELENEBILIR (C++14 tahsis eleme kurali),
  // o yuzden dogrudan ::operator new cagrilir — bu elenemez.
  int gorulen = 0;
  {
    const TahsisKapsami kapsam;
    void* ham = ::operator new(64);
    gorulen = tahsis_adedi;
    ::operator delete(ham);
  }
  EXPECT_GT(gorulen, 0) << "operator new degistirilmemis";
}

TEST(EigenAllocGuardDeathTest, GuardActuallyCatchesEigenAllocation) {
  // KRITIK: Eigen dinamik bellegi operator new ile DEGIL, dogrudan malloc ile
  // alir. Yalnizca operator new saymak Eigen tahsislerini kacirirdi ve test
  // sahte bir 0 raporlardi.
  //
  // Dogru arac Eigen'in kendi kapisidir: EIGEN_RUNTIME_NO_MALLOC +
  // set_is_malloc_allowed(false). Bu test o kapinin gercekten kapattigini
  // dogrular; kapi calismiyorsa asagidaki 0-tahsis testi anlamsiz olurdu.
  EXPECT_DEATH(
      {
        Eigen::internal::set_is_malloc_allowed(false);
        Eigen::MatrixXd m(64, 64); // dinamik: Eigen malloc'a gider
        m.setZero();
      },
      "");
}

TEST(ImuPropagator, PropagateDoesNotAllocate) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  NavCovariance p = NavCovariance::Identity() * 0.02;
  const ImuPropagator prop(ornek_gurultu());
  const ImuSample u = olcum(Vec3(0.2, -0.1, 0.3), Vec3(0.4, 0.3, kG));
  StateMat f;
  StateMat q;

  // Isinma: ilk cagri Eigen tarafinda tembel kurulum yapabilir.
  prop.propagate(x, p, u, 0.005);

  {
    const TahsisKapsami kapsam;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < 100; ++i) {
      prop.propagate(x, p, u, 0.005);
    }
    prop.propagate(x, p, u, 0.005, &f, &q);
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_EQ(tahsis_adedi, 0) << "sicak yol operator new'e gitti";
}

} // namespace
