/// \file
/// S6 — SE_2(3) sarmalayici ve konvansiyon bekcileri (PHASE0.md S6).
///
/// RightPerturbationConvention ve QuaternionStorageOrder KALICI BEKCIDIR:
/// konvansiyon kayarsa bu iki test duser.

#include "kerteriz/state/lie.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace {

using kerteriz::AdjointMat;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::TangentVec;

constexpr Scalar kTol = 1e-10;

/// Deterministik, kimlikten uzak bir grup elemani.
SE23 ornek_x() {
  TangentVec d;
  d.segment<3>(kerteriz::kIdxTheta) << 0.30, -0.20, 0.45;
  d.segment<3>(kerteriz::kIdxVel) << 1.50, -0.75, 0.25;
  d.segment<3>(kerteriz::kIdxPos) << -2.00, 3.25, 0.80;
  return kerteriz::exp_map(d);
}

TangentVec ornek_tau(Scalar olcek) {
  TangentVec d;
  d.segment<3>(kerteriz::kIdxTheta) << 0.12, 0.34, -0.21;
  d.segment<3>(kerteriz::kIdxVel) << -0.60, 0.15, 0.90;
  d.segment<3>(kerteriz::kIdxPos) << 0.70, -1.10, 0.40;
  return olcek * d;
}

// --- PHASE0 S6'nin bes testi ------------------------------------------------

TEST(Lie, ExpLogRoundTrip) {
  // Log(Exp(tau)) ~ tau, kucuk ve orta tau icin.
  for (const Scalar olcek : {1e-8, 1e-4, 0.1, 1.0}) {
    const TangentVec tau = ornek_tau(olcek);
    const TangentVec tekrar = kerteriz::log_map(kerteriz::exp_map(tau));
    EXPECT_LT((tekrar - tau).norm(), kTol) << "olcek = " << olcek;
  }
}

TEST(Lie, PlusMinusRoundTrip) {
  // X (+) (Y (-) X) ~ Y
  const SE23 x = ornek_x();
  const SE23 y = kerteriz::plus(x, ornek_tau(0.4));

  const TangentVec fark = kerteriz::minus(y, x);
  const SE23 tekrar = kerteriz::plus(x, fark);

  EXPECT_LT((kerteriz::minus(tekrar, y)).norm(), kTol);
}

TEST(Lie, RightPerturbationConvention) {
  // KALICI BEKCI. CONVENTIONS §3.1:  X (+) d = X o Exp(d)  — SOL DEGIL.
  const SE23 x = ornek_x();
  const TangentVec d = ornek_tau(0.3);

  const SE23 sag = x.compose(kerteriz::exp_map(d));
  const SE23 sol = kerteriz::exp_map(d).compose(x);

  EXPECT_LT((kerteriz::minus(kerteriz::plus(x, d), sag)).norm(), kTol)
      << "plus() sag pertürbasyon degil";

  // Ikisi ayni olsaydi test hicbir sey ispatlamazdi; once ayristiklarini goster.
  EXPECT_GT((kerteriz::minus(sag, sol)).norm(), 1e-3)
      << "sag ve sol bu ornekte ayrismiyor — test bekci degil";
  EXPECT_GT((kerteriz::minus(kerteriz::plus(x, d), sol)).norm(), 1e-3)
      << "plus() sol pertürbasyonla ortusuyor";
}

TEST(Lie, AdjointIdentity) {
  // Ad tanimi:  X o Exp(tau) o X^-1 = Exp(Ad_X tau).
  // Tersi ACIKCA yazmamak icin esdeger bicim kullanilir:
  //     X o Exp(tau) == Exp(Ad_X tau) o X
  const SE23 x = ornek_x();
  const TangentVec tau = ornek_tau(0.25);

  const AdjointMat ad = kerteriz::adjoint(x);

  const SE23 sol_taraf = x.compose(kerteriz::exp_map(tau));
  const SE23 sag_taraf = kerteriz::exp_map(ad * tau).compose(x);

  EXPECT_LT((kerteriz::minus(sol_taraf, sag_taraf)).norm(), kTol);

  // Kimlik elemaninin adjoint'i birim matristir.
  EXPECT_LT((kerteriz::adjoint(SE23::Identity()) - AdjointMat::Identity()).norm(), kTol);
}

TEST(Lie, QuaternionStorageOrder) {
  // KALICI BEKCI. CONVENTIONS §2 "Eigen tuzagi":
  //   yapici (w, x, y, z)  ama  coeffs() (x, y, z, w) dondurur.
  const Eigen::Quaterniond q(0.1, 0.2, 0.3, 0.4);

  EXPECT_DOUBLE_EQ(q.w(), 0.1);
  EXPECT_DOUBLE_EQ(q.x(), 0.2);
  EXPECT_DOUBLE_EQ(q.y(), 0.3);
  EXPECT_DOUBLE_EQ(q.z(), 0.4);

  EXPECT_DOUBLE_EQ(q.coeffs()[0], 0.2) << "coeffs()[0] x degil";
  EXPECT_DOUBLE_EQ(q.coeffs()[1], 0.3);
  EXPECT_DOUBLE_EQ(q.coeffs()[2], 0.4);
  EXPECT_DOUBLE_EQ(q.coeffs()[3], 0.1) << "coeffs()[3] w degil — skaler sonda olmali";

  // Hamilton konvansiyonu: (q1 * q2) bilesimi R1 * R2'ye karsilik gelir.
  // JPL konvansiyonunda sira ters olurdu.
  const Eigen::Quaterniond q1(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond q2(Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitX()));
  const Eigen::Matrix3d bilesim = (q1 * q2).toRotationMatrix();
  const Eigen::Matrix3d carpim = q1.toRotationMatrix() * q2.toRotationMatrix();
  EXPECT_LT((bilesim - carpim).norm(), kTol) << "Hamilton degil";
}

// --- Tegent sirasi bekcisi ---------------------------------------------------
// manif'in sirasi bizimkinden farkli; permutasyon lie.hpp'de tek yerde yapilir.
// Bu test o donusumun sessizce kaymasini engeller.

TEST(Lie, TangentOrderMatchesConventions) {
  // CONVENTIONS §5.1: [ dtheta(0..2) dv(3..5) dp(6..8) ]
  EXPECT_EQ(kerteriz::kIdxTheta, 0);
  EXPECT_EQ(kerteriz::kIdxVel, 3);
  EXPECT_EQ(kerteriz::kIdxPos, 6);

  // Yalnizca donme bileseni olan bir tegent: konum ve hiz sifir kalmali.
  TangentVec sadece_donme = TangentVec::Zero();
  sadece_donme.segment<3>(kerteriz::kIdxTheta) << 0.0, 0.0, 0.5;

  const SE23 x = kerteriz::exp_map(sadece_donme);
  EXPECT_LT(x.translation().norm(), kTol) << "dtheta konuma sizdi — sira yanlis";
  EXPECT_LT(x.linearVelocity().norm(), kTol) << "dtheta hiza sizdi — sira yanlis";

  // Yalnizca konum bileseni: donme kimlik kalmali.
  TangentVec sadece_konum = TangentVec::Zero();
  sadece_konum.segment<3>(kerteriz::kIdxPos) << 1.0, -2.0, 3.0;

  const SE23 y = kerteriz::exp_map(sadece_konum);
  EXPECT_LT((y.rotation() - Eigen::Matrix3d::Identity()).norm(), kTol)
      << "dp donmeye sizdi — sira yanlis";
  EXPECT_LT(y.linearVelocity().norm(), kTol) << "dp hiza sizdi — sira yanlis";
  EXPECT_LT((y.translation() - Eigen::Vector3d(1.0, -2.0, 3.0)).norm(), kTol);

  // Yalnizca hiz bileseni.
  TangentVec sadece_hiz = TangentVec::Zero();
  sadece_hiz.segment<3>(kerteriz::kIdxVel) << -0.5, 0.25, 4.0;

  const SE23 z = kerteriz::exp_map(sadece_hiz);
  EXPECT_LT(z.translation().norm(), kTol) << "dv konuma sizdi — sira yanlis";
  EXPECT_LT((z.linearVelocity() - Eigen::Vector3d(-0.5, 0.25, 4.0)).norm(), kTol);
}

TEST(Lie, PermutationIsOrthogonal) {
  // P bir permutasyon matrisidir: P^T P = I. adjoint() bu varsayima dayanir.
  const AdjointMat p = kerteriz::tangent_permutation();
  EXPECT_LT((p.transpose() * p - AdjointMat::Identity()).norm(), kTol);
}

} // namespace
