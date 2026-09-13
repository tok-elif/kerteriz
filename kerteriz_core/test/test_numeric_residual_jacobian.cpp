/// \file
/// S7 — sayisal residual Jacobian (INTERFACES §7).
///
/// DoD: analitik turevi BILINEN oyuncak residual'larda sayisal Jacobian
/// analitige 1e-5 icinde esit.

#include "kerteriz/state/lie.hpp"
#include "kerteriz/util/numeric_residual_jacobian.hpp"

#include <gtest/gtest.h>

namespace {

using kerteriz::JacMat;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::TangentVec;

using ResidualVec = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using MatA = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

constexpr int kDof = 9;
constexpr Scalar kTol = 1e-5; // DoD'nin istedigi tolerans

SE23 ornek_x() {
  TangentVec d;
  d.segment<3>(kerteriz::kIdxTheta) << 0.25, -0.15, 0.40;
  d.segment<3>(kerteriz::kIdxVel) << 1.20, -0.60, 0.30;
  d.segment<3>(kerteriz::kIdxPos) << -1.75, 2.50, 0.90;
  return kerteriz::exp_map(d);
}

/// Deterministik, tekrar uretilebilir A matrisi.
MatA ornek_a(int satir) {
  MatA a(satir, kDof);
  for (int i = 0; i < satir; ++i) {
    for (int k = 0; k < kDof; ++k) {
      a(i, k) = 0.1 * static_cast<Scalar>((i + 1) * (k + 2)) - 0.35 * static_cast<Scalar>(i) +
                0.07 * static_cast<Scalar>(k * k);
    }
  }
  return a;
}

// --- Oyuncak 1: r(x) = A * (x (-) x_lin) -> J = A, TAM olarak ---------------
// Sebep: r(x_lin (+) d) = A * Log(x_lin^-1 o x_lin o Exp(d)) = A * d.
// Manifold egriligi girmez; bu yuzden analitik referans tam bilinir.

TEST(NumericResidualJacobian, MatchesAnalyticOnRightMinusResidual) {
  const SE23 x = ornek_x();
  const int dim = 5;
  const MatA a = ornek_a(dim);

  const JacMat j = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23& xp) -> ResidualVec { return a * kerteriz::minus(xp, x); }, dim);

  const MatA sayisal = j.topLeftCorner(dim, kDof);
  EXPECT_LT((sayisal - a).cwiseAbs().maxCoeff(), kTol);
}

// --- Oyuncak 2: r(x) = A * Log(x) -> J = A * Jrinv(Log x) ------------------
// PHASE0.md S7'nin onerdigi bicim. Egrilik burada gercekten devreye girer.

TEST(NumericResidualJacobian, MatchesAnalyticOnLogResidual) {
  const SE23 x = ornek_x();
  const int dim = 4;
  const MatA a = ornek_a(dim);

  const JacMat j = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23& xp) -> ResidualVec { return a * kerteriz::log_map(xp); }, dim);

  // Log(X o Exp(d)) ~ Log(X) + Jrinv(Log X) d
  const MatA analitik = a * kerteriz::right_jacobian_inverse(kerteriz::log_map(x));

  const MatA sayisal = j.topLeftCorner(dim, kDof);
  EXPECT_LT((sayisal - analitik).cwiseAbs().maxCoeff(), kTol) << "sayisal:\n"
                                                              << sayisal << "\nanalitik:\n"
                                                              << analitik;
}

// --- Isaret: cikti J_res'tir, olcum Jacobian'i degil -----------------------

TEST(NumericResidualJacobian, OutputIsResidualJacobianSignPreserved) {
  // r(x) = -(x (-) x_lin)  ->  J = -I.  Arac icinde hicbir negatifleme yoktur;
  // isaret dogrudan residual'dan gelir. ADR-13'un dayandigi ozellik budur.
  const SE23 x = ornek_x();

  const JacMat j = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23& xp) -> ResidualVec { return -kerteriz::minus(xp, x); }, kDof);

  const MatA beklenen = -MatA::Identity(kDof, kDof);
  EXPECT_LT((j.topLeftCorner(kDof, kDof) - beklenen).cwiseAbs().maxCoeff(), kTol);

  // Ters isaretli residual, ters isaretli Jacobian verir — sessiz bir mutlak
  // deger veya negatifleme olsaydi bu iki test ayni sonucu verirdi.
  const JacMat j_arti = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23& xp) -> ResidualVec { return kerteriz::minus(xp, x); }, kDof);

  EXPECT_LT((j_arti.topLeftCorner(kDof, kDof) + j.topLeftCorner(kDof, kDof)).cwiseAbs().maxCoeff(),
            kTol);
}

// --- Sag pertürbasyon kullanildiginin kaniti -------------------------------

TEST(NumericResidualJacobian, UsesRightPerturbation) {
  // r(x) = x (-) x_lin  icin J = I olmasi YALNIZCA sag pertürbasyonda dogrudur.
  // Sol pertürbasyon kullanilsaydi sonuc Ad_x'e bagli cikardi.
  const SE23 x = ornek_x();

  const JacMat j = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23& xp) -> ResidualVec { return kerteriz::minus(xp, x); }, kDof);

  const MatA birim = MatA::Identity(kDof, kDof);
  EXPECT_LT((j.topLeftCorner(kDof, kDof) - birim).cwiseAbs().maxCoeff(), kTol);

  // Ad_x birim degil — yani yukaridaki test gercekten ayirt edici.
  EXPECT_GT((kerteriz::adjoint(x) - kerteriz::AdjointMat::Identity()).cwiseAbs().maxCoeff(), 1e-2);
}

// --- Bicim ve dayaniklilik --------------------------------------------------

TEST(NumericResidualJacobian, UnusedBlocksAreZero) {
  const SE23 x = ornek_x();
  const int dim = 3;
  const MatA a = ornek_a(dim);

  const JacMat j = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23& xp) -> ResidualVec { return a * kerteriz::minus(xp, x); }, dim);

  // Kullanilmayan satirlar ve sutunlar sifir kalmalidir.
  EXPECT_EQ(j.bottomRows(kerteriz::kMaxResidualDim - dim).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ(j.rightCols(kerteriz::kMaxStateDof - kDof).cwiseAbs().maxCoeff(), 0.0);
}

TEST(NumericResidualJacobian, StepSizeIsRobust) {
  const SE23 x = ornek_x();
  const int dim = 4;
  const MatA a = ornek_a(dim);

  const auto fn = [&](const SE23& xp) -> ResidualVec { return a * kerteriz::log_map(xp); };
  const MatA analitik = a * kerteriz::right_jacobian_inverse(kerteriz::log_map(x));

  for (const Scalar eps : {1e-4, 1e-6, 1e-7}) {
    const JacMat j = kerteriz::numeric_residual_jacobian(x, fn, dim, eps);
    EXPECT_LT((j.topLeftCorner(dim, kDof) - analitik).cwiseAbs().maxCoeff(), kTol)
        << "eps = " << eps;
  }
}

TEST(NumericResidualJacobian, ZeroResidualGivesZeroJacobian) {
  const SE23 x = ornek_x();
  const int dim = 2;

  const JacMat j = kerteriz::numeric_residual_jacobian(
      x, [&](const SE23&) -> ResidualVec { return ResidualVec::Zero(dim); }, dim);

  EXPECT_EQ(j.cwiseAbs().maxCoeff(), 0.0);
}

} // namespace
