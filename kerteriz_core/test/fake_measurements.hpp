#pragma once

/// \file
/// S10 DoD icin SAHTE olcum tipleri.
///
/// Faz 0'da gercek olcum yoktur (INTERFACES §3 `Measurement` Faz 1'dedir).
/// Registry mekanizmasinin dogrulanabilmesi icin en az bir "olcum gibi
/// davranan" tip gerekir; bunlar odur. Uretim kodu DEGILDIR, yalnizca test
/// agacindadir ve kurulmaz.
///
/// Modeller kasten Jacobian'i KAPALI FORMDA bilinen turden secildi:
///   r(X) = A * Log(X)   =>   J_res = A * Jr^-1(Log(X))
/// Boylece Faz 1 test govdelerinin nasil gorunecegi birebir temsil edilir:
/// analitik Jacobian, sayisal Jacobian ile karsilastirilir.

#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>

namespace kerteriz::test_fakes {

/// 3 boyutlu sahte mutlak olcum — Faz 1'deki `GnssPosition`'in yerini tutar.
struct SahteGnss {
  static constexpr int kDim = 3;
  using Katsayi = Eigen::Matrix<Scalar, kDim, SE23::DoF>;

  static Katsayi katsayi() {
    Katsayi a;
    // Sabit, kesin tanimli, ozel bir yapisi olmayan katsayilar.
    for (int i = 0; i < kDim; ++i) {
      for (int k = 0; k < SE23::DoF; ++k) {
        a(i, k) = Scalar(0.1) * Scalar(i + 1) - Scalar(0.03) * Scalar(k) + Scalar(0.5);
      }
    }
    return a;
  }

  static Eigen::Matrix<Scalar, kDim, 1> residual(const SE23& x) {
    return (katsayi() * log_map(x)).eval();
  }

  static Eigen::Matrix<Scalar, kDim, SE23::DoF> analytic_jacobian(const SE23& x) {
    return (katsayi() * right_jacobian_inverse(log_map(x))).eval();
  }
};

/// 1 boyutlu sahte olcum — Faz 1'deki `WheelVelocity` gibi skaler bir artik.
struct SahteTeker {
  static constexpr int kDim = 1;
  using Katsayi = Eigen::Matrix<Scalar, kDim, SE23::DoF>;

  static Katsayi katsayi() {
    Katsayi a;
    for (int k = 0; k < SE23::DoF; ++k) {
      a(0, k) = Scalar(1.3) - Scalar(0.11) * Scalar(k);
    }
    return a;
  }

  static Eigen::Matrix<Scalar, kDim, 1> residual(const SE23& x) {
    return (katsayi() * log_map(x)).eval();
  }

  static Eigen::Matrix<Scalar, kDim, SE23::DoF> analytic_jacobian(const SE23& x) {
    return (katsayi() * right_jacobian_inverse(log_map(x))).eval();
  }
};

} // namespace kerteriz::test_fakes
