#pragma once

/// \file
/// Non-holonomic kisiti — INTERFACES §3 · CONVENTIONS §1, §3.
///
/// ============================ KAPSAM =======================================
///
/// Tekerlekli bir arac yana ve yukari kaymaz: govde y ve z hizi sifira
/// yakindir. Kisitin GECERLI OLUP OLMADIGINA bu sinif karar VERMEZ — kayan,
/// paletli veya havalanan platformlarda kapatilmasi yapilandirma/adaptor
/// isidir. Olcum modeli platform davranisini tahmin etmeye calismaz;
/// kovaryans da tam 2x2 verilir, boylece "ne kadar gevsek" cagiranin
/// elindedir.
///
/// ========================== OLCUM MODELI ===================================
///
///   u    = R_WB^T v_WB
///   A    = [0 1 0 ; 0 0 1]           y ve z satirlarini secer
///   h(X) = A u
///   z    = [0, 0]^T
///   r    = -A u
///
/// ========================= RESIDUAL JACOBIAN ================================
///
/// WheelVelocity ile ayni pertürbasyon:  u ~ u^ + dv + [u^]x dtheta.
///
///   J_res,theta = -A [u]x            (sutun 0..2)
///   J_res,v     = -A                 (sutun 3..5)
///
/// Konum, bias ve augmentation sutunlari sifirdir.
///
/// Calisma alaninin kullanilmayan bolgesi CAGIRAN tarafindan sifirlanir.

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <string_view>

namespace kerteriz {

class NonHolonomic final : public Measurement {
 public:
  NonHolonomic(TimeNs stamp_ns, std::string_view name,
               const Eigen::Matrix<Scalar, 2, 2>& covariance_B)
      : stamp_ns_(stamp_ns), name_(name), covariance_b_(covariance_B) {}

  TimeNs stamp_ns() const override { return stamp_ns_; }
  int residual_dim() const override { return kDim; }
  std::string_view name() const override { return name_; }

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    const NavState& x = states.current();
    const Eigen::Matrix3d r_wb = x.extended_pose().rotation();
    const Vec3 u = r_wb.transpose() * x.extended_pose().linearVelocity();

    w.dim = kDim;
    w.r.head<kDim>() = -u.tail<kDim>(); // -[u_y, u_z]
    w.R.topLeftCorner<kDim, kDim>() = covariance_b_;

    // A M, yani M'nin SON IKI SATIRI. Ayri bir A matrisi kurulmaz.
    w.J_res.block<kDim, 3>(0, kIdxTheta) = -capraz(u).bottomRows<kDim>();
    w.J_res.block<kDim, 3>(0, kIdxVel) =
        -Eigen::Matrix<Scalar, 3, 3>::Identity().bottomRows<kDim>();
  }

 private:
  static constexpr int kDim = 2;
  static_assert(kDim <= kMaxResidualDim, "artik boyutu kapasiteyi asiyor");

  TimeNs stamp_ns_;
  std::string_view name_; ///< omru olcumden uzundur (yapilandirmada sahiplenilir)
  Eigen::Matrix<Scalar, 2, 2> covariance_b_;
};

} // namespace kerteriz
