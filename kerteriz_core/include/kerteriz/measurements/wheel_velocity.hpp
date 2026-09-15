#pragma once

/// \file
/// Teker ileri hizi olcumu — INTERFACES §3 · CONVENTIONS §1, §3 · ADR-10.
///
/// ============================ KAPSAM =======================================
///
/// Cekirdek HAM TICK GORMEZ. Teker yaricapi, tur basina tick sayisi ve
/// aktarma orani PLATFORMA OZEL sayilardir; olcum modeline gomulurlerse model
/// tek bir araca baglanir. Bu sinif metre/saniye cinsinden, GOVDE +x
/// dogrultusundaki ileri hizi tuketir (REP-103: x_B ileridir, CONVENTIONS §1).
/// Tick -> m/s cevrimi adaptor katmanindadir.
///
/// ADR-10'daki teker OLCEK FAKTORU cevrimici kalibrasyonu FAZ 3'TEDIR. F1.6
/// yalnizca mevcut 15 DoF cekirdek durumu kullanir; olcek durumu ne okunur ne
/// de Jacobian'a sutun eklenir.
///
/// ========================== OLCUM MODELI ===================================
///
///   u    = R_WB^T v_WB               govde cercevesindeki hiz
///   h(X) = e_x^T u                   e_x = [1 0 0]^T
///   r    = z_forward - e_x^T u
///
/// ========================= RESIDUAL JACOBIAN ================================
///
/// Sag pertürbasyon:  R = R^ Exp(dtheta),  v = v^ + R^ dv.
///
///   u(X (+) d) = Exp(dtheta)^T R^^T (v^ + R^ dv)
///              ~ (I - [dtheta]x)(u^ + dv)
///              ~ u^ + dv - [dtheta]x u^
///              = u^ + dv + [u^]x dtheta
///
/// cunku [dtheta]x u = -[u]x dtheta. Dolayisiyla:
///
///   J_res,theta = -e_x^T [u]x        (sutun 0..2)
///   J_res,v     = -e_x^T             (sutun 3..5)
///
/// Konum, bias ve augmentation sutunlari sifirdir.
///
/// YONELIM TERIMI GERCEKTIR: govde hizi yonelimden gecer, bu yuzden u'nun y
/// veya z bileseni sifirdan farkliyken capraz terim de sifirdan farklidir.
///
/// Calisma alaninin kullanilmayan bolgesi CAGIRAN tarafindan sifirlanir.

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <string_view>

namespace kerteriz {

class WheelVelocity final : public Measurement {
 public:
  WheelVelocity(TimeNs stamp_ns, std::string_view name, Scalar forward_velocity_B, Scalar variance)
      : stamp_ns_(stamp_ns), name_(name), forward_velocity_b_(forward_velocity_B),
        variance_(variance) {}

  TimeNs stamp_ns() const override { return stamp_ns_; }
  int residual_dim() const override { return kDim; }
  std::string_view name() const override { return name_; }

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    const NavState& x = states.current();
    const Eigen::Matrix3d r_wb = x.extended_pose().rotation();
    const Vec3 u = r_wb.transpose() * x.extended_pose().linearVelocity();

    w.dim = kDim;
    w.r[0] = forward_velocity_b_ - u.x();
    w.R(0, 0) = variance_;

    // e_x^T M, yani M'nin ILK SATIRI. Ayri bir e_x vektoru kurulmaz.
    w.J_res.block<kDim, 3>(0, kIdxTheta) = -capraz(u).row(0);
    w.J_res.block<kDim, 3>(0, kIdxVel) = -Eigen::Matrix<Scalar, 1, 3>::Unit(0);
  }

 private:
  static constexpr int kDim = 1;
  static_assert(kDim <= kMaxResidualDim, "artik boyutu kapasiteyi asiyor");

  TimeNs stamp_ns_;
  std::string_view name_; ///< omru olcumden uzundur (yapilandirmada sahiplenilir)
  Scalar forward_velocity_b_;
  Scalar variance_;
};

} // namespace kerteriz
