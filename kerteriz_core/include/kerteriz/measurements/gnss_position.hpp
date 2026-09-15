#pragma once

/// \file
/// GNSS konum olcumu — INTERFACES §3 · CONVENTIONS §1, §3, §7.
///
/// ============================ KAPSAM =======================================
///
/// Bu sinif ENU METRE cinsinden konum tuketir. LLH -> ENU donusumu BURADA
/// YAPILMAZ; orijin politikasi ve cografi donusum ust katmanin isidir
/// (CONVENTIONS §2 `origin_policy`). Cekirdek ROS'suz ve cografi kutuphane
/// bagimsiz kalir (R1, ADR-1).
///
/// ========================== OLCUM MODELI ===================================
///
///   h(X) = p_WB + R_WB p_BS          p_BS: anten lever-arm, GOVDE cercevesinde
///   r    = z_W - h(X)
///
/// ========================= RESIDUAL JACOBIAN ================================
///
/// Sag perturbasyon (CONVENTIONS §3.1) SE_2_3 uzerinde konum hatasini GOVDE
/// cercevesinde tanimlar:  R = R^ Exp(dtheta),  p = p^ + R^ dp.
///
///   r(X (+) d) = z - ( p^ + R^ dp + R^ Exp(dtheta) p_BS )
///              ~ r^ - R^ dp + R^ [p_BS]x dtheta
///
/// cunku [dtheta]x p_BS = -[p_BS]x dtheta. Dolayisiyla:
///
///   J_res,theta = R_WB [p_BS]x        (sutun 0..2)
///   J_res,p     = -R_WB               (sutun 6..8)
///
/// Geri kalan tum sutunlar sifirdir: hiz, bias'lar ve augmentation blogu bu
/// olcumu ETKILEMEZ. p_BS = 0 ozel durumunda yonelim blogu tam sifirdir —
/// lever-arm yokken GNSS konumu yonelim hakkinda bilgi TASIMAZ.
///
/// Calisma alaninin kullanilmayan bolgesi CAGIRAN tarafindan sifirlanir
/// (MeasurementWorkspace::clear, backend her update'te cagirir).

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <cassert>
#include <string_view>

namespace kerteriz {

class GnssPosition final : public Measurement {
 public:
  GnssPosition(TimeNs stamp_ns, std::string_view name, const Vec3& position_W,
               const Eigen::Matrix3d& covariance_W, const Vec3& p_BS = Vec3::Zero())
      : stamp_ns_(stamp_ns), name_(name), position_w_(position_W), covariance_w_(covariance_W),
        p_bs_(p_BS) {}

  TimeNs stamp_ns() const override { return stamp_ns_; }
  int residual_dim() const override { return kDim; }
  std::string_view name() const override { return name_; }

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    const NavState& x = states.current();
    const Eigen::Matrix3d r_wb = x.extended_pose().rotation();
    const Vec3 p_wb = x.extended_pose().translation();

    w.dim = kDim;
    w.r.head<kDim>() = position_w_ - (p_wb + r_wb * p_bs_);
    w.R.topLeftCorner<kDim, kDim>() = covariance_w_;

    // Yalnizca bu olcumun etkiledigi sutunlar yazilir; gerisi cagiranin
    // sifirladigi hâliyle kalir.
    w.J_res.block<kDim, 3>(0, kIdxTheta) = r_wb * capraz(p_bs_);
    w.J_res.block<kDim, 3>(0, kIdxPos) = -r_wb;
  }

 private:
  static constexpr int kDim = 3;
  static_assert(kDim <= kMaxResidualDim, "artik boyutu kapasiteyi asiyor");

  static Eigen::Matrix3d capraz(const Vec3& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
  }

  TimeNs stamp_ns_;
  std::string_view name_; ///< omru olcumden uzundur (yapilandirmada sahiplenilir)
  Vec3 position_w_;
  Eigen::Matrix3d covariance_w_;
  Vec3 p_bs_;
};

} // namespace kerteriz
