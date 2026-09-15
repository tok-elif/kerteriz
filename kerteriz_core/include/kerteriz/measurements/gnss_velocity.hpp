#pragma once

/// \file
/// GNSS hiz olcumu — INTERFACES §3 · CONVENTIONS §1, §3.
///
/// ============================ KAPSAM =======================================
///
/// Bu sinif ENU METRE/SANIYE cinsinden, GOVDE ORIJININE indirgenmis hiz
/// tuketir. Doppler cozumu, LLH/ENU donusumu ve mesaj ayristirma BURADA
/// YAPILMAZ; cekirdek ROS'suz ve cografi kutuphane bagimsiz kalir (R1, ADR-1).
///
/// ANTEN KOLU (lever-arm) DUZELTMESI BURADA YAPILMAZ. Anten hizi
///
///   v_WS = v_WB + R_WB ([w_B]x p_BS)
///
/// olup donme terimi ACISAL HIZ gerektirir; acisal hiz NavState'in parcasi
/// DEGILDIR (CONVENTIONS §5.1'deki cekirdek 15 DoF'ta yoktur). Dolayisiyla bu
/// duzeltme durum uzerinden turetilemez. Ham anten hizini govde orijinine
/// indirgemek, IMU acisal hizina erisimi olan ADAPTOR katmaninin isidir; F1.6
/// sozlesmesi cekirdege ZATEN INDIRGENMIS hiz verildigidir.
///
/// ========================== OLCUM MODELI ===================================
///
///   h(X) = v_WB
///   r    = z_W - v_WB
///
/// ========================= RESIDUAL JACOBIAN ================================
///
/// Sag pertürbasyon (CONVENTIONS §3.1) hiz hatasini GOVDE cercevesinde
/// tanimlar:  v = v^ + R^ dv,  R = R^ Exp(dtheta).
///
///   r(X (+) d) = z - (v^ + R^ dv) = r^ - R^ dv
///
/// Dolayisiyla:
///
///   J_res,v     = -R_WB              (sutun 3..5)
///   J_res,theta = 0                  (sutun 0..2)
///
/// Yonelim sutunu TAM SIFIRDIR: dunya cercevesindeki hiz olcumu, hatanin
/// govde cercevesinde parametrelenmesinden bagimsizdir. Konum, bias'lar ve
/// augmentation sutunlari da sifirdir.
///
/// -R_WB ile -I ayrimi onemlidir; birim olmayan bir yonelimle sinanir.
///
/// Calisma alaninin kullanilmayan bolgesi CAGIRAN tarafindan sifirlanir
/// (MeasurementWorkspace::clear, backend her update'te cagirir).

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <string_view>

namespace kerteriz {

class GnssVelocity final : public Measurement {
 public:
  GnssVelocity(TimeNs stamp_ns, std::string_view name, const Vec3& velocity_W,
               const Eigen::Matrix3d& covariance_W)
      : stamp_ns_(stamp_ns), name_(name), velocity_w_(velocity_W), covariance_w_(covariance_W) {}

  TimeNs stamp_ns() const override { return stamp_ns_; }
  int residual_dim() const override { return kDim; }
  std::string_view name() const override { return name_; }

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    const NavState& x = states.current();

    w.dim = kDim;
    w.r.head<kDim>() = velocity_w_ - x.extended_pose().linearVelocity();
    w.R.topLeftCorner<kDim, kDim>() = covariance_w_;

    // Yalnizca bu olcumun etkiledigi sutun yazilir; gerisi cagiranin
    // sifirladigi hâliyle kalir.
    w.J_res.block<kDim, 3>(0, kIdxVel) = -x.extended_pose().rotation();
  }

 private:
  static constexpr int kDim = 3;
  static_assert(kDim <= kMaxResidualDim, "artik boyutu kapasiteyi asiyor");

  TimeNs stamp_ns_;
  std::string_view name_; ///< omru olcumden uzundur (yapilandirmada sahiplenilir)
  Vec3 velocity_w_;
  Eigen::Matrix3d covariance_w_;
};

} // namespace kerteriz
