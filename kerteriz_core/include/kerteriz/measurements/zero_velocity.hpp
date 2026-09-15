#pragma once

/// \file
/// Sifir hiz guncellemesi (ZUPT) — INTERFACES §3 · CONVENTIONS §1, §3.
///
/// ============================ KAPSAM =======================================
///
/// Aracin durdugu TESPIT EDILDIGINDE uretilir. Durma TESPITI burada YAPILMAZ;
/// bu sinif yalnizca kisitin kendisidir. Tespit mantigi (IMU varyans esigi,
/// teker hizi, arac durumu) adaptor/config katmanina aittir — olcum modeli
/// platform davranisini tahmin etmeye calismaz.
///
/// ========================== OLCUM MODELI ===================================
///
///   h(X) = v_WB
///   z_W  = 0
///   r    = -v_WB
///
/// Artik DUNYA cercevesinde alinir (Faz 1 karari). Govde cercevesinde
/// R^T v_WB yazmak ayni kisiti ifade ederdi ama yonelim sutununu sifirdan
/// farkli yapardi; Faz 1 basit ve dogrudan olani secer.
///
/// ========================= RESIDUAL JACOBIAN ================================
///
/// Sag pertürbasyon:  v = v^ + R^ dv.
///
///   r(X (+) d) = -(v^ + R^ dv) = r^ - R^ dv
///
///   J_res,v     = -R_WB              (sutun 3..5)
///   J_res,theta = 0                  (sutun 0..2)
///
/// ZUPT DOGRUDAN BIR BIAS OLCUMU DEGILDIR. Jiro/ivme bias sutunlarina elle
/// artik veya Jacobian YAZILMAZ; bias'a dusen duzeltme yalnizca kovaryansta
/// birikmis capraz korelasyon uzerinden olusur. Elle bias terimi eklemek
/// gozlemlenebilir olmayan bir yonu gozlemlenebilir gostermek olurdu.
///
/// Calisma alaninin kullanilmayan bolgesi CAGIRAN tarafindan sifirlanir.

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <string_view>

namespace kerteriz {

class ZeroVelocity final : public Measurement {
 public:
  ZeroVelocity(TimeNs stamp_ns, std::string_view name, const Eigen::Matrix3d& covariance_W)
      : stamp_ns_(stamp_ns), name_(name), covariance_w_(covariance_W) {}

  TimeNs stamp_ns() const override { return stamp_ns_; }
  int residual_dim() const override { return kDim; }
  std::string_view name() const override { return name_; }

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    const NavState& x = states.current();

    w.dim = kDim;
    w.r.head<kDim>() = -x.extended_pose().linearVelocity();
    w.R.topLeftCorner<kDim, kDim>() = covariance_w_;

    w.J_res.block<kDim, 3>(0, kIdxVel) = -x.extended_pose().rotation();
  }

 private:
  static constexpr int kDim = 3;
  static_assert(kDim <= kMaxResidualDim, "artik boyutu kapasiteyi asiyor");

  TimeNs stamp_ns_;
  std::string_view name_; ///< omru olcumden uzundur (yapilandirmada sahiplenilir)
  Eigen::Matrix3d covariance_w_;
};

} // namespace kerteriz
