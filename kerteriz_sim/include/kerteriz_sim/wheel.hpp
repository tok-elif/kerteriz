#pragma once

/// \file
/// Teker hizi sentezleyici — PHASE0.md S9.
/// Govde hizi + olcek hatasi + gurultu.
///
/// Teker ileri yonu okur: REP-103'te x_B ileridir (CONVENTIONS §1).
/// Olcek faktoru ADR-10'da durum vektorune giren buyuklugun ta kendisidir;
/// burada bilinen bir hata olarak enjekte edilir ki Faz 3'teki kalibrasyon
/// onu geri bulabilsin.

#include "kerteriz/types.hpp"
#include "kerteriz_sim/rng.hpp"
#include "kerteriz_sim/trajectory.hpp"

namespace kerteriz_sim {

using kerteriz::TimeNs;

struct WheelParams {
  Scalar scale_factor = Scalar(1);    ///< 1.0 = hatasiz
  Scalar speed_noise_std = Scalar(0); ///< m/s
};

struct WheelSample {
  TimeNs stamp_ns = 0;
  Scalar forward_speed = Scalar(0); ///< m/s, govde x ekseni
};

class WheelSynthesizer {
 public:
  WheelSynthesizer(WheelParams params, SeededRng& rng) : params_(params), rng_(&rng) {}

  WheelSample sample(const TrajectorySample& gt) {
    const Vec3 govde_hizi = gt.rotation.transpose() * gt.velocity;

    WheelSample out;
    out.stamp_ns = gt.stamp_ns;
    out.forward_speed =
        params_.scale_factor * govde_hizi.x() + params_.speed_noise_std * rng_->gaussian();
    return out;
  }

 private:
  WheelParams params_;
  SeededRng* rng_;
};
} // namespace kerteriz_sim
