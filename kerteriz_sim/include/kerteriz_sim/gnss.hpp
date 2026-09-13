#pragma once

/// \file
/// GNSS sentezleyici — PHASE0.md S9. Konum + gurultu, yapilandirilabilir hiz.
///
/// Ciktinin cercevesi W'dir (CONVENTIONS §1). Coğrafi donusum YOKTUR:
/// SPEC kapsam disi tablosunda GNSS ham gozlem isleme zaten yok, ve
/// origin_policy "identity" simulasyon icin birinci siniftir.

#include "kerteriz/types.hpp"
#include "kerteriz_sim/rng.hpp"
#include "kerteriz_sim/trajectory.hpp"

namespace kerteriz_sim {

using kerteriz::TimeNs;

struct GnssParams {
  Scalar position_noise_std = Scalar(0); ///< m, eksen basina
  Scalar rate_hz = Scalar(5);            ///< olcum hizi
};

struct GnssSample {
  TimeNs stamp_ns = 0;
  Vec3 position = Vec3::Zero(); ///< p_WB, W
};

class GnssSynthesizer {
 public:
  GnssSynthesizer(GnssParams params, SeededRng& rng) : params_(params), rng_(&rng) {}

  /// Iki olcum arasindaki sure, nanosaniye. Cagiran zamanlamayi buna gore kurar.
  TimeNs period_ns() const {
    if (params_.rate_hz <= Scalar(0)) {
      return 0;
    }
    return static_cast<TimeNs>(Scalar(1e9) / params_.rate_hz);
  }

  GnssSample sample(const TrajectorySample& gt) {
    GnssSample out;
    out.stamp_ns = gt.stamp_ns;
    out.position = gt.position + params_.position_noise_std * rng_->gaussian3();
    return out;
  }

 private:
  GnssParams params_;
  SeededRng* rng_;
};

} // namespace kerteriz_sim
