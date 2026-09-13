#pragma once

/// \file
/// IMU sentezleyici — PHASE0.md S9.
///
/// CONVENTIONS §7'deki olcum modeli:
///   w_m = w + b_g + n_g      db_g = n_bg
///   a_m = a + b_a + n_a      db_a = n_ba
///
/// Ivmeolcerin gordugu OZGUL KUVVETTIR, ivme degil:
///   a = R_WB^T (a_W - g_W)
/// Yer cekimi ENU'da [0, 0, -g]'dir (CONVENTIONS §7), yani durgun bir govde
/// +z yonunde +g olcer.
///
/// Ayrik gurultu olceklemesi:
///   beyaz gurultu   : sigma / sqrt(dt)
///   bias rastgele yuruyus : sigma_b * sqrt(dt)

#include "kerteriz/types.hpp"
#include "kerteriz_sim/rng.hpp"
#include "kerteriz_sim/trajectory.hpp"

#include <cmath>

namespace kerteriz_sim {

using kerteriz::ImuSample;
using kerteriz::TimeNs;

struct ImuParams {
  Scalar gyro_noise_density = Scalar(0);  ///< rad/s/sqrt(Hz)
  Scalar accel_noise_density = Scalar(0); ///< m/s^2/sqrt(Hz)
  Scalar gyro_bias_walk = Scalar(0);      ///< rad/s/sqrt(s)
  Scalar accel_bias_walk = Scalar(0);     ///< m/s^2/sqrt(s)
  Vec3 initial_gyro_bias = Vec3::Zero();
  Vec3 initial_accel_bias = Vec3::Zero();
};

class ImuSynthesizer {
 public:
  ImuSynthesizer(ImuParams params, SeededRng& rng)
      : params_(params), rng_(&rng), gyro_bias_(params.initial_gyro_bias),
        accel_bias_(params.initial_accel_bias) {}

  /// Bias'lari baslangic degerine dondurur. RNG'ye DOKUNMAZ — tohum sahipligi
  /// cagirana aittir, boylece birden fazla sentezleyici tek akisi paylasabilir.
  void reset_bias() {
    gyro_bias_ = params_.initial_gyro_bias;
    accel_bias_ = params_.initial_accel_bias;
  }

  const Vec3& gyro_bias() const { return gyro_bias_; }
  const Vec3& accel_bias() const { return accel_bias_; }

  /// Ground truth ornegi ve onceki ornekten bu yana gecen sureyle olcum uretir.
  /// Bias once dt kadar ilerletilir, sonra olcum olusturulur.
  ImuSample sample(const TrajectorySample& gt, Scalar dt) {
    if (dt > Scalar(0)) {
      const Scalar kok_dt = std::sqrt(dt);
      gyro_bias_ += params_.gyro_bias_walk * kok_dt * rng_->gaussian3();
      accel_bias_ += params_.accel_bias_walk * kok_dt * rng_->gaussian3();
    }

    const Scalar gurultu_olcek = (dt > Scalar(0)) ? Scalar(1) / std::sqrt(dt) : Scalar(0);

    // Ozgul kuvvet, govde cercevesinde.
    const Vec3 ozgul_kuvvet = gt.rotation.transpose() * (gt.acceleration - gravity_world());

    ImuSample out;
    out.stamp_ns = gt.stamp_ns;
    out.gyro = gt.angular_velocity + gyro_bias_ +
               params_.gyro_noise_density * gurultu_olcek * rng_->gaussian3();
    out.accel = ozgul_kuvvet + accel_bias_ +
                params_.accel_noise_density * gurultu_olcek * rng_->gaussian3();
    return out;
  }

 private:
  ImuParams params_;
  SeededRng* rng_;
  Vec3 gyro_bias_;
  Vec3 accel_bias_;
};

} // namespace kerteriz_sim
