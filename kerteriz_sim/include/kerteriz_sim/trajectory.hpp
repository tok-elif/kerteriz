#pragma once

/// \file
/// Analitik yorunge ureteci — PHASE0.md S9.
///
/// Ground truth KAPALI FORMDA verilir; sayisal entegrasyon YOKTUR. Boylece
/// referans, uretilen IMU'nun entegrasyonundan bagimsizdir ve S9 DoD'sinin
/// ikinci yarisi (entegre et, sapmayi olc) anlamli olur.
///
/// Cerceve konvansiyonu CONVENTIONS §1:
///   W = ENU (x=Dogu, y=Kuzey, z=Yukari),  B = REP-103 (x=ileri, y=sol, z=yukari)
///   R_WB : B'den W'ye dondurur.  v_WB, p_WB : W icinde ifade edilir.
///
/// Govde yonelimi: x_B hiz vektoruyle hizalanir (duz ucus, yalniz yaw).
/// Yaw hizi egrilik formulunden kapali formda gelir:
///   w_z = (vx*ay - vy*ax) / (vx^2 + vy^2)

#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace kerteriz_sim {

using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::Vec3;

/// CONVENTIONS §7
inline constexpr Scalar kGravity = Scalar(9.80665);

/// ENU'da yer cekimi vektoru.
inline Vec3 gravity_world() { return Vec3(Scalar(0), Scalar(0), -kGravity); }

enum class TrajectoryKind {
  kConstantVelocity, ///< sabit hiz, donme yok
  kCircle,           ///< yatay daire
  kFigureEight       ///< yatay sekiz
};

struct TrajectoryParams {
  TrajectoryKind kind = TrajectoryKind::kConstantVelocity;
  Vec3 initial_position = Vec3::Zero();
  Vec3 linear_velocity = Vec3(Scalar(1), Scalar(0), Scalar(0)); ///< yalniz kConstantVelocity
  Scalar radius = Scalar(10);                                   ///< daire / sekiz
  Scalar angular_rate = Scalar(0.2);                            ///< rad/s
  Scalar height = Scalar(0);
};

struct TrajectorySample {
  TimeNs stamp_ns = 0;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); ///< R_WB
  Vec3 position = Vec3::Zero();                           ///< p_WB, W
  Vec3 velocity = Vec3::Zero();                           ///< v_WB, W
  Vec3 acceleration = Vec3::Zero();                       ///< a_W (yer cekimi HARIC)
  Vec3 angular_velocity = Vec3::Zero();                   ///< omega_B, B
};

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(TrajectoryParams params, TimeNs start_ns = 0)
      : params_(params), start_ns_(start_ns) {}

  const TrajectoryParams& params() const { return params_; }
  TimeNs start_ns() const { return start_ns_; }

  /// Kapali formda ground truth. CONVENTIONS §6: gecen sure iki TimeNs
  /// farkindan hesaplanir, biriktirilmez.
  TrajectorySample at(TimeNs stamp_ns) const {
    const Scalar t = static_cast<Scalar>(stamp_ns - start_ns_) * Scalar(1e-9);

    TrajectorySample s;
    s.stamp_ns = stamp_ns;

    switch (params_.kind) {
    case TrajectoryKind::kConstantVelocity: {
      s.position = params_.initial_position + params_.linear_velocity * t;
      s.velocity = params_.linear_velocity;
      s.acceleration = Vec3::Zero();
      break;
    }
    case TrajectoryKind::kCircle: {
      const Scalar w = params_.angular_rate;
      const Scalar r = params_.radius;
      const Scalar c = std::cos(w * t);
      const Scalar sn = std::sin(w * t);
      s.position = params_.initial_position + Vec3(r * c, r * sn, params_.height);
      s.velocity = Vec3(-r * w * sn, r * w * c, Scalar(0));
      s.acceleration = Vec3(-r * w * w * c, -r * w * w * sn, Scalar(0));
      break;
    }
    case TrajectoryKind::kFigureEight: {
      // p = [ r sin(wt), (r/2) sin(2wt), h ]
      const Scalar w = params_.angular_rate;
      const Scalar r = params_.radius;
      const Scalar s1 = std::sin(w * t);
      const Scalar c1 = std::cos(w * t);
      const Scalar s2 = std::sin(Scalar(2) * w * t);
      const Scalar c2 = std::cos(Scalar(2) * w * t);
      s.position = params_.initial_position + Vec3(r * s1, Scalar(0.5) * r * s2, params_.height);
      s.velocity = Vec3(r * w * c1, r * w * c2, Scalar(0));
      s.acceleration = Vec3(-r * w * w * s1, Scalar(-2) * r * w * w * s2, Scalar(0));
      break;
    }
    }

    yonelimi_doldur(s);
    return s;
  }

 private:
  /// x_B hiz yonunde; yalniz yaw. Yaw hizi egrilik formulunden.
  void yonelimi_doldur(TrajectorySample& s) const {
    const Scalar vx = s.velocity.x();
    const Scalar vy = s.velocity.y();
    const Scalar hiz_kare = vx * vx + vy * vy;

    if (hiz_kare < kEnKucukHizKare) {
      s.rotation = Eigen::Matrix3d::Identity();
      s.angular_velocity = Vec3::Zero();
      return;
    }

    const Scalar yaw = std::atan2(vy, vx);
    s.rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const Scalar wz = (vx * s.acceleration.y() - vy * s.acceleration.x()) / hiz_kare;
    s.angular_velocity = Vec3(Scalar(0), Scalar(0), wz);
  }

  static constexpr Scalar kEnKucukHizKare = Scalar(1e-12);

  TrajectoryParams params_;
  TimeNs start_ns_;
};

} // namespace kerteriz_sim
