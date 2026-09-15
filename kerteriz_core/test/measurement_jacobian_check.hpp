#pragma once

/// \file
/// Olcum testleri icin ORTAK sayisal Jacobian araci — INTERFACES §7.
///
/// Iki ayri binary kullanir: `test_phase1_measurements` (model testleri) ve
/// `test_measurement_registry` (Denetim (4) govdeleri). Ikisinin ayni araci
/// kullanmasi kasitlidir; ayri kopyalar zamanla ayrisir ve denetim govdesi
/// model testinden daha zayif hale gelirdi.
///
/// PERTÜRBASYON KAYNAGI TEKTIR: `NavState::plus`. Testte ayri bir (+) tanimi
/// YAZILMAZ. Aksi halde uretim ile testin konvansiyonu sessizce ayrisabilir ve
/// karsilastirma kendi kendini dogrulayan bir tekrara donusurdu (ADR-13).

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/nav_state.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>

namespace kerteriz::test_support {

/// Backend'in yaptigini birebir tekrarlar: temizle, sonra doldur.
inline MeasurementWorkspace degerlendir(const Measurement& z, const NavState& x) {
  MeasurementWorkspace w;
  w.clear();
  const StateBundle bundle(x);
  z.evaluate(bundle, w);
  return w;
}

/// Analitik ile merkezi-fark Jacobian'i arasindaki maksimum mutlak fark.
/// Aktif duzenin TAMAMI taranir — augmentation sutunlari da dahildir.
inline Scalar jacobian_hatasi(const Measurement& z, const NavState& x0, Scalar eps = 1e-7) {
  const int dim = z.residual_dim();
  const int n = x0.active_dof();
  const auto w0 = degerlendir(z, x0);

  Eigen::MatrixXd sayisal(dim, n);
  StateVec d = StateVec::Zero();
  for (int i = 0; i < n; ++i) {
    d.setZero();
    d[i] = eps;
    const auto arti = degerlendir(z, x0.plus(d));
    d[i] = -eps;
    const auto eksi = degerlendir(z, x0.plus(d));
    sayisal.col(i) = (arti.r.head(dim) - eksi.r.head(dim)) / (Scalar(2) * eps);
  }
  return (w0.J_res.block(0, 0, dim, n) - sayisal).cwiseAbs().maxCoeff();
}

/// Sinama durumu: yonelim BIRIM DEGIL, hiz govde cercevesinde UC BILESENDE DE
/// sifirdan farkli. Ikisi de kasitli:
///   - birim olmayan R, -R ile -I'yi ayirt eder (GnssVelocity, ZeroVelocity)
///   - u'nun y/z bileseni sifirdan farkli oldugunda [u]x capraz terimi
///     gercekten dolar (WheelVelocity, NonHolonomic)
inline NavState ornek_durum_nav() {
  NavState x;
  const Eigen::Quaternion<Scalar> q(
      Eigen::AngleAxis<Scalar>(0.73, Vec3(0.3, -0.5, 0.81).normalized()));
  x.extended_pose() = SE23(Vec3(12.0, -7.0, 3.5), q, Vec3(2.4, -1.3, 0.85));
  x.gyro_bias() = Vec3(0.013, -0.021, 0.009);
  x.accel_bias() = Vec3(-0.07, 0.11, 0.04);
  return x;
}

/// ornek_durum_nav()'un govde hizi.
inline Vec3 govde_hizi(const NavState& x) {
  return x.extended_pose().rotation().transpose() * x.extended_pose().linearVelocity();
}

} // namespace kerteriz::test_support
