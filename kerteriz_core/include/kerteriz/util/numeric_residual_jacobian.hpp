#pragma once

/// \file
/// Sayisal residual Jacobian — INTERFACES §7.
///
/// Manifold uzerinde MERKEZI FARK:  x (+) (+eps e_i)  ve  x (+) (-eps e_i).
/// Pertürbasyon lie.hpp'deki plus() uzerinden gecer, yani sag pertürbasyon
/// konvansiyonu (CONVENTIONS §3.1) burada da otomatik olarak gecerlidir.
///
/// CIKTI DOGRUDAN J_res'TIR (CONVENTIONS §3.2). residual_fn pertürbe edilmis
/// noktadaki ARTIGI dondurur; bu yuzden isaret cevrilmez, negatiflenmez.
/// Bu, "H" konvansiyonu yerine J_res secilmesinin asil sebebidir (ADR-13):
/// sayisal test ile uretim ayni nesneyi uretir, aralarinda sessiz bir isaret
/// uyusmazligi olamaz.
///
/// NOT: bu dosyadaki yorumlar yasakli sembolleri LITERAL olarak icermez
/// (kerteriz_core/CMakeLists.txt'deki kural).

#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <cassert>
#include <utility>

namespace kerteriz {

/// Merkezi farkla sayisal residual Jacobian.
///
/// \tparam State   `plus(State, tangent)` ile (+) saglayan ve `State::DoF`
///                 tanimlayan manifold tipi (bugun SE23; Faz 1'de NavState).
/// \param residual_fn  pertürbe edilmis durumu alir, DEGERLENDIRILMIS bir
///                     Eigen vektoru dondurur (ifade degil).
/// \param residual_dim artik boyutu; kMaxResidualDim'i asamaz.
/// \param eps          merkezi fark adimi.
///
/// Doner: JacMat'in sol ust (residual_dim x State::DoF) bloğu doludur,
/// gerisi sifirdir — INTERFACES §7'deki kullanim topLeftCorner ile okur.
///
/// Faz 1 notu: NavState geldiginde aktif boyut calisma aninda degisecek ve
/// active_dof alan bir asiri yukleme eklenecek. INTERFACES §7'deki imza
/// NavState'i gosterir; bu surum ayni sekli derleme zamani DoF ile verir.
template <typename State, typename ResidualFn>
JacMat numeric_residual_jacobian(const State& x, ResidualFn&& residual_fn, int residual_dim,
                                 Scalar eps = 1e-6) {
  constexpr int kDof = State::DoF;
  static_assert(kDof <= kMaxStateDof, "durum DoF kapasiteyi asiyor");

  assert(residual_dim > 0 && residual_dim <= kMaxResidualDim);
  assert(eps > Scalar(0));

  using Tangent = Eigen::Matrix<Scalar, kDof, 1>;

  JacMat j = JacMat::Zero();
  Tangent d = Tangent::Zero();

  for (int i = 0; i < kDof; ++i) {
    d.setZero();

    d[i] = eps;
    const auto r_arti = residual_fn(plus(x, d));

    d[i] = -eps;
    const auto r_eksi = residual_fn(plus(x, d));

    for (int k = 0; k < residual_dim; ++k) {
      j(k, i) = (r_arti[k] - r_eksi[k]) / (Scalar(2) * eps);
    }
  }

  return j;
}

} // namespace kerteriz
