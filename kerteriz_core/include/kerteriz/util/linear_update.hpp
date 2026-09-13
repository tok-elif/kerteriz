#pragma once

/// \file
/// CONVENTIONS §4'un birebir uygulamasi — guncelleme denklemi TEK YERDE.
///
/// Faz 1'deki ESKF ve Faz 3'teki InEKF bu fonksiyonu cagirir; isaret ve Joseph
/// mantigi iki kere yazilmaz (PHASE0.md S8).
///
///   S   = J_res P J_res^T + R
///   d   = -P J_res^T S^-1 r
///   A   = I - P J_res^T S^-1 J_res
///   P+  = A P A^T + P J_res^T S^-1 R S^-1 J_res P        (Joseph)
///   P+ <- 1/2 (P+ + P+^T)                                 (simetrizasyon)
///   nis = r^T S^-1 r
///
/// ADR-17 / CONVENTIONS §4.1: S'nin acik tersi HESAPLANMAZ. LDLT ile cozulur.
/// Joseph formu ve simetrizasyon opsiyonel degil, zorunludur.
///
/// ADR-22 (tahsissiz hot path): tum ara matrisler sabit kapasitelidir ve
/// yigitta durur; heap tahsisi yoktur. Eigen'in dinamik boyutlu blok
/// carpimlarinda gecici tahsis yapabilmesine karsi her ara sonuc onceden
/// ayrilmis sabit depoya .noalias() ile yazilir.
///
/// S'nin LDLT'si her zaman tam kMaxResidualDim boyutunda kurulur; kullanilmayan
/// kosegen 1 ile doldurulur ve artigin o girdileri sifirdir. Boylece LDLT nesnesi
/// sabit boyutludur (dinamik olsaydi heap'e giderdi) ve sonuc degismez:
/// blok-kosegen yapida etkin olmayan bolum cozume katki vermez.
///
/// NOT: bu dosyadaki yorumlar yasakli sembolleri LITERAL olarak icermez
/// (kerteriz_core/CMakeLists.txt'deki kural).

#include "kerteriz/types.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <cassert>

namespace kerteriz {

/// Kazanc ve ara carpimlar icin sabit kapasiteli calisma tipi (dof x dim).
using StateResMat = Eigen::Matrix<Scalar, kMaxStateDof, kMaxResidualDim>;

struct LinearUpdateResult {
  Scalar nis;  ///< r^T S^-1 r, serbestlik derecesi = dim
  bool spd_ok; ///< S pozitif tanimli cozulebildi mi
};

/// CONVENTIONS §4 guncellemesi.
///
/// \param j_res    dim x dof, JacMat'in sol ust blogunda
/// \param r        dim, artik
/// \param r_cov    dim x dim, olcum gurultusu
/// \param p        GIRIS/CIKIS kovaryans; yalnizca sol ust dof x dof blogu yazilir
/// \param delta_out  d = -P J_res^T S^-1 r; yalnizca ilk dof girdisi yazilir
///
/// spd_ok false donerse P DEGISTIRILMEZ ve delta_out sifirlanir — bozuk bir
/// cozumle durumu kirletmemek icin.
inline LinearUpdateResult linear_update(const Eigen::Ref<const JacMat>& j_res,
                                        const Eigen::Ref<const ResVec>& r,
                                        const Eigen::Ref<const ResMat>& r_cov, int dim, int dof,
                                        Eigen::Ref<StateMat> p, Eigen::Ref<StateVec> delta_out) {
  assert(dim > 0 && dim <= kMaxResidualDim);
  assert(dof > 0 && dof <= kMaxStateDof);

  LinearUpdateResult out{Scalar(0), false};

  // --- P J^T  (dof x dim) ---------------------------------------------------
  StateResMat pjt = StateResMat::Zero();
  pjt.topLeftCorner(dof, dim).noalias() =
      p.topLeftCorner(dof, dof) * j_res.topLeftCorner(dim, dof).transpose();

  // --- S = J (P J^T) + R ----------------------------------------------------
  // Kullanilmayan bolum birim birakilir; blok-kosegen yapi cozumu degistirmez.
  ResMat s = ResMat::Identity();
  s.topLeftCorner(dim, dim).noalias() = j_res.topLeftCorner(dim, dof) * pjt.topLeftCorner(dof, dim);
  s.topLeftCorner(dim, dim) += r_cov.topLeftCorner(dim, dim);
  if (dim < kMaxResidualDim) {
    const int kalan = kMaxResidualDim - dim;
    s.topRightCorner(dim, kalan).setZero();
    s.bottomLeftCorner(kalan, dim).setZero();
  }

  const Eigen::LDLT<ResMat> ldlt(s);
  out.spd_ok = (ldlt.info() == Eigen::Success) && ldlt.isPositive();
  if (!out.spd_ok) {
    delta_out.setZero();
    return out; // P'ye dokunulmaz
  }

  // --- S^-1 r  ve  nis ------------------------------------------------------
  ResVec rp = ResVec::Zero();
  rp.head(dim) = r.head(dim);

  const ResVec sinv_r = ldlt.solve(rp);
  out.nis = rp.head(dim).dot(sinv_r.head(dim));

  // --- d = -P J^T S^-1 r ----------------------------------------------------
  // Eksi isaret ACIKCA yazilir (ADR-13). J_res yerine -J_res verilirse d'nin
  // isareti doner; denetim (3) bunu yakalar.
  delta_out.setZero();
  delta_out.head(dof).noalias() = -pjt.topLeftCorner(dof, dim) * sinv_r.head(dim);

  // --- S^-1 J P  (dim x dof).  K = P J^T S^-1 = (S^-1 J P)^T, S simetrik -----
  JacMat jp = JacMat::Zero();
  jp.topLeftCorner(dim, dof) = pjt.topLeftCorner(dof, dim).transpose();
  const JacMat sinv_jp = ldlt.solve(jp);

  // --- A = I - K J ----------------------------------------------------------
  StateMat a = StateMat::Identity();
  a.topLeftCorner(dof, dof).noalias() -=
      sinv_jp.topLeftCorner(dim, dof).transpose() * j_res.topLeftCorner(dim, dof);

  // --- Joseph:  P+ = A P A^T + K R K^T --------------------------------------
  StateMat ap = StateMat::Zero();
  ap.topLeftCorner(dof, dof).noalias() = a.topLeftCorner(dof, dof) * p.topLeftCorner(dof, dof);

  StateMat p_yeni = StateMat::Zero();
  p_yeni.topLeftCorner(dof, dof).noalias() =
      ap.topLeftCorner(dof, dof) * a.topLeftCorner(dof, dof).transpose();

  StateResMat kr = StateResMat::Zero();
  kr.topLeftCorner(dof, dim).noalias() =
      sinv_jp.topLeftCorner(dim, dof).transpose() * r_cov.topLeftCorner(dim, dim);

  p_yeni.topLeftCorner(dof, dof).noalias() +=
      kr.topLeftCorner(dof, dim) * sinv_jp.topLeftCorner(dim, dof);

  // --- Simetrizasyon (zorunlu, CONVENTIONS §4) ------------------------------
  p.topLeftCorner(dof, dof) =
      Scalar(0.5) * (p_yeni.topLeftCorner(dof, dof) + p_yeni.topLeftCorner(dof, dof).transpose());

  return out;
}

} // namespace kerteriz
