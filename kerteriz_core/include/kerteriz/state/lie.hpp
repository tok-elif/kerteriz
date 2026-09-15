#pragma once

/// \file
/// SE_2(3) ince sarmalayici. Amaci hesap yapmak degil, KONVANSIYONU TEK YERDE
/// SABITLEMEK (PHASE0.md S6):
///
///   1. Sag pertürbasyon  — CONVENTIONS §3.1:  X (+) d = X o Exp(d)
///   2. Hamilton kuaterniyonu — CONVENTIONS §2 (manif Hamilton kullanir)
///   3. Cekirdek tegent sirasi — CONVENTIONS §5.1
///
/// UYARI — manif'in kendi tegent sirasi BIZIMKINDEN FARKLIDIR:
///
///   bizim (CONVENTIONS §5.1):  [ dtheta(0..2)  dv(3..5)  dp(6..8) ]
///   manif:                     [ lin=dp(0..2)  ang=dtheta(3..5)  lin2=dv(6..8) ]
///
/// Bu dosya donusumu tek noktada yapar. manif tipleri kerteriz_core'un geri
/// kalaninda DOGRUDAN kullanilmaz; buradaki fonksiyonlar uzerinden gecilir.
/// Aksi halde permutasyon her kullanim yerinde tekrarlanir ve er gec biri
/// unutulur.
///
/// NOT: kerteriz_core/CMakeLists.txt'deki kural burada da gecerlidir — bu
/// dosyadaki yorumlar yasakli sembolleri LITERAL olarak icermez.

#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <manif/SE_2_3.h>
#include <manif/SO3.h>

namespace kerteriz {

using SE23 = manif::SE_2_3<Scalar>;
using SE23Tangent = manif::SE_2_3Tangent<Scalar>;

inline constexpr int kSe23Dof = 9;

using TangentVec = Eigen::Matrix<Scalar, kSe23Dof, 1>;
using AdjointMat = Eigen::Matrix<Scalar, kSe23Dof, kSe23Dof>;

static_assert(SE23::DoF == kSe23Dof, "manif SE_2_3 DoF beklenenden farkli");

// --- Tegent uzay indeksleri -------------------------------------------------
// CONVENTIONS §5.1: "Tegent uzay sirasi — DEGISTIRME".
inline constexpr int kIdxTheta = 0; ///< dtheta — rotasyon
inline constexpr int kIdxVel = 3;   ///< dv     — hiz
inline constexpr int kIdxPos = 6;   ///< dp     — konum

// manif'in kendi sirasi. Yalnizca bu dosya bilir.
inline constexpr int kManifIdxPos = 0;   ///< lin()
inline constexpr int kManifIdxTheta = 3; ///< ang()
inline constexpr int kManifIdxVel = 6;   ///< lin2()

/// Bizim sira -> manif sirasi.
inline SE23Tangent to_manif(const TangentVec& d) {
  SE23Tangent t;
  t.coeffs().template segment<3>(kManifIdxTheta) = d.template segment<3>(kIdxTheta);
  t.coeffs().template segment<3>(kManifIdxVel) = d.template segment<3>(kIdxVel);
  t.coeffs().template segment<3>(kManifIdxPos) = d.template segment<3>(kIdxPos);
  return t;
}

/// manif sirasi -> bizim sira.
inline TangentVec from_manif(const SE23Tangent& t) {
  TangentVec d;
  d.template segment<3>(kIdxTheta) = t.coeffs().template segment<3>(kManifIdxTheta);
  d.template segment<3>(kIdxVel) = t.coeffs().template segment<3>(kManifIdxVel);
  d.template segment<3>(kIdxPos) = t.coeffs().template segment<3>(kManifIdxPos);
  return d;
}

/// Permutasyon matrisi P: d_manif = P * d_bizim.
/// Adjoint'i bizim siraya tasimak icin gerekir.
inline AdjointMat tangent_permutation() {
  AdjointMat p = AdjointMat::Zero();
  p.template block<3, 3>(kManifIdxTheta, kIdxTheta).setIdentity();
  p.template block<3, 3>(kManifIdxVel, kIdxVel).setIdentity();
  p.template block<3, 3>(kManifIdxPos, kIdxPos).setIdentity();
  return p;
}

/// so(3) sapka operatoru:  [v]x.  capraz(v) w == v.cross(w).
///
/// Sag pertürbasyon turetmelerinin her birinde gecer ([dtheta]x a = -[a]x dtheta
/// donusumu), o yuzden olcum basliklarinda tekrar tanimlanmaz.
inline Eigen::Matrix<Scalar, 3, 3> capraz(const Vec3& v) {
  Eigen::Matrix<Scalar, 3, 3> m;
  m << Scalar(0), -v.z(), v.y(), v.z(), Scalar(0), -v.x(), -v.y(), v.x(), Scalar(0);
  return m;
}

// --- Grup islemleri, HEPSI bizim sirada -------------------------------------

/// Exp: tegent -> grup.
inline SE23 exp_map(const TangentVec& d) { return to_manif(d).exp(); }

/// Log: grup -> tegent.
inline TangentVec log_map(const SE23& x) { return from_manif(x.log()); }

/// Sag toplama (CONVENTIONS §3.1):  X (+) d = X o Exp(d).
/// SOL DEGIL. Bu satir projedeki pertürbasyon konvansiyonunun tanimidir.
inline SE23 plus(const SE23& x, const TangentVec& d) { return x.rplus(to_manif(d)); }

/// Sag cikarma:  Y (-) X = Log(X^-1 o Y).
inline TangentVec minus(const SE23& y, const SE23& x) { return from_manif(y.rminus(x)); }

/// Sag Jacobian'in tersi, bizim tegent sirasinda.
/// Log(X o Exp(d)) ~ Log(X) + Jrinv(Log X) * d  iliskisinde gecer.
/// S7'deki sayisal Jacobian testinin analitik referansi budur; manif tipleri
/// core'un geri kalaninda dogrudan kullanilmasin diye burada sarmalanir.
inline AdjointMat right_jacobian_inverse(const TangentVec& d) {
  const AdjointMat p = tangent_permutation();
  return p.transpose() * to_manif(d).rjacinv() * p;
}

/// Adjoint, bizim tegent sirasinda:  Ad_bizim = P^T * Ad_manif * P.
inline AdjointMat adjoint(const SE23& x) {
  const AdjointMat p = tangent_permutation();
  return p.transpose() * x.adj() * p;
}

} // namespace kerteriz
