#pragma once

/// \file
/// IMU yayilimi — INTERFACES §2 · CONVENTIONS §1, §3.1, §5.1, §6, §7, §8.1.
///
/// ====================== SECILEN AYRIKLASTIRMA ================================
///
/// Belgelerde tanimli olmadigi icin acikca secildi (F1.2):
///
///   NOMINAL : birinci mertebe strapdown, rotasyon TAM
///   F       : secilen ayrik haritanin TAM Jacobian'i
///   Q       : G Qc G^T dt (birinci mertebe)
///   dt      : dt = 0 no-op; dt < 0 cagiran hatasi (ADR-5 sirasizligi tampon
///             seviyesinde geri sararak cozer, propagator'a ulasmamali)
///
/// F'in ayrik haritanin TAM turevi olmasi kasitlidir: sayisal Jacobian testi
/// boylece gevsek toleransli degil, SIKI bir esitlik testi olur.
///
/// ============================== NOMINAL ======================================
///
/// Olcum modeli (CONVENTIONS §7):  w_m = w + b_g + n_g,  a_m = a + b_a + n_a
/// Nominal debias:                 w = w_m - b_g,        a = a_m - b_a
///
///   a_W = R0 a + g_W
///   R1  = R0 Exp(w dt)
///   v1  = v0 + a_W dt
///   p1  = p0 + v0 dt + (1/2) a_W dt^2
///   b_g, b_a degismez
///
/// Duragan, duz IMU: a_W = 0 iken a_m = -g_W = [0,0,+g] okur; yayilimda
/// a_W = R a_m + g_W = 0 cikar ve hiz sabit kalir.
///
/// ======================= HATA DINAMIGI (SAG PERTURBASYON) ====================
///
/// Hata tanimi X = X^ (+) delta, CONVENTIONS §3.1. SE_2_3 uzerinde sag-plus
/// hiz ve konum pertürbasyonlarini GOVDE cercevesinde tanimlar:
///
///   R = R^ Exp(dtheta),   v = v^ + R^ dv,   p = p^ + R^ dp
///
/// Bu yuzden asagidaki bloklarin hepsinde DR^T carpani vardir; dunya
/// cercevesinde tanimlanmis bir hata icin bu carpan OLMAZDI. Sayisal Jacobian
/// testi tam olarak bu ayrimi yakalar.
///
/// DR = Exp(w dt) olmak uzere:
///
///   dtheta1 = DR^T dtheta                         - Jr(w dt) dt db_g
///   dv1     = DR^T ( dv - [a]x dt dtheta          - dt db_a )
///   dp1     = DR^T ( dp + dv dt - (1/2)[a]x dt^2 dtheta - (1/2) dt^2 db_a )
///   db_g1   = db_g
///   db_a1   = db_a
///
/// ============================ SUREC GURULTUSU ================================
///
/// Yogunluklar sigma/sqrt(Hz)'dir, yani PSD = sigma^2. dt boyunca varyans
/// katkisi sigma^2 dt olur — BIRIM KARESI alinir:
///
///   Q[dtheta] = gyro_noise_density^2  dt        rad^2
///   Q[dv]     = accel_noise_density^2 dt        m^2/s^2
///   Q[db_g]   = gyro_random_walk^2    dt        rad^2/s^2
///   Q[db_a]   = accel_random_walk^2   dt        m^2/s^4
///
/// Konum blogu birinci mertebe Q'da SIFIRDIR; konum gurultusu O(dt^2)
/// mertebesinde dogar ve bu ayriklastirmaya dahil edilmez. Q bu yuzden pozitif
/// YARI-tanimlidir.
///
/// ============================= AUGMENTATION ==================================
///
/// IMU yalnizca cekirdegi ilerletir. Augmentation durumlari nominal olarak
/// sabittir, gecis bloklari birimdir, surec gurultusu almazlar. Kovaryans
/// blok yapisi kullanilarak ilerletilir:
///
///   P_cc' = Fcc P_cc Fcc^T + Qcc
///   P_ca' = Fcc P_ca            (F_aa = I oldugu icin)
///   P_aa' = P_aa
///
/// Boylece cekirdek<->augmentation capraz kovaryansi KORUNUR ve dogru
/// donusturulur. Tam 63x63 carpim yapilmaz; hem gereksiz hem de aktif olmayan
/// kuyruga yazardi.

#include "kerteriz/state/nav_state.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <cassert>
#include <manif/SO3.h>
#include <utility>

namespace kerteriz {

struct ImuNoiseParams {
  Scalar gyro_noise_density;  ///< rad/s/sqrt(Hz)
  Scalar gyro_random_walk;    ///< rad/s^2/sqrt(Hz)
  Scalar accel_noise_density; ///< m/s^2/sqrt(Hz)
  Scalar accel_random_walk;   ///< m/s^3/sqrt(Hz)
};

class ImuPropagator {
 public:
  explicit ImuPropagator(ImuNoiseParams params, Vec3 gravity_W = {0, 0, -9.80665})
      : params_(params), gravity_w_(std::move(gravity_W)) {}

  /// dt: SANIYE cinsinden sure, iki TimeNs farkindan hesaplanmis
  /// (CONVENTIONS §6). Mutlak zamani backend tutar; propagator zaman durumu
  /// tasimaz.
  ///
  /// dt = 0 hicbir sey yapmaz: durum ve P degismez, F = I, Q = 0.
  /// dt < 0 cagiran hatasidir.
  ///
  /// F ve Q verilirse YALNIZCA sol ust active_dof() x active_dof() blogu
  /// anlamlidir; geri kalani deterministik olarak sifirdir. Cikti isaretcileri
  /// teshis icindir, sonucu DEGISTIRMEZ.
  void propagate(NavState& x, NavCovariance& P, const ImuSample& u, Scalar dt,
                 StateMat* F = nullptr, StateMat* Q = nullptr) const {
    assert(dt >= Scalar(0) && "negatif dt: sirasiz IMU ornegi propagator'a ulasti");

    const int n = x.active_dof();

    CoreMat fcc = CoreMat::Identity();
    CoreMat qcc = CoreMat::Zero();

    if (dt > Scalar(0)) {
      gecis_ve_gurultu(x, u, dt, fcc, qcc);
      kovaryansi_ilerlet(P, fcc, qcc, n);
      durumu_ilerlet(x, u, dt);
    }

    if (F != nullptr) {
      F->setZero();
      F->topLeftCorner<kCoreDof, kCoreDof>() = fcc;
      if (n > kCoreDof) {
        F->block(kCoreDof, kCoreDof, n - kCoreDof, n - kCoreDof).setIdentity();
      }
    }
    if (Q != nullptr) {
      Q->setZero();
      Q->topLeftCorner<kCoreDof, kCoreDof>() = qcc;
    }
  }

 private:
  using CoreMat = Eigen::Matrix<Scalar, kCoreDof, kCoreDof>;
  using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

  // CONVENTIONS §5.1 cekirdek teget sirasi.
  static constexpr int kTheta = 0;
  static constexpr int kVel = 3;
  static constexpr int kPos = 6;
  static constexpr int kBg = 9;
  static constexpr int kBa = 12;

  static Mat3 capraz(const Vec3& v) {
    Mat3 m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
  }

  void durumu_ilerlet(NavState& x, const ImuSample& u, Scalar dt) const {
    const Vec3 w = u.gyro - x.gyro_bias();
    const Vec3 a = u.accel - x.accel_bias();

    const Mat3 r0 = x.extended_pose().rotation();
    const Vec3 v0 = x.extended_pose().linearVelocity();
    const Vec3 p0 = x.extended_pose().translation();

    const Vec3 a_w = r0 * a + gravity_w_;
    const Mat3 r1 = r0 * manif::SO3Tangent<Scalar>(w * dt).exp().rotation();
    const Vec3 v1 = v0 + a_w * dt;
    const Vec3 p1 = p0 + v0 * dt + Scalar(0.5) * a_w * dt * dt;

    // KUATERNIYON NORMALLESTIRILIR. r1 bir matris carpimidir ve ondan
    // turetilen kuaterniyon birim normdan sapar; normallestirilmeden
    // saklanirsa `.rotation()` |q|^2 ile olceklenir ve hata her adimda
    // CARPIMSAL buyur. Duzeltme olmadan 100 Hz'de birkac bin adimda rotasyon
    // tumden bozulur (bkz. OrientationStaysOrthonormalOverLongRun).
    Eigen::Quaternion<Scalar> q1(r1);
    q1.normalize();
    x.extended_pose() = SE23(p1, q1, v1);
    // Bias'lar nominal yayilimda DEGISMEZ (CONVENTIONS §7: db = n_b).
  }

  void gecis_ve_gurultu(const NavState& x, const ImuSample& u, Scalar dt, CoreMat& fcc,
                        CoreMat& qcc) const {
    const Vec3 w = u.gyro - x.gyro_bias();
    const Vec3 a = u.accel - x.accel_bias();

    const manif::SO3Tangent<Scalar> wdt(w * dt);
    const Mat3 dr_t = wdt.exp().rotation().transpose(); // DR^T
    const Mat3 jr = wdt.rjac();                         // Jr(w dt)
    const Mat3 ax = capraz(a);

    fcc.setZero();
    fcc.block<3, 3>(kTheta, kTheta) = dr_t;
    fcc.block<3, 3>(kTheta, kBg) = -jr * dt;

    fcc.block<3, 3>(kVel, kTheta) = -dr_t * ax * dt;
    fcc.block<3, 3>(kVel, kVel) = dr_t;
    fcc.block<3, 3>(kVel, kBa) = -dr_t * dt;

    fcc.block<3, 3>(kPos, kTheta) = -Scalar(0.5) * dr_t * ax * dt * dt;
    fcc.block<3, 3>(kPos, kVel) = dr_t * dt;
    fcc.block<3, 3>(kPos, kPos) = dr_t;
    fcc.block<3, 3>(kPos, kBa) = -Scalar(0.5) * dr_t * dt * dt;

    fcc.block<3, 3>(kBg, kBg).setIdentity();
    fcc.block<3, 3>(kBa, kBa).setIdentity();

    // Q = G Qc G^T dt. Yogunluklarin KARESI alinir.
    qcc.setZero();
    const Scalar sg = params_.gyro_noise_density;
    const Scalar sa = params_.accel_noise_density;
    const Scalar sbg = params_.gyro_random_walk;
    const Scalar sba = params_.accel_random_walk;
    qcc.block<3, 3>(kTheta, kTheta).diagonal().setConstant(sg * sg * dt);
    qcc.block<3, 3>(kVel, kVel).diagonal().setConstant(sa * sa * dt);
    qcc.block<3, 3>(kBg, kBg).diagonal().setConstant(sbg * sbg * dt);
    qcc.block<3, 3>(kBa, kBa).diagonal().setConstant(sba * sba * dt);
  }

  /// Blok yapisini kullanarak ilerletir. Sabit boyutlu yerel tamponlar; heap
  /// tahsisi yoktur (CONVENTIONS §8.1).
  void kovaryansi_ilerlet(NavCovariance& P, const CoreMat& fcc, const CoreMat& qcc, int n) const {
    const int m = n - kCoreDof;

    CoreMat ara;
    ara.noalias() = fcc * P.topLeftCorner<kCoreDof, kCoreDof>();
    P.topLeftCorner<kCoreDof, kCoreDof>().noalias() = ara * fcc.transpose();
    P.topLeftCorner<kCoreDof, kCoreDof>() += qcc;

    if (m > 0) {
      Eigen::Matrix<Scalar, kCoreDof, kMaxAugmentDof> capraz_ara;
      capraz_ara.leftCols(m).noalias() = fcc * P.block(0, kCoreDof, kCoreDof, m);
      P.block(0, kCoreDof, kCoreDof, m) = capraz_ara.leftCols(m);
      P.block(kCoreDof, 0, m, kCoreDof) = capraz_ara.leftCols(m).transpose();
      // P_aa degismez.
    }

    // Simetrizasyon (CONVENTIONS §4): yuvarlama kaynakli kaymayi engeller.
    P.topLeftCorner(n, n) =
        Scalar(0.5) * (P.topLeftCorner(n, n) + P.topLeftCorner(n, n).transpose()).eval();
  }

  ImuNoiseParams params_;
  Vec3 gravity_w_;
};

} // namespace kerteriz
