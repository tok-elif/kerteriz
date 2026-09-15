/// \file
/// F1.6 — kalan Faz 1 olcum modelleri. INTERFACES §3 · CONVENTIONS §1, §3.1.
///
/// Dort model tek binary'de sinanir: GnssVelocity, WheelVelocity,
/// NonHolonomic, ZeroVelocity.
///
/// Her model icin sinanan cekirdek: boyut/damga/ad, artik denklemi, kovaryansin
/// birebir tasinmasi, analitik Jacobian'in kapali formu, dokunulmayan sutunlarin
/// sifir kalmasi, ve analitik ile SAYISAL Jacobian'in ortusmesi.
///
/// Sayisal Jacobian projenin KENDI (+) tanimini (NavState::plus) kullanir —
/// testte ikinci bir pertürbasyon konvansiyonu yazilmaz (ADR-13).

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/measurements/gnss_velocity.hpp"
#include "kerteriz/measurements/non_holonomic.hpp"
#include "kerteriz/measurements/wheel_velocity.hpp"
#include "kerteriz/measurements/zero_velocity.hpp"
#include "measurement_jacobian_check.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <gtest/gtest.h>
#include <new>
#include <string>
#include <utility>

namespace {

using kerteriz::capraz;
using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::GnssVelocity;
using kerteriz::ImuNoiseParams;
using kerteriz::kCoreDof;
using kerteriz::kIdxPos;
using kerteriz::kIdxTheta;
using kerteriz::kIdxVel;
using kerteriz::kMaxResidualDim;
using kerteriz::kMaxStateDof;
using kerteriz::Measurement;
using kerteriz::MeasurementWorkspace;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::NonHolonomic;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::StateBundle;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;
using kerteriz::WheelVelocity;
using kerteriz::ZeroVelocity;
using kerteriz::test_support::degerlendir;
using kerteriz::test_support::govde_hizi;
using kerteriz::test_support::jacobian_hatasi;
using kerteriz::test_support::ornek_durum_nav;

constexpr Scalar kGuven = 0.997;
constexpr Scalar kJacTol = 1e-6; ///< test_gnss_position ile AYNI sinif
constexpr int kIdxGyroBias = 9;
constexpr int kIdxAccelBias = 12;

Eigen::Matrix3d kov3() {
  Eigen::Matrix3d c;
  c << 0.09, 0.01, -0.004, 0.01, 0.16, 0.002, -0.004, 0.002, 0.25;
  return c;
}

Eigen::Matrix<Scalar, 2, 2> kov2() {
  Eigen::Matrix<Scalar, 2, 2> c;
  c << 0.04, 0.006, 0.006, 0.09;
  return c;
}

/// Augmentation'li durum: kalici kalibrasyon + bir klon.
NavState genisletilmis_durum() {
  NavState x = ornek_durum_nav();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  return x;
}

/// Iki duzende sayisal Jacobian hatasi; en buyugunu dondurur ve ikisini de
/// yazdirir. Deger TESHIS icindir — gecme kriteri kJacTol'dur, cikti degil.
Scalar jac_hata_raporu(const char* ad, const Measurement& z) {
  const Scalar sade = jacobian_hatasi(z, ornek_durum_nav());
  const Scalar aug = jacobian_hatasi(z, genisletilmis_durum());
  std::printf("    %-14s sayisal Jacobian maks hata: sade %.3e   augmentation %.3e\n", ad, sade,
              aug);
  return std::max(sade, aug);
}

/// Bir olcumun dokunmamasi gereken sutunlarin gercekten sifir oldugunu
/// dogrular. `serbest` bitleri: hangi cekirdek bloklarin dolu OLMASINA izin
/// verildigi. Bias, konum ve augmentation her dort modelde de sifirdir.
void bias_konum_ve_augmentation_sifir(const MeasurementWorkspace& w, const NavState& x) {
  const int dim = w.dim;
  const int n = x.active_dof();
  ASSERT_GT(n, kCoreDof) << "test kurulumu: augmentation olmadan bu kontrol bos gecer";

  EXPECT_LT(w.J_res.block(0, kIdxPos, dim, 3).cwiseAbs().maxCoeff(), 1e-18) << "konum sutunlari";
  EXPECT_LT(w.J_res.block(0, kIdxGyroBias, dim, 3).cwiseAbs().maxCoeff(), 1e-18) << "jiro bias";
  EXPECT_LT(w.J_res.block(0, kIdxAccelBias, dim, 3).cwiseAbs().maxCoeff(), 1e-18) << "ivme bias";
  EXPECT_LT(w.J_res.block(0, kCoreDof, dim, n - kCoreDof).cwiseAbs().maxCoeff(), 1e-18)
      << "augmentation sutunlari";
  EXPECT_LT(w.J_res.block(0, n, dim, kMaxStateDof - n).cwiseAbs().maxCoeff(), 1e-18) << "kuyruk";
  EXPECT_LT(w.J_res.bottomRows(kMaxResidualDim - dim).cwiseAbs().maxCoeff(), 1e-18)
      << "kullanilmayan satirlar";
}

// =============================================================================
// GnssVelocity —  h(X) = v_WB,  r = z_W - v_WB
// =============================================================================

TEST(GnssVelocity, ReportsDimensionStampAndName) {
  const GnssVelocity z(987654321, "gnss_vel", Vec3(1, 2, 3), kov3());
  EXPECT_EQ(z.residual_dim(), 3);
  EXPECT_EQ(z.stamp_ns(), 987654321);
  EXPECT_EQ(z.name(), "gnss_vel");
  EXPECT_TRUE(z.required_clones().empty()) << "mutlak olcum klon istememeli";
}

TEST(GnssVelocity, ResidualIsMeasurementMinusWorldVelocity) {
  const NavState x = ornek_durum_nav();
  const Vec3 z_w(3.1, -0.9, 0.45);
  const auto w = degerlendir(GnssVelocity(1, "g", z_w, kov3()), x);

  EXPECT_EQ(w.dim, 3);
  EXPECT_LT((w.r.head<3>() - (z_w - x.extended_pose().linearVelocity())).cwiseAbs().maxCoeff(),
            1e-15);
}

TEST(GnssVelocity, CovarianceIsCopiedVerbatim) {
  const Eigen::Matrix3d c = kov3();
  const auto w = degerlendir(GnssVelocity(1, "g", Vec3::Zero(), c), ornek_durum_nav());
  EXPECT_LT((w.R.topLeftCorner<3, 3>() - c).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_LT(w.R.bottomRightCorner(kMaxResidualDim - 3, kMaxResidualDim - 3).cwiseAbs().maxCoeff(),
            1e-18);
}

TEST(GnssVelocity, VelocityJacobianIsNegativeRotationAndOrientationIsZero) {
  const NavState x = genisletilmis_durum();
  const auto w = degerlendir(GnssVelocity(1, "g", Vec3(3.1, -0.9, 0.45), kov3()), x);

  const Eigen::Matrix3d beklenen = -x.extended_pose().rotation();
  EXPECT_LT((w.J_res.block<3, 3>(0, kIdxVel) - beklenen).cwiseAbs().maxCoeff(), 1e-15);
  // -I yazilsaydi bu ayrim kaybolurdu; yonelim kasten birim degil.
  EXPECT_GT((beklenen + Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-2)
      << "test kurulumu: yonelim birim olmamali";

  EXPECT_LT((w.J_res.block<3, 3>(0, kIdxTheta)).cwiseAbs().maxCoeff(), 1e-18)
      << "dunya cercevesi hiz olcumu yonelim sutununa yazmamali";
  bias_konum_ve_augmentation_sifir(w, x);
}

TEST(GnssVelocity, AnalyticJacobianMatchesNumeric) {
  const GnssVelocity z(1, "g", Vec3(3.1, -0.9, 0.45), kov3());
  EXPECT_LT(jac_hata_raporu("GnssVelocity", z), kJacTol);
}

// =============================================================================
// WheelVelocity —  u = R^T v,  h(X) = u_x,  r = z - u_x
// =============================================================================

TEST(WheelVelocity, ReportsDimensionStampAndName) {
  const WheelVelocity z(42, "wheel_front", 2.5, 0.04);
  EXPECT_EQ(z.residual_dim(), 1);
  EXPECT_EQ(z.stamp_ns(), 42);
  EXPECT_EQ(z.name(), "wheel_front");
  EXPECT_TRUE(z.required_clones().empty());
}

TEST(WheelVelocity, ResidualIsForwardSpeedMinusBodyXVelocity) {
  const NavState x = ornek_durum_nav();
  const Scalar olculen = 2.5;
  const auto w = degerlendir(WheelVelocity(1, "w", olculen, 0.04), x);

  EXPECT_EQ(w.dim, 1);
  EXPECT_NEAR(w.r[0], olculen - govde_hizi(x).x(), 1e-15);
  EXPECT_NEAR(w.R(0, 0), 0.04, 1e-18);
}

TEST(WheelVelocity, JacobianMatchesClosedFormAndCrossTermIsNonZero) {
  const NavState x = genisletilmis_durum();
  const Vec3 u = govde_hizi(x);
  const auto w = degerlendir(WheelVelocity(1, "w", 2.5, 0.04), x);

  const Eigen::Matrix<Scalar, 1, 3> beklenen_theta = -capraz(u).row(0);
  const Eigen::Matrix<Scalar, 1, 3> beklenen_v = -Eigen::Matrix<Scalar, 1, 3>::Unit(0);

  EXPECT_LT((w.J_res.block<1, 3>(0, kIdxTheta) - beklenen_theta).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT((w.J_res.block<1, 3>(0, kIdxVel) - beklenen_v).cwiseAbs().maxCoeff(), 1e-15);

  // Capraz terim GERCEKTEN dolu: u'nun y/z bileseni sifir olsaydi bu test
  // yonelim sutunu tamamen unutulsa da gecerdi.
  EXPECT_GT(beklenen_theta.cwiseAbs().maxCoeff(), 1e-2)
      << "test kurulumu: u_y/u_z sifira cok yakin";
  bias_konum_ve_augmentation_sifir(w, x);
}

TEST(WheelVelocity, AnalyticJacobianMatchesNumeric) {
  const WheelVelocity z(1, "w", 2.5, 0.04);
  EXPECT_LT(jac_hata_raporu("WheelVelocity", z), kJacTol);
}

// =============================================================================
// NonHolonomic —  A = [e_y; e_z],  h(X) = A u,  z = 0,  r = -A u
// =============================================================================

TEST(NonHolonomic, ReportsDimensionStampAndName) {
  const NonHolonomic z(7, "nhc", kov2());
  EXPECT_EQ(z.residual_dim(), 2);
  EXPECT_EQ(z.stamp_ns(), 7);
  EXPECT_EQ(z.name(), "nhc");
  EXPECT_TRUE(z.required_clones().empty());
}

TEST(NonHolonomic, ResidualIsNegativeLateralAndVerticalBodyVelocity) {
  const NavState x = ornek_durum_nav();
  const Vec3 u = govde_hizi(x);
  const auto w = degerlendir(NonHolonomic(1, "n", kov2()), x);

  EXPECT_EQ(w.dim, 2);
  EXPECT_NEAR(w.r[0], -u.y(), 1e-15);
  EXPECT_NEAR(w.r[1], -u.z(), 1e-15);
  EXPECT_LT((w.R.topLeftCorner<2, 2>() - kov2()).cwiseAbs().maxCoeff(), 1e-18);
}

TEST(NonHolonomic, JacobianMatchesClosedForm) {
  const NavState x = genisletilmis_durum();
  const Vec3 u = govde_hizi(x);
  const auto w = degerlendir(NonHolonomic(1, "n", kov2()), x);

  const Eigen::Matrix<Scalar, 2, 3> beklenen_theta = -capraz(u).bottomRows<2>();
  const Eigen::Matrix<Scalar, 2, 3> beklenen_v =
      -Eigen::Matrix<Scalar, 3, 3>::Identity().bottomRows<2>();

  EXPECT_LT((w.J_res.block<2, 3>(0, kIdxTheta) - beklenen_theta).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT((w.J_res.block<2, 3>(0, kIdxVel) - beklenen_v).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_GT(beklenen_theta.cwiseAbs().maxCoeff(), 1e-2)
      << "test kurulumu: capraz terim sifira yakin";
  bias_konum_ve_augmentation_sifir(w, x);
}

TEST(NonHolonomic, AnalyticJacobianMatchesNumeric) {
  const NonHolonomic z(1, "n", kov2());
  EXPECT_LT(jac_hata_raporu("NonHolonomic", z), kJacTol);
}

// =============================================================================
// ZeroVelocity —  h(X) = v_WB,  z = 0,  r = -v_WB
// =============================================================================

TEST(ZeroVelocity, ReportsDimensionStampAndName) {
  const ZeroVelocity z(555, "zupt", kov3());
  EXPECT_EQ(z.residual_dim(), 3);
  EXPECT_EQ(z.stamp_ns(), 555);
  EXPECT_EQ(z.name(), "zupt");
  EXPECT_TRUE(z.required_clones().empty());
}

TEST(ZeroVelocity, ResidualIsNegativeWorldVelocity) {
  const NavState x = ornek_durum_nav();
  const auto w = degerlendir(ZeroVelocity(1, "z", kov3()), x);

  EXPECT_EQ(w.dim, 3);
  EXPECT_LT((w.r.head<3>() + x.extended_pose().linearVelocity()).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT((w.R.topLeftCorner<3, 3>() - kov3()).cwiseAbs().maxCoeff(), 1e-18);
}

TEST(ZeroVelocity, VelocityJacobianIsNegativeRotationAndBiasColumnsStayZero) {
  const NavState x = genisletilmis_durum();
  const auto w = degerlendir(ZeroVelocity(1, "z", kov3()), x);

  const Eigen::Matrix3d beklenen = -x.extended_pose().rotation();
  EXPECT_LT((w.J_res.block<3, 3>(0, kIdxVel) - beklenen).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_GT((beklenen + Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-2)
      << "test kurulumu: yonelim birim olmamali";
  EXPECT_LT((w.J_res.block<3, 3>(0, kIdxTheta)).cwiseAbs().maxCoeff(), 1e-18);

  // ZUPT dogrudan bias olcumu DEGILDIR: bias sutunlari elle yazilmaz.
  bias_konum_ve_augmentation_sifir(w, x);
}

TEST(ZeroVelocity, AnalyticJacobianMatchesNumeric) {
  const ZeroVelocity z(1, "z", kov3());
  EXPECT_LT(jac_hata_raporu("ZeroVelocity", z), kJacTol);
}

TEST(ZeroVelocity, DiffersFromGnssVelocityOnlyByMeasuredValue) {
  // Iki modelin Jacobian'i ayni kapali formdadir; ayrim ARTIKTADIR.
  // Yanlislikla biri digerinin artigini yazsaydi bu test duserdi.
  const NavState x = ornek_durum_nav();
  const Vec3 z_w(3.1, -0.9, 0.45);

  const auto zupt = degerlendir(ZeroVelocity(1, "z", kov3()), x);
  const auto gnss = degerlendir(GnssVelocity(1, "g", z_w, kov3()), x);

  EXPECT_LT((zupt.J_res - gnss.J_res).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_LT((gnss.r.head<3>() - zupt.r.head<3>() - z_w).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_GT((gnss.r.head<3>() - zupt.r.head<3>()).norm(), 1e-3)
      << "test kurulumu: z_W sifir olmamali";
}

// =============================================================================
// Backend entegrasyonu — GENERIC Measurement yolu
//
// Backend'de sensore ozel dal YOKTUR; dordu de ayni update() yolundan gecer.
// Burada backend invariant'lari model basina TEKRARLANMAZ; yalnizca dort
// modelin bu yoldan dogru dof ile gectigi, duzeltmenin dogru yonde oldugu ve
// acik bir aykiri degerin reddedildigi dogrulanir.
// =============================================================================

EskfConfig config(const NavState& x, Scalar p_diag = 1.0) {
  NavCovariance p = NavCovariance::Zero();
  const int n = x.active_dof();
  p.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * p_diag;
  return EskfConfig{x, p, 1000, ImuNoiseParams{1e-3, 1e-5, 2e-2, 3e-4}, kGuven};
}

TEST(Phase1Backend, AllFourRunThroughGenericPathWithCorrectDof) {
  const NavState x = ornek_durum_nav();
  const Vec3 v_w = x.extended_pose().linearVelocity();
  const Vec3 u = govde_hizi(x);

  // Hepsi mevcut duruma YAKIN: kapiyi rahat gecmeliler.
  const GnssVelocity gv(2000, "gnss_velocity", v_w + Vec3(0.05, -0.03, 0.02), kov3());
  const WheelVelocity wv(2000, "wheel_velocity", u.x() + 0.05, 0.04);
  const NonHolonomic nhc(2000, "non_holonomic", kov2());
  const ZeroVelocity zupt(2000, "zero_velocity", Eigen::Matrix3d::Identity() * 4.0);

  const std::pair<const Measurement*, int> olcumler[] = {{&gv, 3}, {&wv, 1}, {&nhc, 2}, {&zupt, 3}};

  for (const auto& [z, beklenen_dof] : olcumler) {
    SCOPED_TRACE(std::string(z->name()));
    EskfBackend backend(config(x, 1.0));
    kerteriz::FilterBackend& soyut = backend; // somut tip uzerinden dal YOK
    const auto sonuc = soyut.update(*z);

    EXPECT_EQ(sonuc.status, UpdateStatus::kAccepted);
    EXPECT_EQ(sonuc.dof, beklenen_dof);
    ASSERT_TRUE(sonuc.nis.has_value());
    EXPECT_LT(*sonuc.nis, sonuc.threshold);
    EXPECT_NEAR(sonuc.threshold, kerteriz::chi_square_quantile(kGuven, beklenen_dof), 1e-12);
  }
}

TEST(Phase1Backend, CorrectionMovesVelocityTowardTheMeasurement) {
  const NavState x = ornek_durum_nav();
  const Vec3 v0 = x.extended_pose().linearVelocity();

  // GnssVelocity: cozum olculen hiza YAKLASMALI.
  {
    const Vec3 z_w = v0 + Vec3(0.4, -0.25, 0.15);
    EskfBackend backend(config(x, 1.0));
    ASSERT_EQ(
        backend.update(GnssVelocity(2000, "gv", z_w, Eigen::Matrix3d::Identity() * 0.09)).status,
        UpdateStatus::kAccepted);
    EXPECT_LT((backend.state().extended_pose().linearVelocity() - z_w).norm(), (v0 - z_w).norm())
        << "duzeltme olcumden UZAKLASTI — artik isareti ters";
  }
  // ZeroVelocity: hiz SIFIRA yaklasmali.
  {
    EskfBackend backend(config(x, 1.0));
    ASSERT_EQ(backend.update(ZeroVelocity(2000, "z", Eigen::Matrix3d::Identity() * 0.01)).status,
              UpdateStatus::kAccepted);
    EXPECT_LT(backend.state().extended_pose().linearVelocity().norm(), v0.norm())
        << "ZUPT hizi buyuttu";
  }
  // NonHolonomic: yanal/dikey govde hizi kuculmeli.
  {
    EskfBackend backend(config(x, 1.0));
    const Vec3 u0 = govde_hizi(x);
    ASSERT_EQ(backend.update(NonHolonomic(2000, "n", kov2())).status, UpdateStatus::kAccepted);
    const Vec3 u1 = govde_hizi(backend.state());
    EXPECT_LT(u1.tail<2>().norm(), u0.tail<2>().norm()) << "yanal kayma azalmadi";
  }
  // WheelVelocity: ileri govde hizi olculene yaklasmali.
  {
    EskfBackend backend(config(x, 1.0));
    const Scalar u0x = govde_hizi(x).x();
    const Scalar olculen = u0x + 0.6;
    ASSERT_EQ(backend.update(WheelVelocity(2000, "w", olculen, 0.01)).status,
              UpdateStatus::kAccepted);
    EXPECT_LT(std::abs(govde_hizi(backend.state()).x() - olculen), std::abs(u0x - olculen));
  }
}

TEST(Phase1Backend, ObviousOutlierIsChiSquareRejectedAndChangesNothing) {
  NavState x = ornek_durum_nav();
  x.register_calibration("wheel_scale", 1);
  EskfBackend backend(config(x, 0.25));

  const NavState durum_once = backend.state();
  const NavCovariance p_once = backend.covariance();

  // 400 m/s sapma, 0.3 m/s std: NIS esigin cok uzerinde.
  const Vec3 z_w = x.extended_pose().linearVelocity() + Vec3(400.0, -250.0, 90.0);
  const auto sonuc = backend.update(GnssVelocity(2000, "gv", z_w, kov3()));

  ASSERT_EQ(sonuc.status, UpdateStatus::kChiSquareRejected);
  ASSERT_TRUE(sonuc.nis.has_value());
  EXPECT_GT(*sonuc.nis, sonuc.threshold);
  EXPECT_EQ(sonuc.dof, 3);

  EXPECT_LT(backend.state().minus(durum_once).norm(), 1e-18)
      << "reddedilen update durumu degistirdi";
  EXPECT_LT((backend.covariance() - p_once).cwiseAbs().maxCoeff(), 1e-18)
      << "reddedilen update kovaryansi degistirdi";
}

} // namespace

// -----------------------------------------------------------------------------
// Tahsis yasagi — CONVENTIONS §8.1 / ADR-22
//
// operator new GLOBAL scope'ta olmak ZORUNDA. Eigen dinamik bellegi operator
// new ile DEGIL malloc ile alir; iki kapi birlikte kullanilir ve ikisinin de
// gercekten bagli oldugu ayrica dogrulanir.
// -----------------------------------------------------------------------------

namespace {
bool tahsis_sayimi_acik = false;
int tahsis_adedi = 0;

struct TahsisKapsami {
  TahsisKapsami() {
    tahsis_adedi = 0;
    tahsis_sayimi_acik = true;
  }
  ~TahsisKapsami() { tahsis_sayimi_acik = false; }
};
} // namespace

void* operator new(std::size_t n) {
  if (tahsis_sayimi_acik) {
    ++tahsis_adedi;
  }
  void* p = std::malloc(n);
  if (p == nullptr) {
    throw std::bad_alloc();
  }
  return p;
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

namespace {

TEST(Phase1Measurements, AllocationCounterIsActuallyWired) {
  int gorulen = 0;
  {
    const TahsisKapsami kapsam;
    void* ham = ::operator new(64); // dogrudan cagri: derleyici eleyemez
    gorulen = tahsis_adedi;
    ::operator delete(ham);
  }
  EXPECT_GT(gorulen, 0) << "operator new degistirilmemis";
}

TEST(EigenAllocGuardDeathTest, GuardIsWiredInThisBinary) {
  EXPECT_DEATH(
      {
        Eigen::internal::set_is_malloc_allowed(false);
        Eigen::MatrixXd m(64, 64);
        m.setZero();
      },
      "");
}

TEST(Phase1Measurements, NoEvaluatePathAllocates) {
  const NavState x = genisletilmis_durum();
  const StateBundle bundle(x);

  const GnssVelocity gv(2000, "gv", Vec3(3.1, -0.9, 0.45), kov3());
  const WheelVelocity wv(2000, "wv", 2.5, 0.04);
  const NonHolonomic nhc(2000, "nhc", kov2());
  const ZeroVelocity zupt(2000, "zupt", kov3());
  const Measurement* olcumler[] = {&gv, &wv, &nhc, &zupt};

  MeasurementWorkspace w;
  for (const Measurement* z : olcumler) { // isinma
    w.clear();
    z->evaluate(bundle, w);
  }

  {
    const TahsisKapsami kapsam;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < 200; ++i) {
      for (const Measurement* z : olcumler) {
        w.clear();
        z->evaluate(bundle, w);
      }
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_EQ(tahsis_adedi, 0) << "evaluate() sicak yolda heap'e gitti";
}

} // namespace
