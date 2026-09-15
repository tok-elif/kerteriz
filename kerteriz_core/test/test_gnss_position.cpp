/// \file
/// F1.4 — GnssPosition. INTERFACES §3 · CONVENTIONS §3.1.

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/measurements/gnss_position.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>
#include <gtest/gtest.h>
#include <new>

namespace {

using kerteriz::CloneId;
using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::exp_map;
using kerteriz::GnssPosition;
using kerteriz::ImuNoiseParams;
using kerteriz::kCoreDof;
using kerteriz::kIdxPos;
using kerteriz::kIdxTheta;
using kerteriz::kIdxVel;
using kerteriz::kMaxStateDof;
using kerteriz::Measurement;
using kerteriz::MeasurementWorkspace;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::StateBundle;
using kerteriz::StateVec;
using kerteriz::TangentVec;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;

constexpr Scalar kGuven = 0.997;

Eigen::Matrix3d capraz(const Vec3& v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

SE23 poz(const Eigen::Quaterniond& q, const Vec3& p, const Vec3& v = Vec3::Zero()) {
  return SE23(p, q, v);
}

NavState ornek_durum() {
  NavState x;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.73, Vec3(0.3, -0.5, 0.81).normalized()));
  x.extended_pose() = poz(q, Vec3(12.0, -7.0, 3.5), Vec3(1.1, 0.4, -0.2));
  x.gyro_bias() = Vec3(0.013, -0.021, 0.009);
  x.accel_bias() = Vec3(-0.07, 0.11, 0.04);
  return x;
}

Eigen::Matrix3d ornek_kovaryans() {
  Eigen::Matrix3d c;
  c << 2.25, 0.30, -0.10, 0.30, 2.56, 0.05, -0.10, 0.05, 9.00; // yatay 1.5, dikey 3.0
  return c;
}

/// Backend'in yaptigini birebir tekrarlar: temizle, sonra doldur.
MeasurementWorkspace degerlendir(const Measurement& z, const NavState& x) {
  MeasurementWorkspace w;
  w.clear();
  const StateBundle bundle(x);
  z.evaluate(bundle, w);
  return w;
}

// -----------------------------------------------------------------------------
// Temel sozlesme
// -----------------------------------------------------------------------------

TEST(GnssPosition, ReportsDimensionStampAndName) {
  const GnssPosition z(123456789, "gnss_main", Vec3(1.0, 2.0, 3.0), ornek_kovaryans());
  EXPECT_EQ(z.residual_dim(), 3);
  EXPECT_EQ(z.stamp_ns(), 123456789);
  EXPECT_EQ(z.name(), "gnss_main");
  EXPECT_TRUE(z.required_clones().empty()) << "mutlak olcum klon istememeli";
}

TEST(GnssPosition, ResidualIsMeasurementMinusPredictedAntennaPosition) {
  const NavState x = ornek_durum();
  const Vec3 lever(0.10, -0.25, 0.35);
  const Vec3 z_w(20.0, -3.0, 5.0);

  const auto w = degerlendir(GnssPosition(1, "g", z_w, ornek_kovaryans(), lever), x);

  const Vec3 beklenen =
      z_w - (x.extended_pose().translation() + x.extended_pose().rotation() * lever);
  EXPECT_EQ(w.dim, 3);
  EXPECT_LT(((w.r.head<3>()) - beklenen).cwiseAbs().maxCoeff(), 1e-15);
}

TEST(GnssPosition, ZeroLeverArmObservesBodyOriginDirectly) {
  const NavState x = ornek_durum();
  const Vec3 z_w(20.0, -3.0, 5.0);
  const auto w = degerlendir(GnssPosition(1, "g", z_w, ornek_kovaryans()), x);
  EXPECT_LT(((w.r.head<3>()) - (z_w - x.extended_pose().translation())).cwiseAbs().maxCoeff(),
            1e-15);
}

TEST(GnssPosition, CovarianceIsCopiedVerbatim) {
  const Eigen::Matrix3d c = ornek_kovaryans();
  const auto w = degerlendir(GnssPosition(1, "g", Vec3::Zero(), c), ornek_durum());

  EXPECT_LT(((w.R.topLeftCorner<3, 3>()) - c).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "kovaryans birebir tasinmadi";
  // Dis blok temiz kalmali.
  EXPECT_LT(w.R.bottomRightCorner(kerteriz::kMaxResidualDim - 3, kerteriz::kMaxResidualDim - 3)
                .cwiseAbs()
                .maxCoeff(),
            1e-18);
}

// -----------------------------------------------------------------------------
// Analitik Jacobian
// -----------------------------------------------------------------------------

TEST(GnssPosition, PositionJacobianIsNegativeRotation) {
  const NavState x = ornek_durum();
  const auto w = degerlendir(GnssPosition(1, "g", Vec3(5.0, 5.0, 5.0), ornek_kovaryans()), x);

  const Eigen::Matrix3d beklenen = -x.extended_pose().rotation();
  EXPECT_LT(((w.J_res.block<3, 3>(0, kIdxPos)) - beklenen).cwiseAbs().maxCoeff(), 1e-15);
  // Birim olmadigini da dogrula: -I yazilsaydi bu test duserdi.
  EXPECT_GT((beklenen + Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff(), 1e-2)
      << "test kurulumu: yonelim birim olmamali";
}

TEST(GnssPosition, OrientationJacobianIsExactlyZeroWithoutLeverArm) {
  const auto w =
      degerlendir(GnssPosition(1, "g", Vec3(5.0, 5.0, 5.0), ornek_kovaryans()), ornek_durum());
  EXPECT_LT((w.J_res.block<3, 3>(0, kIdxTheta)).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "lever-arm yokken GNSS konumu yonelim bilgisi tasimamali";
}

TEST(GnssPosition, OrientationJacobianMatchesAnalyticFormWithLeverArm) {
  const NavState x = ornek_durum();
  const Vec3 lever(0.10, -0.25, 0.35);
  const auto w =
      degerlendir(GnssPosition(1, "g", Vec3(5.0, 5.0, 5.0), ornek_kovaryans(), lever), x);

  const Eigen::Matrix3d beklenen = x.extended_pose().rotation() * capraz(lever);
  EXPECT_LT(((w.J_res.block<3, 3>(0, kIdxTheta)) - beklenen).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_GT(beklenen.cwiseAbs().maxCoeff(), 1e-3) << "test kurulumu: blok sifirdan farkli olmali";
}

TEST(GnssPosition, VelocityBiasAndAugmentationColumnsAreZero) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  const int n = x.active_dof();

  const auto w = degerlendir(
      GnssPosition(1, "g", Vec3(5.0, 5.0, 5.0), ornek_kovaryans(), Vec3(0.1, 0.2, 0.3)), x);

  EXPECT_LT((w.J_res.block<3, 3>(0, kIdxVel)).cwiseAbs().maxCoeff(), 1e-18) << "hiz sutunu";
  EXPECT_LT((w.J_res.block<3, 3>(0, 9)).cwiseAbs().maxCoeff(), 1e-18) << "jiro bias sutunu";
  EXPECT_LT((w.J_res.block<3, 3>(0, 12)).cwiseAbs().maxCoeff(), 1e-18) << "ivme bias sutunu";
  EXPECT_LT(w.J_res.block(0, kCoreDof, 3, n - kCoreDof).cwiseAbs().maxCoeff(), 1e-18)
      << "augmentation sutunlari";
  EXPECT_LT(w.J_res.block(0, n, 3, kMaxStateDof - n).cwiseAbs().maxCoeff(), 1e-18) << "kuyruk";
}

// -----------------------------------------------------------------------------
// Sayisal Jacobian — projenin KENDI right-plus konvansiyonu kullanilir
// -----------------------------------------------------------------------------

Scalar jacobian_hatasi(const NavState& x0, const GnssPosition& z, Scalar eps = 1e-7) {
  const int n = x0.active_dof();
  const auto w0 = degerlendir(z, x0);

  Eigen::MatrixXd sayisal(3, n);
  StateVec d = StateVec::Zero();
  for (int i = 0; i < n; ++i) {
    d.setZero();
    d[i] = eps;
    const auto arti = degerlendir(z, x0.plus(d));
    d[i] = -eps;
    const auto eksi = degerlendir(z, x0.plus(d));
    sayisal.col(i) = (arti.r.head<3>() - eksi.r.head<3>()) / (2.0 * eps);
  }
  return (w0.J_res.block(0, 0, 3, n) - sayisal).cwiseAbs().maxCoeff();
}

TEST(GnssPosition, AnalyticJacobianMatchesNumericWithoutLeverArm) {
  const GnssPosition z(1, "g", Vec3(20.0, -3.0, 5.0), ornek_kovaryans());
  EXPECT_LT(jacobian_hatasi(ornek_durum(), z), 1e-6);
}

TEST(GnssPosition, AnalyticJacobianMatchesNumericWithLeverArm) {
  const GnssPosition z(1, "g", Vec3(20.0, -3.0, 5.0), ornek_kovaryans(), Vec3(0.10, -0.25, 0.35));
  EXPECT_LT(jacobian_hatasi(ornek_durum(), z), 1e-6);
}

TEST(GnssPosition, AnalyticJacobianMatchesNumericAcrossRandomOrientations) {
  const Vec3 lever(0.42, 0.17, -0.63);
  Scalar en_buyuk = 0.0;
  for (int i = 0; i < 12; ++i) {
    const Scalar a = 0.37 * Scalar(i + 1);
    const Eigen::Quaterniond q(Eigen::AngleAxisd(
        a, Vec3(std::sin(a), std::cos(2 * a), std::sin(3 * a) + 0.5).normalized()));
    NavState x;
    x.extended_pose() = poz(q, Vec3(3.0 * a, -2.0 * a, 1.5 * a), Vec3(0.5, -0.3, 0.2));
    x.gyro_bias() = Vec3(0.01, -0.02, 0.03);
    x.accel_bias() = Vec3(-0.05, 0.06, -0.07);

    const GnssPosition z(1, "g", Vec3(20.0, -3.0, 5.0), ornek_kovaryans(), lever);
    en_buyuk = std::max(en_buyuk, jacobian_hatasi(x, z));
  }
  EXPECT_LT(en_buyuk, 1e-6) << "maks sayisal Jacobian hatasi";
}

TEST(GnssPosition, AnalyticJacobianMatchesNumericWithAugmentation) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();
  const GnssPosition z(1, "g", Vec3(20.0, -3.0, 5.0), ornek_kovaryans(), Vec3(0.1, -0.2, 0.3));
  EXPECT_LT(jacobian_hatasi(x, z), 1e-6);
}

// -----------------------------------------------------------------------------
// Backend entegrasyonu — GENERIC Measurement yolu
//
// Backend'de GNSS'e ozel kod YOKTUR; asagidaki her sey ayni update() yolundan
// gecer.
// -----------------------------------------------------------------------------

EskfConfig config(const NavState& x, Scalar p_diag = 1.0) {
  NavCovariance p = NavCovariance::Zero();
  const int n = x.active_dof();
  p.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * p_diag;
  return EskfConfig{x, p, 1000, ImuNoiseParams{1e-3, 1e-5, 2e-2, 3e-4}, kGuven};
}

TEST(GnssPosition, NearbyMeasurementIsAcceptedAndPullsPositionTowardIt) {
  NavState x = ornek_durum();
  const Vec3 gercek_p = x.extended_pose().translation();
  EskfBackend backend(config(x, 1.0));

  // Gercek konumdan yarim metre sapmis bir olcum: kapiyi rahat gecer.
  const Vec3 z_w = gercek_p + Vec3(0.5, -0.3, 0.2);
  const GnssPosition z(2000, "gnss_main", z_w, Eigen::Matrix3d::Identity() * 2.25);

  const auto sonuc = backend.update(z);

  ASSERT_EQ(sonuc.status, UpdateStatus::kAccepted);
  ASSERT_TRUE(sonuc.nis.has_value());
  EXPECT_EQ(sonuc.dof, 3);
  EXPECT_LT(*sonuc.nis, sonuc.threshold);
  EXPECT_NEAR(sonuc.threshold, kerteriz::chi_square_quantile(kGuven, 3), 1e-12);

  // Duzeltme olcume DOGRU olmali. Artik isareti cevrilirse ters yone gider.
  const Vec3 yeni_p = backend.state().extended_pose().translation();
  EXPECT_LT((yeni_p - z_w).norm(), (gercek_p - z_w).norm())
      << "duzeltme olcumden UZAKLASTI — artik isareti ters";
  EXPECT_LT(backend.covariance()(kIdxPos, kIdxPos), 1.0) << "konum belirsizligi azalmadi";
}

TEST(GnssPosition, FarOutlierIsChiSquareRejectedAndChangesNothing) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  EskfBackend backend(config(x, 0.25));

  const NavState durum_once = backend.state();
  const NavCovariance p_once = backend.covariance();

  // Yuzlerce metre sapma, 1.5 m std: NIS esigin cok uzerinde.
  const Vec3 z_w = x.extended_pose().translation() + Vec3(500.0, -300.0, 100.0);
  const GnssPosition z(2000, "gnss_main", z_w, Eigen::Matrix3d::Identity() * 2.25);

  const auto sonuc = backend.update(z);

  ASSERT_EQ(sonuc.status, UpdateStatus::kChiSquareRejected);
  ASSERT_TRUE(sonuc.nis.has_value());
  EXPECT_GT(*sonuc.nis, sonuc.threshold);
  EXPECT_EQ(sonuc.dof, 3);

  EXPECT_LT(backend.state().minus(durum_once).norm(), 0.0 + 1e-18)
      << "reddedilen GNSS durumu degistirdi";
  EXPECT_LT((backend.covariance() - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "reddedilen GNSS kovaryansi degistirdi";
}

TEST(GnssPosition, LeverArmShiftsTheAcceptedSolution) {
  // Ayni olcum, farkli lever-arm: cozum de farkli olmali. Lever-arm yok
  // sayilsaydi ikisi ayni cikardi.
  NavState x = ornek_durum();
  const Vec3 z_w = x.extended_pose().translation() + Vec3(0.4, -0.2, 0.1);
  const Eigen::Matrix3d c = Eigen::Matrix3d::Identity() * 2.25;

  EskfBackend a(config(x, 1.0));
  EskfBackend b(config(x, 1.0));
  const GnssPosition kolsuz(2000, "g", z_w, c);
  const GnssPosition kollu(2000, "g", z_w, c, Vec3(0.6, -0.4, 0.9));

  ASSERT_EQ(a.update(kolsuz).status, UpdateStatus::kAccepted);
  ASSERT_EQ(b.update(kollu).status, UpdateStatus::kAccepted);

  EXPECT_GT(
      (a.state().extended_pose().translation() - b.state().extended_pose().translation()).norm(),
      1e-3)
      << "lever-arm cozumu etkilemedi";
}

TEST(GnssPosition, BackendUsesNoGnssSpecificPath) {
  // Ayni olcum hem somut hem soyut arayuz uzerinden ayni sonucu vermeli:
  // backend'de tipe ozel dal yok.
  NavState x = ornek_durum();
  const Vec3 z_w = x.extended_pose().translation() + Vec3(0.3, 0.2, -0.1);
  const GnssPosition z(2000, "g", z_w, Eigen::Matrix3d::Identity() * 2.25, Vec3(0.1, 0.2, 0.3));

  EskfBackend somut_a(config(x, 1.0));
  EskfBackend somut_b(config(x, 1.0));
  kerteriz::FilterBackend& soyut = somut_b;

  const auto a = somut_a.update(z);
  const auto b = soyut.update(z);

  EXPECT_EQ(a.status, b.status);
  ASSERT_TRUE(a.nis.has_value() && b.nis.has_value());
  EXPECT_NEAR(*a.nis, *b.nis, 1e-18);
  EXPECT_LT(somut_a.state().minus(somut_b.state()).norm(), 1e-18);
}

} // namespace

// -----------------------------------------------------------------------------
// Tahsis yasagi — CONVENTIONS §8.1 / ADR-22
//
// operator new GLOBAL scope'ta olmak ZORUNDA. Ayrica Eigen dinamik bellegi
// operator new ile DEGIL malloc ile alir; iki kapi birlikte kullanilir ve
// ikisinin de gercekten bagli oldugu ayrica dogrulanir.
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

TEST(GnssPosition, AllocationCounterIsActuallyWired) {
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

TEST(GnssPosition, EvaluateDoesNotAllocate) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();

  const GnssPosition z(2000, "gnss_main", Vec3(20.0, -3.0, 5.0), ornek_kovaryans(),
                       Vec3(0.1, -0.2, 0.3));
  MeasurementWorkspace w;
  const StateBundle bundle(x);

  w.clear(); // isinma
  z.evaluate(bundle, w);

  {
    const TahsisKapsami kapsam;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < 200; ++i) {
      w.clear();
      z.evaluate(bundle, w);
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_EQ(tahsis_adedi, 0) << "evaluate() sicak yolda heap'e gitti";
}

TEST(GnssPosition, BackendUpdateWithGnssDoesNotAllocate) {
  NavState x = ornek_durum();
  EskfBackend backend(config(x, 1.0));
  const Vec3 p0 = x.extended_pose().translation();
  const GnssPosition yakin(2000, "g", p0 + Vec3(0.2, -0.1, 0.05),
                           Eigen::Matrix3d::Identity() * 2.25, Vec3(0.1, 0.0, -0.2));
  const GnssPosition uzak(2000, "g", p0 + Vec3(500.0, -300.0, 100.0),
                          Eigen::Matrix3d::Identity() * 2.25);

  backend.update(yakin); // isinma

  {
    const TahsisKapsami kapsam;
    Eigen::internal::set_is_malloc_allowed(false);
    for (int i = 0; i < 50; ++i) {
      backend.update(yakin);
      backend.update(uzak);
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }
  EXPECT_EQ(tahsis_adedi, 0) << "GNSS update yolu heap'e gitti";
}

} // namespace
