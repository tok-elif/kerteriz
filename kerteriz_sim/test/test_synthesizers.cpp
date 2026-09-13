/// \file
/// S9 — sensor sentezleyiciler ve DoD'nin ikinci yarisi:
/// uretilen IMU'yu entegre et, sapmayi gurultu modeliyle karsilastir.

#include "kerteriz_sim/gnss.hpp"
#include "kerteriz_sim/imu.hpp"
#include "kerteriz_sim/wheel.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <gtest/gtest.h>
#include <vector>

namespace {

using kerteriz::TimeNs;
using kerteriz_sim::GnssParams;
using kerteriz_sim::GnssSynthesizer;
using kerteriz_sim::ImuParams;
using kerteriz_sim::ImuSynthesizer;
using kerteriz_sim::Scalar;
using kerteriz_sim::SeededRng;
using kerteriz_sim::TrajectoryGenerator;
using kerteriz_sim::TrajectoryKind;
using kerteriz_sim::TrajectoryParams;
using kerteriz_sim::Vec3;
using kerteriz_sim::WheelParams;
using kerteriz_sim::WheelSynthesizer;

constexpr TimeNs kSaniye = 1000000000LL;

TrajectoryGenerator daire() {
  TrajectoryParams p;
  p.kind = TrajectoryKind::kCircle;
  p.radius = 6.0;
  p.angular_rate = 0.4;
  p.height = 1.0;
  return TrajectoryGenerator(p);
}

/// Strapdown entegrasyonu. Gurultusuz IMU verildiginde ground truth'u
/// geri vermelidir — sentezin kendi icinde tutarli oldugunun kaniti.
struct EntegrasyonSonucu {
  Vec3 konum;
  Vec3 hiz;
  Eigen::Matrix3d rotasyon;
};

EntegrasyonSonucu entegre_et(TrajectoryGenerator& g, ImuSynthesizer& imu, Scalar dt, Scalar sure) {
  const auto ilk = g.at(0);
  EntegrasyonSonucu s{ilk.position, ilk.velocity, ilk.rotation};

  const auto adim_ns = static_cast<TimeNs>(dt * 1e9);
  const int n = static_cast<int>(sure / dt);

  for (int i = 0; i < n; ++i) {
    const auto gt = g.at(static_cast<TimeNs>(i) * adim_ns);
    const auto olcum = imu.sample(gt, dt);

    // a_W = R * a_m + g_W   (ozgul kuvvetten dunya ivmesine)
    const Vec3 a_w = s.rotasyon * olcum.accel + kerteriz_sim::gravity_world();

    s.konum += s.hiz * dt + Scalar(0.5) * a_w * dt * dt;
    s.hiz += a_w * dt;

    const Vec3 aci = olcum.gyro * dt;
    const Scalar buyukluk = aci.norm();
    if (buyukluk > Scalar(0)) {
      s.rotasyon = s.rotasyon * Eigen::AngleAxisd(buyukluk, aci / buyukluk).toRotationMatrix();
    }
  }
  return s;
}

TEST(ImuSynthesizer, StaticBodyMeasuresGravityOnZ) {
  // ENU'da g_W = [0,0,-g]; durgun govde +z'de +g olcer (CONVENTIONS §7).
  TrajectoryParams p;
  p.kind = TrajectoryKind::kConstantVelocity;
  p.linear_velocity = Vec3::Zero();
  TrajectoryGenerator g(p);

  SeededRng rng(1U);
  ImuSynthesizer imu(ImuParams{}, rng); // gurultusuz, bias'siz

  const auto s = imu.sample(g.at(0), 0.01);
  EXPECT_NEAR(s.accel.x(), 0.0, 1e-12);
  EXPECT_NEAR(s.accel.y(), 0.0, 1e-12);
  EXPECT_NEAR(s.accel.z(), kerteriz_sim::kGravity, 1e-12);
  EXPECT_LT(s.gyro.norm(), 1e-15);
}

TEST(ImuSynthesizer, NoiselessImuIntegratesBackToGroundTruth) {
  // DoD'nin ikinci yarisi. Gurultu ve bias sifirken kalan tek hata sayisal
  // entegrasyon hatasidir; sentez ile ground truth tutarli olmalidir.
  auto g = daire();
  SeededRng rng(3U);
  ImuSynthesizer imu(ImuParams{}, rng);

  const Scalar dt = 1e-4;
  const Scalar sure = 5.0;
  const auto s = entegre_et(g, imu, dt, sure);

  const auto gt = g.at(static_cast<TimeNs>(sure * 1e9));

  EXPECT_LT((s.konum - gt.position).norm(), 1e-2) << "konum sapmasi cok buyuk";
  EXPECT_LT((s.hiz - gt.velocity).norm(), 1e-2) << "hiz sapmasi cok buyuk";
  EXPECT_LT((s.rotasyon - gt.rotation).cwiseAbs().maxCoeff(), 1e-4);
}

TEST(ImuSynthesizer, DriftGrowsWhenNoiseIsAdded) {
  // Gurultu modeli gercekten etkili olmali: ayni yorunge, gurultulu IMU ile
  // gurultusuzden belirgin sekilde daha cok sapmalidir.
  auto g = daire();

  SeededRng rng_temiz(5U);
  ImuSynthesizer imu_temiz(ImuParams{}, rng_temiz);
  const auto temiz = entegre_et(g, imu_temiz, 1e-3, 5.0);

  ImuParams gurultulu;
  gurultulu.gyro_noise_density = 0.01;
  gurultulu.accel_noise_density = 0.05;
  gurultulu.gyro_bias_walk = 1e-4;
  gurultulu.accel_bias_walk = 1e-3;
  SeededRng rng_gurultulu(5U);
  ImuSynthesizer imu_gurultulu(gurultulu, rng_gurultulu);
  const auto sapmis = entegre_et(g, imu_gurultulu, 1e-3, 5.0);

  const auto gt = g.at(static_cast<TimeNs>(5.0 * 1e9));
  const Scalar hata_temiz = (temiz.konum - gt.position).norm();
  const Scalar hata_gurultulu = (sapmis.konum - gt.position).norm();

  EXPECT_GT(hata_gurultulu, hata_temiz * 10.0)
      << "gurultu eklendi ama sapma artmadi — gurultu modeli baglanmamis";
}

TEST(ImuSynthesizer, BiasRandomWalkAccumulates) {
  ImuParams p;
  p.gyro_bias_walk = 1e-2;
  p.accel_bias_walk = 1e-2;
  SeededRng rng(11U);
  ImuSynthesizer imu(p, rng);

  auto g = daire();
  const Vec3 baslangic = imu.gyro_bias();
  for (int i = 0; i < 1000; ++i) {
    imu.sample(g.at(static_cast<TimeNs>(i) * (kSaniye / 100)), 0.01);
  }
  EXPECT_GT((imu.gyro_bias() - baslangic).norm(), 1e-3) << "bias yuruyusu ilerlemiyor";

  imu.reset_bias();
  EXPECT_LT((imu.gyro_bias() - baslangic).norm(), 1e-15);
}

TEST(GnssSynthesizer, PeriodMatchesConfiguredRate) {
  GnssParams p;
  p.rate_hz = 5.0;
  SeededRng rng(1U);
  GnssSynthesizer gnss(p, rng);
  EXPECT_EQ(gnss.period_ns(), 200000000LL);

  p.rate_hz = 0.0;
  GnssSynthesizer kapali(p, rng);
  EXPECT_EQ(kapali.period_ns(), 0);
}

TEST(GnssSynthesizer, NoiselessSampleEqualsGroundTruth) {
  auto g = daire();
  SeededRng rng(1U);
  GnssSynthesizer gnss(GnssParams{0.0, 5.0}, rng);

  const auto gt = g.at(2 * kSaniye);
  const auto s = gnss.sample(gt);
  EXPECT_EQ(s.stamp_ns, gt.stamp_ns);
  EXPECT_LT((s.position - gt.position).norm(), 1e-15);
}

TEST(GnssSynthesizer, NoiseHasConfiguredStandardDeviation) {
  auto g = daire();
  SeededRng rng(23U);
  const Scalar sigma = 0.75;
  GnssSynthesizer gnss(GnssParams{sigma, 5.0}, rng);

  const auto gt = g.at(kSaniye);
  const int n = 20000;
  Scalar toplam_kare = 0;
  for (int i = 0; i < n; ++i) {
    const auto s = gnss.sample(gt);
    toplam_kare += (s.position - gt.position).squaredNorm();
  }
  // Uc eksen bagimsiz => E[||n||^2] = 3 sigma^2
  const Scalar olculen = std::sqrt(toplam_kare / (Scalar(3) * n));
  EXPECT_NEAR(olculen, sigma, sigma * 0.05);
}

TEST(WheelSynthesizer, ReadsBodyForwardSpeed) {
  auto g = daire();
  SeededRng rng(1U);
  WheelSynthesizer teker(WheelParams{1.0, 0.0}, rng);

  const auto gt = g.at(kSaniye);
  const auto s = teker.sample(gt);

  // x_B hiz yonunde oldugu icin ileri hiz, hizin buyuklugudur.
  EXPECT_NEAR(s.forward_speed, gt.velocity.norm(), 1e-12);
}

TEST(WheelSynthesizer, ScaleFactorErrorIsRecoverable) {
  // ADR-10: olcek faktoru durum vektorune girer. Enjekte edilen hata geri
  // hesaplanabilir olmali, yoksa Faz 3'teki kalibrasyon dogrulanamaz.
  auto g = daire();
  SeededRng rng(1U);
  const Scalar olcek = 1.07;
  WheelSynthesizer teker(WheelParams{olcek, 0.0}, rng);

  const auto gt = g.at(kSaniye);
  const auto s = teker.sample(gt);
  EXPECT_NEAR(s.forward_speed / gt.velocity.norm(), olcek, 1e-12);
}

TEST(Synthesizers, SameSeedReproducesIdenticalSensorStreams) {
  // DoD: ayni tohum, ayni cikti. Sensor katmani icin de gecerli.
  auto uret = [](std::uint64_t tohum) {
    auto g = daire();
    SeededRng rng(tohum);
    ImuParams ip;
    ip.gyro_noise_density = 0.01;
    ip.accel_noise_density = 0.05;
    ImuSynthesizer imu(ip, rng);
    GnssSynthesizer gnss(GnssParams{0.5, 5.0}, rng);
    WheelSynthesizer teker(WheelParams{1.02, 0.1}, rng);

    std::vector<Scalar> cikti;
    for (int i = 0; i < 200; ++i) {
      const auto gt = g.at(static_cast<TimeNs>(i) * (kSaniye / 100));
      const auto a = imu.sample(gt, 0.01);
      const auto b = gnss.sample(gt);
      const auto c = teker.sample(gt);
      for (int k = 0; k < 3; ++k) {
        cikti.push_back(a.gyro[k]);
        cikti.push_back(a.accel[k]);
        cikti.push_back(b.position[k]);
      }
      cikti.push_back(c.forward_speed);
    }
    return cikti;
  };

  const auto a = uret(777U);
  const auto b = uret(777U);
  ASSERT_EQ(a.size(), b.size());
  EXPECT_EQ(std::memcmp(a.data(), b.data(), a.size() * sizeof(Scalar)), 0)
      << "ayni tohum farkli sensor akisi uretti";

  const auto c = uret(778U);
  EXPECT_NE(std::memcmp(a.data(), c.data(), a.size() * sizeof(Scalar)), 0)
      << "farkli tohum ayni akisi uretti";
}

} // namespace
