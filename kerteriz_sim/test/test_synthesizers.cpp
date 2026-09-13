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

/// Duragan govde: olcum eksi ground truth TAM OLARAK gurultu terimidir.
TrajectoryGenerator duragan() {
  TrajectoryParams p;
  p.kind = TrajectoryKind::kConstantVelocity;
  p.linear_velocity = Vec3::Zero();
  return TrajectoryGenerator(p);
}

/// Ornek ortalamasi ve ornek standart sapmasi (Bessel duzeltmeli).
/// Ortalamanin sifir oldugu VARSAYILMAZ, kestirilir — boylece bir kayma
/// (bias) gurultu genligi gibi gorunup testi yanlislikla gecirmez.
struct StdBirikec {
  int n = 0;
  Scalar toplam = 0;
  Scalar toplam_kare = 0;

  void ekle(Scalar x) {
    ++n;
    toplam += x;
    toplam_kare += x * x;
  }
  Scalar ortalama() const { return toplam / static_cast<Scalar>(n); }
  Scalar std_sapma() const {
    const Scalar m = ortalama();
    return std::sqrt((toplam_kare - static_cast<Scalar>(n) * m * m) / static_cast<Scalar>(n - 1));
  }
};

/// Bir olcumun uc ekseninin std'si tek birikecte toplanir.
struct ImuGurultuOlcumu {
  Scalar jiro_std = 0;
  Scalar ivme_std = 0;
};

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

TEST(ImuSynthesizer, WhiteNoiseStdMatchesDensityOverSqrtDt) {
  // DoD: entegrasyon hatasi gurultu modeliyle TUTARLI olmali. Yalnizca
  // "gurultu etkili" demek yetmez; olcekleme de dogru olmalidir.
  // Ayrik beyaz gurultu:  sigma_d = yogunluk / sqrt(dt)   (CONVENTIONS §7)
  //
  // Bias yuruyusu ve baslangic bias'i sifir birakilir; geriye kalan tek
  // terim beyaz gurultudur, yani olcum eksi ground truth = gurultu.
  auto g = duragan();
  const auto gt = g.at(0);

  ImuParams p;
  p.gyro_noise_density = 0.02;  // rad/s/sqrt(Hz)
  p.accel_noise_density = 0.08; // m/s^2/sqrt(Hz)

  constexpr int kOrnek = 20000;

  auto olc = [&](Scalar dt) {
    SeededRng rng(4242U);
    ImuSynthesizer imu(p, rng);
    StdBirikec jiro;
    StdBirikec ivme;
    for (int i = 0; i < kOrnek; ++i) {
      const auto s = imu.sample(gt, dt);
      const Vec3 ivme_gurultusu = s.accel - Vec3(0.0, 0.0, kerteriz_sim::kGravity);
      for (int k = 0; k < 3; ++k) {
        jiro.ekle(s.gyro[k]); // duragan govdede gercek w = 0
        ivme.ekle(ivme_gurultusu[k]);
      }
    }
    return ImuGurultuOlcumu{jiro.std_sapma(), ivme.std_sapma()};
  };

  // 60000 orneklik std kestiriminin bagil hatasi ~1/sqrt(2N) = %0.3.
  // %4 tolerans bunun ~13 katidir (tesadufi dusme pratikte imkansiz), ama
  // yanlis usse karsi hala cok dar: dt yerine 1/dt kullanilsa dt=4e-3 icin
  // olculen std 15.8 kat sapardi.
  const Scalar dt1 = 4e-3;
  const auto o1 = olc(dt1);
  const Scalar jiro_beklenen1 = p.gyro_noise_density / std::sqrt(dt1);
  const Scalar ivme_beklenen1 = p.accel_noise_density / std::sqrt(dt1);
  EXPECT_NEAR(o1.jiro_std, jiro_beklenen1, jiro_beklenen1 * 0.04);
  EXPECT_NEAR(o1.ivme_std, ivme_beklenen1, ivme_beklenen1 * 0.04);

  const Scalar dt2 = 1e-2;
  const auto o2 = olc(dt2);
  const Scalar jiro_beklenen2 = p.gyro_noise_density / std::sqrt(dt2);
  const Scalar ivme_beklenen2 = p.accel_noise_density / std::sqrt(dt2);
  EXPECT_NEAR(o2.jiro_std, jiro_beklenen2, jiro_beklenen2 * 0.04);
  EXPECT_NEAR(o2.ivme_std, ivme_beklenen2, ivme_beklenen2 * 0.04);

  // Uslerin kendisini dogrudan sinar. Iki kosuda tohum ve cagri sayisi ayni
  // oldugu icin RNG cekilisleri BIREBIR aynidir; dolayisiyla std orani
  // istatistiksel degil, cebirseldir: sqrt(dt1/dt2). Ornekleme gurultusu
  // payda ve pay'da sadelesir, bu yuzden 1e-12 tolerans mesrudur.
  const Scalar oran_beklenen = std::sqrt(dt1 / dt2);
  EXPECT_NEAR(o2.jiro_std / o1.jiro_std, oran_beklenen, 1e-12)
      << "jiro beyaz gurultusu dt ile 1/sqrt(dt) yasasina gore olceklenmiyor";
  EXPECT_NEAR(o2.ivme_std / o1.ivme_std, oran_beklenen, 1e-12)
      << "ivme beyaz gurultusu dt ile 1/sqrt(dt) yasasina gore olceklenmiyor";
}

TEST(ImuSynthesizer, BiasIncrementStdMatchesWalkTimesSqrtDt) {
  // Rastgele yuruyus artisi:  std(db) = sigma_b * sqrt(dt)   (CONVENTIONS §7)
  // Beyaz gurultu sifir birakilir; olculen tek sey bias artislaridir.
  auto g = duragan();
  const auto gt = g.at(0);

  ImuParams p;
  p.gyro_bias_walk = 3e-3;  // rad/s/sqrt(s)
  p.accel_bias_walk = 7e-3; // m/s^2/sqrt(s)

  constexpr int kOrnek = 20000;

  auto olc = [&](Scalar dt) {
    SeededRng rng(9090U);
    ImuSynthesizer imu(p, rng);
    StdBirikec jiro;
    StdBirikec ivme;
    Vec3 onceki_jiro = imu.gyro_bias();
    Vec3 onceki_ivme = imu.accel_bias();
    for (int i = 0; i < kOrnek; ++i) {
      imu.sample(gt, dt);
      const Vec3 d_jiro = imu.gyro_bias() - onceki_jiro;
      const Vec3 d_ivme = imu.accel_bias() - onceki_ivme;
      for (int k = 0; k < 3; ++k) {
        jiro.ekle(d_jiro[k]);
        ivme.ekle(d_ivme[k]);
      }
      onceki_jiro = imu.gyro_bias();
      onceki_ivme = imu.accel_bias();
    }
    return ImuGurultuOlcumu{jiro.std_sapma(), ivme.std_sapma()};
  };

  const Scalar dt1 = 4e-3;
  const auto o1 = olc(dt1);
  const Scalar jiro_beklenen1 = p.gyro_bias_walk * std::sqrt(dt1);
  const Scalar ivme_beklenen1 = p.accel_bias_walk * std::sqrt(dt1);
  EXPECT_NEAR(o1.jiro_std, jiro_beklenen1, jiro_beklenen1 * 0.04);
  EXPECT_NEAR(o1.ivme_std, ivme_beklenen1, ivme_beklenen1 * 0.04);

  const Scalar dt2 = 2.5e-2;
  const auto o2 = olc(dt2);
  const Scalar jiro_beklenen2 = p.gyro_bias_walk * std::sqrt(dt2);
  const Scalar ivme_beklenen2 = p.accel_bias_walk * std::sqrt(dt2);
  EXPECT_NEAR(o2.jiro_std, jiro_beklenen2, jiro_beklenen2 * 0.04);
  EXPECT_NEAR(o2.ivme_std, ivme_beklenen2, ivme_beklenen2 * 0.04);

  // Us dogrudan: beyaz gurultunun tersi yonde, sqrt(dt2/dt1).
  const Scalar oran_beklenen = std::sqrt(dt2 / dt1);
  EXPECT_NEAR(o2.jiro_std / o1.jiro_std, oran_beklenen, 1e-12)
      << "jiro bias yuruyusu dt ile sqrt(dt) yasasina gore olceklenmiyor";
  EXPECT_NEAR(o2.ivme_std / o1.ivme_std, oran_beklenen, 1e-12)
      << "ivme bias yuruyusu dt ile sqrt(dt) yasasina gore olceklenmiyor";
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

TEST(WheelSynthesizer, NoiseHasConfiguredStandardDeviation) {
  // Teker gurultusu dogrudan m/s cinsinden std'dir — IMU'nun aksine dt
  // olceklemesi YOKTUR. Ayarlanan deger ile olculen std birebir tutmali.
  auto g = daire();
  const auto gt = g.at(kSaniye);
  const Scalar gercek_hiz = gt.velocity.norm();

  SeededRng rng(31U);
  const Scalar sigma = 0.35;
  WheelSynthesizer teker(WheelParams{1.0, sigma}, rng);

  StdBirikec birikec;
  for (int i = 0; i < 20000; ++i) {
    birikec.ekle(teker.sample(gt).forward_speed - gercek_hiz);
  }

  EXPECT_NEAR(birikec.std_sapma(), sigma, sigma * 0.04);
  // Gurultu sifir ortalamali olmali; kacak bir kayma olcek hatasi gibi
  // davranip Faz 3 kalibrasyonunu yanlis dogrular.
  EXPECT_NEAR(birikec.ortalama(), 0.0, sigma * 0.04);
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
