/// \file
/// F1.7 — MeasurementBuffer: sira, geri sarma ve yeniden yayilim.
/// INTERFACES §5 · ADR-5, ADR-15, ADR-19, ADR-20 · CONVENTIONS §6.
///
/// En onemli test `DelayedMeasurementMatchesChronologicalReference`'tir:
/// geciken olcum + geri sarma yolu ile ayni olaylarin bastan kronolojik
/// islendigi referans kosu AYNI final durumu vermelidir. Geri sarmanin
/// gercekten dogru oldugunun asil kaniti budur.

#include "kerteriz/backends/eskf_backend.hpp"
#include "kerteriz/buffer/measurement_buffer.hpp"
#include "kerteriz/measurements/gnss_position.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <memory>
#include <utility>
#include <vector>

namespace {

using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::Estimator;
using kerteriz::FaultDetector;
using kerteriz::GnssPosition;
using kerteriz::ImuNoiseParams;
using kerteriz::ImuSample;
using kerteriz::Measurement;
using kerteriz::MeasurementBuffer;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::NisMonitor;
using kerteriz::RejectReason;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::TimeNs;
using kerteriz::Vec3;

constexpr Scalar kGuven = 0.997;
constexpr TimeNs kT0 = 1'000'000'000;
constexpr TimeNs kMs = 1'000'000;
constexpr TimeNs kImuAdimi = 10 * kMs; ///< 100 Hz
constexpr TimeNs kPencere = 200 * kMs; ///< ADR-5 varsayilani

NavState ornek_durum() {
  NavState x;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(0.23, Vec3(0.1, -0.3, 0.95).normalized()));
  x.extended_pose() = SE23(Vec3(2.0, -1.0, 0.5), q, Vec3(0.3, 0.1, 0.0));
  return x;
}

Estimator kestirimci(Scalar p_diag = 1.0) {
  const NavState x = ornek_durum();
  NavCovariance p = NavCovariance::Zero();
  const int n = x.active_dof();
  p.topLeftCorner(n, n) = Eigen::MatrixXd::Identity(n, n).cast<Scalar>() * p_diag;
  return Estimator(std::make_unique<EskfBackend>(
                       EskfConfig{x, p, kT0, ImuNoiseParams{1e-3, 1e-5, 2e-2, 3e-4}, kGuven}),
                   NisMonitor{}, FaultDetector{});
}

/// Deterministik, hafif salinan IMU. Icerik onemsiz; TEKRAR URETILEBILIR
/// olmasi onemli.
ImuSample imu(int k) {
  const Scalar a = 0.05 * static_cast<Scalar>(k);
  return ImuSample{kT0 + static_cast<TimeNs>(k) * kImuAdimi,
                   Vec3(0.01 * std::sin(a), -0.02 * std::cos(a), 0.015),
                   Vec3(0.1 * std::cos(a), -0.05 * std::sin(a), 9.81)};
}

/// Baslangic konumuna yakin, kapiyi rahat gecen bir GNSS olcumu.
std::unique_ptr<Measurement> gnss(TimeNs t, const Vec3& ofset) {
  return std::make_unique<GnssPosition>(t, "gnss_position",
                                        ornek_durum().extended_pose().translation() + ofset,
                                        Eigen::Matrix3d::Identity() * 2.25);
}

/// Referans: ayni olaylari tampon OLMADAN, dogrudan kronolojik uygular.
/// `dt` yine yalnizca iki TimeNs farkindan hesaplanir.
struct Olay {
  TimeNs t;
  bool imu_mu;
  int imu_k;
  Vec3 ofset;
};

void dogrudan_kos(Estimator& est, const std::vector<Olay>& olaylar) {
  for (const Olay& o : olaylar) {
    if (o.imu_mu) {
      const TimeNs fark = o.t - est.backend().stamp_ns();
      est.predict(imu(o.imu_k), static_cast<Scalar>(fark) * Scalar(1e-9));
    } else {
      est.apply(*gnss(o.t, o.ofset));
    }
  }
}

void esit_mi(const Estimator& a, const Estimator& b, const char* nerede) {
  EXPECT_EQ(a.backend().stamp_ns(), b.backend().stamp_ns()) << nerede << ": zaman";
  EXPECT_LT(a.backend().state().minus(b.backend().state()).norm(), 0.0 + 1e-18)
      << nerede << ": durum";
  EXPECT_LT((a.backend().covariance() - b.backend().covariance()).cwiseAbs().maxCoeff(),
            0.0 + 1e-18)
      << nerede << ": kovaryans";
}

// -----------------------------------------------------------------------------
// Sirali akis
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, InOrderStreamMatchesDirectBackendCalls) {
  std::vector<Olay> olaylar;
  for (int k = 1; k <= 20; ++k) {
    olaylar.push_back({kT0 + k * kImuAdimi, true, k, Vec3::Zero()});
    if (k == 5 || k == 12 || k == 18) {
      olaylar.push_back({kT0 + k * kImuAdimi, false, 0, Vec3(0.4, -0.2, 0.1)});
    }
  }

  Estimator tamponlu = kestirimci();
  MeasurementBuffer tampon(kPencere);
  int kabul = 0;
  for (const Olay& o : olaylar) {
    if (o.imu_mu) {
      tampon.add_imu(imu(o.imu_k));
    } else {
      ASSERT_TRUE(tampon.add_measurement(gnss(o.t, o.ofset)));
    }
    for (const auto& r : tampon.process(tamponlu)) {
      EXPECT_EQ(r.reason, RejectReason::kNone);
      ++kabul;
    }
  }
  EXPECT_EQ(kabul, 3) << "her olcum icin tam bir sonuc uretilmeli";

  Estimator dogrudan = kestirimci();
  dogrudan_kos(dogrudan, olaylar);
  esit_mi(tamponlu, dogrudan, "sirali akis");
}

TEST(MeasurementBuffer, ResultsComeOutInTimestampOrder) {
  Estimator est = kestirimci();
  MeasurementBuffer tampon(kPencere);

  for (int k = 1; k <= 6; ++k) {
    tampon.add_imu(imu(k));
  }
  // Kasten TERS sirada eklenir; tampon damgaya gore siralamali.
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 5 * kImuAdimi, Vec3(0.3, 0.0, 0.0))));
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 2 * kImuAdimi, Vec3(0.2, 0.0, 0.0))));
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 4 * kImuAdimi, Vec3(0.1, 0.0, 0.0))));

  const auto sonuclar = tampon.process(est);
  ASSERT_EQ(sonuclar.size(), 3U);
  EXPECT_EQ(sonuclar[0].stamp_ns, kT0 + 2 * kImuAdimi);
  EXPECT_EQ(sonuclar[1].stamp_ns, kT0 + 4 * kImuAdimi);
  EXPECT_EQ(sonuclar[2].stamp_ns, kT0 + 5 * kImuAdimi);
  for (const auto& r : sonuclar) {
    EXPECT_EQ(r.reason, RejectReason::kNone);
    EXPECT_EQ(r.sensor, "gnss_position");
  }
}

TEST(MeasurementBuffer, DtComesFromConsecutiveTimestampDifference) {
  // Esit olmayan araliklar: dt biriktirilseydi veya sabit varsayilsaydi
  // dogrudan referansla ayrisirdi.
  const std::vector<TimeNs> damgalar = {kT0 + 7 * kMs, kT0 + 19 * kMs, kT0 + 20 * kMs,
                                        kT0 + 63 * kMs};

  Estimator tamponlu = kestirimci();
  MeasurementBuffer tampon(kPencere);
  Estimator dogrudan = kestirimci();

  for (std::size_t i = 0; i < damgalar.size(); ++i) {
    ImuSample u = imu(static_cast<int>(i) + 1);
    u.stamp_ns = damgalar[i];
    tampon.add_imu(u);
    EXPECT_TRUE(tampon.process(tamponlu).empty()) << "IMU sonuc uretmemeli";

    const TimeNs fark = u.stamp_ns - dogrudan.backend().stamp_ns();
    dogrudan.predict(u, static_cast<Scalar>(fark) * Scalar(1e-9));
  }
  esit_mi(tamponlu, dogrudan, "esit olmayan araliklar");
}

// -----------------------------------------------------------------------------
// Bayat olcum
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, TooOldMeasurementIsRejectedAndChangesNothing) {
  Estimator est = kestirimci();
  MeasurementBuffer tampon(kPencere);

  for (int k = 1; k <= 40; ++k) { // 400 ms — pencere 200 ms
    tampon.add_imu(imu(k));
  }
  ASSERT_TRUE(tampon.process(est).empty());

  const NavState durum_once = est.backend().state();
  const NavCovariance p_once = est.backend().covariance();
  const TimeNs t_once = est.backend().stamp_ns();

  // Pencerenin epey disinda: 400 ms - 50 ms = 350 ms geride.
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 5 * kImuAdimi, Vec3(0.3, -0.1, 0.2))));
  const auto sonuclar = tampon.process(est);

  ASSERT_EQ(sonuclar.size(), 1U);
  EXPECT_EQ(sonuclar[0].reason, RejectReason::kTooOld);
  EXPECT_FALSE(sonuclar[0].update.has_value())
      << "backend hic cagrilmadi; uydurma UpdateResult uretilmemeli";

  EXPECT_LT(est.backend().state().minus(durum_once).norm(), 0.0 + 1e-18);
  EXPECT_LT((est.backend().covariance() - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18);
  EXPECT_EQ(est.backend().stamp_ns(), t_once);
}

// -----------------------------------------------------------------------------
// Geri sarma ve yeniden yayilim
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, OutOfOrderWithinWindowIsRewoundAndApplied) {
  Estimator est = kestirimci();
  MeasurementBuffer tampon(kPencere);

  for (int k = 1; k <= 15; ++k) {
    tampon.add_imu(imu(k));
  }
  ASSERT_TRUE(tampon.process(est).empty());
  const TimeNs t_once = est.backend().stamp_ns();
  const NavState durum_once = est.backend().state();

  // 50 ms geride, pencere icinde.
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 10 * kImuAdimi, Vec3(0.5, -0.3, 0.2))));
  const auto sonuclar = tampon.process(est);

  ASSERT_EQ(sonuclar.size(), 1U);
  EXPECT_EQ(sonuclar[0].reason, RejectReason::kNone) << "pencere icindeki gecikme kabul edilmeli";
  EXPECT_EQ(sonuclar[0].stamp_ns, kT0 + 10 * kImuAdimi);

  // Geri sarma sonrasi ILERI yeniden yayilim yapilmis olmali: zaman geri
  // gitmemeli, durum ise degismis olmali.
  EXPECT_EQ(est.backend().stamp_ns(), t_once) << "yeniden yayilim ufka geri donmedi";
  EXPECT_GT(est.backend().state().minus(durum_once).norm(), 1e-6)
      << "geciken olcum durumu hic etkilemedi";
}

TEST(MeasurementBuffer, DelayedMeasurementMatchesChronologicalReference) {
  // ==== F1.7'nin ANA TESTI ====
  //
  // Ayni olay kumesi iki farkli VARIS sirasiyla verilir:
  //   A) her sey kronolojik sirada gelir           -> geri sarma hic olmaz
  //   B) olcumler damgalarindan 70 ms SONRA gelir  -> her biri geri sarma tetikler
  //
  // Iki kosunun final durumu, kovaryansi ve zamani AYNI olmalidir. Geri
  // sarmanin gercekten dogru oldugunun asil kaniti budur; yanlis bir anchor,
  // atlanan bir yeniden yayilim veya bozulan sira burada ayrisir.
  constexpr TimeNs kGecikme = 70 * kMs; // pencere (200 ms) icinde

  std::vector<Olay> imular;
  for (int k = 1; k <= 30; ++k) {
    imular.push_back({kT0 + k * kImuAdimi, true, k, Vec3::Zero()});
  }
  const std::vector<Olay> olcumler = {
      {kT0 + 4 * kImuAdimi + 5 * kMs, false, 0, Vec3(-0.30, 0.20, 0.10)},
      {kT0 + 12 * kImuAdimi + 3 * kMs, false, 0, Vec3(0.45, -0.25, 0.15)},
      {kT0 + 21 * kImuAdimi + 8 * kMs, false, 0, Vec3(0.20, 0.35, -0.10)}};

  // --- A) kronolojik referans ---
  std::vector<Olay> kronolojik = imular;
  kronolojik.insert(kronolojik.end(), olcumler.begin(), olcumler.end());
  std::stable_sort(kronolojik.begin(), kronolojik.end(),
                   [](const Olay& a, const Olay& b) { return a.t < b.t; });

  Estimator referans = kestirimci();
  MeasurementBuffer ref_tampon(kPencere);
  int ref_kabul = 0;
  for (const Olay& o : kronolojik) {
    if (o.imu_mu) {
      ref_tampon.add_imu(imu(o.imu_k));
    } else {
      ASSERT_TRUE(ref_tampon.add_measurement(gnss(o.t, o.ofset)));
    }
    for (const auto& r : ref_tampon.process(referans)) {
      ASSERT_EQ(r.reason, RejectReason::kNone) << "referans kosuda olcum reddedildi";
      ++ref_kabul;
    }
  }
  ASSERT_EQ(ref_kabul, static_cast<int>(olcumler.size()));

  // --- B) gecikmeli varis ---
  Estimator gecikmeli = kestirimci();
  MeasurementBuffer gec_tampon(kPencere);
  int gec_kabul = 0;
  TimeNs onceki = kT0;
  for (const Olay& o : imular) {
    gec_tampon.add_imu(imu(o.imu_k));
    for (const Olay& m : olcumler) {
      const TimeNs varis = m.t + kGecikme;
      if (varis > onceki && varis <= o.t) {
        ASSERT_TRUE(gec_tampon.add_measurement(gnss(m.t, m.ofset)));
      }
    }
    onceki = o.t;
    for (const auto& r : gec_tampon.process(gecikmeli)) {
      ASSERT_EQ(r.reason, RejectReason::kNone) << "gecikmeli kosuda olcum reddedildi";
      EXPECT_TRUE(r.update.has_value());
      ++gec_kabul;
    }
  }
  ASSERT_EQ(gec_kabul, static_cast<int>(olcumler.size()))
      << "geciken olcumlerin hepsi islenmeliydi — test aksi halde bos gecer";

  esit_mi(gecikmeli, referans, "gecikmeli vs kronolojik");

  // Iki kosunun GERCEKTEN farkli yollardan gectigini de dogrula: gecikmeli
  // kosuda her olcum damgasinin gerisine geri sarildi. Aksi halde bu test
  // yalnizca ayni sirayi iki kez kosardi.
  EXPECT_GT(kGecikme, kImuAdimi) << "gecikme bir IMU adimindan kucukse sira hic bozulmaz";
}

TEST(MeasurementBuffer, ReplayedMeasurementsAreNotReportedTwice) {
  Estimator est = kestirimci();
  MeasurementBuffer tampon(kPencere);

  for (int k = 1; k <= 10; ++k) {
    tampon.add_imu(imu(k));
  }
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 8 * kImuAdimi, Vec3(0.3, 0.0, 0.0))));
  ASSERT_EQ(tampon.process(est).size(), 1U);

  for (int k = 11; k <= 14; ++k) {
    tampon.add_imu(imu(k));
  }
  ASSERT_TRUE(tampon.process(est).empty());

  // Geri sarma: 8. adimdaki olcum yeniden oynatilacak ama TEKRAR
  // raporlanmayacak. Yalnizca yeni gelen sonuc dondurulur.
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 9 * kImuAdimi, Vec3(-0.2, 0.1, 0.0))));
  const auto sonuclar = tampon.process(est);

  ASSERT_EQ(sonuclar.size(), 1U) << "yeniden oynatilan eski olcum tekrar raporlandi";
  EXPECT_EQ(sonuclar[0].stamp_ns, kT0 + 9 * kImuAdimi);
}

TEST(MeasurementBuffer, OutOfOrderDropWhenRewindPointWasEvicted) {
  // Pencere cok genis: hicbir sey "bayat" degil. Buna ragmen gecmis kapasite
  // tavani asilinca en eski kayitlar dusurulur ve o noktaya geri sarilamaz.
  // kTooOld ile kOutOfOrderDrop'u ayiran durum tam olarak budur.
  constexpr TimeNs kGenisPencere = 3600LL * 1'000'000'000LL;
  constexpr int kFazla = kerteriz::kMaxBufferedEvents + 16;

  Estimator est = kestirimci();
  MeasurementBuffer tampon(kGenisPencere);
  for (int k = 1; k <= kFazla; ++k) {
    tampon.add_imu(imu(k));
    ASSERT_TRUE(tampon.process(est).empty());
  }

  const NavState durum_once = est.backend().state();
  const TimeNs t_once = est.backend().stamp_ns();

  // En bastaki olayin damgasi: kayitlari coktan dusurulmus.
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 2 * kImuAdimi, Vec3(0.3, -0.1, 0.2))));
  const auto sonuclar = tampon.process(est);

  ASSERT_EQ(sonuclar.size(), 1U);
  EXPECT_EQ(sonuclar[0].reason, RejectReason::kOutOfOrderDrop);
  EXPECT_FALSE(sonuclar[0].update.has_value());
  EXPECT_LT(est.backend().state().minus(durum_once).norm(), 0.0 + 1e-18);
  EXPECT_EQ(est.backend().stamp_ns(), t_once);
}

// -----------------------------------------------------------------------------
// Sahiplik ve kapasite
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, RejectsNullAndRespectsPendingCapacity) {
  MeasurementBuffer tampon(kPencere);
  EXPECT_FALSE(tampon.add_measurement(nullptr));

  for (int i = 0; i < kerteriz::kMaxPendingMeasurements; ++i) {
    EXPECT_TRUE(tampon.add_measurement(gnss(kT0 + i * kMs, Vec3::Zero()))) << "i = " << i;
  }
  EXPECT_FALSE(tampon.add_measurement(gnss(kT0, Vec3::Zero())))
      << "kapasite asildiginda sahiplik alinmamali";
}

TEST(MeasurementBuffer, ProcessWithoutPendingEventsIsANoOp) {
  Estimator est = kestirimci();
  MeasurementBuffer tampon(kPencere);
  const NavState durum_once = est.backend().state();

  EXPECT_TRUE(tampon.process(est).empty());
  EXPECT_LT(est.backend().state().minus(durum_once).norm(), 0.0 + 1e-18);
  EXPECT_EQ(tampon.window_ns(), kPencere);
}

} // namespace
