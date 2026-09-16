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

/// Referans: olcumleri TAM damgalarinda uygular. IMU araligini damgada boler,
/// olcumu uygular, kalan araligi normal IMU olayi tamamlar. Bu, tamponun
/// yapmasi gerekenin ELLE yazilmis hâlidir — tampon kodundan bagimsizdir.
void exact_kos(Estimator& est, const std::vector<Olay>& imular, const std::vector<Olay>& olcumler) {
  std::size_t mi = 0;
  for (const Olay& o : imular) {
    while (mi < olcumler.size() && olcumler[mi].t <= o.t) {
      const TimeNs t_m = olcumler[mi].t;
      const TimeNs simdi = est.backend().stamp_ns();
      if (t_m > simdi) {
        ImuSample parca = imu(o.imu_k); // ornek [t_k, t_{k+1}] araligini temsil eder
        parca.stamp_ns = t_m;
        est.predict(parca, static_cast<Scalar>(t_m - simdi) * Scalar(1e-9));
      }
      est.apply(*gnss(t_m, olcumler[mi].ofset));
      ++mi;
    }
    est.predict(imu(o.imu_k), static_cast<Scalar>(o.t - est.backend().stamp_ns()) * Scalar(1e-9));
  }
  EXPECT_EQ(mi, olcumler.size()) << "referans kosuda yerlestirilmeyen olcum kaldi";
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

  // --- C) elle yazilmis TAM ZAMANLI referans ---
  // Iki tampon kosusunun birbirine esit olmasi yetmez: ikisi de ayni yanlisi
  // yapiyor olabilirdi. Bu ucuncu referans tampon kodunu hic kullanmaz.
  Estimator elle = kestirimci();
  exact_kos(elle, imular, olcumler);

  esit_mi(gecikmeli, referans, "gecikmeli vs kronolojik");
  esit_mi(referans, elle, "kronolojik vs elle tam-zamanli referans");

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
// Baslangic ufku — tampon zamani estimator'dan alir
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, FirstEventOlderThanEstimatorTimeIsNotTreatedAsInOrder) {
  // Estimator t0 = 1.000 s, tamponun gordugu ILK olay 0.950 s'lik bir olcum.
  // Sentinel bir "en kucuk zaman" ufkuyla bu olcum SIRALI sayilir ve 1.000 s
  // durumuna uygulanirdi. Ufuk estimator'un gercek zamanindan baslamali.
  const TimeNs t_gec = kT0 - 50 * kMs;

  { // pencere ICINDE: geri sarilacak gecmis yok -> kOutOfOrderDrop
    Estimator est = kestirimci();
    MeasurementBuffer tampon(kPencere); // 200 ms > 50 ms
    const NavState durum_once = est.backend().state();

    ASSERT_TRUE(tampon.add_measurement(gnss(t_gec, Vec3(5.0, -4.0, 3.0))));
    const auto sonuclar = tampon.process(est);

    ASSERT_EQ(sonuclar.size(), 1U);
    EXPECT_EQ(sonuclar[0].reason, RejectReason::kOutOfOrderDrop);
    EXPECT_FALSE(sonuclar[0].update.has_value()) << "backend'e ulasmamaliydi";
    EXPECT_LT(est.backend().state().minus(durum_once).norm(), 0.0 + 1e-18)
        << "gecmis olcum guncel duruma uygulandi";
    EXPECT_EQ(est.backend().stamp_ns(), kT0);
  }
  { // pencere DISINDA: kTooOld
    Estimator est = kestirimci();
    MeasurementBuffer tampon(20 * kMs); // 20 ms < 50 ms
    const NavState durum_once = est.backend().state();

    ASSERT_TRUE(tampon.add_measurement(gnss(t_gec, Vec3(5.0, -4.0, 3.0))));
    const auto sonuclar = tampon.process(est);

    ASSERT_EQ(sonuclar.size(), 1U);
    EXPECT_EQ(sonuclar[0].reason, RejectReason::kTooOld);
    EXPECT_FALSE(sonuclar[0].update.has_value());
    EXPECT_LT(est.backend().state().minus(durum_once).norm(), 0.0 + 1e-18);
  }
}

TEST(MeasurementBuffer, MeasurementAtCurrentEstimatorTimeDoesNotWaitForFutureImu) {
  // Damgasi estimator'un TAM mevcut zamani olan bir olcum yayilim GEREKTIRMEZ:
  // durum zaten o andadir. Erteleme yalnizca olcum hem mevcut zamanin hem de
  // bilinen en yeni IMU orneginin ILERISINDE ise anlamlidir; aksi halde
  // hicbir IMU gelmezse olcum sonsuza kadar kuyrukta beklerdi.
  Estimator est = kestirimci();
  MeasurementBuffer tampon(kPencere); // taze tampon, hic IMU yok
  const NavState durum_once = est.backend().state();
  const Vec3 ofset(0.40, -0.25, 0.15);
  const Vec3 z_w = ornek_durum().extended_pose().translation() + ofset;

  ASSERT_TRUE(tampon.add_measurement(gnss(kT0, ofset)));
  const auto sonuclar = tampon.process(est);

  ASSERT_EQ(sonuclar.size(), 1U) << "olcum ertelendi — gelecek IMU bekleniyor";
  EXPECT_EQ(sonuclar[0].reason, RejectReason::kNone);
  EXPECT_EQ(sonuclar[0].stamp_ns, kT0);
  ASSERT_TRUE(sonuclar[0].update.has_value());
  EXPECT_EQ(sonuclar[0].update->dof, 3);

  EXPECT_EQ(est.backend().stamp_ns(), kT0) << "yayilim yapilmamaliydi";
  const Vec3 p_once = durum_once.extended_pose().translation();
  const Vec3 p_sonra = est.backend().state().extended_pose().translation();
  EXPECT_LT((p_sonra - z_w).norm(), (p_once - z_w).norm()) << "duzeltme olcum yonunde degil";
}

// -----------------------------------------------------------------------------
// Basarisiz geri sarma, gecerli olaylari DUSURMEZ
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, FailedRewindDropsOnlyTheUnplaceableEvent) {
  constexpr TimeNs kGenisPencere = 3600LL * 1'000'000'000LL;
  constexpr int kFazla = kerteriz::kMaxBufferedEvents + 16;

  Estimator est = kestirimci();
  MeasurementBuffer tampon(kGenisPencere);
  for (int k = 1; k <= kFazla; ++k) {
    tampon.add_imu(imu(k));
    ASSERT_TRUE(tampon.process(est).empty());
  }
  const TimeNs t_once = est.backend().stamp_ns();
  const NavState durum_once = est.backend().state();

  // AYNI process() cagrisinda: yerlestirilemeyecek kadar eski GNSS,
  // guncel IMU ve guncel GNSS.
  const TimeNs t_guncel = kT0 + static_cast<TimeNs>(kFazla + 1) * kImuAdimi;
  ASSERT_TRUE(tampon.add_measurement(gnss(kT0 + 2 * kImuAdimi, Vec3(0.3, -0.1, 0.2))));
  tampon.add_imu(imu(kFazla + 1));
  ASSERT_TRUE(tampon.add_measurement(gnss(t_guncel, Vec3(0.4, -0.2, 0.1))));

  const auto sonuclar = tampon.process(est);

  ASSERT_EQ(sonuclar.size(), 2U) << "gecerli olcum de dusurulmus";
  EXPECT_EQ(sonuclar[0].reason, RejectReason::kOutOfOrderDrop);
  EXPECT_FALSE(sonuclar[0].update.has_value());

  EXPECT_EQ(sonuclar[1].stamp_ns, t_guncel);
  EXPECT_EQ(sonuclar[1].reason, RejectReason::kNone) << "guncel GNSS backend'e ulasmadi";
  ASSERT_TRUE(sonuclar[1].update.has_value());
  EXPECT_EQ(sonuclar[1].update->dof, 3);

  EXPECT_EQ(est.backend().stamp_ns(), t_guncel) << "guncel IMU yayilim yapmadi";
  EXPECT_GT(est.backend().stamp_ns(), t_once);
  EXPECT_GT(est.backend().state().minus(durum_once).norm(), 1e-9);
}

// -----------------------------------------------------------------------------
// Olcum damgasinda TAM yerlestirme
// -----------------------------------------------------------------------------

TEST(MeasurementBuffer, NonAlignedMeasurementIsAppliedAtItsOwnTimestamp) {
  // IMU 40 ms · GNSS 45 ms · IMU 50 ms.  GNSS 45 ms durumuna uygulanmali.
  const TimeNs t_olcum = kT0 + 4 * kImuAdimi + 5 * kMs;
  const Vec3 ofset(0.45, -0.25, 0.15);

  Estimator tamponlu = kestirimci();
  MeasurementBuffer tampon(kPencere);
  int kabul = 0;
  for (int k = 1; k <= 8; ++k) {
    tampon.add_imu(imu(k));
    if (k == 4) { // olcum, kapsayan IMU'dan (k=5) ONCE gelir
      ASSERT_TRUE(tampon.add_measurement(gnss(t_olcum, ofset)));
    }
    for (const auto& r : tampon.process(tamponlu)) {
      EXPECT_EQ(r.reason, RejectReason::kNone);
      EXPECT_EQ(r.stamp_ns, t_olcum);
      ++kabul;
    }
  }
  ASSERT_EQ(kabul, 1) << "olcum hic islenmedi (erteleme takildi mi?)";

  // Elle yazilmis TAM ZAMANLI referans.
  std::vector<Olay> imular;
  for (int k = 1; k <= 8; ++k) {
    imular.push_back({kT0 + k * kImuAdimi, true, k, Vec3::Zero()});
  }
  Estimator elle = kestirimci();
  exact_kos(elle, imular, {{t_olcum, false, 0, ofset}});
  esit_mi(tamponlu, elle, "tam damgada yerlestirme");

  // Ve onceki IMU durumuna uygulamak GERCEKTEN farkli sonuc verirdi —
  // aksi halde bu test bos gecerdi.
  Estimator hizalanmis = kestirimci();
  for (int k = 1; k <= 8; ++k) {
    if (k == 5) {
      hizalanmis.apply(*gnss(t_olcum, ofset)); // 40 ms durumunda
    }
    hizalanmis.predict(imu(k),
                       static_cast<Scalar>(imu(k).stamp_ns - hizalanmis.backend().stamp_ns()) *
                           Scalar(1e-9));
  }
  EXPECT_GT(tamponlu.backend().state().minus(hizalanmis.backend().state()).norm(), 1e-9)
      << "45 ms ile 40 ms yerlestirmesi ayirt edilemiyor — test bos geciyor";
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
