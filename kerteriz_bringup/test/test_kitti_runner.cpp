/// \file
/// F1.8 Checkpoint B — KITTI kosucusu bağlantı katmani.
///
/// SENTETIK FIXTURE kullanir (test/fixtures/OKUBENI.md). Buradan hicbir
/// dogruluk iddiasi cikarilmaz; sinanan sey KOSUCU MANTIGIDIR: uretim zinciri
/// gercekten kullaniliyor mu, ara-degerleme politikasi uygulaniyor mu, cikti
/// deterministik mi, kol parametresi olcume ulasiyor mu.
///
/// Ayristirici testleri Checkpoint A'dadir ve BURADA TEKRARLANMAZ.

#include "kerteriz_bringup/kitti_oxts.hpp"
#include "kerteriz_bringup/kitti_runner.hpp"

#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

using kerteriz::Scalar;
using kerteriz::Vec3;
using kerteriz_bringup::DatasetEvent;
using kerteriz_bringup::DatasetEventKind;
using kerteriz_bringup::KittiConfig;
using kerteriz_bringup::KittiRunnerConfig;
using kerteriz_bringup::run_kitti;

std::vector<DatasetEvent> fixture_olaylari() {
  KittiConfig cfg;
  cfg.dataset_dir = std::string(KERTERIZ_FIXTURE_DIR) + "/kitti_sentetik";
  std::vector<DatasetEvent> ev;
  EXPECT_TRUE(kerteriz_bringup::load_kitti_oxts(cfg, ev).ok);
  return ev;
}

TEST(KittiRunner, RunsThroughProductionChainAndStaysFinite) {
  const auto ev = fixture_olaylari();
  const auto r = run_kitti(ev, KittiRunnerConfig{});

  ASSERT_TRUE(r.ok) << r.message;
  EXPECT_FALSE(r.trajectory.empty());
  EXPECT_TRUE(r.stats.final_state_finite);
  EXPECT_TRUE(r.stats.final_covariance_finite);
  EXPECT_LT(r.stats.final_covariance_symmetry_error, 1e-9);
  EXPECT_GT(r.stats.final_covariance_min_diagonal, 0.0)
      << "aktif kovaryans kosegeni pozitif olmali";

  // Yorunge damga sirasinda ve tekil olmali.
  for (std::size_t i = 1; i < r.trajectory.size(); ++i) {
    EXPECT_LT(r.trajectory[i - 1].stamp_ns, r.trajectory[i].stamp_ns);
  }
}

TEST(KittiRunner, MeasurementsGoThroughTheRealGateNotABypass) {
  // Kosucu kendi guncelleme denklemini yazmiyor: olcumler gercek kapidan
  // geciyor ve sonuclar ardisik duzen sebeplerine gore sayiliyor. Kabul
  // edilen her olcum backend'e ULASMIS demektir.
  const auto ev = fixture_olaylari();
  const auto r = run_kitti(ev, KittiRunnerConfig{});
  ASSERT_TRUE(r.ok) << r.message;

  const auto& gp = r.stats.gnss_position;
  EXPECT_EQ(gp.submitted,
            gp.accepted + gp.chi_square_gated + gp.numerical_failure + gp.buffer_rejected)
      << "verilen olcum sayisi ile raporlanan sonuc sayisi tutmuyor";
  EXPECT_GT(gp.accepted, 0) << "hicbir GNSS olcumu backend'e ulasmamis";
  EXPECT_EQ(r.stats.gnss_position.total, 3);
}

TEST(KittiRunner, InterpolatedFramesAreExcludedByExplicitPolicy) {
  // Fixture'in 3. karesi (indeks 2) veri seti tarafindan ara-degerlenmis
  // olarak isaretlidir (posmode/velmode/orimode = -1).
  const auto ev = fixture_olaylari();
  const auto r = run_kitti(ev, KittiRunnerConfig{});
  ASSERT_TRUE(r.ok) << r.message;

  EXPECT_EQ(r.stats.gnss_position.skipped_interpolated, 1)
      << "ara-degerlenmis GNSS konumu olcum olarak verilmis";
  EXPECT_EQ(r.stats.gnss_velocity.skipped_interpolated, 1)
      << "ara-degerlenmis GNSS hizi olcum olarak verilmis";

  EXPECT_EQ(r.stats.reference_total, 3);
  EXPECT_EQ(r.stats.reference_used, 2) << "ara-degerlenmis referans ATE'ye girmis";
  EXPECT_EQ(r.reference.size(), 2U);
  for (const auto& s : r.reference) {
    EXPECT_NE(s.stamp_ns, ev.back().stamp_ns) << "ara-degerlenmis kare referansa sizmis";
  }

  // IMU DUSURULMEZ — zaman surekliligi gerekir — ama AYRI sayilir.
  EXPECT_EQ(r.stats.imu_events, 3);
  EXPECT_EQ(r.stats.imu_interpolated, 1);
}

TEST(KittiRunner, IsDeterministic) {
  const auto ev = fixture_olaylari();
  const auto a = run_kitti(ev, KittiRunnerConfig{});
  const auto b = run_kitti(ev, KittiRunnerConfig{});
  ASSERT_TRUE(a.ok && b.ok);
  ASSERT_EQ(a.trajectory.size(), b.trajectory.size());

  for (std::size_t i = 0; i < a.trajectory.size(); ++i) {
    EXPECT_EQ(a.trajectory[i].stamp_ns, b.trajectory[i].stamp_ns);
    EXPECT_LT((a.trajectory[i].position_w - b.trajectory[i].position_w).cwiseAbs().maxCoeff(),
              0.0 + 1e-18);
    EXPECT_LT((a.trajectory[i].velocity_w - b.trajectory[i].velocity_w).cwiseAbs().maxCoeff(),
              0.0 + 1e-18);
  }
}

TEST(KittiRunner, LeverArmReachesTheGnssPositionConstructor) {
  // Kol yapilandirmada kalip olcume ULASMASAYDI iki kosu ayni cikardi.
  const auto ev = fixture_olaylari();

  KittiRunnerConfig kolsuz;
  KittiRunnerConfig kollu;
  kollu.gnss_lever_arm_b = Vec3(1.5, -0.8, 0.6);

  const auto a = run_kitti(ev, kolsuz);
  const auto b = run_kitti(ev, kollu);
  ASSERT_TRUE(a.ok && b.ok);
  ASSERT_FALSE(a.trajectory.empty());
  ASSERT_EQ(a.trajectory.size(), b.trajectory.size());

  EXPECT_GT((a.trajectory.back().position_w - b.trajectory.back().position_w).norm(), 1e-3)
      << "kol degeri GnssPosition kurucusuna ulasmiyor";
}

TEST(KittiRunner, InitializesFromFirstNonInterpolatedRecord) {
  const auto ev = fixture_olaylari();
  const auto r = run_kitti(ev, KittiRunnerConfig{});
  ASSERT_TRUE(r.ok) << r.message;

  const DatasetEvent* ilk_ref = nullptr;
  const DatasetEvent* ilk_hiz = nullptr;
  for (const auto& e : ev) {
    if (ilk_ref == nullptr && e.kind == DatasetEventKind::kReferencePose &&
        !e.source_interpolated) {
      ilk_ref = &e;
    }
  }
  ASSERT_NE(ilk_ref, nullptr);
  for (const auto& e : ev) {
    if (e.stamp_ns == ilk_ref->stamp_ns && e.kind == DatasetEventKind::kGnssVelocity) {
      ilk_hiz = &e;
    }
  }
  ASSERT_NE(ilk_hiz, nullptr);

  EXPECT_EQ(r.stats.init_stamp_ns, ilk_ref->stamp_ns);
  EXPECT_LT((r.init.position_w - ilk_ref->position_w).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_LT((r.init.velocity_w - ilk_hiz->velocity_w).cwiseAbs().maxCoeff(), 1e-18);
  EXPECT_NEAR(r.init.orientation_wb.angularDistance(ilk_ref->orientation_wb), 0.0, 1e-12);

  // Govde hizi = R^T v_W. Taban cizgisi de bunu alir (r_l hizlari govde
  // cercevesindedir), bu yuzden tek kod yolundan uretilir.
  const Vec3 beklenen =
      ilk_ref->orientation_wb.toRotationMatrix().transpose() * ilk_hiz->velocity_w;
  EXPECT_LT((r.init.velocity_b - beklenen).cwiseAbs().maxCoeff(), 1e-12);

  // Baslatma karesinin olcumleri filtreye VERILMEZ.
  EXPECT_EQ(r.stats.gnss_position.submitted, 1) << "baslatma karesi olcum olarak da kullanilmis";
}

TEST(KittiRunner, GnssVelocityCanBeDisabledForMatchedComparison) {
  // Harici taban cizgisine GNSS hizi verilemedigi icin esdeger kiyas cifti
  // yalnizca-konum yapilandirmasidir. O yapilandirma gercekten hicbir hiz
  // olcumu VERMEMELI.
  const auto ev = fixture_olaylari();
  KittiRunnerConfig cfg;
  cfg.use_gnss_velocity = false;
  const auto r = run_kitti(ev, cfg);
  ASSERT_TRUE(r.ok) << r.message;

  EXPECT_EQ(r.stats.gnss_velocity.submitted, 0);
  EXPECT_EQ(r.stats.gnss_velocity.accepted, 0);
  EXPECT_GT(r.stats.gnss_position.submitted, 0) << "konum olcumleri de kapanmis";
}

} // namespace
