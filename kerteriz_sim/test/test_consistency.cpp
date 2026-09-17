/// \file
/// F2.2 — NEES / NIS istatistik motoru.
///
/// TESTLER DETERMINISTIKTIR. "Kosumlarin %95'i bandin icinde olmali" gibi bir
/// iddia BURADA YOKTUR: o bir DENEY SONUCUDUR, birim test degismezi degil, ve
/// F2.3'e aittir. Burada cebir, gecersiz kovaryans politikasi, konvansiyon,
/// quantile dogrulugu, band formulleri, NIS dahil etme kurali ve damga
/// sozlesmesi sinanir.

#include "kerteriz_sim/consistency.hpp"
#include "kerteriz_sim/monte_carlo.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

namespace {

using kerteriz::kCoreDof;
using kerteriz::Scalar;
using kerteriz::StateVec;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz::Vec3;
using kerteriz_sim::anees_band;
using kerteriz_sim::anis_band;
using kerteriz_sim::compute_nees;
using kerteriz_sim::CoreCovariance;
using kerteriz_sim::MonteCarloConfig;
using kerteriz_sim::MonteCarloResult;
using kerteriz_sim::NeesStatus;
using kerteriz_sim::NisObservation;
using kerteriz_sim::RunResult;
using kerteriz_sim::StateSample;

/// Gercek/kestirim cifti: gercek durum birim, kestirim ondan `e` kadar sapmis.
/// `plus` ve `minus` birbirinin tersi oldugu icin minus(e) tam olarak e verir.
StateSample ornek(const StateVec& e, const CoreCovariance& p, TimeNs t = 1000) {
  StateSample s;
  s.stamp_ns = t;
  s.truth = kerteriz::NavState{};
  s.estimate = s.truth.plus(e);
  s.covariance = p;
  s.active_dof = kCoreDof;
  return s;
}

StateVec ornek_hata() {
  StateVec e = StateVec::Zero();
  for (int i = 0; i < kCoreDof; ++i) {
    e[i] = Scalar(0.01) * Scalar(i + 1) - Scalar(0.03);
  }
  return e;
}

// -----------------------------------------------------------------------------
// A. NEES cebiri
// -----------------------------------------------------------------------------

TEST(Nees, IdentityCovarianceGivesSquaredErrorNorm) {
  const StateVec e = ornek_hata();
  const auto n = compute_nees(ornek(e, CoreCovariance::Identity()));

  ASSERT_TRUE(n.valid) << kerteriz_sim::to_string(n.status);
  EXPECT_EQ(n.dof, 15);
  EXPECT_NEAR(n.nees, e.head<kCoreDof>().squaredNorm(), 1e-12);
  EXPECT_GT(n.nees, 0.0) << "test kurulumu: hata sifir olmamali";
}

TEST(Nees, DiagonalCovarianceGivesExactWeightedSum) {
  const StateVec e = ornek_hata();
  CoreCovariance p = CoreCovariance::Zero();
  for (int i = 0; i < kCoreDof; ++i) {
    p(i, i) = Scalar(0.5) + Scalar(0.25) * Scalar(i);
  }
  Scalar beklenen = 0;
  for (int i = 0; i < kCoreDof; ++i) {
    beklenen += e[i] * e[i] / p(i, i);
  }

  const auto n = compute_nees(ornek(e, p));
  ASSERT_TRUE(n.valid) << kerteriz_sim::to_string(n.status);
  EXPECT_NEAR(n.nees, beklenen, 1e-12);
}

TEST(Nees, NonDiagonalCovarianceMatchesTheSolveNotAnInverse) {
  // Tam dolu SPD bir P. Beklenen deger LDLT COZUMUNDEN uretilir; explicit ters
  // alinmaz. Uygulama gizlice ters alsaydi bile deger ayni cikardi, ama bu
  // testin isi cebirsel dogruluk: capraz terimler gercekten kullaniliyor mu.
  const StateVec e = ornek_hata();
  CoreCovariance a = CoreCovariance::Zero();
  for (int i = 0; i < kCoreDof; ++i) {
    for (int k = 0; k < kCoreDof; ++k) {
      a(i, k) = Scalar(0.1) * Scalar((i + 1) * (k + 2)) - Scalar(0.05) * Scalar(i * k);
    }
  }
  const CoreCovariance p = a * a.transpose() + CoreCovariance::Identity() * Scalar(1.0);
  const auto n = compute_nees(ornek(e, p));
  ASSERT_TRUE(n.valid) << kerteriz_sim::to_string(n.status);

  const Eigen::Matrix<Scalar, kCoreDof, 1> ec = e.head<kCoreDof>();
  const Scalar beklenen = ec.dot(p.ldlt().solve(ec));
  EXPECT_NEAR(n.nees, beklenen, 1e-9);

  // Kosegen-yalniz yorum YANLIS olurdu: capraz terimler sonucu degistirmeli.
  Scalar kosegen_yalniz = 0;
  for (int i = 0; i < kCoreDof; ++i) {
    kosegen_yalniz += ec[i] * ec[i] / p(i, i);
  }
  EXPECT_GT(std::abs(n.nees - kosegen_yalniz), 1e-6) << "capraz terimler yok sayilmis olabilir";
}

// -----------------------------------------------------------------------------
// B. Gecersiz kovaryans — ONARILMAZ, REDDEDILIR
// -----------------------------------------------------------------------------

TEST(NeesInvalid, SingularCovarianceIsRejectedWithoutJitter) {
  CoreCovariance p = CoreCovariance::Identity();
  p(7, 7) = Scalar(0); // tekil
  const auto n = compute_nees(ornek(ornek_hata(), p));

  EXPECT_FALSE(n.valid);
  EXPECT_EQ(n.status, NeesStatus::kCovarianceNotPositiveDefinite);
  EXPECT_EQ(n.nees, 0.0) << "gecersiz ornek yine de bir sayi uretmis";
}

TEST(NeesInvalid, IndefiniteCovarianceIsRejected) {
  CoreCovariance p = CoreCovariance::Identity();
  p(3, 3) = Scalar(-2.0); // belirsiz
  const auto n = compute_nees(ornek(ornek_hata(), p));

  EXPECT_FALSE(n.valid);
  EXPECT_EQ(n.status, NeesStatus::kCovarianceNotPositiveDefinite);
}

TEST(NeesInvalid, NonFiniteCovarianceIsRejected) {
  CoreCovariance p = CoreCovariance::Identity();
  p(2, 2) = std::numeric_limits<Scalar>::quiet_NaN();
  EXPECT_EQ(compute_nees(ornek(ornek_hata(), p)).status, NeesStatus::kCovarianceNotFinite);

  p = CoreCovariance::Identity();
  p(5, 5) = std::numeric_limits<Scalar>::infinity();
  EXPECT_EQ(compute_nees(ornek(ornek_hata(), p)).status, NeesStatus::kCovarianceNotFinite);
}

TEST(NeesInvalid, MateriallyNonSymmetricCovarianceIsRejected) {
  // Eigen'in LDLT'si varsayilan olarak yalnizca ALT ucgeni okur: acik bir
  // simetri kontrolu olmasaydi bu matris SESSIZCE kabul edilir ve ust ucgen
  // yok sayilirdi.
  CoreCovariance p = CoreCovariance::Identity() * Scalar(2);
  p(1, 4) = Scalar(0.7);
  p(4, 1) = Scalar(-0.7);
  const auto n = compute_nees(ornek(ornek_hata(), p));

  EXPECT_FALSE(n.valid);
  EXPECT_EQ(n.status, NeesStatus::kCovarianceNotSymmetric);
}

TEST(NeesInvalid, RoundingLevelAsymmetryIsStillAccepted) {
  // Olcut MADDI asimetridir; kayan nokta artigi degil. Aksi halde gercek
  // kosularin cogu gereksiz yere gecersiz sayilirdi.
  CoreCovariance p = CoreCovariance::Identity() * Scalar(3);
  p(0, 6) += Scalar(1e-17);
  EXPECT_TRUE(compute_nees(ornek(ornek_hata(), p)).valid);
}

TEST(NeesInvalid, UnexpectedActiveDofIsRejected) {
  StateSample s = ornek(ornek_hata(), CoreCovariance::Identity());
  s.active_dof = kCoreDof + 6; // augmentation varmis gibi
  EXPECT_EQ(compute_nees(s).status, NeesStatus::kUnexpectedDof);
}

// -----------------------------------------------------------------------------
// C. Durum konvansiyonu
// -----------------------------------------------------------------------------

TEST(Nees, ErrorComesFromNavStateMinusIncludingBiases) {
  // Hata YALNIZCA bias bileseninde olsun. Elde kurulmus bir poz farki veya
  // yalnizca 9-DoF poz hatasi kullanilsaydi NEES sifir cikardi.
  StateVec e = StateVec::Zero();
  e.segment<3>(9) = Vec3(0.003, -0.004, 0.005); // jiro bias
  e.segment<3>(12) = Vec3(-0.02, 0.03, 0.01);   // ivme bias

  const auto n = compute_nees(ornek(e, CoreCovariance::Identity()));
  ASSERT_TRUE(n.valid);
  EXPECT_NEAR(n.nees, e.head<kCoreDof>().squaredNorm(), 1e-12);
  EXPECT_GT(n.nees, 0.0) << "bias bilesenleri 15D hataya girmiyor";

  // Poz bloklari gercekten sifir: katki yalnizca bias'tan geliyor.
  const StateSample s = ornek(e, CoreCovariance::Identity());
  const StateVec geri = s.estimate.minus(s.truth);
  EXPECT_LT(geri.head<9>().cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(geri.segment<3>(9).norm(), e.segment<3>(9).norm(), 1e-12);
}

// -----------------------------------------------------------------------------
// D. chi-kare quantile — genellestirilmis DoF
// -----------------------------------------------------------------------------

TEST(ChiSquareLargeDof, MatchesAnalyticTwoDofIdentity) {
  for (const Scalar p : {0.025, 0.5, 0.95, 0.975, 0.997}) {
    EXPECT_NEAR(kerteriz::chi_square_quantile(p, 2), -2.0 * std::log(1.0 - p), 1e-12)
        << "p = " << p;
  }
}

TEST(ChiSquareLargeDof, MatchesFrozenReferenceValues) {
  // Referanslar: scipy.stats.chi2.ppf. Toleranslar OLCULEN bagil hataya gore
  // secildi (dof=15 ~1e-16, dof=150 ~2e-15, dof=7500 ~1e-12); keyfi degil.
  struct Ref {
    Scalar p;
    int dof;
    Scalar beklenen;
    Scalar bagil_tol;
  };
  const Ref refs[] = {
      {0.025, 15, 6.262137795043253, 1e-13},   {0.975, 15, 27.488392863442975, 1e-13},
      {0.025, 150, 117.9845154029029, 1e-13},  {0.975, 150, 185.80044700379327, 1e-13},
      {0.025, 7500, 7261.854301690299, 1e-10}, {0.975, 7500, 7741.934237424176, 1e-10},
  };
  for (const Ref& r : refs) {
    const Scalar x = kerteriz::chi_square_quantile(r.p, r.dof);
    EXPECT_TRUE(std::isfinite(x)) << "dof = " << r.dof;
    EXPECT_LT(std::abs(x - r.beklenen) / r.beklenen, r.bagil_tol)
        << "p = " << r.p << " dof = " << r.dof << " hesap = " << x;
  }
}

TEST(ChiSquareLargeDof, IsMonotonicInConfidenceAndDof) {
  for (const int dof : {15, 150, 1500, 7500}) {
    EXPECT_LT(kerteriz::chi_square_quantile(0.025, dof), kerteriz::chi_square_quantile(0.5, dof));
    EXPECT_LT(kerteriz::chi_square_quantile(0.5, dof), kerteriz::chi_square_quantile(0.975, dof));
    EXPECT_TRUE(std::isfinite(kerteriz::chi_square_quantile(0.975, dof)));
  }
  EXPECT_LT(kerteriz::chi_square_quantile(0.975, 15), kerteriz::chi_square_quantile(0.975, 150));
  EXPECT_LT(kerteriz::chi_square_quantile(0.975, 1500), kerteriz::chi_square_quantile(0.975, 7500));
}

// -----------------------------------------------------------------------------
// Betimleyici medyan
// -----------------------------------------------------------------------------

TEST(DescriptiveMedian, OddCountTakesTheMiddleElement) {
  EXPECT_NEAR(kerteriz_sim::median({5.0}), 5.0, 1e-15);
  EXPECT_NEAR(kerteriz_sim::median({3.0, 1.0, 2.0}), 2.0, 1e-15);
  EXPECT_NEAR(kerteriz_sim::median({9.0, 1.0, 7.0, 3.0, 5.0}), 5.0, 1e-15);
}

TEST(DescriptiveMedian, EvenCountAveragesTheTwoMiddleElements) {
  // `v[n/2]` tek basina UST-ORTA ogeyi verir ve medyani yukari kaydirir.
  // E1 serisi 150 damgadir, yani tam olarak bu durum.
  EXPECT_NEAR(kerteriz_sim::median({1.0, 2.0}), 1.5, 1e-15);
  EXPECT_NEAR(kerteriz_sim::median({4.0, 1.0, 3.0, 2.0}), 2.5, 1e-15);
  EXPECT_NEAR(kerteriz_sim::median({10.0, 20.0, 30.0, 41.0, 50.0, 60.0}), 35.5, 1e-15);

  // Ust-orta oge ile AYNI OLMADIGI acikca gosterilir: aksi halde test hatali
  // uygulamayi da gecirirdi.
  const std::vector<Scalar> v = {1.0, 2.0, 3.0, 100.0};
  EXPECT_NEAR(kerteriz_sim::median(v), 2.5, 1e-15);
  EXPECT_NE(kerteriz_sim::median(v), v[v.size() / 2]);
}

TEST(DescriptiveMedian, DoesNotMutateTheCallersVector) {
  const std::vector<Scalar> girdi = {9.0, 1.0, 5.0, 3.0};
  std::vector<Scalar> kopya = girdi;
  EXPECT_NEAR(kerteriz_sim::median(kopya), 4.0, 1e-15);
  EXPECT_EQ(kopya, girdi) << "cagiranin dizisi siralanmis";
}

TEST(DescriptiveMedian, EmptySeriesIsZero) { EXPECT_EQ(kerteriz_sim::median({}), 0.0); }

// -----------------------------------------------------------------------------
// E. ANEES bandi
// -----------------------------------------------------------------------------

TEST(AneesBand, SingleSampleReducesToTheOrdinaryChiSquareInterval) {
  const auto b = anees_band(1, 15);
  EXPECT_NEAR(b.lower, kerteriz::chi_square_quantile(0.025, 15), 1e-12);
  EXPECT_NEAR(b.upper, kerteriz::chi_square_quantile(0.975, 15), 1e-12);
  EXPECT_EQ(b.expected_mean, 15.0);
  EXPECT_LT(b.lower, b.expected_mean);
  EXPECT_GT(b.upper, b.expected_mean);
}

TEST(AneesBand, EnsembleUsesTotalDofAndNarrowsWithM) {
  const auto b32 = anees_band(32, 15);
  EXPECT_NEAR(b32.lower, kerteriz::chi_square_quantile(0.025, 32 * 15) / 32.0, 1e-12);
  EXPECT_NEAR(b32.upper, kerteriz::chi_square_quantile(0.975, 32 * 15) / 32.0, 1e-12);
  EXPECT_EQ(b32.expected_mean, 15.0) << "beklenen merkez M'den bagimsiz olmali";

  // M buyudukce band DARALIR — topluluk toplamanin butun anlami budur.
  const auto b1 = anees_band(1, 15);
  EXPECT_GT(b32.lower, b1.lower);
  EXPECT_LT(b32.upper, b1.upper);

  // 500 kosumluk uretim deneyi (F2.3) de hesaplanabilmeli.
  const auto b500 = anees_band(500, 15);
  EXPECT_TRUE(std::isfinite(b500.lower) && std::isfinite(b500.upper));
  EXPECT_LT(b500.lower, 15.0);
  EXPECT_GT(b500.upper, 15.0);
}

TEST(AneesBand, ConsistencyConfidenceIsNotTheGateConfidence) {
  EXPECT_EQ(kerteriz_sim::kDefaultConsistencyConfidence, 0.95);
  const auto varsayilan = anees_band(8, 15);
  const auto kapi_gibi = anees_band(8, 15, 0.997);
  EXPECT_LT(kapi_gibi.lower, varsayilan.lower) << "0.997 bandi daha genis olmali";
  EXPECT_GT(kapi_gibi.upper, varsayilan.upper);
}

TEST(AnisBand, UsesSummedDofNotAHardCodedThree) {
  // Karisik artik boyutlari: 3 + 3 + 1 = 7.
  const auto b = anis_band(3, 7);
  EXPECT_NEAR(b.lower, kerteriz::chi_square_quantile(0.025, 7) / 3.0, 1e-12);
  EXPECT_NEAR(b.upper, kerteriz::chi_square_quantile(0.975, 7) / 3.0, 1e-12);
  EXPECT_NEAR(b.expected_mean, 7.0 / 3.0, 1e-12);

  // Sabit 3 gomulu olsaydi bu iki band ayni cikardi.
  const auto sabit_uc = anis_band(3, 9);
  EXPECT_NE(b.expected_mean, sabit_uc.expected_mean);
}

// -----------------------------------------------------------------------------
// F. NIS toplama
// -----------------------------------------------------------------------------

NisObservation gozlem(TimeNs t, Scalar nis, int dof, UpdateStatus st, bool var = true) {
  NisObservation o;
  o.stamp_ns = t;
  o.nis = nis;
  o.dof = dof;
  o.status = st;
  o.has_nis = var;
  return o;
}

MonteCarloResult sahte_mc(std::vector<std::vector<NisObservation>> kosular) {
  MonteCarloResult mc;
  for (auto& g : kosular) {
    RunResult r;
    r.nis_observations = std::move(g);
    mc.runs.push_back(std::move(r));
  }
  return mc;
}

TEST(NisAggregation, RejectedObservationsAreIncludedNotOnlyAccepted) {
  // Yalnizca kabul edilenleri almak UST KUYRUGU keser ve tutarlilik sonucunu
  // sistematik olarak IYIMSER gosterirdi. Reddedilen gozlem BUYUK NIS tasir.
  const auto mc = sahte_mc({
      {gozlem(100, 2.0, 3, UpdateStatus::kAccepted)},
      {gozlem(100, 40.0, 3, UpdateStatus::kChiSquareRejected)},
  });
  const auto s = kerteriz_sim::aggregate_nis(mc);

  ASSERT_TRUE(s.status.ok) << s.status.message;
  ASSERT_EQ(s.points.size(), 1U);
  EXPECT_EQ(s.points[0].valid_count, 2) << "reddedilen gozlem disarida birakilmis";
  EXPECT_EQ(s.points[0].undefined_count, 0);
  EXPECT_NEAR(s.points[0].mean_nis, 21.0, 1e-12);
  EXPECT_EQ(s.points[0].total_dof, 6);
  EXPECT_NEAR(s.points[0].expected_mean, 3.0, 1e-12);
}

TEST(NisAggregation, UndefinedNisIsExcludedAndCounted) {
  const auto mc = sahte_mc({
      {gozlem(100, 2.0, 3, UpdateStatus::kAccepted)},
      {gozlem(100, 0.0, 3, UpdateStatus::kNumericalFailure, /*var=*/false)},
      {gozlem(100, 4.0, 3, UpdateStatus::kAccepted)},
  });
  const auto s = kerteriz_sim::aggregate_nis(mc);

  ASSERT_TRUE(s.status.ok) << s.status.message;
  ASSERT_EQ(s.points.size(), 1U);
  EXPECT_EQ(s.points[0].valid_count, 2);
  EXPECT_EQ(s.points[0].undefined_count, 1) << "tanimsiz NIS sayilmamis";
  EXPECT_EQ(s.undefined_total, 1);
  EXPECT_NEAR(s.points[0].mean_nis, 3.0, 1e-12) << "tanimsiz gozlem ortalamaya karismis";
  EXPECT_EQ(s.points[0].total_dof, 6) << "tanimsiz gozlemin dof'u toplama girmis";
}

TEST(NisAggregation, MixedResidualDimensionsSumTheirDof) {
  const auto mc = sahte_mc({
      {gozlem(100, 3.0, 3, UpdateStatus::kAccepted)},
      {gozlem(100, 1.0, 1, UpdateStatus::kAccepted)},
      {gozlem(100, 2.0, 2, UpdateStatus::kAccepted)},
  });
  const auto s = kerteriz_sim::aggregate_nis(mc);

  ASSERT_TRUE(s.status.ok) << s.status.message;
  EXPECT_EQ(s.points[0].total_dof, 6) << "dof toplami yanlis (3 gomulu olabilir)";
  EXPECT_NEAR(s.points[0].expected_mean, 2.0, 1e-12);
  EXPECT_NEAR(s.points[0].lower_95, kerteriz::chi_square_quantile(0.025, 6) / 3.0, 1e-12);
}

// -----------------------------------------------------------------------------
// G. Damga sozlesmesi
// -----------------------------------------------------------------------------

TEST(Aggregation, MismatchedTimestampGridIsAnExplicitFailure) {
  // Uzunluklar ESIT ama damgalar farkli: indis eslemesi ilgisiz ornekleri
  // karsilastirirdi. Sessiz hizalama YOK.
  const auto mc = sahte_mc({
      {gozlem(100, 1.0, 3, UpdateStatus::kAccepted), gozlem(200, 1.0, 3, UpdateStatus::kAccepted)},
      {gozlem(100, 1.0, 3, UpdateStatus::kAccepted), gozlem(250, 1.0, 3, UpdateStatus::kAccepted)},
  });
  const auto s = kerteriz_sim::aggregate_nis(mc);

  EXPECT_FALSE(s.status.ok) << "uyusmayan damga izgarasi sessizce hizalandi";
  EXPECT_NE(s.status.message.find("250"), std::string::npos) << s.status.message;
  EXPECT_TRUE(s.points.empty());
}

TEST(Aggregation, DifferentSampleCountIsAnExplicitFailure) {
  const auto mc = sahte_mc({
      {gozlem(100, 1.0, 3, UpdateStatus::kAccepted), gozlem(200, 1.0, 3, UpdateStatus::kAccepted)},
      {gozlem(100, 1.0, 3, UpdateStatus::kAccepted)},
  });
  const auto s = kerteriz_sim::aggregate_nis(mc);
  EXPECT_FALSE(s.status.ok);
  EXPECT_TRUE(s.points.empty());
}

TEST(Aggregation, MatchingGridsAggregateAcrossRunsPerTimestamp) {
  const auto mc = sahte_mc({
      {gozlem(100, 1.0, 3, UpdateStatus::kAccepted), gozlem(200, 5.0, 3, UpdateStatus::kAccepted)},
      {gozlem(100, 3.0, 3, UpdateStatus::kAccepted), gozlem(200, 9.0, 3, UpdateStatus::kAccepted)},
  });
  const auto s = kerteriz_sim::aggregate_nis(mc);

  ASSERT_TRUE(s.status.ok) << s.status.message;
  ASSERT_EQ(s.points.size(), 2U);
  // Her damgada KOSULAR uzerinde ortalama; zaman uzerinde havuzlama YOK.
  EXPECT_EQ(s.points[0].stamp_ns, 100);
  EXPECT_NEAR(s.points[0].mean_nis, 2.0, 1e-12);
  EXPECT_EQ(s.points[1].stamp_ns, 200);
  EXPECT_NEAR(s.points[1].mean_nis, 7.0, 1e-12);
}

// -----------------------------------------------------------------------------
// Gercek zincir uzerinde uctan uca baglanti
// -----------------------------------------------------------------------------

TEST(Aggregation, WorksOnTheRealProductionChainRecords) {
  MonteCarloConfig cfg;
  cfg.scenario = kerteriz_sim::noisy_scenario();
  cfg.scenario.duration_s = 2.0;
  cfg.runs = 4;
  cfg.record_state_samples = true;
  cfg.state_sample_stride = 10;

  const MonteCarloResult mc = kerteriz_sim::run_monte_carlo(cfg);
  const auto nees = kerteriz_sim::aggregate_nees(mc);
  const auto nis = kerteriz_sim::aggregate_nis(mc);

  ASSERT_TRUE(nees.status.ok) << nees.status.message;
  ASSERT_TRUE(nis.status.ok) << nis.status.message;
  ASSERT_FALSE(nees.points.empty());
  ASSERT_FALSE(nis.points.empty());

  EXPECT_EQ(nees.runs, 4);
  for (const auto& p : nees.points) {
    EXPECT_EQ(p.valid_count + p.invalid_count, 4);
    if (p.valid_count > 0) {
      EXPECT_TRUE(std::isfinite(p.mean_nees));
      EXPECT_GT(p.mean_nees, 0.0);
      EXPECT_EQ(p.expected_mean, 15.0);
      EXPECT_LT(p.lower_95, p.upper_95);
    }
  }
  for (const auto& p : nis.points) {
    EXPECT_EQ(p.valid_count + p.undefined_count, 4);
    if (p.valid_count > 0) {
      EXPECT_EQ(p.total_dof, 3 * p.valid_count) << "GnssPosition dof = 3";
      EXPECT_TRUE(std::isfinite(p.mean_nis));
      EXPECT_LT(p.lower_95, p.upper_95);
    }
  }
  // Bu test bir TUTARLILIK HUKMU vermez; yalnizca motorun gercek kayitlar
  // uzerinde calistigini gosterir.
}

} // namespace
