/// \file
/// F2.1 — Monte Carlo kosum altyapisi.
///
/// SINANAN SEY ALTYAPIDIR, ISTATISTIK DEGIL. Burada "kosumlarin %95'i guven
/// bandinda olmali" gibi bir iddia YOKTUR; oyle bir esik birim testinde
/// kararsiz olur ve zaten F2.2/F2.3'un isidir. Burada yalnizca sunlar
/// dogrulanir: tohum turetme kurali, determinizm, on ek kararliligi, akis
/// bagimsizligi ve kayit sozlesmesi.

#include "kerteriz_sim/monte_carlo.hpp"

#include <gtest/gtest.h>
#include <set>
#include <vector>

namespace {

using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::UpdateStatus;
using kerteriz_sim::derive_seeds;
using kerteriz_sim::MonteCarloConfig;
using kerteriz_sim::MonteCarloResult;
using kerteriz_sim::RunResult;
using kerteriz_sim::RunSeeds;
using kerteriz_sim::ScenarioConfig;

constexpr std::uint64_t kAna = 20260916ULL;

/// Testler hizli kalsin diye KISA senaryo. SPEC E1'in 500 kosumluk uretim
/// deneyi F2.3'tur ve birim test yuku DEGILDIR.
ScenarioConfig kisa_senaryo() {
  ScenarioConfig c = kerteriz_sim::noisy_scenario();
  c.duration_s = 3.0;
  return c;
}

MonteCarloConfig mc(int runs, std::uint64_t master = kAna) {
  MonteCarloConfig c;
  c.scenario = kisa_senaryo();
  c.runs = runs;
  c.master_seed = master;
  return c;
}

/// Iki kosumun BIT-BIREBIR ayni olup olmadigi.
void birebir_ayni(const RunResult& a, const RunResult& b, const char* nerede) {
  EXPECT_EQ(a.seeds.imu, b.seeds.imu) << nerede;
  EXPECT_EQ(a.seeds.gnss, b.seeds.gnss) << nerede;
  EXPECT_EQ(a.counters.total, b.counters.total) << nerede;
  EXPECT_EQ(a.counters.accepted, b.counters.accepted) << nerede;
  EXPECT_EQ(a.counters.rejected, b.counters.rejected) << nerede;
  EXPECT_EQ(a.counters.numerical_failures, b.counters.numerical_failures) << nerede;
  EXPECT_EQ(a.nis_sum, b.nis_sum) << nerede;
  EXPECT_EQ(a.nis_min, b.nis_min) << nerede;
  EXPECT_EQ(a.nis_max, b.nis_max) << nerede;
  EXPECT_EQ(a.final_stamp_ns, b.final_stamp_ns) << nerede;
  EXPECT_EQ(a.final_state.minus(b.final_state).norm(), 0.0) << nerede << ": durum";
  EXPECT_EQ((a.final_covariance - b.final_covariance).cwiseAbs().maxCoeff(), 0.0)
      << nerede << ": kovaryans";
}

// -----------------------------------------------------------------------------
// Tohum turetme
// -----------------------------------------------------------------------------

TEST(MonteCarloSeeds, DerivationIsAPureFunctionOfMasterAndRunIndex) {
  // Tohum ureteci durumsuzdur: ayni girdi her cagride ayni ciktiyi verir ve
  // cagri sirasi onemsizdir. Sirayla uretilen bir akis olsaydi bu tutmazdi.
  for (int i = 0; i < 6; ++i) {
    const RunSeeds a = derive_seeds(kAna, i);
    const RunSeeds b = derive_seeds(kAna, i);
    EXPECT_EQ(a.imu, b.imu) << "i = " << i;
    EXPECT_EQ(a.gnss, b.gnss) << "i = " << i;
  }
  // Ters sirada sorulunca da ayni.
  EXPECT_EQ(derive_seeds(kAna, 5).imu, derive_seeds(kAna, 5).imu);
}

TEST(MonteCarloSeeds, ImuAndGnssStreamsAreIndependentlySeeded) {
  // Biri digerinin devami DEGILDIR: farkli alan tuzlari kullanilir.
  std::set<std::uint64_t> hepsi;
  for (int i = 0; i < 32; ++i) {
    const RunSeeds s = derive_seeds(kAna, i);
    EXPECT_NE(s.imu, s.gnss) << "i = " << i << ": IMU ve GNSS tohumu ayni";
    hepsi.insert(s.imu);
    hepsi.insert(s.gnss);
  }
  EXPECT_EQ(hepsi.size(), 64U) << "tohumlar cakisiyor";
}

TEST(MonteCarloSeeds, DifferentMasterSeedsGiveDifferentRunSeeds) {
  for (int i = 0; i < 8; ++i) {
    EXPECT_NE(derive_seeds(kAna, i).imu, derive_seeds(kAna + 1, i).imu) << "i = " << i;
    EXPECT_NE(derive_seeds(kAna, i).gnss, derive_seeds(kAna + 1, i).gnss) << "i = " << i;
  }
}

// -----------------------------------------------------------------------------
// Determinizm ve on ek kararliligi
// -----------------------------------------------------------------------------

TEST(MonteCarlo, SameMasterSeedProducesIdenticalOutput) {
  const MonteCarloResult a = kerteriz_sim::run_monte_carlo(mc(3));
  const MonteCarloResult b = kerteriz_sim::run_monte_carlo(mc(3));

  ASSERT_EQ(a.runs.size(), 3U);
  ASSERT_EQ(a.runs.size(), b.runs.size());
  EXPECT_EQ(a.master_seed, b.master_seed);
  for (std::size_t i = 0; i < a.runs.size(); ++i) {
    birebir_ayni(a.runs[i], b.runs[i], "ayni ana tohum");
  }
}

TEST(MonteCarlo, GrowingNDoesNotChangeEarlierRuns) {
  // ON EK KARARLILIGI. N'i buyutmek eski kosumlari degistirseydi birikmis
  // sonuclar gecersiz olurdu ve deney "N'i artir, bak ne oluyor" ile
  // kirletilebilirdi.
  const MonteCarloResult kisa = kerteriz_sim::run_monte_carlo(mc(3));
  const MonteCarloResult uzun = kerteriz_sim::run_monte_carlo(mc(5));

  ASSERT_EQ(kisa.runs.size(), 3U);
  ASSERT_EQ(uzun.runs.size(), 5U);
  for (std::size_t i = 0; i < kisa.runs.size(); ++i) {
    birebir_ayni(kisa.runs[i], uzun.runs[i], "on ek kararliligi");
  }
}

TEST(MonteCarlo, DifferentMasterSeedChangesStochasticOutput) {
  // Esitlik testlerinin ANLAMLI oldugunu gosterir.
  const MonteCarloResult a = kerteriz_sim::run_monte_carlo(mc(3, kAna));
  const MonteCarloResult b = kerteriz_sim::run_monte_carlo(mc(3, kAna + 1));

  ASSERT_EQ(a.runs.size(), b.runs.size());
  int farkli = 0;
  for (std::size_t i = 0; i < a.runs.size(); ++i) {
    EXPECT_NE(a.runs[i].seeds.imu, b.runs[i].seeds.imu) << "i = " << i;
    if (a.runs[i].nis_sum != b.runs[i].nis_sum) {
      ++farkli;
    }
  }
  EXPECT_EQ(farkli, static_cast<int>(a.runs.size())) << "ana tohum degisti ama sonuc ayni";
}

TEST(MonteCarlo, RunIndexIsExplicitAndOrdered) {
  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(mc(4));
  ASSERT_EQ(r.runs.size(), 4U);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(r.runs[static_cast<std::size_t>(i)].run_index, i);
    EXPECT_EQ(r.runs[static_cast<std::size_t>(i)].seeds.imu, derive_seeds(kAna, i).imu);
    EXPECT_EQ(r.runs[static_cast<std::size_t>(i)].seeds.gnss, derive_seeds(kAna, i).gnss);
  }
}

TEST(MonteCarlo, ZeroOrNegativeRunCountYieldsEmptyResult) {
  EXPECT_TRUE(kerteriz_sim::run_monte_carlo(mc(0)).runs.empty());
  EXPECT_TRUE(kerteriz_sim::run_monte_carlo(mc(-3)).runs.empty());
}

// -----------------------------------------------------------------------------
// Kayit sozlesmesi
// -----------------------------------------------------------------------------

TEST(MonteCarlo, RecordedSamplesAreFiniteSymmetricAndOrdered) {
  MonteCarloConfig cfg = mc(2);
  cfg.record_state_samples = true;
  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(cfg);

  ASSERT_EQ(r.runs.size(), 2U);
  for (const RunResult& kosu : r.runs) {
    ASSERT_FALSE(kosu.samples.empty()) << "ornek kaydi istendi ama uretilmedi";

    TimeNs onceki = -1;
    for (const auto& s : kosu.samples) {
      EXPECT_GT(s.stamp_ns, onceki) << "damgalar kesin artan degil";
      onceki = s.stamp_ns;

      EXPECT_EQ(s.active_dof, kerteriz::kCoreDof);
      EXPECT_TRUE(s.truth.extended_pose().coeffs().allFinite());
      EXPECT_TRUE(s.estimate.extended_pose().coeffs().allFinite());
      EXPECT_TRUE(s.estimate.gyro_bias().allFinite());
      EXPECT_TRUE(s.estimate.accel_bias().allFinite());
      EXPECT_TRUE(s.covariance.allFinite()) << "kovaryans sonlu degil";
      EXPECT_LT((s.covariance - s.covariance.transpose()).cwiseAbs().maxCoeff(), 1e-9)
          << "kovaryans simetrisi bozuk";
      EXPECT_GT(s.covariance.diagonal().minCoeff(), 0.0) << "kovaryans kosegeni pozitif degil";
    }
  }
}

// -----------------------------------------------------------------------------
// Gercek durum 15 DoF'tur: bias'lar da GERCEK simule edilen degerlerdir
// -----------------------------------------------------------------------------

/// Sifirdan farkli, deterministik baslangic bias'lari.
kerteriz::Vec3 kJiroBias(0.004, -0.002, 0.007);
kerteriz::Vec3 kIvmeBias(-0.030, 0.050, 0.010);

ScenarioConfig bias_senaryosu(bool yuruyus) {
  ScenarioConfig c = kisa_senaryo();
  c.imu_sim.initial_gyro_bias = kJiroBias;
  c.imu_sim.initial_accel_bias = kIvmeBias;
  if (!yuruyus) {
    c.imu_sim.gyro_bias_walk = 0.0;
    c.imu_sim.accel_bias_walk = 0.0;
  }
  return c;
}

TEST(MonteCarloTruth, ConstantBiasAppearsExactlyInEveryTruthSample) {
  // Yuruyus SIFIR: gercek bias tum kosu boyunca SABIT ve yapilandirilan degere
  // TAM esit olmalidir. Kayit bias'i sifir biraksaydi bu test duserdi.
  MonteCarloConfig cfg = mc(1);
  cfg.scenario = bias_senaryosu(/*yuruyus=*/false);
  cfg.record_state_samples = true;

  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(cfg);
  ASSERT_EQ(r.runs.size(), 1U);
  ASSERT_FALSE(r.runs[0].samples.empty());

  for (const auto& s : r.runs[0].samples) {
    EXPECT_EQ((s.truth.gyro_bias() - kJiroBias).cwiseAbs().maxCoeff(), 0.0)
        << "gercek jiro bias'i kayitta yok veya yanlis, damga " << s.stamp_ns;
    EXPECT_EQ((s.truth.accel_bias() - kIvmeBias).cwiseAbs().maxCoeff(), 0.0)
        << "gercek ivme bias'i kayitta yok veya yanlis, damga " << s.stamp_ns;
  }
}

TEST(MonteCarloTruth, InitialEstimateIsTheConfiguredBiasPlusTheInitialError) {
  // t = 0'daki GERCEK durum, sentezleyicinin BASLANGIC bias'larini tasimali;
  // baslangic hatasi onun UZERINE uygulanir. Sifir varsayilsaydi kestirimin
  // bias'i gercek degerden tam olarak kJiroBias kadar fazladan saparadi ve bu
  // sapma hicbir yerde gorunmezdi.
  //
  // Yuruyus SIFIR ve ilk GNSS guncellemesi 0.2 s'de gelir; yayilim bias'i
  // nominal olarak DEGISTIRMEZ. Dolayisiyla ilk ornekte hatanin bias blogu
  // TAM OLARAK yapilandirilan baslangic hatasi olmalidir.
  MonteCarloConfig cfg = mc(1);
  cfg.scenario = bias_senaryosu(/*yuruyus=*/false);
  cfg.record_state_samples = true;

  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(cfg);
  ASSERT_EQ(r.runs.size(), 1U);
  ASSERT_FALSE(r.runs[0].samples.empty());

  const auto& ilk = r.runs[0].samples.front();
  const kerteriz::Vec3 jiro_hatasi = cfg.scenario.initial_error.segment<3>(9);
  const kerteriz::Vec3 ivme_hatasi = cfg.scenario.initial_error.segment<3>(12);

  EXPECT_EQ((ilk.estimate.gyro_bias() - (kJiroBias + jiro_hatasi)).cwiseAbs().maxCoeff(), 0.0)
      << "baslangic kestirimi yapilandirilan gercek bias uzerine kurulmamis";
  EXPECT_EQ((ilk.estimate.accel_bias() - (kIvmeBias + ivme_hatasi)).cwiseAbs().maxCoeff(), 0.0)
      << "baslangic kestirimi yapilandirilan gercek bias uzerine kurulmamis";

  const kerteriz::StateVec e = ilk.estimate.minus(ilk.truth);
  EXPECT_EQ((e.segment<3>(9) - jiro_hatasi).cwiseAbs().maxCoeff(), 0.0)
      << "jiro bias hatasi yapilandirilan baslangic hatasina esit degil";
  EXPECT_EQ((e.segment<3>(12) - ivme_hatasi).cwiseAbs().maxCoeff(), 0.0)
      << "ivme bias hatasi yapilandirilan baslangic hatasina esit degil";
  // Kurulum kontrolu: yapilandirilan bias sifir olsaydi bu test bos gecerdi.
  ASSERT_GT(kJiroBias.norm(), 0.0);
  ASSERT_GT(kIvmeBias.norm(), 0.0);
}

TEST(MonteCarloTruth, RandomWalkMakesLaterTruthBiasDifferFromInitial) {
  // Yuruyus ACIK: gercek bias zamanla KAYAR. Kayit yalnizca baslangic degerini
  // tutsaydi (veya sifir birakssaydi) bu test duserdi.
  MonteCarloConfig cfg = mc(1);
  cfg.scenario = bias_senaryosu(/*yuruyus=*/true);
  cfg.record_state_samples = true;

  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(cfg);
  ASSERT_EQ(r.runs.size(), 1U);
  const auto& ornekler = r.runs[0].samples;
  ASSERT_GT(ornekler.size(), 10U);

  EXPECT_GT((ornekler.back().truth.gyro_bias() - kJiroBias).norm(), 0.0)
      << "jiro bias'i hic yurumemis";
  EXPECT_GT((ornekler.back().truth.accel_bias() - kIvmeBias).norm(), 0.0)
      << "ivme bias'i hic yurumemis";

  // Ilk ornek zaten BIR adim yurumustur: sentezleyici olcumu uretmeden once
  // bias'i ilerletir.
  EXPECT_NE(ornekler.front().truth.gyro_bias(), ornekler.back().truth.gyro_bias());
}

TEST(MonteCarloTruth, RecordedBiasMatchesTheSynthesizerThatProducedThatSample) {
  // Kaydedilen bias, O DAMGANIN olcumunu ureten bias'in TA KENDISI mi?
  //
  // Rastgele yuruyus denklemi burada TEKRARLANMAZ — dogrulugun kaynagi yine
  // ImuSynthesizer'dir. Ayni tohum ve ayni dt dizisiyle BAGIMSIZ bir
  // sentezleyici adim adim kosturulur; her adimdan SONRAKI bias'i, kayittaki
  // ile karsilastirilir. Kayit bir adim geride (sample() oncesi) alinmis olsaydi
  // bu test duserdi.
  MonteCarloConfig cfg = mc(1);
  cfg.scenario = bias_senaryosu(/*yuruyus=*/true);
  cfg.record_state_samples = true;

  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(cfg);
  ASSERT_EQ(r.runs.size(), 1U);
  const auto& ornekler = r.runs[0].samples;
  ASSERT_FALSE(ornekler.empty());

  const ScenarioConfig& sc = cfg.scenario;
  kerteriz_sim::SeededRng rng(kerteriz_sim::derive_seeds(kAna, 0).imu);
  kerteriz_sim::ImuSynthesizer ayna(sc.imu_sim, rng);
  kerteriz_sim::TrajectoryGenerator yorunge(sc.trajectory);

  const auto adim_sayisi =
      static_cast<int>(sc.duration_s * static_cast<Scalar>(kerteriz_sim::kNanosecondsPerSecond) /
                       static_cast<Scalar>(sc.imu_period_ns));
  TimeNs onceki = 0;
  std::size_t i = 0;
  for (int k = 1; k <= adim_sayisi; ++k) {
    const TimeNs t = static_cast<TimeNs>(k) * sc.imu_period_ns;
    const Scalar dt = static_cast<Scalar>(t - onceki) * 1e-9;
    ayna.sample(yorunge.at(t), dt); // bias'i ILERLETIR
    onceki = t;

    if (i < ornekler.size() && ornekler[i].stamp_ns == t) {
      EXPECT_EQ((ornekler[i].truth.gyro_bias() - ayna.gyro_bias()).cwiseAbs().maxCoeff(), 0.0)
          << "damga " << t << ": kayittaki jiro bias'i olcumu ureten bias degil";
      EXPECT_EQ((ornekler[i].truth.accel_bias() - ayna.accel_bias()).cwiseAbs().maxCoeff(), 0.0)
          << "damga " << t << ": kayittaki ivme bias'i olcumu ureten bias degil";
      ++i;
    }
  }
  EXPECT_EQ(i, ornekler.size()) << "her kayit damgasi eslesmedi";
}

TEST(MonteCarlo, ErrorIsComputableWithTheProjectsOwnConvention) {
  // F2.2 hatayi NavState::minus ile alacak — elde uydurulmus bir Euler farkiyla
  // DEGIL. Kayit bunu mumkun kiliyor mu, burada dogrulanir.
  //
  // Senaryo SIFIRDAN FARKLI gercek bias tasir: bias bloklari boylece yalnizca
  // "sonlu" degil, FIZIKSEL OLARAK ANLAMLIDIR.
  MonteCarloConfig cfg = mc(1);
  cfg.scenario = bias_senaryosu(/*yuruyus=*/true);
  cfg.record_state_samples = true;
  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(cfg);
  ASSERT_EQ(r.runs.size(), 1U);
  ASSERT_FALSE(r.runs[0].samples.empty());

  for (const auto& s : r.runs[0].samples) {
    const kerteriz::StateVec e = s.estimate.minus(s.truth);
    ASSERT_TRUE(e.allFinite());
    EXPECT_EQ(e.tail(kerteriz::kMaxStateDof - kerteriz::kCoreDof).cwiseAbs().maxCoeff(), 0.0)
        << "augmentation kuyrugu sifir olmali";
    // Gercek bias sifir DEGIL: 15 boyutlu hatanin bias kismi gercek bir
    // buyuklugun farki.
    EXPECT_GT(s.truth.gyro_bias().norm(), 0.0) << "gercek jiro bias'i sifir kalmis";
    EXPECT_GT(s.truth.accel_bias().norm(), 0.0) << "gercek ivme bias'i sifir kalmis";
  }

  // Baslangic hatasi kasitlidir; ilk ornekte hata GERCEKTEN sifirdan farkli
  // olmali, yoksa bu test bos gecerdi.
  const auto& ilk = r.runs[0].samples.front();
  const kerteriz::StateVec e0 = ilk.estimate.minus(ilk.truth);
  EXPECT_GT(e0.norm(), 1e-3);
  // Bias bileseni de sifirdan farkli olmali: filtre bias'lari gercek degerden
  // sapmis durumda baslar (baslangic_hatasi() bias terimleri tasir).
  EXPECT_GT(e0.segment<3>(9).norm(), 1e-6) << "jiro bias hatasi sifir";
  EXPECT_GT(e0.segment<3>(12).norm(), 1e-6) << "ivme bias hatasi sifir";
}

TEST(MonteCarlo, SampleStrideThinsTheRecordWithoutChangingTheFilter) {
  MonteCarloConfig yogun = mc(1);
  yogun.record_state_samples = true;
  MonteCarloConfig seyrek = mc(1);
  seyrek.record_state_samples = true;
  seyrek.state_sample_stride = 10;

  const MonteCarloResult a = kerteriz_sim::run_monte_carlo(yogun);
  const MonteCarloResult b = kerteriz_sim::run_monte_carlo(seyrek);
  ASSERT_EQ(a.runs.size(), 1U);
  ASSERT_EQ(b.runs.size(), 1U);

  EXPECT_LT(b.runs[0].samples.size(), a.runs[0].samples.size());
  // Kayit sikligi FILTREYI ETKILEMEZ.
  birebir_ayni(a.runs[0], b.runs[0], "ornek adimi");
}

TEST(MonteCarlo, NisObservationsCarryDegreesOfFreedom) {
  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(mc(1));
  ASSERT_EQ(r.runs.size(), 1U);
  const RunResult& kosu = r.runs[0];

  ASSERT_FALSE(kosu.nis_observations.empty()) << "NIS gozlemi kaydedilmemis";
  EXPECT_EQ(static_cast<int>(kosu.nis_observations.size()), kosu.counters.total);

  TimeNs onceki = -1;
  for (const auto& g : kosu.nis_observations) {
    EXPECT_EQ(g.dof, 3) << "GnssPosition artik boyutu 3 olmali";
    EXPECT_GT(g.stamp_ns, onceki) << "NIS gozlemleri sirali degil";
    onceki = g.stamp_ns;
    if (g.status == UpdateStatus::kNumericalFailure) {
      EXPECT_FALSE(g.has_nis) << "ADR-24: sayisal basarisizlikta NIS tanimsizdir";
    } else {
      EXPECT_TRUE(g.has_nis);
      EXPECT_GE(g.nis, 0.0);
    }
  }
}

TEST(MonteCarlo, NumericalFailuresAreReportedNotSilentlyDropped) {
  const MonteCarloResult r = kerteriz_sim::run_monte_carlo(mc(4));
  ASSERT_EQ(r.runs.size(), 4U) << "kosu sessizce atilmis";

  int toplam = 0;
  int kosu_sayisi = 0;
  for (const RunResult& kosu : r.runs) {
    toplam += kosu.counters.numerical_failures;
    if (kosu.counters.numerical_failures > 0) {
      ++kosu_sayisi;
    }
    // Sayaclar kendi icinde tutarli olmali.
    EXPECT_EQ(kosu.counters.accepted + kosu.counters.rejected + kosu.counters.numerical_failures,
              kosu.counters.total);
  }
  EXPECT_EQ(r.total_numerical_failures, toplam) << "toplam sayisal basarisizlik tutmuyor";
  EXPECT_EQ(r.runs_with_numerical_failure, kosu_sayisi);
}

// -----------------------------------------------------------------------------
// Duman testiyle ayni kod yolu
// -----------------------------------------------------------------------------

TEST(MonteCarlo, SharesTheSameRunnerAsTheSmokeTest) {
  // Duman testi ve Monte Carlo AYNI `run_single`'i cagirir. Ikinci bir filtre
  // dongusu olsaydi ayni tohumlarla ayni sonucu vermeleri garanti olmazdi.
  const RunSeeds s = derive_seeds(kAna, 0);
  const RunResult dogrudan = kerteriz_sim::run_single(kisa_senaryo(), s, 0);
  const MonteCarloResult mcr = kerteriz_sim::run_monte_carlo(mc(1));

  ASSERT_EQ(mcr.runs.size(), 1U);
  birebir_ayni(dogrudan, mcr.runs[0], "dogrudan vs Monte Carlo");
}

} // namespace
