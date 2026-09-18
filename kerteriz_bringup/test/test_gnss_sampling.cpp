/// \file
/// F2.4-C — deterministik GNSS konum seyreltme politikasi.
///
/// Politika SAF ve PAYLASIMLIDIR: hem Kerteriz kosucusu hem harici taban
/// cizgisi yayincisi AYNI fonksiyondan gecer. Iki dosyaya kopyalansaydi
/// "ayni algoritmayi kullandim" bir iddia olarak kalirdi; burada damga
/// kumelerinin BIREBIR esitligi sinaniyor.
///
/// BU BIR AYAR DENEYI DEGILDIR. Filtre matematigi, Q, P0, R ve kapi
/// degismemistir; yalnizca GNSS duzeltme SIKLIGI deterministik olarak azalir.

#include "kerteriz_bringup/gnss_sampling.hpp"

#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

using kerteriz::TimeNs;
using kerteriz_bringup::build_sampling_plan;
using kerteriz_bringup::DatasetEvent;
using kerteriz_bringup::DatasetEventKind;
using kerteriz_bringup::GnssSamplingPolicy;
using kerteriz_bringup::initialization_stamp;
using kerteriz_bringup::SamplingPlan;

constexpr TimeNs kAdim = 100000000; // 100 ms

/// `n` kare: her karede IMU, GNSS konum, GNSS hiz ve referans poz.
/// `interpolated` kume indisleri ara-degerlenmis isaretlenir.
std::vector<DatasetEvent> kareler(int n, const std::vector<int>& interpolated = {}) {
  std::vector<DatasetEvent> out;
  for (int k = 0; k < n; ++k) {
    const bool ara = std::find(interpolated.begin(), interpolated.end(), k) != interpolated.end();
    const TimeNs t = static_cast<TimeNs>(k) * kAdim;
    for (const auto kind : {DatasetEventKind::kImu, DatasetEventKind::kGnssPosition,
                            DatasetEventKind::kGnssVelocity, DatasetEventKind::kReferencePose}) {
      DatasetEvent e;
      e.stamp_ns = t;
      e.kind = kind;
      e.source_interpolated = ara;
      out.push_back(e);
    }
  }
  return out;
}

SamplingPlan plan(const std::vector<DatasetEvent>& ev, int stride, bool enabled = true) {
  GnssSamplingPolicy p;
  p.enabled = enabled;
  p.stride = stride;
  return build_sampling_plan(ev, p);
}

// -----------------------------------------------------------------------------
// Baslatma damgasi
// -----------------------------------------------------------------------------

TEST(GnssSampling, InitializationStampIsFirstNonInterpolatedReference) {
  bool ok = false;
  // Ilk iki kare ara-degerlenmis: t0 ucuncu karedir.
  EXPECT_EQ(initialization_stamp(kareler(5, {0, 1}), ok), 2 * kAdim);
  EXPECT_TRUE(ok);

  initialization_stamp({}, ok);
  EXPECT_FALSE(ok) << "bos girdi sessizce 0 dondurmemeli";
}

// -----------------------------------------------------------------------------
// 1-2. Ordinal secimi ve t0
// -----------------------------------------------------------------------------

TEST(GnssSampling, SelectsExactlyEveryStrideOrdinalAfterInit) {
  // 31 kare -> t0 = kare 0, ondan SONRA 30 aday (ordinal 1..30).
  const auto p = plan(kareler(31), 10);
  ASSERT_TRUE(p.status.ok) << p.status.message;
  EXPECT_EQ(p.init_stamp_ns, 0);
  EXPECT_EQ(p.candidate_count, 30);

  // ordinal 10, 20, 30 -> kare 10, 20, 30
  const std::vector<TimeNs> beklenen{10 * kAdim, 20 * kAdim, 30 * kAdim};
  EXPECT_EQ(p.selected_slot_stamps, beklenen);
  EXPECT_EQ(p.measurement_stamps, beklenen);
  EXPECT_EQ(p.selected_slot_count, 3);
  EXPECT_EQ(p.selected_usable_count, 3);
  EXPECT_EQ(p.selected_interpolated_skipped, 0);
}

TEST(GnssSampling, InitFrameIsNeverASelectedMeasurement) {
  for (const int stride : {1, 2, 10}) {
    const auto p = plan(kareler(25), stride);
    ASSERT_TRUE(p.status.ok);
    for (const TimeNs t : p.measurement_stamps) {
      EXPECT_GT(t, p.init_stamp_ns) << "t0 olcum olarak secilmis (stride " << stride << ")";
    }
    for (const TimeNs t : p.selected_slot_stamps) {
      EXPECT_GT(t, p.init_stamp_ns);
    }
  }
}

// -----------------------------------------------------------------------------
// 3. Ara-degerlenmis secili slot FAZI KAYDIRMAZ
// -----------------------------------------------------------------------------

TEST(GnssSampling, InterpolatedSelectedSlotDoesNotShiftThePhase) {
  // Ordinal 10 (kare 10) ara-degerlenmis. Ordinal 11 (kare 11) normal.
  // Beklenen: 11 OLCUM DEGIL; bir sonraki secili slot yine ordinal 20.
  const auto p = plan(kareler(31, {10}), 10);
  ASSERT_TRUE(p.status.ok);

  EXPECT_EQ(p.selected_slot_stamps, (std::vector<TimeNs>{10 * kAdim, 20 * kAdim, 30 * kAdim}))
      << "faz kaymis";
  EXPECT_EQ(p.measurement_stamps, (std::vector<TimeNs>{20 * kAdim, 30 * kAdim}))
      << "ara-degerlenmis slot olcum olarak kullanilmis veya yerine 11 secilmis";
  EXPECT_EQ(p.selected_interpolated_skipped, 1);
  EXPECT_EQ(p.selected_usable_count, 2);

  for (const TimeNs t : p.measurement_stamps) {
    EXPECT_NE(t, 11 * kAdim) << "ara-degerlenmis slotun yerine sonraki kare secilmis";
  }
}

TEST(GnssSampling, OrdinalCounterAdvancesThroughInterpolatedCandidates) {
  // Ordinal sayaci ara-degerlenmis adaylarda da ILERLER. Ilerlemeseydi kare 11
  // ordinal 10 olur ve secilirdi.
  const auto p = plan(kareler(31, {5}), 10);
  ASSERT_TRUE(p.status.ok);
  EXPECT_EQ(p.candidate_count, 30);
  EXPECT_EQ(p.selected_slot_stamps, (std::vector<TimeNs>{10 * kAdim, 20 * kAdim, 30 * kAdim}));
}

// -----------------------------------------------------------------------------
// 4-5. Determinizm
// -----------------------------------------------------------------------------

TEST(GnssSampling, SameEventListGivesSameSelection) {
  const auto ev = kareler(45, {3, 20, 21});
  const auto a = plan(ev, 10);
  const auto b = plan(ev, 10);
  EXPECT_EQ(a.selected_slot_stamps, b.selected_slot_stamps);
  EXPECT_EQ(a.measurement_stamps, b.measurement_stamps);
  EXPECT_EQ(a.withheld_reference_stamps, b.withheld_reference_stamps);
}

TEST(GnssSampling, InvalidStrideIsAnExplicitError) {
  for (const int stride : {0, -1, -10}) {
    const auto p = plan(kareler(10), stride);
    EXPECT_FALSE(p.status.ok) << "stride " << stride << " sessizce duzeltilmis";
    EXPECT_TRUE(p.measurement_stamps.empty());
  }
}

// -----------------------------------------------------------------------------
// 6. Legacy — seyreltme KAPALI
// -----------------------------------------------------------------------------

TEST(GnssSampling, DisabledPolicySelectsEveryPostInitCandidate) {
  const auto p = plan(kareler(20, {7}), /*stride=*/10, /*enabled=*/false);
  ASSERT_TRUE(p.status.ok);
  EXPECT_EQ(p.candidate_count, 19);
  EXPECT_EQ(p.selected_slot_count, 19) << "kapaliyken tum adaylar secili sayilmali";
  EXPECT_EQ(p.selected_usable_count, 18) << "ara-degerlenmis olan olcum olamaz";
  EXPECT_EQ(p.selected_interpolated_skipped, 1);
  EXPECT_TRUE(p.withheld_reference_stamps.empty()) << "legacy modda withheld kume BOS olmali";
}

// -----------------------------------------------------------------------------
// 9-10. Withheld referans
// -----------------------------------------------------------------------------

TEST(GnssSampling, MeasurementAndWithheldSetsAreDisjoint) {
  const auto p = plan(kareler(45, {4, 20, 33}), 10);
  ASSERT_TRUE(p.status.ok);
  ASSERT_FALSE(p.measurement_stamps.empty());
  ASSERT_FALSE(p.withheld_reference_stamps.empty());

  for (const TimeNs t : p.measurement_stamps) {
    EXPECT_EQ(std::count(p.withheld_reference_stamps.begin(), p.withheld_reference_stamps.end(), t),
              0)
        << "damga hem olcum hem withheld: " << t;
  }
}

TEST(GnssSampling, EveryEligibleReferenceIsEitherSelectedOrWithheld) {
  const std::vector<int> ara{4, 20, 33};
  const auto ev = kareler(45, ara);
  const auto p = plan(ev, 10);
  ASSERT_TRUE(p.status.ok);

  int uygun = 0;
  for (const auto& e : ev) {
    if (e.kind != DatasetEventKind::kReferencePose) {
      continue;
    }
    if (e.stamp_ns <= p.init_stamp_ns || e.source_interpolated) {
      continue;
    }
    ++uygun;
    const bool secili =
        std::count(p.selected_slot_stamps.begin(), p.selected_slot_stamps.end(), e.stamp_ns) > 0;
    const bool withheld = std::count(p.withheld_reference_stamps.begin(),
                                     p.withheld_reference_stamps.end(), e.stamp_ns) > 0;
    EXPECT_NE(secili, withheld) << "damga " << e.stamp_ns << " ikisi de veya hicbiri";
  }
  EXPECT_EQ(uygun, p.selected_usable_count + p.withheld_reference_count)
      << "uygun referanslar tam olarak ikiye bolunmemis";
  EXPECT_EQ(p.withheld_reference_count, static_cast<int>(p.withheld_reference_stamps.size()));
}

TEST(GnssSampling, InterpolatedFramesAreNeitherMeasurementNorWithheld) {
  const auto p = plan(kareler(31, {10, 17}), 10);
  ASSERT_TRUE(p.status.ok);
  for (const TimeNs t : {10 * kAdim, 17 * kAdim}) {
    EXPECT_EQ(std::count(p.measurement_stamps.begin(), p.measurement_stamps.end(), t), 0);
    EXPECT_EQ(std::count(p.withheld_reference_stamps.begin(), p.withheld_reference_stamps.end(), t),
              0);
  }
}

TEST(GnssSampling, CandidateRateIsDerivedFromTheSequenceNotHardCoded) {
  // Her KITTI dizisinin ornekleme araligi farklidir; tek bir diziden olculmus
  // sabit bir Hz gomulseydi coklu dizi kosusunda yanlis deger raporlanirdi.
  const auto p = plan(kareler(21), 10); // 100 ms adim -> 10 Hz
  ASSERT_TRUE(p.status.ok);
  EXPECT_EQ(p.candidate_count, 20);
  EXPECT_EQ(p.candidate_span_ns, 19 * kAdim);
  EXPECT_NEAR(kerteriz_bringup::candidate_rate_hz(p), 10.0, 1e-9);

  // Bos ve tek adayli durumlarda 0 doner, bolme yapilmaz.
  SamplingPlan bos;
  EXPECT_EQ(kerteriz_bringup::candidate_rate_hz(bos), 0.0);
}

} // namespace
