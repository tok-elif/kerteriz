/// \file
/// F1.1 — NavState. INTERFACES §1 · CONVENTIONS §3.1, §5.

#include "kerteriz/state/nav_state.hpp"

#include <Eigen/Geometry>
#include <cstdlib>
#include <gtest/gtest.h>
#include <new>
#include <type_traits>

namespace {

using kerteriz::AugmentKind;
using kerteriz::CloneId;
using kerteriz::exp_map;
using kerteriz::kCloneDof;
using kerteriz::kCoreDof;
using kerteriz::kInvalidClone;
using kerteriz::kMaxAugmentDof;
using kerteriz::kMaxStateDof;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::SE23;
using kerteriz::StateVec;
using kerteriz::TangentVec;
using kerteriz::Vec3;

TangentVec ornek_tanjant() {
  TangentVec d;
  d << 0.10, -0.20, 0.30, 1.10, -0.40, 0.70, -2.30, 0.90, 1.70;
  return d;
}

NavState ornek_durum() {
  NavState x;
  x.extended_pose() = exp_map(ornek_tanjant());
  x.gyro_bias() = Vec3(0.01, -0.02, 0.03);
  x.accel_bias() = Vec3(-0.4, 0.5, -0.6);
  return x;
}

/// Aktif boyuta gore dolu, kuyrugu sifir bir delta.
StateVec delta_uret(int aktif, Scalar olcek = 1.0) {
  StateVec d = StateVec::Zero();
  for (int i = 0; i < aktif; ++i) {
    d[i] = olcek * (0.01 * Scalar(i + 1) - 0.13);
  }
  return d;
}

// -----------------------------------------------------------------------------
// Cekirdek
// -----------------------------------------------------------------------------

TEST(NavState, DefaultStateHasCoreDofOnly) {
  const NavState x;
  EXPECT_EQ(x.active_dof(), kCoreDof);
  EXPECT_EQ(x.active_dof(), 15);
  EXPECT_EQ(x.augment_dof(), 0);
}

TEST(NavState, DefaultCoreIsIdentityAndZeroBias) {
  const NavState x;
  EXPECT_LT(kerteriz::log_map(x.extended_pose()).norm(), 1e-15);
  EXPECT_LT(x.gyro_bias().norm(), 1e-15);
  EXPECT_LT(x.accel_bias().norm(), 1e-15);
}

TEST(NavState, BiasAccessorsReadAndWriteIndependently) {
  NavState x;
  x.gyro_bias() = Vec3(1.0, 2.0, 3.0);
  x.accel_bias() = Vec3(-4.0, -5.0, -6.0);

  EXPECT_TRUE(x.gyro_bias().isApprox(Vec3(1.0, 2.0, 3.0)));
  EXPECT_TRUE(x.accel_bias().isApprox(Vec3(-4.0, -5.0, -6.0)));

  const NavState& sabit = x;
  EXPECT_TRUE(sabit.gyro_bias().isApprox(Vec3(1.0, 2.0, 3.0)));
}

// -----------------------------------------------------------------------------
// plus / minus — CONVENTIONS §3.1 sag perturbasyon
// -----------------------------------------------------------------------------

TEST(NavState, PlusMinusRoundTrip) {
  const NavState x = ornek_durum();
  const StateVec d = delta_uret(kCoreDof);

  const NavState y = x.plus(d);
  const StateVec geri = y.minus(x);

  for (int i = 0; i < kCoreDof; ++i) {
    EXPECT_NEAR(geri[i], d[i], 1e-9) << "bilesen " << i;
  }
}

TEST(NavState, MinusOfSelfIsZero) {
  const NavState x = ornek_durum();
  const StateVec d = x.minus(x);
  EXPECT_LT(d.norm(), 1e-12);
}

TEST(NavState, PlusUsesRightPerturbationOnExtendedPose) {
  // CONVENTIONS §3.1: X (+) d = X o Exp(d). Sol perturbasyon Exp(d) o X
  // olurdu ve bu test duserdi.
  NavState x;
  x.extended_pose() = exp_map(ornek_tanjant());

  StateVec d = StateVec::Zero();
  d.head(9) << 0.05, -0.07, 0.11, 0.20, -0.30, 0.40, 0.60, -0.50, 0.25;

  const NavState y = x.plus(d);
  const SE23 sag = kerteriz::plus(x.extended_pose(), TangentVec(d.head(9)));
  const SE23 sol = kerteriz::exp_map(TangentVec(d.head(9))).compose(x.extended_pose());

  EXPECT_LT(kerteriz::minus(y.extended_pose(), sag).norm(), 1e-12);
  EXPECT_GT(kerteriz::minus(y.extended_pose(), sol).norm(), 1e-3) << "sol perturbasyon kullanilmis";
}

TEST(NavState, TangentOrderMatchesConventions) {
  // CONVENTIONS §5.1: [dtheta(0..2) dv(3..5) dp(6..8) db_g(9..11) db_a(12..14)]
  const NavState x;

  StateVec d = StateVec::Zero();
  d[9] = 0.7; // db_g x
  EXPECT_NEAR(x.plus(d).gyro_bias().x(), 0.7, 1e-15);
  EXPECT_LT(x.plus(d).accel_bias().norm(), 1e-15);

  d.setZero();
  d[12] = -0.9; // db_a x
  EXPECT_NEAR(x.plus(d).accel_bias().x(), -0.9, 1e-15);
  EXPECT_LT(x.plus(d).gyro_bias().norm(), 1e-15);

  // dp bilesenleri konumu tasir, dv hizi.
  d.setZero();
  d[kerteriz::kIdxPos] = 2.0;
  EXPECT_NEAR(x.plus(d).extended_pose().translation().x(), 2.0, 1e-12);
  EXPECT_LT(x.plus(d).extended_pose().linearVelocity().norm(), 1e-12);

  d.setZero();
  d[kerteriz::kIdxVel] = 3.0;
  EXPECT_NEAR(x.plus(d).extended_pose().linearVelocity().x(), 3.0, 1e-12);
  EXPECT_LT(x.plus(d).extended_pose().translation().norm(), 1e-12);
}

TEST(NavState, PlusIgnoresUnusedTail) {
  // INTERFACES §1: plus() kuyrugu yok sayar.
  const NavState x = ornek_durum();

  StateVec temiz = delta_uret(kCoreDof);
  StateVec kirli = temiz;
  for (int i = kCoreDof; i < kMaxStateDof; ++i) {
    kirli[i] = 1234.5;
  }

  const StateVec a = x.plus(temiz).minus(x);
  const StateVec b = x.plus(kirli).minus(x);
  EXPECT_LT((a - b).norm(), 1e-15) << "kuyruk plus() sonucunu etkiledi";
}

TEST(NavState, MinusZeroesUnusedTail) {
  // INTERFACES §1: minus() kuyrugu sifirlar.
  const NavState x = ornek_durum();
  const NavState y = x.plus(delta_uret(kCoreDof));

  const StateVec d = y.minus(x);
  EXPECT_LT(d.tail(kMaxStateDof - kCoreDof).cwiseAbs().maxCoeff(), 0.0 + 1e-18)
      << "kuyruk sifirlanmadi";
}

// -----------------------------------------------------------------------------
// Kalici kalibrasyon
// -----------------------------------------------------------------------------

TEST(NavState, RegisterCalibrationReturnsOffsetAfterCore) {
  NavState x;
  const auto ofs = x.register_calibration("wheel_scale", 1);
  ASSERT_TRUE(ofs.has_value());
  EXPECT_EQ(*ofs, kCoreDof);
  EXPECT_EQ(x.calibration_offset("wheel_scale"), kCoreDof);
  EXPECT_EQ(x.active_dof(), kCoreDof + 1);
  EXPECT_EQ(x.augment_dof(), 1);
}

TEST(NavState, MultipleCalibrationsKeepRegistrationOrder) {
  NavState x;
  const auto a = x.register_calibration("wheel_scale", 1);
  const auto b = x.register_calibration("imu_gnss_extrinsic", 6);
  const auto c = x.register_calibration("time_offset", 1);

  ASSERT_TRUE(a && b && c);
  EXPECT_EQ(*a, kCoreDof);
  EXPECT_EQ(*b, kCoreDof + 1);
  EXPECT_EQ(*c, kCoreDof + 7);
  EXPECT_EQ(x.augment_dof(), 8);
  EXPECT_EQ(x.active_dof(), kCoreDof + 8);
}

TEST(NavState, DuplicateCalibrationNameIsRejected) {
  // Sozlesme: isimler benzersizdir. Mukerrer kayit yapilandirma hatasidir ve
  // mevcut kaydi DEGISTIRMEZ.
  NavState x;
  const auto ilk = x.register_calibration("wheel_scale", 1);
  ASSERT_TRUE(ilk.has_value());
  const int onceki_dof = x.active_dof();

  EXPECT_FALSE(x.register_calibration("wheel_scale", 1).has_value()) << "ayni dof";
  EXPECT_FALSE(x.register_calibration("wheel_scale", 3).has_value()) << "farkli dof";

  EXPECT_EQ(x.active_dof(), onceki_dof) << "reddedilen kayit boyutu degistirdi";
  EXPECT_EQ(x.calibration_offset("wheel_scale"), *ilk) << "mevcut ofset degisti";
}

TEST(NavState, CalibrationRejectedWhenCapacityExceeded) {
  // CONVENTIONS §5.3: kapasite asilirsa yapilandirma basarisiz olur,
  // sessizce kirpilmaz.
  NavState x;
  ASSERT_TRUE(x.register_calibration("buyuk", kMaxAugmentDof).has_value());
  EXPECT_EQ(x.augment_dof(), kMaxAugmentDof);

  EXPECT_FALSE(x.register_calibration("tasan", 1).has_value());
  EXPECT_EQ(x.augment_dof(), kMaxAugmentDof) << "sessiz truncation";
  EXPECT_EQ(x.active_dof(), kMaxStateDof);
}

TEST(NavState, CalibrationLargerThanCapacityIsRejectedOutright) {
  NavState x;
  EXPECT_FALSE(x.register_calibration("cok_buyuk", kMaxAugmentDof + 1).has_value());
  EXPECT_EQ(x.augment_dof(), 0);
}

// -----------------------------------------------------------------------------
// Klonlar
// -----------------------------------------------------------------------------

TEST(NavState, PushCloneAllocatesCloneDofAfterCore) {
  NavState x;
  const CloneId id = x.push_clone();

  EXPECT_NE(id, kInvalidClone);
  EXPECT_TRUE(x.has_clone(id));
  EXPECT_EQ(x.clone_offset(id), kCoreDof);
  EXPECT_EQ(x.augment_dof(), kCloneDof);
  EXPECT_EQ(x.active_dof(), kCoreDof + kCloneDof);
}

TEST(NavState, UnknownCloneIsNotPresent) {
  NavState x;
  EXPECT_FALSE(x.has_clone(kInvalidClone));
  EXPECT_FALSE(x.has_clone(CloneId{42}));
}

TEST(NavState, MultipleClonesFollowPushOrder) {
  NavState x;
  const CloneId a = x.push_clone();
  const CloneId b = x.push_clone();
  const CloneId c = x.push_clone();

  EXPECT_EQ(x.clone_offset(a), kCoreDof);
  EXPECT_EQ(x.clone_offset(b), kCoreDof + kCloneDof);
  EXPECT_EQ(x.clone_offset(c), kCoreDof + 2 * kCloneDof);
  EXPECT_EQ(x.augment_dof(), 3 * kCloneDof);
}

TEST(NavState, DropCloneShrinksAndCompactsLaterClones) {
  // INTERFACES §1: drop_clone boyutu kucultur. Sonraki klonlar asagi kayar;
  // kimlikleri gecerli kalir.
  NavState x;
  const CloneId a = x.push_clone();
  const CloneId b = x.push_clone();
  const CloneId c = x.push_clone();

  x.drop_clone(b);

  EXPECT_FALSE(x.has_clone(b));
  EXPECT_TRUE(x.has_clone(a));
  EXPECT_TRUE(x.has_clone(c));

  EXPECT_EQ(x.clone_offset(a), kCoreDof);
  EXPECT_EQ(x.clone_offset(c), kCoreDof + kCloneDof) << "sonraki klon asagi kaymadi";
  EXPECT_EQ(x.augment_dof(), 2 * kCloneDof);
  EXPECT_EQ(x.active_dof(), kCoreDof + 2 * kCloneDof);
}

TEST(NavState, DroppedCloneIdIsNotReused) {
  NavState x;
  const CloneId a = x.push_clone();
  x.drop_clone(a);
  const CloneId b = x.push_clone();

  EXPECT_NE(a, b) << "kimlik yeniden kullanildi; bayat CloneId sessizce gecerli olur";
  EXPECT_FALSE(x.has_clone(a));
  EXPECT_TRUE(x.has_clone(b));
}

TEST(NavState, PushCloneReturnsInvalidWhenCapacityExhausted) {
  NavState x;
  const int azami = kMaxAugmentDof / kCloneDof;
  for (int i = 0; i < azami; ++i) {
    ASSERT_NE(x.push_clone(), kInvalidClone) << "klon " << i;
  }
  EXPECT_EQ(x.augment_dof(), azami * kCloneDof);

  EXPECT_EQ(x.push_clone(), kInvalidClone);
  EXPECT_EQ(x.augment_dof(), azami * kCloneDof) << "sessiz truncation";
}

// -----------------------------------------------------------------------------
// Kalibrasyon + klon birlikte — CONVENTIONS §5.2 duzeni
// -----------------------------------------------------------------------------

TEST(NavState, LayoutIsCoreThenPersistentThenClones) {
  NavState x;
  const auto kal1 = x.register_calibration("wheel_scale", 1);
  const CloneId klon1 = x.push_clone();
  const auto kal2 = x.register_calibration("time_offset", 1);
  const CloneId klon2 = x.push_clone();

  ASSERT_TRUE(kal1 && kal2);
  ASSERT_NE(klon1, kInvalidClone);
  ASSERT_NE(klon2, kInvalidClone);

  // Kalibrasyonlar ARADA kaydedilse bile klonlardan ONCE gelir.
  EXPECT_EQ(x.calibration_offset("wheel_scale"), kCoreDof);
  EXPECT_EQ(x.calibration_offset("time_offset"), kCoreDof + 1);
  EXPECT_EQ(x.clone_offset(klon1), kCoreDof + 2);
  EXPECT_EQ(x.clone_offset(klon2), kCoreDof + 2 + kCloneDof);
  EXPECT_EQ(x.augment_dof(), 2 + 2 * kCloneDof);
}

TEST(NavState, DropCloneDoesNotMoveCalibrations) {
  NavState x;
  x.register_calibration("wheel_scale", 1);
  const CloneId a = x.push_clone();
  const CloneId b = x.push_clone();

  x.drop_clone(a);

  EXPECT_EQ(x.calibration_offset("wheel_scale"), kCoreDof);
  EXPECT_EQ(x.clone_offset(b), kCoreDof + 1);
}

TEST(NavState, CloneCapacityAccountsForCalibrations) {
  NavState x;
  ASSERT_TRUE(x.register_calibration("dolgu", kMaxAugmentDof - kCloneDof).has_value());

  ASSERT_NE(x.push_clone(), kInvalidClone);
  EXPECT_EQ(x.active_dof(), kMaxStateDof);
  EXPECT_EQ(x.push_clone(), kInvalidClone);
}

// -----------------------------------------------------------------------------
// plus / minus augmentation ile
// -----------------------------------------------------------------------------

TEST(NavState, PlusMinusRoundTripWithAugmentation) {
  NavState x = ornek_durum();
  x.register_calibration("wheel_scale", 1);
  x.push_clone();

  const int aktif = x.active_dof();
  ASSERT_EQ(aktif, kCoreDof + 1 + kCloneDof);

  const StateVec d = delta_uret(aktif);
  const NavState y = x.plus(d);
  const StateVec geri = y.minus(x);

  for (int i = 0; i < aktif; ++i) {
    EXPECT_NEAR(geri[i], d[i], 1e-9) << "bilesen " << i;
  }
  EXPECT_LT(geri.tail(kMaxStateDof - aktif).cwiseAbs().maxCoeff(), 1e-18);
}

TEST(NavState, PlusPreservesLayout) {
  NavState x;
  x.register_calibration("wheel_scale", 1);
  const CloneId id = x.push_clone();

  const NavState y = x.plus(delta_uret(x.active_dof()));

  EXPECT_EQ(y.active_dof(), x.active_dof());
  EXPECT_EQ(y.calibration_offset("wheel_scale"), kCoreDof);
  EXPECT_TRUE(y.has_clone(id));
  EXPECT_EQ(y.clone_offset(id), kCoreDof + 1);
}

TEST(NavState, AugmentValuesMoveWithCompaction) {
  // drop_clone yalnizca ofsetleri degil DEGERLERI de tasimali; aksi halde
  // hayatta kalan klonun degeri silinen klonunkine kayar.
  NavState x;
  const CloneId a = x.push_clone();
  const CloneId b = x.push_clone();

  StateVec d = StateVec::Zero();
  d[x.clone_offset(a)] = 1.0;
  d[x.clone_offset(b)] = 2.0;
  NavState y = x.plus(d);

  const StateVec once = y.minus(x);
  const Scalar b_degeri = once[y.clone_offset(b)];
  ASSERT_NEAR(b_degeri, 2.0, 1e-15);

  y.drop_clone(a);
  NavState referans = x;
  referans.drop_clone(a);

  const StateVec sonra = y.minus(referans);
  EXPECT_NEAR(sonra[y.clone_offset(b)], b_degeri, 1e-15) << "deger tasinmadi";
}

} // namespace

// -----------------------------------------------------------------------------
// Tahsis yasagi — CONVENTIONS §8.1 / ADR-22
//
// Durum vektoru sabit kapasitelidir ve aktif boyutun degismesi YENIDEN TAHSIS
// gerektirmez. Bunu iddia etmek yerine olculur: global operator new sayilir.
// Sayac yalnizca olculen bolgede aciktir, gtest'in kendi tahsisleri disarida
// kalir.
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

TEST(NavState, LifecycleDoesNotAllocate) {
  NavState x;
  StateVec d = StateVec::Zero();

  {
    const TahsisKapsami kapsam;
    x.register_calibration("wheel_scale", 1);
    const CloneId a = x.push_clone();
    const CloneId b = x.push_clone();
    x.clone_offset(a);
    x.calibration_offset("wheel_scale");
    x.has_clone(b);
    d = x.minus(x);
    const NavState y = x.plus(d);
    x.drop_clone(a);
    (void)y.active_dof();
  }

  EXPECT_EQ(tahsis_adedi, 0) << "durum yasam dongusu heap'e gitti";
}

TEST(NavState, StorageIsFixedSize) {
  // Sabit kapasite: boyut derleme zamaninda bilinir ve aktif dof'tan bagimsizdir.
  NavState a;
  NavState b;
  b.register_calibration("wheel_scale", 6);
  b.push_clone();
  EXPECT_EQ(sizeof(a), sizeof(b));
  EXPECT_TRUE(std::is_nothrow_move_constructible<NavState>::value);
}

} // namespace
