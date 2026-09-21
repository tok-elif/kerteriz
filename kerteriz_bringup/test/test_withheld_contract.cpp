/// \file
/// F2.4-C/D withheld deney sozlesmesi — CLI seviyesindeki fail-fast kapisi.
///
/// Sinanan sey DENEY YAPILANDIRMASIDIR, filtre degil. Sozlesme saf oldugu
/// icin bu testler veri seti, dosya sistemi ve ROS calisma zamani GEREKTIRMEZ.
///
/// Onemli olan negatif kapsam: sozlesme yalnizca `--withheld-reference`
/// istendiginde konusur. Legacy ve withheld istemeyen seyreltme kosulari
/// kisitlanmis olsaydi genel `gnss_sampling` davranisi degismis olurdu.

#include "kerteriz_bringup/withheld_contract.hpp"

#include <gtest/gtest.h>
#include <string>

namespace {

using kerteriz_bringup::check_withheld_contract;
using kerteriz_bringup::kWithheldProtocolStride;
using kerteriz_bringup::WithheldCliRequest;

/// Sozlesmeyi TAM saglayan istek. Testler bundan tek tek alan bozar.
WithheldCliRequest tam() {
  WithheldCliRequest r;
  r.withheld_requested = true;
  r.stride_given = true;
  r.stride = kWithheldProtocolStride;
  r.gnss_velocity_disabled = true;
  r.sampling_manifest_given = true;
  return r;
}

// -----------------------------------------------------------------------------
// Negatif kapsam — sozlesme yalnizca withheld istendiginde konusur
// -----------------------------------------------------------------------------

TEST(WithheldContract, SaysNothingWhenWithheldIsNotRequested) {
  // Legacy: hicbir bayrak yok.
  EXPECT_TRUE(check_withheld_contract(WithheldCliRequest{}).ok);

  // Withheld ISTENMEYEN bir seyreltme kosusu: stride serbest, GNSS hizi acik,
  // manifest yok. Bunlarin kisitlanmasi genel sampling davranisini
  // degistirmek olurdu.
  WithheldCliRequest serbest;
  serbest.withheld_requested = false;
  serbest.stride_given = true;
  serbest.stride = 3;
  serbest.gnss_velocity_disabled = false;
  serbest.sampling_manifest_given = false;
  EXPECT_TRUE(check_withheld_contract(serbest).ok)
      << "withheld istenmeyen kosu kisitlanmis — genel sampling davranisi degismis";
}

// -----------------------------------------------------------------------------
// Pozitif yol
// -----------------------------------------------------------------------------

TEST(WithheldContract, AcceptsTheFrozenProtocol) {
  const auto s = check_withheld_contract(tam());
  EXPECT_TRUE(s.ok) << s.message;
  EXPECT_TRUE(s.message.empty());
}

TEST(WithheldContract, ProtocolStrideIsTheFrozenTen) {
  // Deger F2.4-C'de donduruldu (results/f2.4/sequence_manifest.md). Sessizce
  // degistirilirse withheld deneyi baska bir deney olur.
  EXPECT_EQ(kWithheldProtocolStride, 10);
}

// -----------------------------------------------------------------------------
// Her bayrak TEK BASINA reddi tetiklemeli (mutasyon duyarliligi)
// -----------------------------------------------------------------------------

TEST(WithheldContract, MissingStrideIsRejected) {
  auto r = tam();
  r.stride_given = false;
  r.stride = 1;
  const auto s = check_withheld_contract(r);
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.message.find("--gnss-position-stride"), std::string::npos) << s.message;
}

TEST(WithheldContract, WrongStrideIsRejected) {
  for (const int yanlis : {1, 2, 5, 9, 11, 20}) {
    auto r = tam();
    r.stride = yanlis;
    const auto s = check_withheld_contract(r);
    EXPECT_FALSE(s.ok) << "stride " << yanlis << " kabul edilmis";
    EXPECT_NE(s.message.find(std::to_string(yanlis)), std::string::npos)
        << "hata mesaji verilen degeri soylemeli: " << s.message;
  }
}

TEST(WithheldContract, EnabledGnssVelocityIsRejected) {
  // Bu unutulursa Kerteriz ~10 Hz hiz olcumu alir, harici taban cizgisi hic
  // almaz; kiyas sessizce asimetrik olur.
  auto r = tam();
  r.gnss_velocity_disabled = false;
  const auto s = check_withheld_contract(r);
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.message.find("--no-gnss-velocity"), std::string::npos) << s.message;
}

TEST(WithheldContract, MissingSamplingManifestIsRejected) {
  auto r = tam();
  r.sampling_manifest_given = false;
  const auto s = check_withheld_contract(r);
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.message.find("--sampling-manifest"), std::string::npos) << s.message;
}

// -----------------------------------------------------------------------------
// Hata mesaji eksigin TAMAMINI sayar
// -----------------------------------------------------------------------------

TEST(WithheldContract, ReportsEveryViolationAtOnce) {
  WithheldCliRequest r;
  r.withheld_requested = true; // digerleri varsayilan: hepsi eksik

  const auto s = check_withheld_contract(r);
  ASSERT_FALSE(s.ok);
  EXPECT_NE(s.message.find("--gnss-position-stride"), std::string::npos) << s.message;
  EXPECT_NE(s.message.find("--no-gnss-velocity"), std::string::npos) << s.message;
  EXPECT_NE(s.message.find("--sampling-manifest"), std::string::npos) << s.message;
}

} // namespace
