/// \file
/// S9 — determinizm sozlesmesi. DoD: ayni tohum, bit-bit ayni cikti.

#include "kerteriz_sim/rng.hpp"

#include <cstring>
#include <gtest/gtest.h>
#include <vector>

namespace {

using kerteriz_sim::Scalar;
using kerteriz_sim::SeededRng;

std::vector<Scalar> akis(std::uint64_t tohum, int n) {
  SeededRng rng(tohum);
  std::vector<Scalar> v;
  v.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    v.push_back(rng.gaussian());
  }
  return v;
}

/// Bit seviyesinde karsilastirma — 1e-15 "yakinlik" DEGIL.
bool bit_bit_ayni(const std::vector<Scalar>& a, const std::vector<Scalar>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  return std::memcmp(a.data(), b.data(), a.size() * sizeof(Scalar)) == 0;
}

TEST(SeededRng, SameSeedProducesBitIdenticalStream) {
  const auto a = akis(20260913U, 500);
  const auto b = akis(20260913U, 500);
  EXPECT_TRUE(bit_bit_ayni(a, b)) << "ayni tohum farkli akis uretti";
}

TEST(SeededRng, DifferentSeedProducesDifferentStream) {
  // Bu olmadan yukaridaki test bos bir iddia olurdu.
  const auto a = akis(1U, 500);
  const auto b = akis(2U, 500);
  EXPECT_FALSE(bit_bit_ayni(a, b)) << "farkli tohumlar ayni akisi verdi";
}

TEST(SeededRng, ResetRewindsStream) {
  SeededRng rng(42U);
  std::vector<Scalar> once;
  for (int i = 0; i < 64; ++i) {
    once.push_back(rng.gaussian());
  }

  rng.reset();
  std::vector<Scalar> sonra;
  for (int i = 0; i < 64; ++i) {
    sonra.push_back(rng.gaussian());
  }

  EXPECT_TRUE(bit_bit_ayni(once, sonra));
}

TEST(SeededRng, RawEngineMatchesStandardMt19937_64) {
  // Motor akisi standart tarafindan birebir tanimlidir; sabit bir referansla
  // karsilastirmak, bir gun motorun sessizce degistirilmesini yakalar.
  std::mt19937_64 referans(12345U);
  SeededRng rng(12345U);
  for (int i = 0; i < 32; ++i) {
    EXPECT_EQ(rng.raw(), referans());
  }
}

TEST(SeededRng, Uniform01StaysInRange) {
  SeededRng rng(7U);
  for (int i = 0; i < 10000; ++i) {
    const Scalar u = rng.uniform01();
    ASSERT_GE(u, Scalar(0));
    ASSERT_LT(u, Scalar(1));
  }
}

TEST(SeededRng, GaussianHasExpectedMomentsL) {
  SeededRng rng(99U);
  const int n = 200000;
  Scalar toplam = 0;
  Scalar toplam_kare = 0;
  for (int i = 0; i < n; ++i) {
    const Scalar x = rng.gaussian();
    toplam += x;
    toplam_kare += x * x;
  }
  const Scalar ort = toplam / n;
  const Scalar var = toplam_kare / n - ort * ort;

  EXPECT_NEAR(ort, Scalar(0), Scalar(0.02));
  EXPECT_NEAR(var, Scalar(1), Scalar(0.02));
}

} // namespace
