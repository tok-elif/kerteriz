/// \file
/// F1.3 — chi-kare CDF ve quantile.
///
/// REFERANS DEGERLER BAGIMSIZDIR. Implementasyonun dogrulugu kendi
/// fonksiyonuyla dogrulanmaz:
///   - dof = 2 ve dof = 4 icin CDF'in KAPALI FORMU vardir (gama kodu
///     kullanilmaz), quantile ona karsi sinanir;
///   - dof = 1 icin quantile normal dagilimin karesidir;
///   - geri kalanlar yayimlanmis chi-kare tablo degerleridir.

#include "kerteriz/util/chi_square.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace {

using kerteriz::chi_square_cdf;
using kerteriz::chi_square_quantile;
using kerteriz::kMaxResidualDim;
using kerteriz::Scalar;

/// dof = 2: CDF(x) = 1 - exp(-x/2). Gama fonksiyonundan BAGIMSIZ.
Scalar cdf_dof2(Scalar x) { return Scalar(1) - std::exp(-x / Scalar(2)); }

/// dof = 4: CDF(x) = 1 - exp(-x/2) (1 + x/2). Yine bagimsiz.
Scalar cdf_dof4(Scalar x) {
  return Scalar(1) - std::exp(-x / Scalar(2)) * (Scalar(1) + x / Scalar(2));
}

TEST(ChiSquare, CdfMatchesClosedFormForEvenDof) {
  for (const Scalar x : {0.1, 0.5, 1.0, 2.5, 5.0, 9.0, 15.0, 30.0}) {
    EXPECT_NEAR(chi_square_cdf(x, 2), cdf_dof2(x), 1e-12) << "dof=2, x=" << x;
    EXPECT_NEAR(chi_square_cdf(x, 4), cdf_dof4(x), 1e-12) << "dof=4, x=" << x;
  }
}

TEST(ChiSquare, CdfIsMonotoneAndBounded) {
  for (int dof = 1; dof <= kMaxResidualDim; ++dof) {
    Scalar onceki = chi_square_cdf(0.0, dof);
    EXPECT_NEAR(onceki, 0.0, 1e-15) << "dof=" << dof;
    for (Scalar x = 0.25; x <= 60.0; x += 0.25) {
      const Scalar simdi = chi_square_cdf(x, dof);
      EXPECT_GE(simdi, onceki - 1e-15) << "dof=" << dof << " x=" << x;
      EXPECT_LE(simdi, 1.0 + 1e-12);
      onceki = simdi;
    }
    EXPECT_GT(chi_square_cdf(200.0, dof), 0.999999) << "dof=" << dof;
  }
}

TEST(ChiSquare, QuantileMatchesPublishedTableAt95) {
  // Yayimlanmis chi-kare tablosu, c = 0.95.
  struct Vaka {
    int dof;
    Scalar beklenen;
  };
  const Vaka vakalar[] = {
      {1, 3.841459}, {2, 5.991465}, {3, 7.814728}, {6, 12.591587}, {12, 21.026070},
  };
  for (const auto& v : vakalar) {
    EXPECT_NEAR(chi_square_quantile(0.95, v.dof), v.beklenen, 1e-5) << "dof=" << v.dof;
  }
}

TEST(ChiSquare, QuantileMatchesIndependentClosedFormAt997) {
  // dof = 2: CDF kapali formundan x = -2 ln(1 - c).
  const Scalar c = 0.997;
  EXPECT_NEAR(chi_square_quantile(c, 2), -2.0 * std::log(1.0 - c), 1e-9);

  // dof = 1: chi-kare_1 quantile = (normal quantile)^2.
  // Phi^-1(0.9985) = 2.967738  ->  8.807468
  EXPECT_NEAR(chi_square_quantile(c, 1), 8.807468, 1e-4);

  // dof = 4: kapali form uzerinden ikiye bolerek bagimsiz cozum.
  Scalar alt = 0.0;
  Scalar ust = 100.0;
  for (int i = 0; i < 200; ++i) {
    const Scalar orta = 0.5 * (alt + ust);
    if (cdf_dof4(orta) < c) {
      alt = orta;
    } else {
      ust = orta;
    }
  }
  EXPECT_NEAR(chi_square_quantile(c, 4), 0.5 * (alt + ust), 1e-9);
}

TEST(ChiSquare, QuantileInvertsCdfForEveryDof) {
  for (int dof = 1; dof <= kMaxResidualDim; ++dof) {
    for (const Scalar c : {0.5, 0.9, 0.95, 0.99, 0.997, 0.9999}) {
      const Scalar x = chi_square_quantile(c, dof);
      EXPECT_NEAR(chi_square_cdf(x, dof), c, 1e-9) << "dof=" << dof << " c=" << c;
    }
  }
}

TEST(ChiSquare, QuantileIsMonotoneInConfidenceAndDof) {
  for (int dof = 1; dof <= kMaxResidualDim; ++dof) {
    EXPECT_LT(chi_square_quantile(0.90, dof), chi_square_quantile(0.95, dof));
    EXPECT_LT(chi_square_quantile(0.95, dof), chi_square_quantile(0.997, dof));
  }
  for (int dof = 1; dof < kMaxResidualDim; ++dof) {
    EXPECT_LT(chi_square_quantile(0.95, dof), chi_square_quantile(0.95, dof + 1));
  }
}

TEST(ChiSquare, QuantileIsDeterministic) {
  for (int dof = 1; dof <= kMaxResidualDim; ++dof) {
    EXPECT_EQ(chi_square_quantile(0.997, dof), chi_square_quantile(0.997, dof));
  }
}

} // namespace
