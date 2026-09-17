#pragma once

/// \file
/// chi-kare CDF ve quantile — F1.3 kapisi ve F2.2 tutarlilik bantlari icin.
///
/// CONVENTIONS §10/§11 kapiyi GUVEN SEVIYESI ile tanimlar
/// (`gate: {chi2_confidence: 0.997}`) ve sabit %95/%99 esigi acikca yasaklar.
/// Dolayisiyla guven seviyesini serbestlik derecesine gore esige ceviren bir
/// ters chi-kare gerekir. Yeni bagimlilik eklenmez; C++17 ve <cmath> yeterlidir.
///
/// chi-kare_k CDF(x) = P(k/2, x/2),  P = duzenlestirilmis ALT eksik gama.
///
/// Iterasyon sayilari SABIT ustten sinirlidir; sonsuz dongu yoktur ve sonuc
/// deterministiktir. Hesap kurucuda yapilir, sicak yolda yalnizca onbellek
/// okunur.

#include "kerteriz/types.hpp"

#include <cassert>
#include <cmath>

namespace kerteriz {
namespace detail {

inline constexpr int kGammaMaxIter = 300;
inline constexpr Scalar kGammaEps = 1e-15;
inline constexpr Scalar kGammaTiny = 1e-300;

/// Duzenlestirilmis alt eksik gama P(a, x). Seri ve surekli kesir bicimleri
/// yakinsama bolgelerine gore secilir (Numerical Recipes gser/gcf kalibi).
inline Scalar regularized_gamma_p(Scalar a, Scalar x) {
  assert(a > Scalar(0));
  if (x <= Scalar(0)) {
    return Scalar(0);
  }
  const Scalar log_on = a * std::log(x) - x - std::lgamma(a);

  if (x < a + Scalar(1)) {
    Scalar ap = a;
    Scalar del = Scalar(1) / a;
    Scalar toplam = del;
    for (int i = 0; i < kGammaMaxIter; ++i) {
      ap += Scalar(1);
      del *= x / ap;
      toplam += del;
      if (std::abs(del) < std::abs(toplam) * kGammaEps) {
        break;
      }
    }
    return toplam * std::exp(log_on);
  }

  Scalar b = x + Scalar(1) - a;
  Scalar c = Scalar(1) / kGammaTiny;
  Scalar d = Scalar(1) / b;
  Scalar h = d;
  for (int i = 1; i <= kGammaMaxIter; ++i) {
    const Scalar an = -Scalar(i) * (Scalar(i) - a);
    b += Scalar(2);
    d = an * d + b;
    if (std::abs(d) < kGammaTiny) {
      d = kGammaTiny;
    }
    c = b + an / c;
    if (std::abs(c) < kGammaTiny) {
      c = kGammaTiny;
    }
    d = Scalar(1) / d;
    const Scalar del = d * c;
    h *= del;
    if (std::abs(del - Scalar(1)) < kGammaEps) {
      break;
    }
  }
  return Scalar(1) - std::exp(log_on) * h; // 1 - Q(a, x)
}

} // namespace detail

/// chi-kare kumulatif dagilim fonksiyonu.
inline Scalar chi_square_cdf(Scalar x, int dof) {
  assert(dof >= 1);
  return detail::regularized_gamma_p(Scalar(dof) / Scalar(2), x / Scalar(2));
}

/// chi-kare quantile: CDF(x) = confidence olan x.
///
/// Parantezleme + ikiye bolme. Ikisinin de iterasyon sayisi sabit ustten
/// sinirlidir. Newton kullanilmaz: CDF kuyruklarda cok duz oldugu icin Newton
/// adimi parantezin disina cikabilir ve saglamlastirma gerektirirdi; 200 adim
/// ikiye bolme cift duyarlikta zaten tam cozunurluge iner.
/// SERBESTLIK DERECESI SINIRI BURADA DEGILDIR.
///
/// Bu fonksiyon bir donem `dof <= kMaxResidualDim` ile sinirliydi. O sinir
/// OLCUM ARTIK BOYUTUNUN sinirdir ve ait oldugu yer olcum/backend
/// sozlesmesidir: `EskfBackend` esik onbellegini zaten 1..kMaxResidualDim
/// araliginda kurar ve `Measurement::residual_dim()` o kapasiteyi
/// static_assert ile zorlar. Matematik yardimcisina konuldugunda, olcumle
/// hicbir ilgisi olmayan kullanimlari da kesiyordu: F2.2'nin tutarlilik
/// bantlari 15, M*15 ve nihayetinde 500*15 = 7500 serbestlik derecesi ister.
///
/// FILTRENIN AZAMI ARTIK BOYUTU DEGISMEDI (`kMaxResidualDim`); yalnizca bu
/// yardimcinin sozlesmesi genellestirildi.
///
/// BUYUK DoF DOGRULUGU. Seri kolu sabit 300 iterasyonla sinirlidir, bu yuzden
/// cok buyuk `dof`'ta alt kuyrukta iterasyon tavani once doludur. Olculen
/// bagil hata: dof=15 ~1e-16, dof=150 ~2e-15, dof=7500 ~1e-12. Bantlar icin
/// fazlasiyla yeterlidir ve test_chi_square bu degerleri dondurulmus
/// referanslara karsi sinar.
inline Scalar chi_square_quantile(Scalar confidence, int dof) {
  assert(confidence > Scalar(0) && confidence < Scalar(1));
  assert(dof >= 1);

  Scalar alt = Scalar(0);
  Scalar ust = Scalar(1);
  for (int i = 0; i < 200 && chi_square_cdf(ust, dof) < confidence; ++i) {
    alt = ust;
    ust *= Scalar(2);
  }
  for (int i = 0; i < 200; ++i) {
    const Scalar orta = Scalar(0.5) * (alt + ust);
    if (chi_square_cdf(orta, dof) < confidence) {
      alt = orta;
    } else {
      ust = orta;
    }
  }
  return Scalar(0.5) * (alt + ust);
}

} // namespace kerteriz
