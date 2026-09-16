#pragma once

/// \file
/// Asgari oteleme ATE (mutlak yorunge hatasi) — degerlendirme katmani.
///
/// ============================== KAPSAM ======================================
///
/// Faz 2'nin `evo` tabanli deney harness'i BURAYA CEKILMEZ (SPEC §8). Bu dosya
/// yalnizca Faz 1 entegrasyon dogrulamasi icin gereken en kucuk olcuttur:
///
///   e_i      = || p_kestirim(t_i) - p_referans(t_i) ||
///   ATE_RMSE = sqrt( ortalama(e_i^2) )
///
/// ====================== HIZALAMA YAPILMAZ — BILEREK =========================
///
/// Keyfi bir Sim(3) veya rijit hizalama UYGULANMAZ. Referans ve kestirim AYNI
/// yerel ENU cercevesinde ve AYNI orijinde uretilir (adaptor ikisini de ayni
/// `EnuProjector` ile kurar), dolayisiyla ortak bir cerceve zaten vardir.
/// Hizalama eklemek, olcumun neyi olctugunu degistirirdi: kestirimin mutlak
/// konumu degil, yalnizca sekli degerlendirilmis olurdu.
///
/// ========================== ZAMAN ESLESTIRME ================================
///
/// Her REFERANS ornegi icin damgasi en yakin KESTIRIM ornegi secilir. Fark
/// `max_dt_ns`'i asarsa o ornek eslesmez ve ortalamaya GIRMEZ. Esitlikte daha
/// ERKEN damga secilir — kural deterministiktir, girdi sirasina bagli degildir.
/// Enterpolasyon yapilmaz; yapilsaydi hangi modelin (dogrusal, kubik) secildigi
/// sonucu etkilerdi ve bu secim belgelenmemis bir serbestlik olurdu.

#include "kerteriz/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace kerteriz_bringup {

using kerteriz::Scalar;
using kerteriz::TimeNs;
using kerteriz::Vec3;

/// Zaman damgali konum ornegi.
struct TrajectorySample {
  TimeNs stamp_ns = 0;
  Vec3 position_w = Vec3::Zero();
};

struct AteResult {
  bool ok = false;        ///< eslesen ornek var mi
  Scalar rmse_m = 0;      ///< sqrt(ortalama(e^2))
  Scalar max_error_m = 0; ///< en buyuk tekil hata
  int matched = 0;        ///< ortalamaya giren ornek sayisi
  int unmatched = 0;      ///< `max_dt_ns` disinda kalan referans ornegi sayisi
};

/// `sorted_estimate` DAMGA SIRASINDA olmalidir; cagiran bunu saglar.
inline AteResult translational_ate(const std::vector<TrajectorySample>& reference,
                                   const std::vector<TrajectorySample>& sorted_estimate,
                                   TimeNs max_dt_ns) {
  AteResult r;
  if (reference.empty() || sorted_estimate.empty()) {
    return r;
  }

  Scalar kare_toplam = 0;
  for (const auto& ref : reference) {
    // Ilk damga >= ref olan aday ve bir oncesi karsilastirilir.
    const auto it =
        std::lower_bound(sorted_estimate.begin(), sorted_estimate.end(), ref.stamp_ns,
                         [](const TrajectorySample& s, TimeNs t) { return s.stamp_ns < t; });

    const TrajectorySample* en_iyi = nullptr;
    TimeNs en_iyi_fark = 0;
    auto degerlendir = [&](const TrajectorySample& aday) {
      const TimeNs fark = aday.stamp_ns > ref.stamp_ns ? aday.stamp_ns - ref.stamp_ns
                                                       : ref.stamp_ns - aday.stamp_ns;
      // Esitlikte ERKEN damga kazanir: onceki aday (daha erken) once bakilir
      // ve yalnizca KESIN olarak daha kucuk fark onu iter.
      if (en_iyi == nullptr || fark < en_iyi_fark) {
        en_iyi = &aday;
        en_iyi_fark = fark;
      }
    };
    if (it != sorted_estimate.begin()) {
      degerlendir(*(it - 1));
    }
    if (it != sorted_estimate.end()) {
      degerlendir(*it);
    }

    if (en_iyi == nullptr || en_iyi_fark > max_dt_ns) {
      ++r.unmatched;
      continue;
    }
    const Scalar e = (en_iyi->position_w - ref.position_w).norm();
    kare_toplam += e * e;
    r.max_error_m = std::max(r.max_error_m, e);
    ++r.matched;
  }

  if (r.matched == 0) {
    return r;
  }
  r.rmse_m = std::sqrt(kare_toplam / static_cast<Scalar>(r.matched));
  r.ok = true;
  return r;
}

} // namespace kerteriz_bringup
