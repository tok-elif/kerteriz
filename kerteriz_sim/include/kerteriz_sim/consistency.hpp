#pragma once

/// \file
/// NEES / NIS tutarlilik istatistikleri — F2.2.
///
/// ======================= BU DOSYA CIZIM YAPMAZ ==============================
///
/// Burada yalnizca SAYI uretilir. Grafik, etiket, dosya yolu, sonuc dizini
/// politikasi YOKTUR — onlar F2.3/F2.5'tir. Tipler cizimden bagimsizdir ve
/// dogrudan F2.1'in `StateSample` / `NisObservation` kayitlari uzerinde calisir.
///
/// ============================ NEES TANIMI ===================================
///
///   e    = estimate.minus(truth).head<15>()      (CONVENTIONS §3.1 SAG
///                                                 perturbasyon; elde Euler
///                                                 farki KURULMAZ)
///   NEES = e^T P^-1 e                            P = kaydedilen 15x15 blok
///
/// P'nin acik tersi ALINMAZ (ADR-17): LDLT ile cozulur ve pivotlarin KESIN
/// pozitifligi ayrica zorlanir — `isPositive()` tek basina yetmez, Eigen 3.4'te
/// pozitif YARI-tanimli matris icin de true doner.
///
/// ===================== SAYISAL PATOLOJI GORUNUR KALIR =======================
///
/// P sonlu degilse, yeterince simetrik degilse veya pozitif tanimli degilse:
/// titresim EKLENMEZ, ozdeger KIRPILMAZ, sessizce simetriklestirilmez. Ornek
/// GECERSIZ isaretlenir ve sebebi sayilir. Bir sayisal patolojiyi "onarmak"
/// onu gorunmez kilmak olurdu; tutarlilik calismasinin amaci tam tersidir.
///
/// ==================== TOPLAMA: ZAMAN UZERINDE DEGIL =========================
///
/// Guven bandi yorumu TOPLULUK (ensemble) tabanlidir. Her deterministik damgada
/// topluluk, o damgadaki N kosudur. TEK bir kosunun ardisik zaman ornekleri
/// bagimsiz chi-kare denemeleri gibi HAVUZLANMAZ — zamansal ornekler
/// korelelidir ve boyle bir havuzlama bandi sahte bicimde daraltirdi.

#include "kerteriz/util/chi_square.hpp"
#include "kerteriz_sim/monte_carlo.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <string>
#include <vector>

namespace kerteriz_sim {

using kerteriz::TimeNs;

/// TUTARLILIK guven seviyesi. FILTRE KAPISININ GUVENI DEGILDIR.
///
/// Kapi `chi2_confidence` (varsayilan 0.997) bir OLCUMUN elenip elenmeyecegine
/// karar verir; bu ise kestirimcinin kovaryansinin dogru olup olmadigini
/// sinayan bandin genisligidir. Ikisini ayni sayiya baglamak iki farkli
/// sorunun ayni cevabi paylasmasi olurdu.
inline constexpr Scalar kDefaultConsistencyConfidence = Scalar(0.95);

/// Kovaryans simetri toleransi — bagil. `P` ve `P^T` farkinin en buyuk mutlak
/// ogesi, `P`'nin olceginin bu kati kadarini asarsa ornek gecersizdir.
inline constexpr Scalar kDefaultSymmetryTolerance = Scalar(1e-9);

// -----------------------------------------------------------------------------
// Tek ornek NEES
// -----------------------------------------------------------------------------

enum class NeesStatus {
  kValid,
  kErrorNotFinite,                ///< estimate.minus(truth) sonlu degil
  kCovarianceNotFinite,           ///< P'de NaN/Inf
  kCovarianceNotSymmetric,        ///< maddeten simetrik degil
  kCovarianceNotPositiveDefinite, ///< LDLT kesin pozitif pivot vermedi
  kUnexpectedDof                  ///< aktif duzen cekirdek degil
};

inline const char* to_string(NeesStatus s) {
  switch (s) {
  case NeesStatus::kValid:
    return "gecerli";
  case NeesStatus::kErrorNotFinite:
    return "hata vektoru sonlu degil";
  case NeesStatus::kCovarianceNotFinite:
    return "kovaryans sonlu degil";
  case NeesStatus::kCovarianceNotSymmetric:
    return "kovaryans simetrik degil";
  case NeesStatus::kCovarianceNotPositiveDefinite:
    return "kovaryans pozitif tanimli degil";
  case NeesStatus::kUnexpectedDof:
    return "beklenmeyen aktif serbestlik derecesi";
  }
  return "bilinmeyen";
}

struct NeesSample {
  TimeNs stamp_ns = 0;
  Scalar nees = 0;
  int dof = 0;
  bool valid = false;
  NeesStatus status = NeesStatus::kValid;
};

/// Tek bir `StateSample` icin NEES. Cizimden ve dosyadan bagimsizdir.
inline NeesSample compute_nees(const StateSample& s,
                               Scalar symmetry_tolerance = kDefaultSymmetryTolerance) {
  NeesSample out;
  out.stamp_ns = s.stamp_ns;
  out.dof = kerteriz::kCoreDof;

  if (s.active_dof != kerteriz::kCoreDof) {
    out.status = NeesStatus::kUnexpectedDof;
    return out;
  }

  // Hata PROJENIN KENDI konvansiyonundan gelir; burada Euler farki kurulmaz.
  const kerteriz::StateVec tam = s.estimate.minus(s.truth);
  const Eigen::Matrix<Scalar, kerteriz::kCoreDof, 1> e = tam.head<kerteriz::kCoreDof>();
  if (!e.allFinite()) {
    out.status = NeesStatus::kErrorNotFinite;
    return out;
  }

  const CoreCovariance& p = s.covariance;
  if (!p.allFinite()) {
    out.status = NeesStatus::kCovarianceNotFinite;
    return out;
  }

  // Simetri ACIKCA sinanir. Eigen'in LDLT'si varsayilan olarak yalnizca ALT
  // ucgeni okur; simetrik olmayan bir P sessizce kabul edilir ve ust ucgen yok
  // sayilirdi. Burada P DUZELTILMEZ, yalnizca REDDEDILIR.
  const Scalar olcek = std::max(Scalar(1), p.cwiseAbs().maxCoeff());
  if ((p - p.transpose()).cwiseAbs().maxCoeff() > symmetry_tolerance * olcek) {
    out.status = NeesStatus::kCovarianceNotSymmetric;
    return out;
  }

  // ADR-17: acik ters alinmaz. Kesin pozitif tanimlilik, linear_update ile AYNI
  // olcutle zorlanir — isPositive() tek basina yetmez.
  const Eigen::LDLT<CoreCovariance> ldlt(p);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive() ||
      ldlt.vectorD().minCoeff() <= Scalar(0)) {
    out.status = NeesStatus::kCovarianceNotPositiveDefinite;
    return out;
  }

  const Eigen::Matrix<Scalar, kerteriz::kCoreDof, 1> pinv_e = ldlt.solve(e);
  const Scalar nees = e.dot(pinv_e);
  if (!std::isfinite(nees)) {
    out.status = NeesStatus::kCovarianceNotPositiveDefinite;
    return out;
  }

  out.nees = nees;
  out.valid = true;
  out.status = NeesStatus::kValid;
  return out;
}

// -----------------------------------------------------------------------------
// chi-kare bantlari
// -----------------------------------------------------------------------------

struct ChiSquareBand {
  Scalar lower = 0;
  Scalar upper = 0;
  Scalar expected_mean = 0;
};

/// M bagimsiz, her biri n serbestlik dereceli NEES orneginin ORTALAMASI icin
/// band. Toplam M*n serbestlik derecelidir; ortalama icin M'e bolunur.
inline ChiSquareBand anees_band(int m, int n, Scalar confidence = kDefaultConsistencyConfidence) {
  ChiSquareBand b;
  if (m <= 0 || n <= 0) {
    return b;
  }
  const Scalar alpha = Scalar(1) - confidence;
  const int toplam_dof = m * n;
  b.lower = kerteriz::chi_square_quantile(alpha / Scalar(2), toplam_dof) / Scalar(m);
  b.upper = kerteriz::chi_square_quantile(Scalar(1) - alpha / Scalar(2), toplam_dof) / Scalar(m);
  b.expected_mean = Scalar(n);
  return b;
}

/// M gecerli NIS gozleminin ORTALAMASI icin band.
///
/// `total_dof` TOPLAMDIR, M * sabit degil: gozlemler farkli artik boyutlarina
/// sahip olabilir. Bugun `GnssPosition` dof = 3 verir ama 3 buraya GOMULMEZ.
inline ChiSquareBand anis_band(int m, int total_dof,
                               Scalar confidence = kDefaultConsistencyConfidence) {
  ChiSquareBand b;
  if (m <= 0 || total_dof <= 0) {
    return b;
  }
  const Scalar alpha = Scalar(1) - confidence;
  b.lower = kerteriz::chi_square_quantile(alpha / Scalar(2), total_dof) / Scalar(m);
  b.upper = kerteriz::chi_square_quantile(Scalar(1) - alpha / Scalar(2), total_dof) / Scalar(m);
  b.expected_mean = Scalar(total_dof) / Scalar(m);
  return b;
}

// -----------------------------------------------------------------------------
// Topluluk toplama
// -----------------------------------------------------------------------------

struct NeesTimePoint {
  TimeNs stamp_ns = 0;
  Scalar mean_nees = 0;
  Scalar expected_mean = 0;
  Scalar lower_95 = 0;
  Scalar upper_95 = 0;
  int valid_count = 0;
  int invalid_count = 0;
};

struct NisTimePoint {
  TimeNs stamp_ns = 0;
  Scalar mean_nis = 0;
  Scalar expected_mean = 0;
  Scalar lower_95 = 0;
  Scalar upper_95 = 0;
  int valid_count = 0;
  int undefined_count = 0;
  int total_dof = 0;
};

/// Toplama sonucu. Damga izgaralari uyusmuyorsa SESSIZCE HIZALANMAZ.
struct AggregationStatus {
  bool ok = false;
  std::string message;

  static AggregationStatus success() { return AggregationStatus{true, {}}; }
  static AggregationStatus failure(std::string m) { return AggregationStatus{false, std::move(m)}; }
};

struct NeesSeries {
  AggregationStatus status;
  int runs = 0;
  std::vector<NeesTimePoint> points;
  int invalid_total = 0;
};

struct NisSeries {
  AggregationStatus status;
  int runs = 0;
  std::vector<NisTimePoint> points;
  int undefined_total = 0;
};

namespace detail {

/// Damga izgaralarinin BIREBIR ayni oldugunu dogrular.
///
/// Indislerin ortusecegini VARSAYMAK yeterli degildir: uzunluklar esit olsa
/// bile farkli damgalar ayni indise dusebilir ve ilgisiz ornekler
/// karsilastirilirdi. Uyusmazlik hata olarak bildirilir.
template <typename Kayit, typename DamgaAl>
AggregationStatus damga_izgarasi_dogrula(const std::vector<std::vector<Kayit>>& kosular,
                                         DamgaAl damga, const char* ne) {
  if (kosular.empty()) {
    return AggregationStatus::failure(std::string(ne) + ": kosu yok");
  }
  const std::size_t n = kosular.front().size();
  if (n == 0) {
    return AggregationStatus::failure(std::string(ne) + ": kosu 0 hic kayit tasimiyor");
  }
  for (std::size_t k = 1; k < kosular.size(); ++k) {
    if (kosular[k].size() != n) {
      return AggregationStatus::failure(std::string(ne) + ": kosu " + std::to_string(k) +
                                        " kayit sayisi " + std::to_string(kosular[k].size()) +
                                        ", kosu 0 ise " + std::to_string(n));
    }
    for (std::size_t i = 0; i < n; ++i) {
      if (damga(kosular[k][i]) != damga(kosular.front()[i])) {
        return AggregationStatus::failure(std::string(ne) + ": kosu " + std::to_string(k) +
                                          " indis " + std::to_string(i) + " damgasi " +
                                          std::to_string(damga(kosular[k][i])) + ", kosu 0 ise " +
                                          std::to_string(damga(kosular.front()[i])));
      }
    }
  }
  return AggregationStatus::success();
}

} // namespace detail

/// Her damgada KOSULAR UZERINDE ortalama NEES ve bandi.
inline NeesSeries aggregate_nees(const MonteCarloResult& mc,
                                 Scalar confidence = kDefaultConsistencyConfidence,
                                 Scalar symmetry_tolerance = kDefaultSymmetryTolerance) {
  NeesSeries out;
  out.runs = static_cast<int>(mc.runs.size());

  std::vector<std::vector<StateSample>> izgara;
  izgara.reserve(mc.runs.size());
  for (const auto& k : mc.runs) {
    izgara.push_back(k.samples);
  }
  out.status = detail::damga_izgarasi_dogrula(
      izgara, [](const StateSample& s) { return s.stamp_ns; }, "NEES");
  if (!out.status.ok) {
    return out;
  }

  const std::size_t n = izgara.front().size();
  out.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    NeesTimePoint p;
    p.stamp_ns = izgara.front()[i].stamp_ns;
    Scalar toplam = 0;
    for (const auto& kosu : izgara) {
      const NeesSample ns = compute_nees(kosu[i], symmetry_tolerance);
      if (ns.valid) {
        toplam += ns.nees;
        ++p.valid_count;
      } else {
        ++p.invalid_count;
      }
    }
    out.invalid_total += p.invalid_count;

    if (p.valid_count > 0) {
      p.mean_nees = toplam / static_cast<Scalar>(p.valid_count);
      const ChiSquareBand b = anees_band(p.valid_count, kerteriz::kCoreDof, confidence);
      p.expected_mean = b.expected_mean;
      p.lower_95 = b.lower;
      p.upper_95 = b.upper;
    }
    out.points.push_back(p);
  }
  return out;
}

/// Her damgada KOSULAR UZERINDE ortalama NIS ve bandi.
///
/// KABUL EDILEN VE CHI-KARE ILE REDDEDILEN gozlemlerin IKISI DE dahildir.
/// Yalnizca kabul edilenleri almak ust kuyrugu keser ve tutarlilik sonucunu
/// sistematik olarak IYIMSER gosterirdi. Yalnizca NIS'in GERCEKTEN TANIMSIZ
/// oldugu gozlemler (kNumericalFailure / has_nis = false) disarida birakilir ve
/// ayrica SAYILIR.
inline NisSeries aggregate_nis(const MonteCarloResult& mc,
                               Scalar confidence = kDefaultConsistencyConfidence) {
  NisSeries out;
  out.runs = static_cast<int>(mc.runs.size());

  std::vector<std::vector<NisObservation>> izgara;
  izgara.reserve(mc.runs.size());
  for (const auto& k : mc.runs) {
    izgara.push_back(k.nis_observations);
  }
  out.status = detail::damga_izgarasi_dogrula(
      izgara, [](const NisObservation& o) { return o.stamp_ns; }, "NIS");
  if (!out.status.ok) {
    return out;
  }

  const std::size_t n = izgara.front().size();
  out.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    NisTimePoint p;
    p.stamp_ns = izgara.front()[i].stamp_ns;
    Scalar toplam = 0;
    for (const auto& kosu : izgara) {
      const NisObservation& o = kosu[i];
      if (!o.has_nis) {
        ++p.undefined_count; // NIS tanimsiz — uydurulmaz, sayilir
        continue;
      }
      toplam += o.nis;
      p.total_dof += o.dof;
      ++p.valid_count;
    }
    out.undefined_total += p.undefined_count;

    if (p.valid_count > 0) {
      p.mean_nis = toplam / static_cast<Scalar>(p.valid_count);
      const ChiSquareBand b = anis_band(p.valid_count, p.total_dof, confidence);
      p.expected_mean = b.expected_mean;
      p.lower_95 = b.lower;
      p.upper_95 = b.upper;
    }
    out.points.push_back(p);
  }
  return out;
}

} // namespace kerteriz_sim
