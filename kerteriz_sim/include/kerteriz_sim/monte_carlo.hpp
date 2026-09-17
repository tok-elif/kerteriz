#pragma once

/// \file
/// Monte Carlo kosum altyapisi — F2.1.
///
/// ====================== BU DOSYA ISTATISTIK URETMEZ =========================
///
/// Burada N bagimsiz realizasyon KOSULUR ve kayitlari toplanir. NEES, NIS
/// bandi, chi-kare siniri veya kapsama HESAPLANMAZ — o F2.2'dir; E1 sekli ve
/// 500 kosumluk uretim deneyi F2.3'tur. Altyapinin belirli bir grafige
/// baglanmamasi kasitlidir.
///
/// ========================= TOHUM TURETME KURALI =============================
///
/// Kosu basina tohumlar (master_seed, run_index) ciftinin SAF FONKSIYONUDUR:
///
///   kosu_anahtari = splitmix64(master_seed + kKosuAdimi * run_index)
///   initial_seed  = splitmix64(kosu_anahtari ^ kBaslangicAlani)
///   imu_seed      = splitmix64(kosu_anahtari ^ kImuAlani)
///   gnss_seed     = splitmix64(kosu_anahtari ^ kGnssAlani)
///
/// Bu tanimin iki sonucu vardir ve ikisi de test edilir:
///
///   1. ON EK KARARLILIGI. run_index'in tohumu N'e BAGLI DEGILDIR, bu yuzden
///      N = 3 ile N = 5 kosumlarinin ilk ucu BIREBIR aynidir. Tohumlar sirayla
///      ureteceginden turetilseydi (ornegin tek bir akistan cekilseydi), N'i
///      buyutmek eski kosumlari da degistirir ve birikmis sonuclari
///      gecersiz kilardi.
///   2. AKIS BAGIMSIZLIGI. UC akis da farkli alan tuzlariyla turetilir; biri
///      digerinin devami degildir. Baslangic durumu akisinin ayri olmasi
///      ozellikle onemlidir: baslangic ornekleme politikasini degistirmek
///      IMU/GNSS gurultu dizilerini KAYDIRMAMALIDIR, aksi halde iki degisiklik
///      birbirine karisir ve kiyas anlamini yitirir.
///
/// SplitMix64 kasten secildi: tanimi bit seviyesinde sabittir, durumu yoktur
/// ve tek cagride iyi karisim verir. Duvar saati veya `std::random_device`
/// KULLANILMAZ — deney tekrar uretilebilir olmali.

#include "kerteriz_sim/experiment.hpp"

#include <cstdint>
#include <vector>

namespace kerteriz_sim {

namespace detail {

/// SplitMix64 — sabit, durumsuz karistirici.
inline std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ULL;
  std::uint64_t z = x;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

/// Ardisik kosu anahtarlarini ayirmak icin buyuk, tek sayili adim.
inline constexpr std::uint64_t kKosuAdimi = 0x632BE59BD9B4E019ULL;
inline constexpr std::uint64_t kImuAlani = 0x9E3779B97F4A7C15ULL;
inline constexpr std::uint64_t kGnssAlani = 0xC2B2AE3D27D4EB4FULL;
inline constexpr std::uint64_t kBaslangicAlani = 0x165667B19E3779F9ULL;

} // namespace detail

/// (master_seed, run_index) -> kosu tohumlari. SAF FONKSIYON.
inline RunSeeds derive_seeds(std::uint64_t master_seed, int run_index) {
  const auto i = static_cast<std::uint64_t>(static_cast<std::int64_t>(run_index));
  const std::uint64_t anahtar = detail::splitmix64(master_seed + detail::kKosuAdimi * i);
  RunSeeds s;
  s.initial_state = detail::splitmix64(anahtar ^ detail::kBaslangicAlani);
  s.imu = detail::splitmix64(anahtar ^ detail::kImuAlani);
  s.gnss = detail::splitmix64(anahtar ^ detail::kGnssAlani);
  return s;
}

struct MonteCarloConfig {
  ScenarioConfig scenario = noisy_scenario();
  int runs = 8;
  std::uint64_t master_seed = 20260916ULL;

  /// Zaman serisi kaydi Monte Carlo'da varsayilan olarak KAPALIDIR: ornek
  /// basina gercek + kestirim + 15x15 kovaryans tutulur ve 500 realizasyonda
  /// bellek hizla buyur. F2.2 acacaktir.
  bool record_state_samples = false;
  int state_sample_stride = 1;
};

struct MonteCarloResult {
  std::uint64_t master_seed = 0;
  std::vector<RunResult> runs;

  /// Baslangic ornekleme veya kosum basarisizligi yasayan realizasyonlar.
  /// Bunlar da `runs` icinde KALIR (sessizce atilmaz); ayrica sayilirlar.
  int failed_runs = 0;
  int total_numerical_failures = 0;
  /// En az bir sayisal basarisizlik yasayan kosu sayisi. Boyle kosular
  /// SESSIZCE ATILMAZ; sonucta kalir ve ayrica sayilir.
  int runs_with_numerical_failure = 0;
};

/// N bagimsiz realizasyon. Seri ve deterministiktir.
///
/// PARALELLESTIRME YOK. Once tohumlu seri sonucun tekrar uretilebilirligi
/// sabitlenir; paralellik ancak kosu seviyesindeki tekrar uretilebilirligi
/// KORUYARAK eklenebilir ve bugun gerekli degildir.
inline MonteCarloResult run_monte_carlo(const MonteCarloConfig& cfg) {
  MonteCarloResult out;
  out.master_seed = cfg.master_seed;
  if (cfg.runs <= 0) {
    return out;
  }
  out.runs.reserve(static_cast<std::size_t>(cfg.runs));

  ScenarioConfig senaryo = cfg.scenario;
  senaryo.record_state_samples = cfg.record_state_samples;
  senaryo.state_sample_stride = cfg.state_sample_stride;

  for (int i = 0; i < cfg.runs; ++i) {
    RunResult r = run_single(senaryo, derive_seeds(cfg.master_seed, i), i);
    if (!r.ok) {
      ++out.failed_runs;
    }
    out.total_numerical_failures += r.counters.numerical_failures;
    if (r.counters.numerical_failures > 0) {
      ++out.runs_with_numerical_failure;
    }
    out.runs.push_back(std::move(r));
  }
  return out;
}

} // namespace kerteriz_sim
