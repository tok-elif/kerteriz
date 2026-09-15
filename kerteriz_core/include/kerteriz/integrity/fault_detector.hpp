#pragma once

/// \file
/// Ariza tespiti — INTERFACES §6 · ADR-15, ADR-20.
///
/// ======================= FAZ 1'DE NE VAR, NE YOK ===========================
///
/// F1.7'de YALNIZCA SNAPSHOT ISKELETI. Gercek davranis FAZ 4'tur.
///
/// VAR:  FaultState deger tipi, capture()/restore() cifti.
/// YOK:  sensor saglik makinesi, dislama politikasi, FDI, koruma seviyesi.
///
/// Gerekce nis_monitor.hpp ile aynidir: ADR-15'in "istisna yoktur" kurali
/// bilesen eklendiginde degil, BUGUN yer ayrilarak zorlanir.

#include "kerteriz/integrity/nis_monitor.hpp"
#include "kerteriz/types.hpp"

#include <array>
#include <cstdint>
#include <type_traits>

namespace kerteriz {

/// FaultDetector'un tasidigi tum mutable durum.
///
/// ICERIK FAZ 4'E AITTIR. Bugunku alan YER TUTUCUDUR; Faz 1 onu ne yazar ne
/// okur. Bos struct yerine tek sabit kapasiteli alan tutulmasinin sebebi
/// NisState'teki ile aynidir: round-trip testi boylece bos gecmez.
struct FaultState {
  std::array<std::uint8_t, kMaxTrackedSensors> yer_tutucu{};

  friend bool operator==(const FaultState& a, const FaultState& b) {
    return a.yer_tutucu == b.yer_tutucu;
  }
  friend bool operator!=(const FaultState& a, const FaultState& b) { return !(a == b); }
};

static_assert(std::is_trivially_copyable<FaultState>::value, "snapshot deger semantikli olmali");

/// Faz 4 bileseninin PASIF iskeleti.
///
/// `evaluate()` ve `is_excluded()` INTERFACES §6'da tanimlidir ama BURADA
/// YOKTUR. Estimator Faz 1'de hicbir sensoru dislamaz; dislama iddiasi tasiyan
/// bir no-op `is_excluded()` "hicbir sey disli degil" diye YANLIS bir guvence
/// verirdi.
class FaultDetector {
 public:
  FaultState capture() const { return durum_; }
  void restore(const FaultState& s) { durum_ = s; }

 private:
  FaultState durum_{};
};

} // namespace kerteriz
