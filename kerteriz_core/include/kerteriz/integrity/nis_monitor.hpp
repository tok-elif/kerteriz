#pragma once

/// \file
/// NIS izleyicisi — INTERFACES §6 · ADR-15, ADR-20.
///
/// ======================= FAZ 1'DE NE VAR, NE YOK ===========================
///
/// Bu dosya F1.7'de YALNIZCA SNAPSHOT ISKELETIDIR. Gercek davranis FAZ 4'tur.
///
/// VAR:  NisState deger tipi, capture()/restore() cifti.
/// YOK:  kabul istatistigi, ortalama NIS, kabul orani, binom bandi,
///       sensor saglik makinesi, esik ayari.
///
/// Neden bugun var. ADR-15'in kapsam kurali "replay sonucunu etkileyen tum
/// mutable state snapshot'a dahildir — istisna yoktur" der; ADR-20 bu kurali
/// bilesen bazina dagitir. Iskelet bugun kurulmazsa, Faz 4'te bileseni
/// PipelineSnapshot'a eklemeyi unutmak SESSIZ bir determinizm hatasi uretir:
/// replay calisir gorunur ama ayni olaylar farkli sonuc verir. Bos bir yer
/// bugun ayrilirsa o hata dogamaz.

#include "kerteriz/types.hpp"

#include <array>
#include <cstddef>
#include <type_traits>

namespace kerteriz {

/// Bir binary'nin izleyebilecegi en fazla sensor sayisi. Sabit kapasite
/// (CONVENTIONS §8.1): snapshot deger semantiklidir ve heap tasimaz.
inline constexpr int kMaxTrackedSensors = 16;

/// NisMonitor'un tasidigi tum mutable durum.
///
/// ICERIK FAZ 4'E AITTIR ve orada tamamen degisecektir. Bugunku alan bir YER
/// TUTUCUDUR; Faz 1 onu NE YAZAR NE OKUR.
///
/// Tamamen bos bir struct yazilabilirdi, ama o zaman "restore gercekten
/// degeri tasidi mi" sorusu SINANAMAZDI: her capture() ayni bos degeri
/// dondururdu ve round-trip testi bos gecerdi. Tek bir sabit kapasiteli alan,
/// iskeletin bugun gercekten test edilebilmesini saglar.
struct NisState {
  std::array<Scalar, kMaxTrackedSensors> yer_tutucu{};

  friend bool operator==(const NisState& a, const NisState& b) {
    return a.yer_tutucu == b.yer_tutucu;
  }
  friend bool operator!=(const NisState& a, const NisState& b) { return !(a == b); }
};

static_assert(std::is_trivially_copyable<NisState>::value, "snapshot deger semantikli olmali");

/// Faz 4 bileseninin PASIF iskeleti.
///
/// `record()`, `mean_nis()`, `acceptance_rate()` ve `acceptance_within_band()`
/// INTERFACES §6'da tanimlidir ama BURADA YOKTUR. Yarim bir uygulama, yanlis
/// bir kabul orani bildiren calisir bir arayuzden daha durustur: olmayan
/// fonksiyon derlenmez, yanlis olan sessizce kullanilir.
class NisMonitor {
 public:
  NisState capture() const { return durum_; }
  void restore(const NisState& s) { durum_ = s; }

 private:
  NisState durum_{};
};

} // namespace kerteriz
