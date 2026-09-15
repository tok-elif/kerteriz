#pragma once

/// \file
/// Ardisik duzen sonucu ve red sebebi — INTERFACES §5 · ADR-19, ADR-24.
///
/// RED SEBEBI ARDISIK DUZEN SEVIYESINDEDIR (ADR-19). Backend NE OLDUGUNU
/// soyler (`UpdateStatus`), ardisik duzen NEDEN oldugunu siniflandirir
/// (`RejectReason`). Bu yuzden `kTooOld` veya `kSensorExcluded` gibi sebepler
/// `UpdateResult`'a EKLENMEZ: backend bunlari bilemez.

#include "kerteriz/backends/filter_backend.hpp"
#include "kerteriz/types.hpp"

#include <cstdlib>
#include <optional>
#include <string_view>

namespace kerteriz {

enum class RejectReason {
  kNone,             ///< kabul edildi
  kTooOld,           ///< tampon penceresinin disinda          -> tampon karari
  kOutOfOrderDrop,   ///< geri sarma mumkun degil              -> tampon karari
  kSensorDisabled,   ///< yapilandirmada kapali                -> yapilandirma
  kSensorExcluded,   ///< butunluk katmani disladi             -> butunluk karari (Faz 4)
  kOriginNotSet,     ///< cografi olcum, orijin henuz yok      -> yapilandirma
  kChiSquareGate,    ///< filtre kapisi eledi                  -> backend karari
  kNumericalFailure, ///< innovation kovaryansi cozulemedi     -> backend karari (ADR-24)
  kCloneUnavailable  ///< gerekli klon yok veya dusurulmus     -> durum karari
};

/// Backend sonucu -> red sebebi eslemesi (INTERFACES §5, ADR-24).
///
///   kAccepted          -> kNone
///   kChiSquareRejected -> kChiSquareGate
///   kNumericalFailure  -> kNumericalFailure
///
/// `default:` YAZILMAZ. Yeni bir UpdateStatus eklendiginde derleyici eksik
/// dali bildirsin isteriz; `default` onu sessizce yutardi. Switch sonrasindaki
/// abort yalnizca gecersiz bir enum degeri icin ulasilabilirdir.
inline RejectReason reject_reason_of(UpdateStatus status) {
  switch (status) {
  case UpdateStatus::kAccepted:
    return RejectReason::kNone;
  case UpdateStatus::kChiSquareRejected:
    return RejectReason::kChiSquareGate;
  case UpdateStatus::kNumericalFailure:
    return RejectReason::kNumericalFailure;
  }
  std::abort();
}

/// Bir olcumun ardisik duzendeki TAM sonucu.
struct ProcessingResult {
  std::string_view sensor;
  TimeNs stamp_ns = 0;
  RejectReason reason = RejectReason::kNone;
  /// Yalnizca olcum backend'e ULASTIYSA doludur. `kTooOld` gibi tampon
  /// kararlarinda bostur — backend hic cagrilmamistir, uydurma bir
  /// UpdateResult uretmek olmayan bir filtre islemi raporlamak olurdu.
  std::optional<UpdateResult> update;
};

} // namespace kerteriz
