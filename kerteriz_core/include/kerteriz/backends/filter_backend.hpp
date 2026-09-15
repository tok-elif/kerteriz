#pragma once

/// \file
/// Filtre backend'i soyut arayuzu — INTERFACES §4 · ADR-19, ADR-20, ADR-24.
///
/// Uygulamalar: `EskfBackend` (Faz 1) · `InekfBackend` (Faz 3) ·
/// `IteratedEskfBackend` (opsiyonel). Ucu de ayni `Measurement` nesnelerini
/// tuketir; olcum siniflari backend'i BILMEZ.

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/state/nav_state.hpp"
#include "kerteriz/types.hpp"

#include <optional>

namespace kerteriz {

/// Filtre guncellemesinin sonucu (ADR-24).
enum class UpdateStatus {
  kAccepted,          ///< nis dolu, state/P guncellendi
  kChiSquareRejected, ///< nis dolu, state/P DEGISMEDI
  kNumericalFailure   ///< S pozitif tanimli cozulemedi: nis BOS, state/P DEGISMEDI
};

/// YALNIZCA filtre seviyesi bilgisi tasir — "bayat olcum" veya "FDI disladi"
/// gibi sebepler buraya ait DEGILDIR (ADR-19).
struct UpdateResult {
  UpdateStatus status;
  std::optional<Scalar> nis; ///< yalnizca kNumericalFailure'da bos
  int dof;                   ///< = residual_dim(), her durumda gecerli
  Scalar threshold;          ///< kullanilan chi-kare esigi, her durumda gecerli
};

/// Backend'in KENDI durumu. Butunluk katmanini ICERMEZ (ADR-20).
struct BackendSnapshot {
  NavState state;
  NavCovariance covariance;
  TimeNs stamp_ns; ///< zaman snapshot'in parcasidir (ADR-15)
};

class FilterBackend {
 public:
  FilterBackend() = default;
  virtual ~FilterBackend() = default;
  FilterBackend(const FilterBackend&) = default;
  FilterBackend& operator=(const FilterBackend&) = default;
  FilterBackend(FilterBackend&&) = default;
  FilterBackend& operator=(FilterBackend&&) = default;

  /// dt saniye; backend mutlak zamani u.stamp_ns'ten alir.
  virtual void predict(const ImuSample& u, Scalar dt) = 0;
  /// CONVENTIONS §4'teki Joseph + simetrizasyon uygulanir. Explicit inverse yasak.
  virtual UpdateResult update(const Measurement& z) = 0;

  virtual const NavState& state() const = 0;
  virtual const NavCovariance& covariance() const = 0;
  virtual TimeNs stamp_ns() const = 0;
  virtual EstimatorMode mode() const = 0;

  /// Geri sarma sozlesmesi. reset(x, P) YOKTUR — zaman ve duzen kayboldugu
  /// icin sessiz semantik hata uretiyordu (ADR-15).
  /// Backend YALNIZCA kendi durumunu kaydeder (ADR-20).
  virtual BackendSnapshot save_snapshot() const = 0;
  virtual void restore_snapshot(const BackendSnapshot&) = 0;

  virtual CloneId push_clone() = 0;
  virtual void drop_clone(CloneId) = 0;

  /// Zayif gozlemlenebilir alt uzaylar — teshis icin (ADR-18, Faz 4).
  virtual ArrayView<const WeakDirection> weak_directions() const { return {}; }
};

} // namespace kerteriz
