#pragma once

/// \file
/// Orkestratör ve TAM snapshot sahibi — INTERFACES §5.1 · ADR-15, ADR-19, ADR-20.
///
/// Backend butunluk katmanini bilmez, butunluk katmani backend'i bilmez.
/// Ikisini birlestiren ve `PipelineSnapshot`'in sahibi olan katman budur.
/// Tam snapshot `FilterBackend::save_snapshot()`'a konulsaydi backend kendisine
/// ait olmayan durumu bilmek zorunda kalirdi — ADR-19'un red sebebi icin
/// reddettigi katman karismasinin aynisi (ADR-20).
///
/// FAZ 1 KAPSAMI. `apply()` bugun kapilama ve dislama UYGULAMAZ; backend'in
/// update sonucunu ardisik duzen sonucuna esler. NisMonitor ve FaultDetector
/// pasif iskelettir (Faz 4). Bu durum `kSensorDisabled`, `kSensorExcluded`,
/// `kOriginNotSet` ve `kCloneUnavailable` sebeplerinin Faz 1'de HIC
/// URETILMEDIGI anlamina gelir; sebepler sozlesmede tanimli, uretimleri ilgili
/// fazda gelir.

#include "kerteriz/backends/filter_backend.hpp"
#include "kerteriz/buffer/processing_result.hpp"
#include "kerteriz/integrity/fault_detector.hpp"
#include "kerteriz/integrity/nis_monitor.hpp"
#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/types.hpp"

#include <cassert>
#include <memory>
#include <utility>

namespace kerteriz {

/// Replay'i etkileyen TUM mutable durum — alt bilesen snapshot'larinin
/// bilesimi (ADR-15 kapsam kurali, ADR-20 sahiplik kurali).
///
/// KURAL: yeni stateful bir bilesen eklenirse BURAYA da eklenir. Istisna
/// yoktur; aksi halde replay calisir gorunur ama deterministik olmaz.
struct PipelineSnapshot {
  BackendSnapshot backend;
  NisState nis;     ///< NisMonitor::capture()
  FaultState fault; ///< FaultDetector::capture()
};

class Estimator {
 public:
  Estimator(std::unique_ptr<FilterBackend> backend, NisMonitor nis, FaultDetector fault)
      : backend_(std::move(backend)), nis_(nis), fault_(fault) {
    assert(backend_ != nullptr && "Estimator backend'siz kurulamaz");
  }

  /// Yalnizca delege eder. Mutlak zaman backend'de `u.stamp_ns`'ten gelir;
  /// `dt` saniye cinsindendir ve CAGIRAN tarafindan iki TimeNs farkindan
  /// hesaplanir (CONVENTIONS §6). Burada zaman BIRIKTIRILMEZ.
  void predict(const ImuSample& u, Scalar dt) { backend_->predict(u, dt); }

  /// Olcume ozel dal YOKTUR: dordu de ayni generic yoldan gecer.
  ProcessingResult apply(const Measurement& z) {
    const UpdateResult sonuc = backend_->update(z);
    return ProcessingResult{z.name(), z.stamp_ns(), reject_reason_of(sonuc.status), sonuc};
  }

  /// Tam snapshot — alt bilesenlerden TOPLANIR.
  PipelineSnapshot save_snapshot() const {
    return PipelineSnapshot{backend_->save_snapshot(), nis_.capture(), fault_.capture()};
  }

  /// Tam snapshot — alt bilesenlere DAGITILIR. Hicbiri atlanmaz.
  void restore_snapshot(const PipelineSnapshot& s) {
    backend_->restore_snapshot(s.backend);
    nis_.restore(s.nis);
    fault_.restore(s.fault);
  }

  const FilterBackend& backend() const { return *backend_; }
  EstimatorMode mode() const { return backend_->mode(); }

  /// Faz 4 bilesenlerine erisim — bugun yalnizca snapshot testleri icin.
  const NisMonitor& nis_monitor() const { return nis_; }
  const FaultDetector& fault_detector() const { return fault_; }

 private:
  std::unique_ptr<FilterBackend> backend_;
  NisMonitor nis_;
  FaultDetector fault_;
};

} // namespace kerteriz
