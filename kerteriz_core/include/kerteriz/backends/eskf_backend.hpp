#pragma once

/// \file
/// ESKF backend — INTERFACES §4 · CONVENTIONS §3, §4, §6 · ADR-19, ADR-20, ADR-24.
///
/// ============================ AKIS =========================================
///
///   predict : ImuPropagator'a DELEGE eder; mutlak zaman u.stamp_ns'ten alinir
///   update  : workspace temizle -> evaluate -> NIS -> kapi -> (kabulse)
///             linear_update -> state.plus(delta)
///
/// Yayilim denklemi burada TEKRAR YAZILMAZ; guncelleme matematigi de
/// `linear_update()`'in disinda ikinci kez kurulmaz.
///
/// ===================== KAPI MUTASYONDAN ONCE ===============================
///
/// `linear_update()` NIS ile birlikte P'yi de gunceller, dolayisiyla kapi icin
/// kullanilamaz. NIS `innovation_nis()` ile P'ye dokunulmadan hesaplanir; red
/// halinde ne durum ne kovaryans degisir. Once mutate edip geri alma YOKTUR.
///
/// ======================= F1.3 SINIRLARI ====================================
///
/// mode() daima kNominal dondurur. ADR-18 gecis SIRASINI donduruyor ama gecisi
/// tetikleyen OLAYLAR (origin politikasi, sensor sagligi) backend'in gormedigi
/// seylerdir; F1.3'te gecis mantigi UYGULANMAZ ve uydurulmaz.
///
/// Kapi backend seviyesinde TEK bir guven seviyesi kullanir. CONVENTIONS §11
/// gate'i sensor basina tanimlar, fakat dondurulmus `Measurement` arayuzunde
/// gate alani yoktur; sensor basina cozunurluk Estimator geldiginde mumkun
/// olacaktir.

#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/process/imu_propagator.hpp"
#include "kerteriz/state/nav_state.hpp"
#include "kerteriz/types.hpp"
#include "kerteriz/util/chi_square.hpp"
#include "kerteriz/util/linear_update.hpp"

#include <array>
#include <cassert>
#include <optional>
#include <utility>

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

struct EskfConfig {
  NavState x0;
  NavCovariance P0;
  TimeNs t0;
  ImuNoiseParams imu;
  Scalar chi2_confidence;
};

class EskfBackend {
 public:
  explicit EskfBackend(EskfConfig config)
      : state_(std::move(config.x0)), covariance_(std::move(config.P0)), stamp_ns_(config.t0),
        propagator_(config.imu) {
    assert(config.chi2_confidence > Scalar(0) && config.chi2_confidence < Scalar(1) &&
           "chi2_confidence (0,1) araliginda olmalidir");
    // Esikler KURUCUDA hesaplanir; sicak yolda yalnizca onbellek okunur.
    for (int dof = 1; dof <= kMaxResidualDim; ++dof) {
      esikler_[static_cast<std::size_t>(dof - 1)] =
          chi_square_quantile(config.chi2_confidence, dof);
    }
  }

  /// dt saniye; backend mutlak zamani u.stamp_ns'ten alir.
  void predict(const ImuSample& u, Scalar dt) {
    propagator_.propagate(state_, covariance_, u, dt);
    stamp_ns_ = u.stamp_ns; // dt BIRIKTIRILMEZ (CONVENTIONS §6)
  }

  /// CONVENTIONS §4'teki Joseph + simetrizasyon `linear_update()` icinde
  /// uygulanir. Explicit inverse yasak.
  UpdateResult update(const Measurement& z) {
    const int dim = z.residual_dim();
    assert(dim > 0 && dim <= kMaxResidualDim);
    const Scalar esik = esik_of(dim);

    calisma_alani_.clear();
    const StateBundle bundle(state_);
    z.evaluate(bundle, calisma_alani_);
    assert(calisma_alani_.dim == dim && "evaluate() beyan edilen residual_dim'i doldurmadi");

    const int dof = state_.active_dof();

    // KAPI MUTASYONDAN ONCE: P'ye dokunulmaz.
    const LinearUpdateResult olcum = innovation_nis(calisma_alani_.J_res, calisma_alani_.r,
                                                    calisma_alani_.R, dim, dof, covariance_);

    if (!olcum.spd_ok) {
      // ADR-24: NIS tanimsiz. Sentinel uretilmez.
      return UpdateResult{UpdateStatus::kNumericalFailure, std::nullopt, dim, esik};
    }
    if (olcum.nis > esik) {
      return UpdateResult{UpdateStatus::kChiSquareRejected, olcum.nis, dim, esik};
    }

    StateVec delta = StateVec::Zero();
    const LinearUpdateResult guncelleme = linear_update(
        calisma_alani_.J_res, calisma_alani_.r, calisma_alani_.R, dim, dof, covariance_, delta);
    assert(guncelleme.spd_ok && "ayni S iki farkli SPD sonucu verdi");

    // delta = -P J_res^T S^-1 r  (ADR-13). Isaret BURADA cevrilmez.
    state_ = state_.plus(delta);
    return UpdateResult{UpdateStatus::kAccepted, guncelleme.nis, dim, esik};
  }

  const NavState& state() const { return state_; }
  const NavCovariance& covariance() const { return covariance_; }
  TimeNs stamp_ns() const { return stamp_ns_; }
  EstimatorMode mode() const { return EstimatorMode::kNominal; }

  BackendSnapshot save_snapshot() const { return BackendSnapshot{state_, covariance_, stamp_ns_}; }

  void restore_snapshot(const BackendSnapshot& s) {
    state_ = s.state;
    covariance_ = s.covariance;
    stamp_ns_ = s.stamp_ns;
  }

  /// Klon, guncel poza TAM KORELE baslar (ADR-14). Secici Jc, klon tegetini
  /// [dtheta, dp] cekirdek indekslerine baglar:
  ///
  ///   P_xc = P Jc^T,   P_cc = Jc P Jc^T
  ///
  /// NavState::push_clone() basarisiz olursa kovaryans DEGISMEZ.
  CloneId push_clone() {
    const int n = state_.active_dof();
    const CloneId id = state_.push_clone();
    if (id == kInvalidClone) {
      return kInvalidClone;
    }
    const int ofs = state_.clone_offset(id);
    assert(ofs == n && "klon blogu aktif duzenin sonuna eklenmedi");

    // P_cc once: ust sol n x n blogundan okur, yazma o bolgeye dokunmaz.
    for (int i = 0; i < kCloneDof; ++i) {
      for (int k = 0; k < kCloneDof; ++k) {
        covariance_(ofs + i, ofs + k) = covariance_(kKlonKaynak[i], kKlonKaynak[k]);
      }
    }
    for (int i = 0; i < kCloneDof; ++i) {
      covariance_.col(ofs + i).head(n) = covariance_.col(kKlonKaynak[i]).head(n);
      covariance_.row(ofs + i).head(n) = covariance_.row(kKlonKaynak[i]).head(n);
    }
    return id;
  }

  /// Marjinalizasyon: ilgili 6 satir/sutun KALDIRILIR. Jointly-Gaussian bir
  /// blogu marjinallestirmek tam olarak budur — Schur tumleyeni GEREKMEZ, o
  /// kosullandirma islemidir.
  void drop_clone(CloneId id) {
    assert(state_.has_clone(id) && "var olmayan klon dusuruluyor");
    const int n = state_.active_dof();
    const int ofs = state_.clone_offset(id);
    const int yeni = n - kCloneDof;

    // Sutunlari sola kaydir (hedef < kaynak, ileri sira guvenli).
    for (int c = ofs; c < yeni; ++c) {
      covariance_.col(c).head(n) = covariance_.col(c + kCloneDof).head(n);
    }
    // Satirlari yukari kaydir.
    for (int r = ofs; r < yeni; ++r) {
      covariance_.row(r).head(yeni) = covariance_.row(r + kCloneDof).head(yeni);
    }
    // Bosalan band deterministik olarak sifirlanir.
    covariance_.block(yeni, 0, kCloneDof, n).setZero();
    covariance_.block(0, yeni, n, kCloneDof).setZero();

    state_.drop_clone(id);
  }

  ArrayView<const WeakDirection> weak_directions() const { return {}; }

 private:
  /// Klon tegeti [dtheta, dp] — cekirdek indeksleri (CONVENTIONS §5.1).
  static constexpr std::array<int, kCloneDof> kKlonKaynak{0, 1, 2, 6, 7, 8};

  Scalar esik_of(int dim) const { return esikler_[static_cast<std::size_t>(dim - 1)]; }

  NavState state_;
  NavCovariance covariance_;
  TimeNs stamp_ns_;
  ImuPropagator propagator_;
  std::array<Scalar, kMaxResidualDim> esikler_{};
  MeasurementWorkspace calisma_alani_{};
};

} // namespace kerteriz
