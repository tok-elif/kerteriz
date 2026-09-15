#pragma once

/// \file
/// Olcum modeli soyut altyapisi — INTERFACES §3.
///
/// F1.3 yalnizca SOYUT tipleri getirir. Somut sensor modelleri (`GnssPosition`,
/// `WheelVelocity`, ...) F1.4 ve sonrasindadir.
///
/// Genisleme noktasi burasidir: yeni sensor = bu arayuzun yeni bir uygulamasi.

#include "kerteriz/state/nav_state.hpp"
#include "kerteriz/types.hpp"

#include <cassert>
#include <cstdlib>
#include <string_view>

namespace kerteriz {

/// Olcumun gorecegi durumlar: guncel durum + talep ettigi klonlar (ADR-9, ADR-14).
///
/// KLON ERISIMI HAKKINDA. Dondurulmus kurucu yalnizca `current` alir; klon
/// NavState'lerini iceri gecirecek bir yol YOKTUR. Dolayisiyla bu kurulumda
/// klon kumesi her zaman bostur ve `clone()` sozlesmedeki "Yoksa cagri hatadir"
/// dalina duser. Bu bir eksiklik degil, imzanin dogrudan sonucudur: klon
/// tuketicisi `RelativePose` Faz 3'tedir ve klon degerini `NavState` olarak
/// sunmanin semantigi o fazda ele alinacaktir. Burada uydurma veri URETILMEZ.
class StateBundle {
 public:
  explicit StateBundle(const NavState& current) : current_(&current) {}

  const NavState& current() const { return *current_; }

  /// Olcumun required_clones() ile talep ettigi klon. Yoksa cagri hatadir.
  ///
  /// F1.3'te klon kumesi HER ZAMAN bostur, dolayisiyla bu yol her cagrida
  /// hatadir ve HER BUILD'DE sonlandirir.
  ///
  /// `assert` TEK BASINA YETMEZ: NDEBUG altinda kalkar ve ardindan gelen
  /// `return *current_` calisirdi — yani guncel durum, istenen KLON gibi
  /// sunulurdu. Sessiz yanlis veri, tanimsiz davranistan daha tehlikelidir.
  /// Bu yuzden kosulsuz `std::abort()` kullanilir; `assert` yalnizca debug
  /// build'de okunabilir bir mesaj birakmak icindir.
  [[noreturn]] const NavState& clone(CloneId id) const {
    (void)id;
    (void)current_;
    assert(false && "F1.3: StateBundle klon tasimaz; klon tuketicisi Faz 3'tedir");
    std::abort();
  }

 private:
  const NavState* current_;
};

/// Tahsissiz calisma alani (CONVENTIONS §8.1). Cagiran sahiptir, olcum doldurur.
struct MeasurementWorkspace {
  ResVec r;     ///< artik,   ilk dim satiri gecerli
  JacMat J_res; ///< residual Jacobian, dim x state.active_dof() blogu gecerli
  ResMat R;     ///< olcum gurultusu, dim x dim blogu gecerli
  int dim = 0;  ///< bu olcumun gercek boyutu

  /// Deterministik temizlik. Backend her update'te cagirir; boylece bir olcumun
  /// doldurmadigi bolge onceki olcumden artakalan degeri TASIYAMAZ.
  void clear() {
    r.setZero();
    J_res.setZero();
    R.setZero();
    dim = 0;
  }
};

class Measurement {
 public:
  Measurement() = default;
  virtual ~Measurement() = default;
  Measurement(const Measurement&) = default;
  Measurement& operator=(const Measurement&) = default;
  Measurement(Measurement&&) = default;
  Measurement& operator=(Measurement&&) = default;

  virtual TimeNs stamp_ns() const = 0;
  virtual int residual_dim() const = 0; ///< <= kMaxResidualDim
  /// YAML'daki sensor adi. Omru olcumden uzundur (yapilandirmada sahiplenilir).
  virtual std::string_view name() const = 0;

  /// Bu olcumun ihtiyac duydugu klonlar. Bos ise mutlak olcumdur.
  virtual ArrayView<const CloneId> required_clones() const { return {}; }

  /// w.r = z (-) h(X),  w.J_res = dr(X (+) d)/dd,  w.R = gurultu,
  /// w.dim = residual_dim()
  ///
  /// J_res sutunlari durumun TAM aktif duzenine goredir. Tahsis yapmaz.
  virtual void evaluate(const StateBundle& states, MeasurementWorkspace& w) const = 0;
};

} // namespace kerteriz
