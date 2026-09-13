#pragma once

/// \file
/// Tohumlu rastgele sayi ureteci — PHASE0.md S9.
///
/// DETERMINIZM SOZLESMESI. Ayni tohum ayni akisi uretir. Bunun icin:
///
///   - Motor std::mt19937_64. Standart bu motorun ciktisini BIREBIR tanimlar.
///   - Normal dagilim ELDE yazilir (Box-Muller). std::normal_distribution
///     KULLANILMAZ: standart onun algoritmasini ve ic durumunu belirtmez, yani
///     ayni tohum farkli standart kutuphanelerde farkli sayilar verebilir.
///   - [0,1) esleme bit seviyesinde sabittir: (raw >> 11) * 2^-53.
///
/// Sinir: log/sqrt/sin/cos degerleri IEEE754 double olsa da libm'ler arasinda
/// son bitte farklilasabilir. Bit-bit ayniliк garantisi AYNI derleyici ve libm
/// icin gecerlidir; ham motor akisi ise her yerde ayni.

#include "kerteriz/types.hpp"

#include <cmath>
#include <cstdint>
#include <random>

namespace kerteriz_sim {

using kerteriz::Scalar;
using kerteriz::Vec3;

class SeededRng {
 public:
  explicit SeededRng(std::uint64_t seed) : seed_(seed), gen_(seed) {}

  std::uint64_t seed() const { return seed_; }

  /// Tohuma geri doner. Ayni cagri dizisi ayni sayilari verir.
  void reset() {
    gen_.seed(seed_);
    has_spare_ = false;
    spare_ = Scalar(0);
  }

  /// Ham motor ciktisi — standart tarafindan birebir tanimli.
  std::uint64_t raw() { return gen_(); }

  /// [0, 1) duzgun dagilim. Esleme sabittir.
  Scalar uniform01() {
    return static_cast<Scalar>(gen_() >> 11) * (Scalar(1) / Scalar(9007199254740992ULL));
  }

  /// N(0, 1). Box-Muller, yedek deger saklanir.
  Scalar gaussian() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    Scalar u1 = uniform01();
    if (u1 < kMinU) {
      u1 = kMinU; // log(0) korumasi
    }
    const Scalar u2 = uniform01();
    const Scalar buyukluk = std::sqrt(Scalar(-2) * std::log(u1));
    const Scalar aci = Scalar(2) * kPi * u2;
    spare_ = buyukluk * std::sin(aci);
    has_spare_ = true;
    return buyukluk * std::cos(aci);
  }

  Vec3 gaussian3() {
    Vec3 v;
    v.x() = gaussian();
    v.y() = gaussian();
    v.z() = gaussian();
    return v;
  }

 private:
  static constexpr Scalar kPi = Scalar(3.14159265358979323846);
  static constexpr Scalar kMinU = Scalar(1e-300);

  std::uint64_t seed_;
  std::mt19937_64 gen_;
  bool has_spare_ = false;
  Scalar spare_ = Scalar(0);
};

} // namespace kerteriz_sim
