#pragma once

/// \file
/// Ortak tipler — INTERFACES.md §0'in birebir uygulamasi.
///
/// Bu dosya TEK BASINA derlenebilir: yalnizca standart kutuphane ve Eigen/Core
/// gerektirir, kerteriz basligi icermez.
///
/// Denetim (2) (CLAUDE.md §6.1) burada derleme zamaninda zorlanir; kayit
/// sirasindaki kapasite kontrolu NavState ile (Faz 1) gelir.

#include <cstddef>
#include <cstdint>
#include <string_view>

#include <Eigen/Core>

namespace kerteriz {

using Scalar = double;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

/// Mutlak zaman. CONVENTIONS §6: timestamp asla double saniye degildir.
using TimeNs = std::int64_t;

/// Durum boyutu sinirlari — CONVENTIONS §5.3
inline constexpr int kCoreDof = 15;
inline constexpr int kMaxAugmentDof = 48;
inline constexpr int kMaxStateDof = kCoreDof + kMaxAugmentDof;
inline constexpr int kMaxResidualDim = 12;

// --- Denetim (2): durum kapasitesi -----------------------------------------
// kMaxStateDof toplamdan turetilir; bu static_assert birinin onu duz bir
// sayiyla degistirmesine karsi bekcidir.
static_assert(kCoreDof + kMaxAugmentDof == kMaxStateDof, "state capacity mismatch");
static_assert(kMaxResidualDim > 0 && kMaxResidualDim <= kMaxStateDof,
              "residual dimension out of range");

/// Sabit kapasiteli, calisma aninda aktif boyutu degisen tipler.
using StateVec = Eigen::Matrix<Scalar, kMaxStateDof, 1>;
using StateMat = Eigen::Matrix<Scalar, kMaxStateDof, kMaxStateDof>;
using ResVec = Eigen::Matrix<Scalar, kMaxResidualDim, 1>;
using ResMat = Eigen::Matrix<Scalar, kMaxResidualDim, kMaxResidualDim>;
using JacMat = Eigen::Matrix<Scalar, kMaxResidualDim, kMaxStateDof>;

/// C++17 uyumlu tahsissiz gorunum. std::span C++20 oldugu icin kullanilmaz (R8).
template <typename T>
class ArrayView {
public:
  constexpr ArrayView() = default;
  constexpr ArrayView(const T *data, std::size_t size) : data_(data), size_(size) {}

  constexpr const T *data() const { return data_; }
  constexpr std::size_t size() const { return size_; }
  constexpr bool empty() const { return size_ == 0; }
  constexpr const T &operator[](std::size_t i) const { return data_[i]; }
  constexpr const T *begin() const { return data_; }
  constexpr const T *end() const { return data_ + size_; }

private:
  const T *data_ = nullptr;
  std::size_t size_ = 0;
};

/// Zayif gozlemlenebilir yon teshisi (ADR-18).
struct WeakDirection {
  std::string_view label; ///< "yaw", "wheel_scale", ...
  Scalar sigma;           ///< bu yondeki standart sapma
};

struct ImuSample {
  TimeNs stamp_ns;
  Vec3 gyro;  // rad/s, olculen (bias dahil)
  Vec3 accel; // m/s^2, olculen (bias dahil)
};

/// Klon tanitici. kInvalidClone: klon yok.
enum class CloneId : std::int32_t {};
inline constexpr CloneId kInvalidClone{-1};

/// Kestirimci seviyesi calisma modu. SensorHealth (INTERFACES §6) ile
/// KARISTIRILMAZ: bu sistemin durumu, o tek bir sensorun durumu.
enum class EstimatorMode {
  kUninitialized, ///< orijin veya baslangic durumu henuz yok
  kInitializing,  ///< toplaniyor (statik hizalama, ilk fix bekleniyor)
  kNominal,       ///< tum beklenen sensorler saglikli
  kDegraded,      ///< calisiyor ama bir veya daha fazla yardim kaynagi yok
  kFaulted        ///< kestirim guvenilir degil
};

} // namespace kerteriz
