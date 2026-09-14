#pragma once

/// \file
/// Navigasyon durumu — INTERFACES §1 · CONVENTIONS §3.1, §5 · ADR-14.
///
/// Sabit 15-DoF cekirdek + sinirli kapasiteli augmentation deposu. Tek
/// depolama, IKI AYRI YASAM DONGUSU (ADR-14):
///
///   PersistentCalibration — yapilandirmada kaydedilir, oturum boyunca kalir
///   Clone                 — calisma aninda push/drop edilir
///
/// Aktif duzen (CONVENTIONS §5.2):
///
///   [ cekirdek (15) | kalici (kayit sirasina gore) | klonlar (push sirasina gore) ]
///
/// Depolama basta kMaxStateDof kapasitesindedir; active_dof() calisma aninda
/// degisir, heap'te yeniden boyutlandirma YAPILMAZ (CONVENTIONS §5.3, §8.1).
///
/// KOVARYANS BURADA YOKTUR. drop_clone() yalnizca durum duzenini ve degerlerini
/// yonetir; kovaryans marjinalizasyonu backend'in isidir (INTERFACES §4).

#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <array>
#include <cassert>
#include <cstdint>
#include <optional>
#include <string_view>

namespace kerteriz {

/// Augmentation blok turu — yasam dongusunu belirler.
enum class AugmentKind {
  kPersistentCalibration, ///< yapilandirmada acilir, oturum boyunca kalir, marjinallestirilmez
  kClone                  ///< calisma aninda push/drop, kullanildiktan sonra marjinallestirilir
};

/// Bir klon blogunun boyutu.
///
/// `push_clone()` dof parametresi ALMAZ, yani boyut derleme zamani sabitidir.
/// Deger poz (yonelim + konum) olarak secildi: ADR-9 stochastic cloning icin
/// Roumeliotis & Burdick 2002'ye atif yapar ve o yontem pozu klonlar; tuketici
/// de goreli bir POZ olcumu olan `RelativePose`'dur. Hiz bileseni goreli poz
/// olcumune girmez, bu yuzden klona dahil edilmez.
inline constexpr int kCloneDof = 6;

/// Her blok en az 1 DoF tuketir; blok sayisi bu yuzden kMaxAugmentDof'u asamaz.
inline constexpr int kMaxAugmentBlocks = kMaxAugmentDof;

class NavState {
 public:
  // ---------------------------------------------------------------------------
  // Cekirdek — derleme zamani, her zaman var
  // ---------------------------------------------------------------------------
  manif::SE_2_3d& extended_pose() { return pose_; }
  const manif::SE_2_3d& extended_pose() const { return pose_; }

  Vec3& gyro_bias() { return gyro_bias_; }
  const Vec3& gyro_bias() const { return gyro_bias_; }

  Vec3& accel_bias() { return accel_bias_; }
  const Vec3& accel_bias() const { return accel_bias_; }

  // ---------------------------------------------------------------------------
  // Augmentation — calisma aninda
  // ---------------------------------------------------------------------------
  int active_dof() const { return kCoreDof + augment_dof_; }
  int augment_dof() const { return augment_dof_; }

  /// Yapilandirma sirasinda cagrilir. Kapasite asilirsa std::nullopt.
  ///
  /// Isimler BENZERSIZDIR: zaten kayitli bir ad ikinci kez verilirse kayit
  /// reddedilir ve mevcut kayit hic degismez. Ayni dof ile gelse bile
  /// idempotent kabul edilmez — kalibrasyon kaydi yapilandirma asamasinda tek
  /// seferliktir.
  ///
  /// `name`'in omru NavState'ten uzun olmalidir; yapilandirma sahiplenir
  /// (INTERFACES §3'teki `Measurement::name` ile ayni kural). Tahsis yapilmaz.
  std::optional<int> register_calibration(std::string_view name, int dof) {
    if (dof <= 0 || dof > kMaxAugmentDof - augment_dof_) {
      return std::nullopt;
    }
    if (blok_sayisi_ >= kMaxAugmentBlocks) {
      return std::nullopt;
    }
    if (kalibrasyon_indeksi(name) >= 0) {
      return std::nullopt;
    }

    // Kalibrasyonlar klonlardan ONCE gelir. Daha once klon push edilmisse
    // onlarin degerleri ve kayitlari yukari kaydirilir.
    const int ofset = kCoreDof + kalici_dof_;
    klonlari_kaydir(kalici_dof_, dof);

    // Klon KAYITLARI da bir sira yukari kayar; aksi halde yeni kalibrasyon
    // ilk klonun kaydinin uzerine yazardi.
    for (int k = blok_sayisi_; k > kalici_blok_sayisi_; --k) {
      bloklar_[k] = bloklar_[k - 1];
    }

    Blok b;
    b.kind = AugmentKind::kPersistentCalibration;
    b.name = name;
    b.id = kInvalidClone;
    b.dof = dof;
    bloklar_[kalici_blok_sayisi_] = b;
    ++kalici_blok_sayisi_;
    ++blok_sayisi_;
    kalici_dof_ += dof;
    augment_dof_ += dof;
    return ofset;
  }

  /// Kayitli olmayan ad cagiran hatasidir (INTERFACES §3'teki klon aramasiyla
  /// ayni kural).
  int calibration_offset(std::string_view name) const {
    const int i = kalibrasyon_indeksi(name);
    assert(i >= 0 && "kayitli olmayan kalibrasyon adi");
    return blok_ofseti(i);
  }

  /// Calisma aninda. Kapasite yoksa kInvalidClone.
  CloneId push_clone() {
    if (kCloneDof > kMaxAugmentDof - augment_dof_ || blok_sayisi_ >= kMaxAugmentBlocks) {
      return kInvalidClone;
    }

    Blok b;
    b.kind = AugmentKind::kClone;
    b.name = {};
    b.id = CloneId{siradaki_klon_kimligi_};
    b.dof = kCloneDof;
    bloklar_[blok_sayisi_] = b;
    ++blok_sayisi_;
    augment_dof_ += kCloneDof;
    ++siradaki_klon_kimligi_;
    return b.id;
  }

  /// Marjinallestirir, boyutu kucultur. Sonraki klonlar asagi kayar;
  /// kimlikleri gecerli kalir ve YENIDEN KULLANILMAZ.
  ///
  /// Kovaryans burada ele ALINMAZ — NavState kovaryansin sahibi degildir.
  void drop_clone(CloneId id) {
    const int i = klon_indeksi(id);
    assert(i >= 0 && "var olmayan klon dusuruluyor");

    const int ofset = blok_ofseti(i);
    const int dof = bloklar_[i].dof;
    const int kuyruk = kCoreDof + augment_dof_ - (ofset + dof);

    // Degerler de tasinir; yalnizca ofset tablosunu guncellemek hayatta kalan
    // klonun degerini silinenin yerine kaydirirdi.
    if (kuyruk > 0) {
      augment_.segment(ofset - kCoreDof, kuyruk) = augment_.segment(ofset - kCoreDof + dof, kuyruk);
    }
    augment_.segment(augment_dof_ - dof, dof).setZero();

    for (int k = i; k + 1 < blok_sayisi_; ++k) {
      bloklar_[k] = bloklar_[k + 1];
    }
    --blok_sayisi_;
    augment_dof_ -= dof;
  }

  int clone_offset(CloneId id) const {
    const int i = klon_indeksi(id);
    assert(i >= 0 && "var olmayan klon ofseti isteniyor");
    return blok_ofseti(i);
  }

  bool has_clone(CloneId id) const { return klon_indeksi(id) >= 0; }

  // ---------------------------------------------------------------------------
  // Manifold islemleri — CONVENTIONS §3.1 sag perturbasyon
  // ---------------------------------------------------------------------------

  /// X (+) delta = X o Exp(delta). Kuyrugu (aktif boyutun otesi) YOK SAYAR.
  NavState plus(const Eigen::Ref<const StateVec>& delta) const {
    NavState out = *this;
    out.pose_ = kerteriz::plus(pose_, TangentVec(delta.template head<kSe23Dof>()));
    out.gyro_bias_ += delta.template segment<3>(kIdxGyroBias);
    out.accel_bias_ += delta.template segment<3>(kIdxAccelBias);
    if (augment_dof_ > 0) {
      out.augment_.head(augment_dof_) += delta.segment(kCoreDof, augment_dof_);
    }
    return out;
  }

  /// delta = X (-) other = Log(other^-1 o X). Kuyrugu SIFIRLAR.
  /// Iki durumun aktif duzeni ayni olmalidir.
  StateVec minus(const NavState& other) const {
    assert(augment_dof_ == other.augment_dof_ && "farkli duzenli durumlar cikariliyor");

    StateVec d = StateVec::Zero();
    d.template head<kSe23Dof>() = kerteriz::minus(pose_, other.pose_);
    d.template segment<3>(kIdxGyroBias) = gyro_bias_ - other.gyro_bias_;
    d.template segment<3>(kIdxAccelBias) = accel_bias_ - other.accel_bias_;
    if (augment_dof_ > 0) {
      d.segment(kCoreDof, augment_dof_) =
          augment_.head(augment_dof_) - other.augment_.head(augment_dof_);
    }
    return d;
  }

 private:
  // CONVENTIONS §5.1 cekirdek teget sirasi; ilk dokuzu lie.hpp'de tanimli.
  static constexpr int kIdxGyroBias = 9;
  static constexpr int kIdxAccelBias = 12;
  static_assert(kIdxAccelBias + 3 == kCoreDof, "cekirdek teget sirasi bozuk");

  struct Blok {
    AugmentKind kind = AugmentKind::kClone;
    std::string_view name;      ///< yalnizca kalibrasyon; omru yapilandirmaya ait
    CloneId id = kInvalidClone; ///< yalnizca klon
    int dof = 0;
  };

  /// i. blogun durum vektorundeki mutlak ofseti.
  int blok_ofseti(int i) const {
    int ofset = kCoreDof;
    for (int k = 0; k < i; ++k) {
      ofset += bloklar_[k].dof;
    }
    return ofset;
  }

  int kalibrasyon_indeksi(std::string_view name) const {
    for (int i = 0; i < kalici_blok_sayisi_; ++i) {
      if (bloklar_[i].name == name) {
        return i;
      }
    }
    return -1;
  }

  int klon_indeksi(CloneId id) const {
    if (id == kInvalidClone) {
      return -1;
    }
    for (int i = kalici_blok_sayisi_; i < blok_sayisi_; ++i) {
      if (bloklar_[i].id == id) {
        return i;
      }
    }
    return -1;
  }

  /// Klon bolgesini `dof` kadar yukari kaydirir (yeni kalibrasyon icin yer acar).
  void klonlari_kaydir(int klon_baslangici, int dof) {
    const int klon_dofu = augment_dof_ - klon_baslangici;
    if (klon_dofu <= 0) {
      return;
    }
    for (int k = klon_dofu - 1; k >= 0; --k) {
      augment_[klon_baslangici + dof + k] = augment_[klon_baslangici + k];
    }
    augment_.segment(klon_baslangici, dof).setZero();
  }

  manif::SE_2_3d pose_ = manif::SE_2_3d::Identity();
  Vec3 gyro_bias_ = Vec3::Zero();
  Vec3 accel_bias_ = Vec3::Zero();

  /// Augmentation degerleri Oklidyendir. Kalibrasyon durumlari (ADR-10 teker
  /// olcek faktoru gibi) zaten Oklidyendir; klon bloklarinin manifold yapisi
  /// dondurulmus sozlesmede TANIMLI DEGILDIR (deger erisimcisi yoktur), bu
  /// yuzden burada varsayim uretilmez.
  Eigen::Matrix<Scalar, kMaxAugmentDof, 1> augment_ =
      Eigen::Matrix<Scalar, kMaxAugmentDof, 1>::Zero();

  std::array<Blok, kMaxAugmentBlocks> bloklar_{};
  int blok_sayisi_ = 0;
  int kalici_blok_sayisi_ = 0; ///< bloklar_[0, kalici_blok_sayisi_) kalibrasyondur
  int kalici_dof_ = 0;
  int augment_dof_ = 0;
  std::int32_t siradaki_klon_kimligi_ = 0;
};

} // namespace kerteriz
