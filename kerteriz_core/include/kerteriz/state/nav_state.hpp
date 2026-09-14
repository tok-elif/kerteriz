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
/// Iki blok turu AYNI SEKILDE MODELLENMEZ:
///
///   PersistentCalibration — Oklidyen skaler blok (ADR-10 teker olcek faktoru
///                           gibi). Toplama ile guncellenir.
///   Clone                 — SE(3) POZ degeri (R_WB, p_WB). push_clone() o
///                           andaki pozu kopyalar; blok bir sayi yigini degil,
///                           tarihsel bir durumdur. Teget [dtheta, dp], sag
///                           perturbasyon — cekirdekle ayni konvansiyon.
///
/// KOVARYANS BURADA YOKTUR. drop_clone() yalnizca durum duzenini ve degerlerini
/// yonetir; kovaryans marjinalizasyonu backend'in isidir (INTERFACES §4).

#include "kerteriz/state/lie.hpp"
#include "kerteriz/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cassert>
#include <cstdint>
#include <manif/SE3.h>
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

/// Ayni anda tutulabilecek azami klon.
inline constexpr int kMaxClones = kMaxAugmentDof / kCloneDof;

namespace detail {

/// Klon degeri bir SE(3) pozudur: (R_WB, p_WB).
using ClonePose = manif::SE3<Scalar>;
using CloneTangent = Eigen::Matrix<Scalar, kCloneDof, 1>;

/// Kerteriz klon teget sirasi — cekirdekteki [dtheta ... dp] mantigiyla ayni
/// oncelik: once yonelim, sonra konum.
inline constexpr int kCloneIdxTheta = 0;
inline constexpr int kCloneIdxPos = 3;

/// manif'in SE3 teget sirasi TERSTIR: lin() once, ang() sonra. lie.hpp'deki
/// SE_2_3 permutasyonuyla ayni tuzak; burada da acikca cevrilir.
inline constexpr int kManifCloneIdxPos = 0;
inline constexpr int kManifCloneIdxTheta = 3;

inline manif::SE3Tangent<Scalar> clone_to_manif(const CloneTangent& d) {
  manif::SE3Tangent<Scalar> t;
  t.coeffs().template segment<3>(kManifCloneIdxPos) = d.template segment<3>(kCloneIdxPos);
  t.coeffs().template segment<3>(kManifCloneIdxTheta) = d.template segment<3>(kCloneIdxTheta);
  return t;
}

inline CloneTangent clone_from_manif(const manif::SE3Tangent<Scalar>& t) {
  CloneTangent d;
  d.template segment<3>(kCloneIdxTheta) = t.coeffs().template segment<3>(kManifCloneIdxTheta);
  d.template segment<3>(kCloneIdxPos) = t.coeffs().template segment<3>(kManifCloneIdxPos);
  return d;
}

/// Sag-plus / sag-minus — CONVENTIONS §3.1, cekirdekle ayni konvansiyon.
inline ClonePose clone_plus(const ClonePose& x, const CloneTangent& d) {
  return x.rplus(clone_to_manif(d));
}

inline CloneTangent clone_minus(const ClonePose& y, const ClonePose& x) {
  return clone_from_manif(y.rminus(x));
}

} // namespace detail

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

    // Kalibrasyonlar klonlardan ONCE gelir.
    //
    // Klon DEGERLERI kaydirilmaz: kalibrasyon degerleri ile klon pozlari AYRI
    // depolarda tutulur, dolayisiyla kalibrasyon eklemek klon degerlerine hic
    // dokunmaz. Kaymasi gereken yalnizca OFSETLERDIR.
    const int ofset = kCoreDof + kalici_dof_;

    // Klon KAYITLARI bir sira yukari kayar; aksi halde yeni kalibrasyon
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

    // Klon degeri O ANDAKI pozdur; blok ayirmak yetmez. Hiz ve bias'lar
    // klona girmez (kCloneDof = 6).
    klon_pozlari_[klon_sayisi()] =
        detail::ClonePose(pose_.translation(), Eigen::Quaternion<Scalar>(pose_.rotation()));

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

    // POZ DEGERLERI de sikistirilir; yalnizca ofset tablosunu guncellemek
    // hayatta kalan klonun degerini silinenin kimligine kaydirirdi.
    const int j = i - kalici_blok_sayisi_;
    for (int k = j; k + 1 < klon_sayisi(); ++k) {
      klon_pozlari_[k] = klon_pozlari_[k + 1];
    }
    klon_pozlari_[klon_sayisi() - 1].setIdentity();

    for (int k = i; k + 1 < blok_sayisi_; ++k) {
      bloklar_[k] = bloklar_[k + 1];
    }
    --blok_sayisi_;
    augment_dof_ -= kCloneDof;
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
    if (kalici_dof_ > 0) {
      out.kalibrasyon_.head(kalici_dof_) += delta.segment(kCoreDof, kalici_dof_);
    }
    for (int j = 0; j < klon_sayisi(); ++j) {
      const int ofset = blok_ofseti(kalici_blok_sayisi_ + j);
      out.klon_pozlari_[j] = detail::clone_plus(
          klon_pozlari_[j], detail::CloneTangent(delta.segment<kCloneDof>(ofset)));
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
    if (kalici_dof_ > 0) {
      d.segment(kCoreDof, kalici_dof_) =
          kalibrasyon_.head(kalici_dof_) - other.kalibrasyon_.head(kalici_dof_);
    }
    for (int j = 0; j < klon_sayisi(); ++j) {
      const int ofset = blok_ofseti(kalici_blok_sayisi_ + j);
      d.segment<kCloneDof>(ofset) = detail::clone_minus(klon_pozlari_[j], other.klon_pozlari_[j]);
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

  int klon_sayisi() const { return blok_sayisi_ - kalici_blok_sayisi_; }

  manif::SE_2_3d pose_ = manif::SE_2_3d::Identity();
  Vec3 gyro_bias_ = Vec3::Zero();
  Vec3 accel_bias_ = Vec3::Zero();

  /// Kalici kalibrasyon degerleri — Oklidyen, kayit sirasina gore bitisik.
  /// Kalibrasyonlar duzende her zaman once geldigi icin indeks dogrudan
  /// (ofset - kCoreDof)'tur.
  Eigen::Matrix<Scalar, kMaxAugmentDof, 1> kalibrasyon_ =
      Eigen::Matrix<Scalar, kMaxAugmentDof, 1>::Zero();

  /// Klon POZLARI — push sirasina gore, kalibrasyon deposundan AYRI.
  /// Ayri depo, kalibrasyon kaydinin klon degerlerini kaydirmasini gereksiz
  /// kilar ve klonun Oklidyen olmayan yapisini korur. Sabit kapasite, tahsis yok.
  std::array<detail::ClonePose, kMaxClones> klon_pozlari_{};

  std::array<Blok, kMaxAugmentBlocks> bloklar_{};
  int blok_sayisi_ = 0;
  int kalici_blok_sayisi_ = 0; ///< bloklar_[0, kalici_blok_sayisi_) kalibrasyondur
  int kalici_dof_ = 0;
  int augment_dof_ = 0;
  std::int32_t siradaki_klon_kimligi_ = 0;
};

} // namespace kerteriz
