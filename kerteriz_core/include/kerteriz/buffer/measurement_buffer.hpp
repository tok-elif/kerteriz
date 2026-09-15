#pragma once

/// \file
/// Sirasiz olcum tamponu ve yeniden yayilim — INTERFACES §5 · ADR-5, ADR-15, ADR-20.
///
/// ========================== TEMEL FIKIR ====================================
///
/// Gercek sensorler gec ve sirasiz gelir. Geciken bir olcumu GUNCEL duruma
/// uygulamak sistematik hata uretir; atmak bilgi kaybidir (ADR-5). Bu yuzden
/// kisa bir pencerede olay gecmisi tutulur: geciken olcum gelince o ana geri
/// sarilir, olay dizisi kronolojik yeniden oynatilir.
///
/// ========================= ZAMAN SOZLESMESI ================================
///
/// Mutlak zaman HER ZAMAN `TimeNs`. `dt` yalnizca iki `TimeNs` farkindan
/// uretilir ve BIRIKTIRILMEZ (CONVENTIONS §6). Tamponun kendisi mutlak zamani
/// kayan noktaya hic cevirmez.
///
/// ======================= TAHSIS SINIRI (ADR-22) ============================
///
/// Burasi SAHIPLIK KATMANIDIR, sayisal sicak yol degil. `std::unique_ptr`
/// ve olay gecmisi icin tahsis SERBESTTIR (INTERFACES §5, CLAUDE.md §8).
/// Tahsis yasagi `predict`/`update`/`evaluate` icin gecerlidir ve orada
/// korunur. Sonuc deposu yine de sabit kapasitelidir: dondurulen gorunum
/// tamponun kendi deposuna bakar, sahiplik tasimaz.
///
/// ========================== FAZ 1 SEMANTIGI ================================
///
/// * `process()` yalnizca BU CAGRIDA gelen olcumler icin sonuc dondurur.
///   Geri sarma sirasinda yeniden oynatilan ESKI olcumlerin sonuclari tekrar
///   raporlanmaz — ilk islendiklerinde raporlanmislardi.
/// * Ayni damgaya sahip olaylarda IMU once islenir: olcum yayilmis durumu
///   gorsun. Ayni damgali IKI OLCUM arasindaki sira GELIS SIRASIDIR; bu
///   yuzden ayni damgaya gecikmeli gelen bir olcum, ayni damgali digerinin
///   ARDINA yerlesir. Faz 1 bunu boyle kabul eder.
/// * Pencere disinda kalan olcum `kTooOld`. Gecmis kapasitesi tukendigi icin
///   geri sarma noktasi elde kalmamissa `kOutOfOrderDrop`.
/// * Pencere disinda kalan IMU ornegi sessizce atilir: IMU'nun raporlanacak
///   bir `ProcessingResult` kanali YOKTUR (INTERFACES §5 sonucu olcume bagar).

#include "kerteriz/buffer/estimator.hpp"
#include "kerteriz/buffer/processing_result.hpp"
#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace kerteriz {

/// Pencere icinde saklanabilecek en fazla olay. Sabit ust sinir: ADR-5
/// "maliyet pencere boyutuyla artar" der, bu da o artisin tavanidir. Tavan
/// asilirsa en eski olaylar dusurulur; o noktadan once geri sarma artik
/// mumkun olmadigi icin ilgili olcum `kOutOfOrderDrop` alir.
inline constexpr int kMaxBufferedEvents = 512;

/// Tek `process()` cagrisinda islenebilecek en fazla bekleyen olcum, ve ayni
/// zamanda sabit kapasiteli sonuc deposunun boyutu.
inline constexpr int kMaxPendingMeasurements = 64;

class MeasurementBuffer {
 public:
  explicit MeasurementBuffer(TimeNs window_ns) : pencere_ns_(window_ns) {}

  MeasurementBuffer(const MeasurementBuffer&) = delete;
  MeasurementBuffer& operator=(const MeasurementBuffer&) = delete;
  MeasurementBuffer(MeasurementBuffer&&) = default;
  MeasurementBuffer& operator=(MeasurementBuffer&&) = default;
  ~MeasurementBuffer() = default;

  TimeNs window_ns() const { return pencere_ns_; }

  void add_imu(const ImuSample& u) {
    Olay e;
    e.stamp_ns = u.stamp_ns;
    e.imu = true;
    e.u = u;
    e.yeni = true;
    bekleyen_.push_back(std::move(e));
  }

  /// Sahipligi alir. `false` yalnizca tampon dolu veya isaretci bos ise
  /// doner — BAYATLIK KARARI DEGILDIR. Bayat olcum de kuyruga girer ve
  /// `process()` icinde `kTooOld` sonucu uretir; aksi halde cagiran neden
  /// reddedildigini ogrenemezdi.
  bool add_measurement(std::unique_ptr<Measurement> z) {
    if (z == nullptr) {
      return false;
    }
    if (bekleyen_olcum_sayisi() >= kMaxPendingMeasurements) {
      return false;
    }
    Olay e;
    e.stamp_ns = z->stamp_ns();
    e.imu = false;
    e.z = std::move(z);
    e.yeni = true;
    bekleyen_.push_back(std::move(e));
    return true;
  }

  /// Zaman sirasina gore isler; geciken olcum icin Estimator'un snapshot'ina
  /// doner. Donen gorunum BIR SONRAKI `process()` cagrisina kadar gecerlidir.
  ArrayView<const ProcessingResult> process(Estimator& est) {
    sonuc_sayisi_ = 0;
    if (bekleyen_.empty()) {
      return {sonuclar_.data(), 0};
    }

    std::stable_sort(bekleyen_.begin(), bekleyen_.end(), once_gelir);

    const TimeNs ufuk = std::max(son_ns_, bekleyen_.back().stamp_ns);
    const TimeNs alt_sinir = pencere_alt_siniri(ufuk);

    // 1. Pencere disindakiler elenir. Olcum icin sebep raporlanir.
    std::vector<Olay> kalan;
    kalan.reserve(bekleyen_.size());
    for (auto& e : bekleyen_) {
      if (e.stamp_ns < alt_sinir) {
        if (!e.imu) {
          sonuc_ekle(
              ProcessingResult{e.z->name(), e.stamp_ns, RejectReason::kTooOld, std::nullopt});
        }
        continue;
      }
      kalan.push_back(std::move(e));
    }
    bekleyen_.clear();

    if (kalan.empty()) {
      budala();
      return {sonuclar_.data(), static_cast<std::size_t>(sonuc_sayisi_)};
    }

    const TimeNs en_erken = kalan.front().stamp_ns;
    if (en_erken >= son_ns_) {
      // 2a. Sirali akis — geri sarma gerekmez.
      for (auto& e : kalan) {
        uygula(est, std::move(e));
      }
    } else if (!geri_sar(est, en_erken)) {
      // 2b. Geri sarma noktasi elde kalmamis.
      for (auto& e : kalan) {
        if (!e.imu) {
          sonuc_ekle(ProcessingResult{e.z->name(), e.stamp_ns, RejectReason::kOutOfOrderDrop,
                                      std::nullopt});
        }
      }
    } else {
      // 2c. Geri sarildi: kopartilan gecmis ile yeni olaylar birlestirilip
      // kronolojik yeniden oynatilir. stable_sort oldugu icin ayni damgada
      // ESKI olay onde kalir.
      std::vector<Olay> yeniden = std::move(kopartilan_);
      kopartilan_.clear();
      for (auto& e : kalan) {
        yeniden.push_back(std::move(e));
      }
      std::stable_sort(yeniden.begin(), yeniden.end(), once_gelir);
      for (auto& e : yeniden) {
        uygula(est, std::move(e));
      }
    }

    budala();
    return {sonuclar_.data(), static_cast<std::size_t>(sonuc_sayisi_)};
  }

 private:
  struct Olay {
    TimeNs stamp_ns = 0;
    bool imu = false;
    bool yeni = false; ///< bu `process()` cagrisinda mi geldi (sonuc raporlanir mi)
    ImuSample u{};
    std::unique_ptr<Measurement> z;
  };

  /// Islenmis bir olay ve UYGULANMADAN ONCEKI tam durum.
  ///
  /// Snapshot KASTEN `Olay`'in disindadir: bekleyen kuyrukta ve siralama
  /// sirasinda olaylar tasinir, snapshot ise 30 KB'in uzerindedir. Bekleyen
  /// olayin tasimadigi bir snapshot ne kopyalanir ne de henuz yazilmamis
  /// hâliyle dolasir.
  struct Kayit {
    Olay olay;
    PipelineSnapshot once;
  };

  /// Ayni damgada IMU once gelir: olcum yayilmis durumu gorsun.
  static bool once_gelir(const Olay& a, const Olay& b) {
    if (a.stamp_ns != b.stamp_ns) {
      return a.stamp_ns < b.stamp_ns;
    }
    return static_cast<int>(!a.imu) < static_cast<int>(!b.imu);
  }

  TimeNs pencere_alt_siniri(TimeNs ufuk) const {
    // Tasma korumasi: cok buyuk pencere "hicbir sey bayat degil" demektir.
    if (ufuk < std::numeric_limits<TimeNs>::min() + pencere_ns_) {
      return std::numeric_limits<TimeNs>::min();
    }
    return ufuk - pencere_ns_;
  }

  int bekleyen_olcum_sayisi() const {
    return static_cast<int>(
        std::count_if(bekleyen_.begin(), bekleyen_.end(), [](const Olay& e) { return !e.imu; }));
  }

  void sonuc_ekle(const ProcessingResult& r) {
    if (sonuc_sayisi_ < kMaxPendingMeasurements) {
      sonuclar_[static_cast<std::size_t>(sonuc_sayisi_)] = r;
      ++sonuc_sayisi_;
    }
  }

  /// `t`'den itibaren geri sarar. Basarili ise gecmisin o noktadan sonraki
  /// kismi `kopartilan_`a tasinir ve estimator o andaki tam snapshot'a doner.
  bool geri_sar(Estimator& est, TimeNs t) {
    const auto ilk = std::find_if(gecmis_.begin(), gecmis_.end(),
                                  [t](const Kayit& k) { return k.olay.stamp_ns >= t; });
    if (ilk == gecmis_.end()) {
      return false; // gecmisin tamami t'den once: geri sarilacak bir sey yok
    }
    if (ilk->once.backend.stamp_ns > t) {
      // En eski elimizdeki snapshot bile t'den SONRA — aradaki kayitlar
      // kapasite tavani yuzunden dusurulmus. Bu noktaya donulemez; uydurma
      // bir baslangictan yeniden oynatmak sessiz yanlis olurdu.
      return false;
    }

    est.restore_snapshot(ilk->once);
    son_ns_ = ilk->once.backend.stamp_ns;

    kopartilan_.clear();
    for (auto it = ilk; it != gecmis_.end(); ++it) {
      it->olay.yeni = false; // yeniden oynatilanlar TEKRAR raporlanmaz
      kopartilan_.push_back(std::move(it->olay));
    }
    gecmis_.erase(ilk, gecmis_.end());
    return true;
  }

  void uygula(Estimator& est, Olay&& e) {
    PipelineSnapshot once = est.save_snapshot();

    if (e.imu) {
      // dt YALNIZCA iki TimeNs farkindan; biriktirme yok (CONVENTIONS §6).
      const TimeNs fark = e.stamp_ns - est.backend().stamp_ns();
      est.predict(e.u, static_cast<Scalar>(fark) * Scalar(1e-9));
    } else {
      const ProcessingResult r = est.apply(*e.z);
      if (e.yeni) {
        sonuc_ekle(r);
      }
    }

    son_ns_ = std::max(son_ns_, e.stamp_ns);
    gecmis_.push_back(Kayit{std::move(e), std::move(once)});
  }

  /// Pencere disinda kalan ve kapasiteyi asan gecmisi dusurur.
  void budala() {
    const TimeNs alt_sinir = pencere_alt_siniri(son_ns_);
    while (!gecmis_.empty() && gecmis_.front().olay.stamp_ns < alt_sinir) {
      gecmis_.pop_front();
    }
    while (gecmis_.size() > static_cast<std::size_t>(kMaxBufferedEvents)) {
      gecmis_.pop_front();
    }
  }

  TimeNs pencere_ns_;
  TimeNs son_ns_ = std::numeric_limits<TimeNs>::min();

  std::vector<Olay> bekleyen_;
  std::deque<Kayit> gecmis_;
  std::vector<Olay> kopartilan_;

  std::array<ProcessingResult, kMaxPendingMeasurements> sonuclar_{};
  int sonuc_sayisi_ = 0;
};

} // namespace kerteriz
