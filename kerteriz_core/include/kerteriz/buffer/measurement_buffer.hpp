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
/// UFKUN BASLANGICI. Tampon kendi zaman ufkunu ilk `process()` cagrisinda
/// estimator'un GERCEK zamanindan (`backend().stamp_ns()`) alir. Sentinel bir
/// "en kucuk zaman" ile baslamak, estimator'un baslangic anindan ONCEKI bir
/// olcumu SIRALI sayardi ve onu guncel duruma uygulardi — tam olarak ADR-5'in
/// engellemek icin var oldugu sessiz sapma.
///
/// ================== OLCUM DAMGASINDA TAM YERLESTIRME =======================
///
/// Bir olcum iki IMU ornegi ARASINA duserse, onceki IMU durumuna uygulanmaz.
/// Mevcut ayriklastirma (imu_propagator.hpp, F1.2) ve backend sozlesmesi
/// birlikte tek anlamlidir: `predict(u, dt)` mutlak zamani `u.stamp_ns`
/// yapar, yani ornek `[u.stamp_ns - dt, u.stamp_ns]` araligini temsil eder ve
/// o aralik boyunca (w, a) SABIT tutulur. Dolayisiyla arayi olcum damgasinda
/// ikiye bolmek ayni konvansiyonun dogrudan sonucudur:
///
///   predict(u_{k+1} damgasi t_m yapilmis, t_m - t_k)   -> durum t_m'de
///   apply(z)                                            -> olcum t_m'de
///   predict(u_{k+1}, t_{k+1} - t_m)                     -> durum t_{k+1}'de
///
/// Ikinci bolum ayri bir olay olarak KAYDEDILMEZ; dizideki IMU olayina
/// sirasi geldiginde dt kendiliginden `t_{k+1} - t_m` cikar.
///
/// ERTELEME. Damgasi hem mevcut zamandan hem de bilinen EN YENI IMU
/// orneginden sonra olan bir olcum yerlestirilemez: o araligi kapsayan IMU
/// verisi henuz yoktur. Damgasi tam mevcut zamanda olan olcum ERTELENMEZ —
/// yayilim gerektirmez, durum zaten o andadir. Yerlestirilemeyen boyle bir
/// olcum kuyrukta BEKLER ve o cagride sonuc uretmez; kapsayan IMU gelince
/// normal gecikme yolundan islenir. Alternatif — onu guncel duruma uygulamak —
/// gelecekteki bir olcumu gecmis bir duruma yazmak olurdu.
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
/// * `process()` yalnizca BU CAGRIDA islenen YENI olcumler icin sonuc
///   dondurur. Geri sarma sirasinda yeniden oynatilan ESKI olcumlerin
///   sonuclari tekrar raporlanmaz — ilk islendiklerinde raporlanmislardi.
/// * Ayni damgaya sahip olaylarda IMU once islenir: olcum yayilmis durumu
///   gorsun. Ayni damgali IKI OLCUM arasindaki sira GELIS SIRASIDIR.
/// * Pencere disinda kalan olcum `kTooOld`. Geri sarma noktasi elde
///   kalmamissa `kOutOfOrderDrop`. Bu iki karar OLAY BASINADIR: yerlestirilemeyen
///   tek bir eski olay, ayni cagrideki gecerli olaylari DUSURMEZ.
/// * Pencere disinda kalan IMU ornegi sessizce atilir: IMU'nun raporlanacak
///   bir `ProcessingResult` kanali YOKTUR (INTERFACES §5 sonucu olcume bagar).

#include "kerteriz/buffer/estimator.hpp"
#include "kerteriz/buffer/processing_result.hpp"
#include "kerteriz/measurements/measurement.hpp"
#include "kerteriz/types.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdlib>
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
    en_yeni_imu_ns_ = std::max(en_yeni_imu_ns_, u.stamp_ns);
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
    if (!baslatildi_) {
      son_ns_ = est.backend().stamp_ns(); // ufuk estimator'un GERCEK zamanindan
      baslatildi_ = true;
    }
    if (bekleyen_.empty()) {
      return {sonuclar_.data(), 0};
    }

    std::stable_sort(bekleyen_.begin(), bekleyen_.end(), once_gelir);

    const TimeNs ufuk = std::max(son_ns_, en_yeni_imu_ns_);
    const TimeNs alt_sinir = pencere_alt_siniri(ufuk);

    // Siniflandirma OLAY BASINADIR. Yerlestirilemeyen bir olay yalnizca
    // KENDISI dusurulur; aynı cagrideki gecerli olaylar islenmeye devam eder.
    std::vector<Olay> kalan;
    std::vector<Olay> ertelenen;
    kalan.reserve(bekleyen_.size());
    for (auto& e : bekleyen_) {
      if (e.stamp_ns < alt_sinir) {
        if (!e.imu) {
          sonuc_ekle(
              ProcessingResult{e.z->name(), e.stamp_ns, RejectReason::kTooOld, std::nullopt});
        }
        continue;
      }
      if (e.stamp_ns < son_ns_ && !geri_sarilabilir(e.stamp_ns)) {
        if (!e.imu) {
          sonuc_ekle(ProcessingResult{e.z->name(), e.stamp_ns, RejectReason::kOutOfOrderDrop,
                                      std::nullopt});
        }
        continue;
      }
      // Erteleme yalnizca olcum HEM mevcut zamanin HEM DE bilinen en yeni IMU
      // orneginin ILERISINDE ise anlamlidir. `son_ns_` bu noktada estimator'un
      // gercek zamanidir (ilk cagride backend().stamp_ns()'ten alinir, sonra
      // sirali yolda ona esit kalir). Damgasi tam mevcut zamanda olan bir
      // olcum yayilim GEREKTIRMEZ; onu da ertelemek, hicbir IMU gelmezse
      // kuyrukta sonsuza kadar bekletmek olurdu.
      if (!e.imu && e.stamp_ns > son_ns_ && e.stamp_ns > en_yeni_imu_ns_) {
        ertelenen.push_back(std::move(e)); // kapsayan IMU henuz yok
        continue;
      }
      kalan.push_back(std::move(e));
    }
    bekleyen_ = std::move(ertelenen);

    if (!kalan.empty()) {
      const TimeNs en_erken = kalan.front().stamp_ns;
      if (en_erken >= son_ns_) {
        oynat(est, kalan); // sirali akis
      } else {
        const bool sarildi = geri_sar(est, en_erken);
        if (!sarildi) {
          std::abort(); // siniflandirma asamasi bunu garanti etti
        }
        std::vector<Olay> yeniden = std::move(kopartilan_);
        kopartilan_.clear();
        for (auto& e : kalan) {
          yeniden.push_back(std::move(e));
        }
        // stable_sort: ayni damgada ESKI olay onde kalir.
        std::stable_sort(yeniden.begin(), yeniden.end(), once_gelir);
        oynat(est, yeniden);
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
  /// sirasinda olaylar tasinir, snapshot ise 30 KB'in uzerindedir.
  struct Kayit {
    Olay olay;
    PipelineSnapshot once;
  };

  static Scalar saniye(TimeNs fark) { return static_cast<Scalar>(fark) * Scalar(1e-9); }

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

  /// `t` anina geri sarilabilir mi? Gecmisi DEGISTIRMEZ; siniflandirma
  /// asamasinda olay basina sorulur.
  bool geri_sarilabilir(TimeNs t) const {
    const auto ilk = std::find_if(gecmis_.begin(), gecmis_.end(),
                                  [t](const Kayit& k) { return k.olay.stamp_ns >= t; });
    return ilk != gecmis_.end() && ilk->once.backend.stamp_ns <= t;
  }

  /// `t`'ye geri sarar; gecmisin o noktadan sonraki kismi `kopartilan_`a
  /// tasinir ve estimator o andaki tam snapshot'a doner.
  bool geri_sar(Estimator& est, TimeNs t) {
    const auto ilk = std::find_if(gecmis_.begin(), gecmis_.end(),
                                  [t](const Kayit& k) { return k.olay.stamp_ns >= t; });
    if (ilk == gecmis_.end() || ilk->once.backend.stamp_ns > t) {
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

  /// `i`'den SONRA gelen, `t`'yi kapsayan ilk IMU ornegi. Ayni damgali IMU
  /// zaten `i`'den once islenmis olur (once_gelir), o yuzden yalnizca ileri
  /// bakilir.
  static const ImuSample* kapsayan_imu(const std::vector<Olay>& dizi, std::size_t i, TimeNs t) {
    for (std::size_t j = i + 1; j < dizi.size(); ++j) {
      if (dizi[j].imu && dizi[j].stamp_ns >= t) {
        return &dizi[j].u;
      }
    }
    return nullptr;
  }

  void oynat(Estimator& est, std::vector<Olay>& dizi) {
    for (std::size_t i = 0; i < dizi.size(); ++i) {
      PipelineSnapshot once = est.save_snapshot();
      Olay& e = dizi[i];

      if (e.imu) {
        est.predict(e.u, saniye(e.stamp_ns - est.backend().stamp_ns()));
      } else {
        const TimeNs simdi = est.backend().stamp_ns();
        if (e.stamp_ns > simdi) {
          // Olcum iki IMU ornegi arasinda: araligi damgada BOL.
          const ImuSample* kapsayan = kapsayan_imu(dizi, i, e.stamp_ns);
          if (kapsayan == nullptr) {
            // Siniflandirma asamasi bunu imkansiz kilar. Sessizce onceki
            // duruma uygulamak, olcumu YANLIS ana yazmak olurdu.
            std::abort();
          }
          ImuSample parca = *kapsayan;
          parca.stamp_ns = e.stamp_ns;
          est.predict(parca, saniye(e.stamp_ns - simdi));
        }
        const ProcessingResult r = est.apply(*e.z);
        if (e.yeni) {
          sonuc_ekle(r);
        }
      }

      son_ns_ = std::max(son_ns_, e.stamp_ns);
      gecmis_.push_back(Kayit{std::move(e), std::move(once)});
    }
  }

  /// Pencere disinda kalan ve kapasiteyi asan gecmisi dusurur.
  void budala() {
    const TimeNs alt_sinir = pencere_alt_siniri(std::max(son_ns_, en_yeni_imu_ns_));
    while (!gecmis_.empty() && gecmis_.front().olay.stamp_ns < alt_sinir) {
      gecmis_.pop_front();
    }
    while (gecmis_.size() > static_cast<std::size_t>(kMaxBufferedEvents)) {
      gecmis_.pop_front();
    }
  }

  TimeNs pencere_ns_;
  bool baslatildi_ = false;
  TimeNs son_ns_ = 0;
  TimeNs en_yeni_imu_ns_ = std::numeric_limits<TimeNs>::min();

  std::vector<Olay> bekleyen_;
  std::deque<Kayit> gecmis_;
  std::vector<Olay> kopartilan_;

  std::array<ProcessingResult, kMaxPendingMeasurements> sonuclar_{};
  int sonuc_sayisi_ = 0;
};

} // namespace kerteriz
