#pragma once

/// \file
/// Deterministik GNSS konum seyreltme politikasi — F2.4-C.
///
/// ============================ NEDEN VAR =====================================
///
/// KITTI adaptorunde `kGnssPosition.position_w` ile `kReferencePose.position_w`
/// AYNI OXTS `p_enu` cozumunden gelir. Bu yuzden tam hizli ATE, buyuk olcude
/// "filtre kendi GNSS konum girdisini ne kadar yakindan izliyor" sorusunu
/// olcer; bagimsiz bir dogruluk deneyi DEGILDIR.
///
/// Bu dosya o karismayi ORTADAN KALDIRMAZ — bu veriyle kaldirilamaz. Daha
/// temiz bir soru sormayi mumkun kilar: filtrelere yalnizca SEYREK ve
/// deterministik bir GNSS konum altkumesi verildiginde, filtreye HIC
/// VERILMEMIS ara damgalarda ne oluyor?
///
/// Degerlendirme yine de bagimsiz ground truth DEGILDIR: withheld konumlar da
/// ayni OXTS/INS cozum ailesinden gelir.
///
/// ======================= NEDEN PAYLASIMLI VE SAF ============================
///
/// Ayni politika hem Kerteriz kosucusunda hem harici taban cizgisi
/// yayincisinda kullanilir. Iki dosyaya kopyalansaydi "ayni algoritmayi
/// kullandim" dogrulanamaz bir iddia olarak kalirdi ve iki yol zamanla
/// ayrisirdi. Burada tek tanim vardir ve damga kumelerinin BIREBIR esitligi
/// test edilir.
///
/// ========================== SECIM SOZLESMESI ================================
///
/// `t0` = ilk ARA-DEGERLENMEMIS referans poz damgasi (kosucunun baslatma
/// kaydiyla AYNI kural).
///
/// `t0`'dan SONRAKI tum `kGnssPosition` olaylari veri seti sirasiyla 1'den
/// baslayarak sirali numaralanir. Sayac ARA-DEGERLENMIS adaylarda da ILERLER.
///
///   secili slot  <=>  ordinal % stride == 0
///
/// Secili bir slot ara-degerlenmisse:
///   * olcum olarak KULLANILMAZ,
///   * yerine bir sonraki kare SECILMEZ,
///   * faz KAYDIRILMAZ.
/// Mevcut ara-degerleme politikasi aynen korunur.
///
/// `t0` hicbir zaman olcum degildir.
///
/// WITHHELD referans: `t > t0`, ara-degerlenmemis ve karsilik gelen GNSS
/// ordinali SECILI OLMAYAN referans pozlar. Olcum ve withheld kumelerinin
/// kesisimi BOSTUR.

#include "kerteriz_bringup/dataset_event.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace kerteriz_bringup {

using kerteriz::TimeNs;

/// Seyreltme yapilandirmasi. `enabled = false` LEGACY davranistir: `t0`
/// sonrasindaki tum adaylar secilidir ve withheld kume bostur.
struct GnssSamplingPolicy {
  bool enabled = false;
  int stride = 1;
};

struct SamplingPlanStatus {
  bool ok = false;
  std::string message;

  static SamplingPlanStatus success() { return SamplingPlanStatus{true, {}}; }
  static SamplingPlanStatus failure(std::string m) {
    return SamplingPlanStatus{false, std::move(m)};
  }
};

/// Bir damganin deneydeki rolu. Manifest bu degerlerle yazilir.
enum class SamplingRole {
  kMeasurement,                 ///< filtreye VERILDI
  kSelectedInterpolatedSkipped, ///< slot secildi ama kayit ara-degerlenmis
  kWithheldReference            ///< filtreye VERILMEDI, degerlendirmeye girer
};

inline const char* to_string(SamplingRole r) {
  switch (r) {
  case SamplingRole::kMeasurement:
    return "measurement";
  case SamplingRole::kSelectedInterpolatedSkipped:
    return "selected_interpolated_skipped";
  case SamplingRole::kWithheldReference:
    return "withheld_reference";
  }
  return "unknown";
}

/// Manifest satiri. `ordinal` yalnizca GNSS konum adaylari icin anlamlidir;
/// withheld referanslar icin karsilik gelen adayin ordinali yazilir.
struct SamplingRecord {
  TimeNs stamp_ns = 0;
  int ordinal = 0;
  SamplingRole role = SamplingRole::kMeasurement;
};

/// Denetlenebilir secim plani. Hangi damganin neden hangi kumede oldugu
/// sonradan dogrulanabilsin diye sayilar ve listeler birlikte tutulur.
struct SamplingPlan {
  SamplingPlanStatus status;
  TimeNs init_stamp_ns = 0;

  int candidate_count = 0;               ///< t0 sonrasi tum GNSS konum olaylari
  int selected_slot_count = 0;           ///< ordinal % stride == 0 (ara-degerlenmis DAHIL)
  int selected_usable_count = 0;         ///< secili VE ara-degerlenmemis -> olcum
  int selected_interpolated_skipped = 0; ///< secili ama ara-degerlenmis
  int withheld_reference_count = 0;

  /// Ilk adaydan sonuncusuna gecen sure. Efektif aday/olcum hizi buradan
  /// TURETILIR; bir diziden olculmus sabit bir Hz degeri GOMULMEZ, cunku her
  /// KITTI dizisinin ornekleme araligi farklidir.
  TimeNs candidate_span_ns = 0;

  std::vector<TimeNs> selected_slot_stamps;      ///< faz denetimi icin
  std::vector<TimeNs> measurement_stamps;        ///< filtreye VERILECEK olanlar
  std::vector<TimeNs> withheld_reference_stamps; ///< degerlendirme kumesi

  /// Damga sirasinda, denetim icin. "Ayni algoritmayi kullandim" demek yerine
  /// hangi damganin hangi rolde oldugu dogrudan gosterilebilsin diye.
  std::vector<SamplingRecord> records;
};

/// Aday akisinin EFEKTIF hizi [Hz]. Tek bir diziden olculmus sabit bir deger
/// gomulmesin diye plandan turetilir; her KITTI dizisinin ornekleme araligi
/// farklidir. Aday sayisi 2'den az veya span sifirsa 0 doner.
inline double candidate_rate_hz(const SamplingPlan& plan) {
  if (plan.candidate_count < 2 || plan.candidate_span_ns <= 0) {
    return 0.0;
  }
  const double span_s = static_cast<double>(plan.candidate_span_ns) * 1e-9;
  return static_cast<double>(plan.candidate_count - 1) / span_s;
}

/// Kosucunun baslatma kaydiyla AYNI kural: ilk ara-degerlenmemis referans poz.
inline TimeNs initialization_stamp(const std::vector<DatasetEvent>& events, bool& ok) {
  for (const auto& e : events) {
    if (e.kind == DatasetEventKind::kReferencePose && !e.source_interpolated) {
      ok = true;
      return e.stamp_ns;
    }
  }
  ok = false;
  return 0;
}

/// Sirali bir damga listesinde arama. Listeler olay sirasindan uretildigi icin
/// zaten artandir.
inline bool contains_stamp(const std::vector<TimeNs>& sorted, TimeNs t) {
  return std::binary_search(sorted.begin(), sorted.end(), t);
}

/// Secim planini uretir. SAF: girdi olay listesi disinda hicbir duruma bakmaz.
inline SamplingPlan build_sampling_plan(const std::vector<DatasetEvent>& events,
                                        const GnssSamplingPolicy& policy) {
  SamplingPlan plan;

  if (policy.enabled && policy.stride <= 0) {
    // Sessizce duzeltilmez: gecersiz bir deney yapilandirmasi, gorunmez
    // bicimde "makul" bir degere cevrilirse rapor edilen sozlesme ile
    // gercekte kosulan sozlesme ayrisir.
    plan.status = SamplingPlanStatus::failure("GNSS konum stride'i pozitif olmalidir");
    return plan;
  }

  bool var = false;
  const TimeNs t0 = initialization_stamp(events, var);
  if (!var) {
    plan.status = SamplingPlanStatus::failure("ara-degerlenmemis referans poz yok");
    return plan;
  }
  plan.init_stamp_ns = t0;

  // --- 1. Adaylari numarala ve secili slotlari belirle ---------------------
  int ordinal = 0;
  for (const auto& e : events) {
    if (e.kind != DatasetEventKind::kGnssPosition || e.stamp_ns <= t0) {
      continue;
    }
    ++ordinal; // sayac ARA-DEGERLENMIS adaylarda da ilerler
    ++plan.candidate_count;

    const bool secili = !policy.enabled || (ordinal % policy.stride == 0);
    if (!secili) {
      continue;
    }
    ++plan.selected_slot_count;
    plan.selected_slot_stamps.push_back(e.stamp_ns);

    if (e.source_interpolated) {
      ++plan.selected_interpolated_skipped; // yerine sonraki kare SECILMEZ
      plan.records.push_back(
          SamplingRecord{e.stamp_ns, ordinal, SamplingRole::kSelectedInterpolatedSkipped});
      continue;
    }
    ++plan.selected_usable_count;
    plan.measurement_stamps.push_back(e.stamp_ns);
    plan.records.push_back(SamplingRecord{e.stamp_ns, ordinal, SamplingRole::kMeasurement});
  }

  if (plan.candidate_count > 0) {
    TimeNs ilk = 0;
    TimeNs son = 0;
    bool basladi = false;
    for (const auto& e : events) {
      if (e.kind != DatasetEventKind::kGnssPosition || e.stamp_ns <= t0) {
        continue;
      }
      if (!basladi) {
        ilk = e.stamp_ns;
        basladi = true;
      }
      son = e.stamp_ns;
    }
    plan.candidate_span_ns = son - ilk;
  }

  // GNSS konum adaylarinin damga -> ordinal esleme tablosu; withheld
  // kayitlarina dogru ordinali yazabilmek icin.
  std::vector<std::pair<TimeNs, int>> ordinal_tablosu;
  {
    int o = 0;
    for (const auto& e : events) {
      if (e.kind != DatasetEventKind::kGnssPosition || e.stamp_ns <= t0) {
        continue;
      }
      ordinal_tablosu.emplace_back(e.stamp_ns, ++o);
    }
  }

  // --- 2. Withheld referans ------------------------------------------------
  // Legacy modda (seyreltme kapali) tum adaylar secili oldugundan withheld
  // kume zaten bostur; ayrica hesaplamaya gerek yok ama ayni kod yolundan
  // gecmesi sozlesmeyi tek yerde tutar.
  for (const auto& e : events) {
    if (e.kind != DatasetEventKind::kReferencePose || e.stamp_ns <= t0) {
      continue;
    }
    if (e.source_interpolated) {
      continue; // mevcut ara-degerleme politikasi: referans da olamaz
    }
    if (contains_stamp(plan.selected_slot_stamps, e.stamp_ns)) {
      continue;
    }
    plan.withheld_reference_stamps.push_back(e.stamp_ns);
    const auto it =
        std::lower_bound(ordinal_tablosu.begin(), ordinal_tablosu.end(), e.stamp_ns,
                         [](const std::pair<TimeNs, int>& a, TimeNs t) { return a.first < t; });
    const int ord = (it != ordinal_tablosu.end() && it->first == e.stamp_ns) ? it->second : 0;
    plan.records.push_back(SamplingRecord{e.stamp_ns, ord, SamplingRole::kWithheldReference});
  }
  std::stable_sort(
      plan.records.begin(), plan.records.end(),
      [](const SamplingRecord& a, const SamplingRecord& b) { return a.stamp_ns < b.stamp_ns; });
  plan.withheld_reference_count = static_cast<int>(plan.withheld_reference_stamps.size());

  plan.status = SamplingPlanStatus::success();
  return plan;
}

} // namespace kerteriz_bringup
