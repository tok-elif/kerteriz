#pragma once

/// \file
/// F2.4-C/D withheld deney sozlesmesi — CLI seviyesinde FAIL-FAST.
///
/// ============================ NEDEN VAR =====================================
///
/// `--withheld-reference` istemek, "F2.4-C'de dondurulan withheld deneyini
/// kosuyorum" demektir. O deneyin sozlesmesi tek bir bayrakta degil, BIRDEN
/// COK bayragin birlikte dogru verilmesinde yasar:
///
///   stride 10 · GNSS hizi kapali · secim manifesti yaziliyor
///
/// Bayraklardan biri unutulursa kosu yine de BASARIYLA biter ve makul gorunen
/// bir CSV uretir. Ornegin `--no-gnss-velocity` unutulursa Kerteriz ~10 Hz
/// hiz olcumu alir, harici taban cizgisi hicbir hiz olcumu almaz; ortaya cikan
/// "Kerteriz daha iyi" sonucu deneyin degil, asimetrinin sonucudur. Bu sinif
/// hatalar sonradan bakildiginda ayirt edilemez, bu yuzden KOSU BASLAMADAN
/// once hata verilir.
///
/// ============================== KAPSAM ======================================
///
/// Bu dosya YALNIZCA `--withheld-reference` istendiginde konusur. Genel
/// `gnss_sampling` davranisi DEGISMEZ:
///
///   * seyreltme opt-in olmaya devam eder,
///   * legacy (bayraksiz) kosu etkilenmez,
///   * withheld istenmeyen bir seyreltme kosusunda stride serbesttir ve
///     GNSS hizi yasak DEGILDIR.
///
/// Sozlesme SAFTIR: dosya sistemine, veri setine ve zamana bakmaz; yalnizca
/// ayristirilmis CLI durumuna bakar. Bu yuzden testi gercek veri seti
/// gerektirmez.

#include <string>

namespace kerteriz_bringup {

/// F2.4-C'de DONDURULMUS stride. Kaynak: `results/f2.4/sequence_manifest.md`
/// "Dondurulmus deney protokolu" bloku. Deney gorulduKTEN sonra degistirilmez;
/// baska bir stride ile kosmak withheld deneyi DEGILDIR.
constexpr int kWithheldProtocolStride = 10;

/// Ayristirilmis CLI durumunun sozlesmeyi ilgilendiren kismi.
struct WithheldCliRequest {
  bool withheld_requested = false;      ///< `--withheld-reference` verildi
  bool stride_given = false;            ///< `--gnss-position-stride` verildi
  int stride = 0;                       ///< verilen stride degeri
  bool gnss_velocity_disabled = false;  ///< `--no-gnss-velocity` verildi
  bool sampling_manifest_given = false; ///< `--sampling-manifest` verildi
};

struct WithheldContractStatus {
  bool ok = true;
  std::string message;
};

/// Withheld deney sozlesmesini dogrular.
///
/// `withheld_requested` false ise HER ZAMAN basarilidir — legacy ve withheld
/// istemeyen seyreltme kosulari bu fonksiyondan etkilenmez.
///
/// Ihlaller TEK SEFERDE toplanir: operatorun ucuncu denemede ucuncu bayragi
/// ogrenmesi yerine eksigin tamami bir kerede yazilir.
inline WithheldContractStatus check_withheld_contract(const WithheldCliRequest& istek) {
  if (!istek.withheld_requested) {
    return WithheldContractStatus{};
  }

  std::string eksik;
  const auto ekle = [&eksik](const std::string& s) {
    eksik += "\n  * ";
    eksik += s;
  };

  if (!istek.stride_given) {
    ekle("--gnss-position-stride " + std::to_string(kWithheldProtocolStride) +
         " verilmeli (withheld kume yalnizca seyreltme acikken tanimlidir)");
  } else if (istek.stride != kWithheldProtocolStride) {
    ekle("--gnss-position-stride " + std::to_string(kWithheldProtocolStride) +
         " olmalidir; verilen: " + std::to_string(istek.stride) +
         " (F2.4-C'de dondurulmus deger, results/f2.4/sequence_manifest.md)");
  }
  if (!istek.gnss_velocity_disabled) {
    ekle("--no-gnss-velocity verilmeli (harici taban cizgisi GNSS hizi ALMAZ; "
         "verilmezse kiyas asimetrik olur)");
  }
  if (!istek.sampling_manifest_given) {
    ekle("--sampling-manifest <csv> verilmeli (hangi damganin hangi rolde "
         "oldugu sonradan denetlenebilsin diye)");
  }

  if (eksik.empty()) {
    return WithheldContractStatus{};
  }
  return WithheldContractStatus{false,
                                "--withheld-reference F2.4-C/D withheld deneyini istemektir; "
                                "sozlesme ihlal edildi:" +
                                    eksik};
}

} // namespace kerteriz_bringup
