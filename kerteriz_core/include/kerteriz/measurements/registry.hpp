#pragma once

/// \file
/// Olcum registry'si ve Jacobian testi zorunlulugu.
/// PHASE0.md S10 · CLAUDE.md §6.1 Denetim (4) · INTERFACES §3.
///
/// Faz 0'da OLCUM YOKTUR. Burada kurulan sey MEKANIZMADIR: Faz 1'de ilk
/// `Measurement` yazildigi anda denetim kendiliginden islesin diye.
///
/// Denetim (4) dosya TARAMAZ. CLAUDE.md §6.1 bunu acikca yasaklar
/// ("Dosya tarayan kirilgan script yazma."). Kayitlar programin kendi
/// icindedir; denetim de bir testtir. Bir olcum kaydolur ama Jacobian
/// testi kaydolmazsa test paketi DUSER.
///
/// Sicak yol DEGILDIR (ADR-22): kayitlar statik baslatmada, denetim test
/// icinde kosar. Bu yuzden `std::vector` burada mesrudur.

#include <algorithm>
#include <string_view>
#include <type_traits>
#include <vector>

namespace kerteriz::measurements {

/// Bir olcum tipinin kaydi.
struct MeasurementEntry {
  std::string_view type_name;   ///< C++ tip adi, makronun urettigi metin
  std::string_view config_name; ///< YAML'daki sensor adi (INTERFACES §3, ADR-2)
  const char* file = nullptr;
  int line = 0;
};

/// Jacobian testinin govdesi. Denetim testi bunlari gercekten CAGIRIR;
/// yalnizca varliklarini saymak, kaydi bos bir beyana cevirirdi.
using JacobianTestFn = void (*)();

struct JacobianTestEntry {
  std::string_view type_name;
  JacobianTestFn run = nullptr;
  const char* file = nullptr;
  int line = 0;
};

/// Kayit defteri. Deger tipidir — kuresel ornegin yani sira YEREL ornekler de
/// kurulabilir. Denetimin kendisi bu sayede test edilebilir: ihlalli bir
/// registry kurulup denetimin onu gercekten yakaladigi dogrulanir.
class Registry {
 public:
  void add_measurement(const MeasurementEntry& entry) { measurements_.push_back(entry); }
  void add_jacobian_test(const JacobianTestEntry& entry) { jacobian_tests_.push_back(entry); }

  const std::vector<MeasurementEntry>& measurements() const { return measurements_; }
  const std::vector<JacobianTestEntry>& jacobian_tests() const { return jacobian_tests_; }

 private:
  std::vector<MeasurementEntry> measurements_;
  std::vector<JacobianTestEntry> jacobian_tests_;
};

/// Tek binary'nin kuresel defteri.
///
/// Fonksiyon-ici statik (Meyers) kullanilir: kayitlar baska ceviri
/// birimlerinin statik baslatmasindan gelir ve statik baslatma SIRASI
/// tanimsizdir. Defterin kendisi ilk kullanimda kurulur, bu yuzden hangi
/// ceviri biriminin once baslatildigi onemsizdir.
inline Registry& global_registry() {
  static Registry defter;
  return defter;
}

/// DENETIM (4) — Jacobian testi kaydedilmemis olcum tipleri.
/// Bos donmelidir; donmezse test paketi duser.
inline std::vector<std::string_view> measurements_without_jacobian_test(const Registry& reg) {
  std::vector<std::string_view> eksik;
  for (const auto& olcum : reg.measurements()) {
    const auto& testler = reg.jacobian_tests();
    const bool var = std::any_of(testler.begin(), testler.end(), [&](const JacobianTestEntry& t) {
      return t.type_name == olcum.type_name;
    });
    if (!var) {
      eksik.push_back(olcum.type_name);
    }
  }
  return eksik;
}

/// Kaydi olmayan bir tip icin Jacobian testi kaydedilmis mi.
/// Yazim hatasini yakalar: `KERTERIZ_REGISTER_JACOBIAN_TEST(GnssPositon)` testi
/// kaydeder, `GnssPosition` ise testsiz kalir — iki liste de tek basina
/// bakildiginda dolu gorunur, esleme ise tutmaz.
inline std::vector<std::string_view> jacobian_tests_without_measurement(const Registry& reg) {
  std::vector<std::string_view> sahipsiz;
  for (const auto& test : reg.jacobian_tests()) {
    const auto& olcumler = reg.measurements();
    const bool var = std::any_of(olcumler.begin(), olcumler.end(), [&](const MeasurementEntry& m) {
      return m.type_name == test.type_name;
    });
    if (!var) {
      sahipsiz.push_back(test.type_name);
    }
  }
  return sahipsiz;
}

/// Ayni YAML adini iki tip talep ediyorsa fabrika belirsizlesir (INTERFACES §8).
inline std::vector<std::string_view> duplicate_config_names(const Registry& reg) {
  std::vector<std::string_view> cift;
  const auto& olcumler = reg.measurements();
  for (std::size_t i = 0; i < olcumler.size(); ++i) {
    for (std::size_t k = i + 1; k < olcumler.size(); ++k) {
      if (olcumler[i].config_name == olcumler[k].config_name) {
        cift.push_back(olcumler[i].config_name);
      }
    }
  }
  return cift;
}

namespace detail {

/// Tip TAM mi (yalnizca ileri bildirilmis degil mi)?
///
/// `sizeof` eksik tip icin gecersizdir; SFINAE ile yakalanir. `sizeof(T) > 0`
/// YAZILMAZ: sabit ile karsilastirma clang-tidy'nin bugprone-sizeof-expression
/// kontrolunu tetikler ve hakli olarak — o kalip genellikle gercek bir hatadir.
template <typename T, typename = void>
struct IsCompleteType : std::false_type {};

template <typename T>
struct IsCompleteType<T, decltype(void(sizeof(T)))> : std::true_type {};

} // namespace detail

/// Makrolarin kullandigi kayit nesneleri. Kurucu kayit yapar.
///
/// TIP PARAMETRESI KASITLIDIR. Makro yalnizca `#Type` ile metin uretseydi,
/// kaydedilen ad gercek bir tipe karsilik gelmek ZORUNDA olmazdi: iki tarafta
/// ayni yazim hatasi yapilmasi (olcum ve testi `GnssPositon` olarak kaydetmek)
/// hicbir `GnssPositon` tipi olmadan Denetim (4)'u gecirirdi. Registrar tip
/// parametresi aldigi icin boyle bir kayit DERLENMEZ.
///
/// `sizeof` tam tip zorunlulugu getirir; yalnizca ileri bildirilmis bir sinif
/// da yetmez. Faz 1'de fabrika geri cagrisi eklendiginde `Type`, `Measurement`
/// turevi olma zorunlulugu ile de sinirlanacaktir; bugun boyle bir taban
/// sinif YOKTUR, o yuzden uydurulmaz.
///
/// KALAN SINIR: olcum ile testin ESLESMESI hala `type_name` metni uzerinden
/// yapilir. Iki farkli ad alanindaki AYNI yazimli iki tip birbirinin testine
/// eslesebilir. Bugun bunu kapatmak eslesme olcutunu degistirmeyi gerektirir
/// (tip kimligi anahtari); S10 kapsaminda degildir. Yazim hatasi sinifi —
/// asil delik — tip parametresiyle kapatilmistir.
template <typename Type>
class MeasurementRegistrar {
  static_assert(detail::IsCompleteType<Type>::value, "kayit TAM bir tip gerektirir");

 public:
  MeasurementRegistrar(std::string_view type_name, std::string_view config_name, const char* file,
                       int line) {
    global_registry().add_measurement({type_name, config_name, file, line});
  }
};

template <typename Type>
class JacobianTestRegistrar {
  static_assert(detail::IsCompleteType<Type>::value, "kayit TAM bir tip gerektirir");

 public:
  JacobianTestRegistrar(std::string_view type_name, JacobianTestFn run, const char* file,
                        int line) {
    global_registry().add_jacobian_test({type_name, run, file, line});
  }
};

} // namespace kerteriz::measurements

/// Makro ici benzersiz ad uretimi.
///
/// Ad artik `Type`'tan TURETILEMEZ: `Type` gercek bir tip ifadesidir ve
/// nitelikli olabilir (`kerteriz::test_fakes::SahteGnss`); `::` bir
/// tanimlayicida yer alamaz. Bu yuzden satir numarasi kullanilir. Tek bir
/// makro genislemesi icindeki tum `__LINE__` gecisleri ayni degeri verir,
/// dolayisiyla beyan ile tanim ayni ada cozulur.
#define KERTERIZ_BIRLESTIR_ICSEL(a, b) a##b
#define KERTERIZ_BIRLESTIR(a, b) KERTERIZ_BIRLESTIR_ICSEL(a, b)

/// Bir olcum tipini fabrikaya kaydeder (INTERFACES §3).
/// Olcum sinifinin yanina, kendi ceviri biriminde yazilir.
///
/// `Type` GERCEK BIR TIP olmalidir; kapsamda cozulmezse derleme duser.
#define KERTERIZ_REGISTER_MEASUREMENT(Type, ConfigName)                                            \
  namespace {                                                                                      \
  const ::kerteriz::measurements::MeasurementRegistrar<Type>                                       \
      KERTERIZ_BIRLESTIR(kerteriz_olcum_kaydi_, __LINE__){#Type, ConfigName, __FILE__, __LINE__};  \
  }                                                                                                \
  static_assert(true, "noktali virgul ile bitir")

/// Bir olcum tipinin Jacobian testini kaydeder VE govdesini tanimlar.
///
/// Makro govde BEKLER — bu kasitlidir:
///
///   KERTERIZ_REGISTER_JACOBIAN_TEST(GnssPosition) {
///     ... numeric_residual_jacobian ile analitik Jacobian karsilastirmasi ...
///   }
///
/// Kayit ile testin kendisi tek yapidir. Govdesiz bir kayit derlenmez:
/// ic-baglantili fonksiyon beyan edilip adresi alinir ama tanimlanmazsa
/// ceviri birimi baglanmaz. Yani "testi olmayan test kaydi" mumkun degildir.
///
/// Govde gtest makrolari kullanabilir; registry gtest'i TANIMAZ. Testler
/// denetim testi icinde cagrilir, bu yuzden EXPECT_*/ASSERT_* dogru raporlanir.
/// `Type` burada da GERCEK BIR TIPTIR — kayit ile test ayni tipe baglanir.
#define KERTERIZ_REGISTER_JACOBIAN_TEST(Type)                                                      \
  static void KERTERIZ_BIRLESTIR(kerteriz_jacobian_testi_, __LINE__)();                            \
  namespace {                                                                                      \
  const ::kerteriz::measurements::JacobianTestRegistrar<Type>                                      \
      KERTERIZ_BIRLESTIR(kerteriz_jacobian_kaydi_, __LINE__){                                      \
          #Type, &KERTERIZ_BIRLESTIR(kerteriz_jacobian_testi_, __LINE__), __FILE__, __LINE__};     \
  }                                                                                                \
  static void KERTERIZ_BIRLESTIR(kerteriz_jacobian_testi_, __LINE__)()
