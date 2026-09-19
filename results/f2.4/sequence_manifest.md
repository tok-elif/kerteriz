# F2.4-D · Çoklu KITTI dizi protokolü — ÖN KAYIT

**Bu dosya sonuç içermez.** Amacı, hangi dizilerin hangi gerekçeyle seçildiğini
**herhangi bir APE sonucu üretilmeden önce** dondurmaktır. Sonuçlara bakıp
dizi listesini değiştirmek, deneyi kanıt olmaktan çıkarır.

Donduran commit: bu dosyanın eklendiği commit.
Deney protokolü: **F2.4-C'den aynen devralındı, değiştirilmedi.**

## Dondurulmuş deney protokolü (F2.4-C)

```
GNSS konum stride      10
GNSS hiz               kapali (iki kestirimcide de)
IMU                    tam hizli
Q / P0 / R / kapi      degismedi
sampling policy        gnss_sampling.hpp (paylasimli, saf)
degerlendirme          yalnizca withheld, ara-degerlenmemis referans damgalari
stamp esitligi         Kerteriz ve robot_localization AYNI secili GNSS kumesi
metrik                 ortak destek uzerinde oteleme APE, HIZALAMA YOK
tolerans               20 ms
RL tekrari             dizi basina 3
sonuc sonrasi          parametre degisikligi YOK
```

## Seçilen 6 dizi

Seçim ölçütleri (önceden belirlendi, sonuç metriklerine bakılmadan):

* `0013` **zorunlu** — F2.4-A/B/C'nin legacy çapası, tüm dondurulmuş sayılar ondan
* kategori çeşitliliği: 2 × City, 2 × Residential, 2 × Road
* sürüş uzunluğu/dinamiği çeşitliliği: ~15 s'den ~95 s'e
* çok kısa veya OXTS'i bozuk diziler seçilmedi
* tamamı **2011_09_26** takvim gününden: tek bir `2011_09_26_calib.zip` yeterli
  olsun ve oturumlar arası gizli bir fark girmesin diye. Adaptör `calib`
  okumaz; bu yine de edinme yükünü ve değişken sayısını azaltır.

| # | dizi | kategori | kare | süre | durum |
|---|---|---|---|---|---|
| 1 | `2011_09_26_drive_0013_sync` | City | **144 (ölçüldü)** | **14.810 s (ölçüldü)** | **MEVCUT** |
| 2 | `2011_09_26_drive_0009_sync` | City | **447 (ölçüldü)** | **46.172 s (ölçüldü)** | **MEVCUT** |
| 3 | `2011_09_26_drive_0022_sync` | Residential | **800 (ölçüldü)** | **82.774 s (ölçüldü)** | **MEVCUT** |
| 4 | `2011_09_26_drive_0039_sync` | Residential | **395 (ölçüldü)** | **40.672 s (ölçüldü)** | **MEVCUT** |
| 5 | `2011_09_26_drive_0027_sync` | Road | **188 (ölçüldü)** | **19.371 s (ölçüldü)** | **MEVCUT** |
| 6 | `2011_09_26_drive_0101_sync` | Road | **936 (ölçüldü)** | **96.624 s (ölçüldü)** | **MEVCUT** |

**Katalog doğrulaması — birincil altının tamamı.** Altı dizi de indirildi.
Manifest yazılırken kaydedilen katalog kare sayıları ile ölçülen kare sayıları
**altısında da birebir** uyuştu; **sapma yoktur.** Dizi seçimi değişmedi.

| dizi | katalog kare | ölçülen kare | sapma | katalog süre | ölçülen süre |
|---|---|---|---|---|---|
| `0013` | 144 | 144 | 0 | ~15 s | 14.810 s |
| `0009` | 447 | 447 | 0 | ~46 s | 46.172 s |
| `0022` | 800 | 800 | 0 | ~83 s | 82.774 s |
| `0039` | 395 | 395 | 0 | ~41 s | 40.672 s |
| `0027` | 188 | 188 | 0 | ~19 s | 19.371 s |
| `0101` | 936 | 936 | 0 | ~97 s | 96.624 s |

Ölçülen envanter (birincil altı). Sayılar **deponun gerçek ayrıştırıcı ve
örnekleme politikası kodundan** okundu (`kitti_oxts.hpp`, `gnss_sampling.hpp`);
hiçbiri elle tahmin edilmedi. Kestirim çalıştırılmadı, sonuç üretilmedi.

| dizi | kare | damga | süre | ara-değerlenmiş | kullanılabilir referans | başlatma sonrası aday | aday hızı |
|---|---|---|---|---|---|---|---|
| `0013` | 144 | 144 | 14.810 s | 0 | 144 | 143 | 9.6529 Hz |
| `0009` | 447 | 447 | 46.172 s | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ |
| `0022` | 800 | 800 | 82.774 s | 0 | 800 | 799 | 9.6524 Hz |
| `0039` | 395 | 395 | 40.672 s | 0 | 395 | 394 | 9.6889 Hz |
| `0027` | 188 | 188 | 19.371 s | 0 | 188 | 187 | 9.6570 Hz |
| `0101` | 936 | 936 | 96.624 s | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ |

Boş alan bırakılmaz: `0009` ve `0101` için bu dört alan **ÖLÇÜLEMEDİ** ve
gerekçesi aşağıdadır. Kare/damga/süre alanları bu dizilerde de gerçek
`parse_kitti_timestamp` ile ÖLÇÜLDÜ; katalogdan alınmadı.

### ÖLÇÜLEMEDİ gerekçesi — ayrıştırıcı ara-değerlenmiş kareyi reddediyor

KITTI, kısa OXTS kesintilerinde kaydın tüm alanlarını doğrusal ara-değerler ve
bunu son üç kipi (`posmode`, `velmode`, `orimode`) `-1` yaparak işaretler.
Ölçülen gerçek: bu karelerde veri seti **kategorik alanları da ondalık biçimde**
yazıyor — `-1.00000000000000`, normal karelerdeki `4 8 4 4 0` yerine.

Depodaki ayrıştırıcı kategorik alanların **tam tamsayı** olmasını şart koşar
(`tam_isaretli_sayi`, F1.8-A'da bilinçli olarak konmuş bir veri-kalitesi
kapısı: `3.7` sessizce `3` olmasın diye). Ondalık nokta gördüğü anda satırı
reddeder, `load_kitti_oxts` ilk redde tüm diziyi düşürür.

İndirilmiş on dizinin **3832 karesi tek tek denendi**; ondalık-kategorik kare
kümesi ile `-1` işaretli kare kümesi **birebir aynıdır, sıfır istisna**:

| dizi | kare | reddedilen | ilk reddedilen kare |
|---|---|---|---|
| `0013` | 144 | 0 | — |
| `0009` | 447 | **4** | 178 |
| `0022` | 800 | 0 | — |
| `0039` | 395 | 0 | — |
| `0027` | 188 | 0 | — |
| `0101` | 936 | **2** | 239 |
| `0032` | 390 | 0 | — |
| `0070` | 420 | **20** | 0 |
| `0079` | 100 | **1** | 47 |
| `0104` | 312 | **12** | 45 |

Yani kapı, tam da veri setinin **kendi eksik-bilgi işaretini taşıyan** kareleri
reddediyor. **Bu bir karardır, sessizce verilmez:** ayrıştırıcıya
dokunulmamıştır, seçim değiştirilmemiştir, sayı uydurulmamıştır. Birincil set
fiziksel olarak 6/6 mevcuttur; ayrıştırılabilirlik 4/6'dır ve bu fark burada
açıkça kayıtlıdır.
## YARDIMCI KÜME — DONDURULDU (sonuç görülmeden)

Aşağıdaki dört dizi sistemde mevcuttur ve **bu bölüm herhangi bir sonuç,
APE, RMSE veya koşu çıktısı görülmeden donduruldu.** Seçim ölçütü yalnızca
"aynı takvim gününden, edinilebilir ek dizi"dir; hiçbiri bir metriğe bakılarak
alınmamıştır.

| dizi | kare | damga | süre | ara-değerlenmiş | kullanılabilir referans | başlatma sonrası aday | aday hızı |
|---|---|---|---|---|---|---|---|
| `0032` | 390 | 390 | 40.312 s | 0 | 390 | 389 | 9.6489 Hz |
| `0070` | 420 | 420 | 43.307 s | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ |
| `0079` | 100 | 100 | 10.240 s | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ |
| `0104` | 312 | 312 | 32.171 s | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ | ÖLÇÜLEMEDİ |

`0070`, `0079` ve `0104` için gerekçe birincil kümedekiyle **aynıdır**:
ara-değerlenmiş kareler ondalık kategorik alan taşıyor ve ayrıştırıcı kapısı
onları reddediyor (yukarıdaki bölüm).

**Kategori.** `0032` için `Road` daha önce kaydedilmişti ve değiştirilmedi.
`0070`, `0079` ve `0104` için **kategori bilgisi YAZILMAMIŞTIR**: elde
doğrulanabilir yerel bir kaynak yoktur ve kategori uydurulmaz. Gerekirse resmî
KITTI raw sayfasından doğrulanıp sonradan eklenir.

Dördü için de bağlayıcı kurallar:

* **Ön kayıtlı BİRİNCİL sete dahil DEĞİLDİRLER.**
* **6/6 tamamlanma sayımına girmezler.** Tamamlanma yalnızca `0013`, `0009`,
  `0022`, `0039`, `0027`, `0101` üzerinden sayılır.
* **Birincil istatistiğe / toplu sonuca katılmazlar** — ortalama, aralık,
  oran, sıralama, hiçbirine.
* Yalnızca **dayanıklılık (robustness) ve mekanizma gözlemi** için
  kullanılabilirler.
* **Seçim sonuç görülmeden yapıldı.** Sonuç görüldükten sonra bu listeye dizi
  eklenmesi veya bir dizinin birincil kümeye terfi ettirilmesi ön kaydı
  geçersiz kılar.

## Gerekli dosya yapısı

Her dizi için yalnız `oxts/` gerekir; görüntü ve Velodyne **okunmaz**:

```
2011_09_26_drive_XXXX_sync/
└── oxts/
    ├── timestamps.txt
    └── data/
        ├── 0000000000.txt
        └── ...
```

Resmi kaynak `cvlibs.net` KITTI raw sayfasıdır ve **kayıt/kabul gerektirir**.
Gayriresmî ayna ile bu koşul aşılmaz; veri seti depoya da konmaz.

## Taban çizgisi kök-neden denetimi

`0032` ve `0039` dizilerinde harici `robot_localization` taban çizgisinin
ıraksamasına dair denetim kaydı: [`baseline_audit.md`](baseline_audit.md).
Özet: `sensor_timeout` ölçüm düşürüyor, KITTI IMU eksen/işaret konvansiyonu
yanlış, **ivmeölçer yer çekimi içermiyor** ve "GNSS füzyona girmiyor"
hipotezleri **elendi**. Büyüyen sorun güncellemeler arası ölü-hesap
sürüklenmesidir.

Dikkat: elenen hipotez *ivmeölçerin yer çekimi içerip içermediğidir* — yani
`ax/ay/az` yer çekimi içeren özgül kuvvettir ve
`imu0_remove_gravitational_acceleration: true` doğru ayardır.
Bu, yer çekimini tablodan çıkarmaz:
**yönelim sürüklenmesi → artık yer çekimi sızıntısı** hâlâ ayakta duran tek
mekanizma adayıdır ve **nedenselliği izole edilmemiştir** (süre ile dönme
dinamiği dört diziyle ayrıştırılamıyor).

Mevcut dondurulmuş RL yapılandırması seyrek/uzun dizilerde adil bir doğruluk
kıyası olarak yorumlanmamalıdır.

## Final rapor sözleşmesi

Sonuçların hangi alanlarla sunulacağı, hangi kabul kapılarından geçeceği ve
hangi toplu yorumların **yapılmayacağı** — `0009`, `0022`, `0101` görülmeden
önce donduruldu: [`final_result_contract.md`](final_result_contract.md).

## Bilimsel sınır

`0013` için ölçülen her şey ve ileride bu altı dizi için ölçülecek her şey,
referans ile filtreye verilen GNSS'in **aynı OXTS/INS çözüm ailesinden**
gelmesi kısıtını taşır. Seyreltme deneyi doğrudan-ölçüm-noktasında-değerlendirme
karışmasını azaltır; **bağımsız ground truth sağlamaz.**

`same OXTS family != independent ground truth`
