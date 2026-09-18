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

| # | dizi | kategori | kare (katalog) | süre (~) | durum |
|---|---|---|---|---|---|
| 1 | `2011_09_26_drive_0013_sync` | City | **144 (ölçüldü)** | **14.810 s (ölçüldü)** | **MEVCUT** |
| 2 | `2011_09_26_drive_0009_sync` | City | 447 | ~46 s | eksik |
| 3 | `2011_09_26_drive_0022_sync` | Residential | 800 | ~83 s | eksik |
| 4 | `2011_09_26_drive_0039_sync` | Residential | **395 (ölçüldü)** | **40.672 s (ölçüldü)** | **MEVCUT** |
| 5 | `2011_09_26_drive_0027_sync` | Road | **188 (ölçüldü)** | **19.371 s (ölçüldü)** | **MEVCUT** |
| 6 | `2011_09_26_drive_0101_sync` | Road | 936 | ~97 s | eksik |

**Katalog doğrulaması.** `0027` ve `0039` indirildi; ölçülen kare sayıları
(188 ve 395) manifest yazılırken kaydedilen katalog değerleriyle **birebir**
uyuştu. Dizi seçimi değişmedi.

Ölçülen envanter (indirilmiş üçü):

| dizi | kare | süre | ara-değerlenmiş | kullanılabilir referans | başlatma sonrası aday | aday hızı |
|---|---|---|---|---|---|---|
| `0013` | 144 | 14.810 s | 0 | 144 | 143 | 9.6529 Hz |
| `0027` | 188 | 19.371 s | 0 | 188 | 187 | 9.6570 Hz |
| `0039` | 395 | 40.672 s | 0 | 395 | 394 | 9.6889 Hz |

## Ön kayıtlı sete DAHİL OLMAYAN yardımcı dizi

`2011_09_26_drive_0032_sync` (Road, 390 kare, 40.312 s) de sistemde mevcuttur
ve aynı protokolle koşturulmuştur. **Ön kayıtlı altılının parçası DEĞİLDİR**
ve ön kayıtlı toplu sonuca **katılmaz**; ayrı, yardımcı gözlem olarak
raporlanır. Sonuç görüldükten sonra listeye eklenmesi ön kaydı geçersiz
kılardı.

**Kare sayıları `0013`, `0027` ve `0039` için ÖLÇÜLMÜŞTÜR; `0009`, `0022` ve
`0101` için KATALOG BİLGİSİDİR ve doğrulanmamıştır.**
İndirildiğinde D1 envanteri yeniden çıkarılacak; gerçek sayı katalogdan
saparsa manifest sapma notuyla güncellenir — dizi seçimi değişmez.

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
Özet: `sensor_timeout`, eksen konvansiyonu, yer çekimi yorumu ve "GNSS
füzyona girmiyor" hipotezleri **elendi**; büyüyen sorun güncellemeler arası
ölü-hesap sürüklenmesidir. Mevcut dondurulmuş RL yapılandırması seyrek/uzun
dizilerde adil bir doğruluk kıyası olarak yorumlanmamalıdır.

## Bilimsel sınır

`0013` için ölçülen her şey ve ileride bu altı dizi için ölçülecek her şey,
referans ile filtreye verilen GNSS'in **aynı OXTS/INS çözüm ailesinden**
gelmesi kısıtını taşır. Seyreltme deneyi doğrudan-ölçüm-noktasında-değerlendirme
karışmasını azaltır; **bağımsız ground truth sağlamaz.**

`same OXTS family != independent ground truth`
