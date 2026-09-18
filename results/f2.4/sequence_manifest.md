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
| 1 | `2011_09_26_drive_0013_sync` | City | **144 (ölçüldü)** | **14.81 s (ölçüldü)** | **MEVCUT** |
| 2 | `2011_09_26_drive_0009_sync` | City | 447 | ~46 s | eksik |
| 3 | `2011_09_26_drive_0022_sync` | Residential | 800 | ~83 s | eksik |
| 4 | `2011_09_26_drive_0039_sync` | Residential | 395 | ~41 s | eksik |
| 5 | `2011_09_26_drive_0027_sync` | Road | 188 | ~19 s | eksik |
| 6 | `2011_09_26_drive_0101_sync` | Road | 936 | ~97 s | eksik |

**Kare sayıları `0013` dışında KATALOG BİLGİSİDİR ve doğrulanmamıştır.**
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

## Bilimsel sınır

`0013` için ölçülen her şey ve ileride bu altı dizi için ölçülecek her şey,
referans ile filtreye verilen GNSS'in **aynı OXTS/INS çözüm ailesinden**
gelmesi kısıtını taşır. Seyreltme deneyi doğrudan-ölçüm-noktasında-değerlendirme
karışmasını azaltır; **bağımsız ground truth sağlamaz.**

`same OXTS family != independent ground truth`
