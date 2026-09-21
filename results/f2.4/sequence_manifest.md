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
| `0009` | 447 | 447 | 46.172 s | 4 | 443 | 446 | 9.6609 Hz |
| `0022` | 800 | 800 | 82.774 s | 0 | 800 | 799 | 9.6524 Hz |
| `0039` | 395 | 395 | 40.672 s | 0 | 395 | 394 | 9.6889 Hz |
| `0027` | 188 | 188 | 19.371 s | 0 | 188 | 187 | 9.6570 Hz |
| `0101` | 936 | 936 | 96.624 s | 2 | 934 | 935 | 9.6763 Hz |

**Birincil küme fiziksel olarak 6/6 MEVCUT ve 6/6 AYRIŞTIRILABİLİR.**
Hiçbir alan ÖLÇÜLEMEDİ değildir; altı dizinin tüm envanter alanları gerçek
ayrıştırıcı ve gerçek örnekleme politikasıyla okunmuştur.

### TARİHSEL KAYIT — ayrıştırıcı engeli: bulundu ve çözüldü

**Durum: ÇÖZÜLDÜ.** Engel `15bcb5b` commit'inde tespit edilip kayda geçti;
biçim-uyumluluğu düzeltmesi **bu dosyanın güncellendiği commit'te** yapıldı.
Kayıt silinmiyor: neyin neden ölçülemediği ve nasıl çözüldüğü, sonradan
"zaten çalışıyordu" diye okunmasın diye duruyor.

**Bulgu.** KITTI, kısa OXTS kesintilerinde kaydın tüm alanlarını doğrusal
ara-değerler ve bunu son üç kipi (`posmode`, `velmode`, `orimode`) `-1`
yaparak işaretler. Ölçülen gerçek: bu karelerde veri seti **30 alanın tamamını**
tek tip `%.14f` ile yazıyor — kategorik alanlar dahil. Yani işaret
`-1.00000000000000` olarak geliyor, normal karelerdeki `4 8 4 4 0` yerine.

Depodaki ayrıştırıcı kategorik alanların **tam tamsayı** olmasını şart
koşuyordu (`tam_isaretli_sayi`, F1.8-A'da bilinçli konmuş bir veri-kalitesi
kapısı: `3.7` sessizce `3` olmasın diye). Ondalık nokta gördüğü anda satırı
reddediyor, `load_kitti_oxts` ilk redde tüm diziyi düşürüyordu.

Ölçüm sırasında indirilmiş on dizinin **3832 karesi tek tek denendi**;
ondalık-kategorik kare kümesi ile `-1` işaretli kare kümesi **birebir aynı
çıktı, sıfır istisna**:

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

Yani kapı, tam da veri setinin **kendi eksik-bilgi işaretini taşıyan**
kareleri reddediyordu. O turda ayrıştırıcıya dokunulmadı, seçim değiştirilmedi,
sayı uydurulmadı; birincil set fiziksel 6/6 ama ayrıştırılabilir 4/6 olarak
raporlandı ve karar kullanıcıya bırakıldı.

**Çözüm — biçim uyumluluğu, kapı gevşetmesi DEĞİL.** Karar verilmeden önce on
dizinin **20660 kategorik simgesinin tamamı** tarandı:

| simge sınıfı | adet |
|---|---|
| tamsayı (`4`, `-1`) | 20465 |
| ondalık, kesir kısmı **tamamen sıfır** (`-1.00000000000000`) | 195 |
| ondalık, kesir kısmı sıfırdan farklı | **0** |
| üssel gösterim | **0** |
| bozuk / artık simgeli | **0** |

195 simgenin tamamı 14 basamaklı ve tamamı sıfır; etkilenen 39 karenin
hepsinde beş kategorik alanın **beşi de** ondalık (kısmi vaka yok).

Buna göre `tam_isaretli_sayi` **dar** biçimde genişletildi: kesir kısmı
tamamen sıfırsa simge kabul edilir, aksi halde reddedilir. Karar metinsel ve
tamsayı aritmetiğiyle verilir; **kayan noktaya çevirme veya cast yoktur** —
o yol tam da önlenmek istenen yuvarlama semantiğini geri getirirdi.

F1.8-A'nın **sessiz kırpma yasağı aynen durur**: `3.7` ve `5.0001` hâlâ
reddedilir, çünkü kırpılacak bir şey olduğunda kırpmıyoruz, **reddediyoruz**.
`2e0`, `5.`, `.0`, artık simge ve taşma da reddedilmeye devam eder.
Eksik-bilgi işaretinin **değeri** taşınır, biçimi değil: `-1.00000000000000`
ile `-1` aynı işarettir ve `interpolated_missing` ikisinde de kurulur.

**NCLT ayrıştırıcısına dokunulmadı** — gerekçe KITTI'ye özgü bir yazım
biçimidir.
## YARDIMCI KÜME — DONDURULDU (sonuç görülmeden)

Aşağıdaki dört dizi sistemde mevcuttur ve **bu bölüm herhangi bir sonuç,
APE, RMSE veya koşu çıktısı görülmeden donduruldu.** Seçim ölçütü yalnızca
"aynı takvim gününden, edinilebilir ek dizi"dir; hiçbiri bir metriğe bakılarak
alınmamıştır.

| dizi | kare | damga | süre | ara-değerlenmiş | kullanılabilir referans | başlatma sonrası aday | aday hızı |
|---|---|---|---|---|---|---|---|
| `0032` | 390 | 390 | 40.312 s | 0 | 390 | 389 | 9.6489 Hz |
| `0070` | 420 | 420 | 43.307 s | 20 | 400 | 418 | 9.6748 Hz |
| `0079` | 100 | 100 | 10.240 s | 1 | 99 | 99 | 9.6643 Hz |
| `0104` | 312 | 312 | 32.171 s | 12 | 300 | 311 | 9.6660 Hz |

Dördü de ayrıştırılabilir; `0070`/`0079`/`0104` alanları da biçim uyumluluğu
düzeltmesinden sonra ölçüldü. `0070`'te ilk kare ara-değerlenmiş olduğu için
başlatma damgası ikinci kareden alınır — aday sayısının 418 olması bundandır.

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
