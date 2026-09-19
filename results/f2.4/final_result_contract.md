# F2.4-D · Final sonuç ve rapor sözleşmesi — ÖN KAYIT

> **Bu dosya sonuç içermez ve sonuç içermeyecektir.**
> Amacı, `0009`, `0022` ve `0101` sonuçları **görülmeden önce** final F2.4
> raporunun hangi alanları taşıyacağını, hangi kapılardan geçeceğini ve hangi
> yorumlara **izin verilmediğini** dondurmaktır.
>
> Sonuç sunumu sonuçtan sonra tasarlanırsa, tasarım sonuca göre seçilmiş olur.
> Bu dosyanın tek işi o serbestliği şimdi harcamaktır.

Donduran commit: bu dosyanın eklendiği commit.
Deney protokolü: [`sequence_manifest.md`](sequence_manifest.md) — **değiştirilmedi**.
Bu sözleşme yazıldığında eksik olan diziler: `0009`, `0022`, `0101`.

## Değiştirme kuralı

Bu dosya **sonuçlar görüldükten sonra genişletilmez**. Sonuçtan sonra:

* yeni bir alan **eklenmez**,
* yeni bir toplu (aggregate) ölçüt **eklenmez**,
* kabul kapısı **gevşetilmez**,
* dizi listesi **değişmez**.

Sonuçtan **önce** bir alanın eksik olduğu fark edilirse bu dosya güncellenebilir;
güncelleme ayrı bir commit olur ve hangi sonuçların o anda **görülmemiş** olduğunu
yazar. Sonuçtan sonra tek meşru değişiklik, bir alanın **ölçülemediğini** FAIL
olarak kaydetmektir.

---

## 1. Birincil ön kayıtlı küme

Final tabloya **yalnızca** şu altı dizi girer:

```
2011_09_26_drive_0013_sync
2011_09_26_drive_0009_sync
2011_09_26_drive_0022_sync
2011_09_26_drive_0039_sync
2011_09_26_drive_0027_sync
2011_09_26_drive_0101_sync
```

`2011_09_26_drive_0032_sync` bu kümeye **girmez** — hiçbir koşulda, hiçbir
istatistikte, hiçbir tamamlanma sayımında. Bkz. §5.

---

## 2. Zorunlu alanlar — dizi başına şema

Final rapor **dizi başına bir blok** olarak sunulur (23 alanlı tek bir geniş
tablo okunmaz). Alanların **hiçbiri atlanamaz**; ölçülemeyen alan boş
bırakılmaz, `ÖLÇÜLEMEDİ` + gerekçe yazılır.

| # | alan | birim / tip | kaynak |
|---|---|---|---|
| 1 | `sequence` | dizi adı | manifest |
| 2 | `category` | City / Residential / Road | manifest |
| 3 | `measured_frame_count` | tamsayı | koşucu özeti (`referans toplam`) |
| 4 | `measured_duration_s` | saniye | koşucu özeti (ilk–son damga) |
| 5 | `interpolated_frame_count` | tamsayı | koşucu özeti (`ara-degerlenmis`) |
| 6 | `usable_reference_count` | tamsayı | koşucu özeti (`ATE'ye giren`) |
| 7 | `post_init_gnss_candidate_count` | tamsayı | `başlatma sonrası aday` |
| 8 | `candidate_rate_hz` | Hz | `aday hızı` — **diziden türetilir**, gömülü sabit değil |
| 9 | `selected_gnss_count` | tamsayı | `seçili kullanılabilir ölçüm` |
| 10 | `kerteriz_measurement_stamp_digest` | uint64 | koşucu: `ölçüm damga özeti` |
| 11 | `rl_measurement_stamp_digest` | uint64 | yayıncı log'u: `damga_ozeti=` |
| 12 | `digest_equality` | **PASS / FAIL** | 10 ve 11'in birebir karşılaştırması |
| 13 | `withheld_strict_common_support_n` | tamsayı | `trajectory_eval` — ORTAK destek |
| 14 | `kerteriz_withheld_rmse_m` | metre | `trajectory_eval`, hizalama YOK |
| 15 | `kerteriz_withheld_max_m` | metre | `trajectory_eval` |
| 16 | `rl_run1_rmse_m` | metre | RL tekrar 1 |
| 17 | `rl_run2_rmse_m` | metre | RL tekrar 2 |
| 18 | `rl_run3_rmse_m` | metre | RL tekrar 3 |
| 19 | `rl_mean_rmse_m` | metre | 16–18 ortalaması |
| 20 | `rl_rmse_min_max_range_m` | metre (min, maks) | 16–18 yayılımı |
| 21 | `rl_max_error_m` | metre | üç koşudaki en büyük tekil hata |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | metre | `evo_ape -r trans_part` ile iç ATE farkı |
| 23 | `kerteriz_rejection_diagnostics` | üçlü sayım | koşucu: `chi-kare-kapisi`, `sayısal-başarısızlık`, `tampon-reddi` |

**Alan 19–21 dizi içi betimleyicidir.** Diziler arası birleştirilmeleri §4'te
yasaklanmıştır.

**Alan 22 için ad zorunludur:** rapor bunu *"evo ölçüt-aritmetiği çapraz
kontrolü"* diye adlandırır, *"bağımsız doğrulama"* diye **adlandırmaz**
(gerekçe §6.5).

---

## 3. Kabul kapıları

Bir dizinin final tabloya **geçerli sonuç** olarak girebilmesi için **on iki
kapının hepsi** PASS olmalıdır:

| # | kapı | PASS ölçütü |
|---|---|---|
| G1 | envanter tam | §2'nin 1–9 numaralı alanları ölçülmüş |
| G2 | dondurulmuş stride | `--gnss-position-stride 10`, iki tarafta da |
| G3 | GNSS hızı kapalı | koşucuda `--no-gnss-velocity`; yayıncıda hiç yayınlanmıyor |
| G4 | filtre değişmedi | Q / P₀ / R / kapı güveni F2.4-C'deki değerler |
| G5 | seçim politikadan geliyor | `selected_gnss_count`, paylaşımlı `gnss_sampling` planından |
| G6 | **digest eşitliği** | alan 10 == alan 11, birebir |
| G7 | kesişim boş | ölçüm kümesi ∩ withheld kümesi = ∅ |
| G8 | ortak destek boş değil | `withheld_strict_common_support_n` > 0 |
| G9 | üç RL tekrarı tam | üçü de koşmuş ve kaydedilmiş |
| G10 | evo çapraz kontrolü | **en az bir** RL koşusu için `evo_ape -r trans_part` yapılmış |
| G11 | hizalama yok | ne Sim(3) ne rijit fit; ham ENU |
| G12 | post-hoc ayar yok | sonuç görüldükten sonra hiçbir parametre değişmemiş |

### FAIL nasıl gösterilir

**FAIL olan dizi tablodan çıkarılmaz.** Sessizce dışlamak, ön kaydı geçersiz
kılar ve seçim yanlılığı üretir. FAIL eden dizi:

* tabloda **kalır**,
* durumu `FAIL` yazılır,
* **hangi kapının** düştüğü (G1…G12) ve **neden** düştüğü yazılır,
* sayısal alanları ölçülebildiği kadarıyla **yine de raporlanır**; ölçülemeyenler
  `ÖLÇÜLEMEDİ` olur,
* §4'ün izin verdiği betimleyici aralıklara **katılmaz**, ama aralığın yanında
  "n diziden k'si FAIL" olarak **görünür**.

---

## 4. Toplama (aggregation) politikası — sonuç görülmeden donduruldu

### Birincil sonuç

**F2.4-D'nin birincil sonucu DİZİ BAŞINA sonuçlardır.** Altı dizi, altı blok.
Birleştirilmiş tek bir sayı birincil sonuç **değildir**.

### Yasak — üretilmeyecek

* genel **"Kerteriz vs RL kazananı"**
* **"X kat daha iyi"** türü tek cümlelik üstünlük ifadesi
* toplam skor, doğruluk sıralaması (accuracy ranking)
* **global pooled RMSE** (dizileri birleştiren tek RMSE)
* **süre ağırlıklı** (duration-weighted) skor veya benzeri ağırlıklandırma
* `0032`'nin herhangi bir toplu sayıya katılması

Gerekçe [`baseline_audit.md`](baseline_audit.md)'dedir: mevcut dondurulmuş RL
seyrek taban çizgisi, uzun ve dinamik dizilerde **adil bir doğruluk kıyası
değildir**. Bu kurulumdan çıkan bir "kazanan", kestirimcinin değil deney
kurulumunun sonucudur.

### RMSE oranı

Dizi başına RMSE oranı **tutulabilir**, ancak yalnızca **sequence-level
betimleyici tanı** (descriptive diagnostic) olarak ve yukarıdaki sınırla
**birlikte** yazılır. Oranlar diziler arası ortalanmaz, sıralanmaz ve tek bir
sayıya indirgenmez.

### 6/6 tamamlandığında izin verilen betimleyici özetler

Altı dizi de tamamlandığında — **istenirse** — yalnızca şunlar yazılabilir:

1. Kerteriz dizi başına withheld RMSE **aralığı** (min–maks, dizi adlarıyla)
2. RL dizi başına withheld RMSE **aralığı** (min–maks, dizi adlarıyla)
3. `withheld_strict_common_support_n` **aralığı**
4. Kaç dizide RL ölü-hesap ıraksaması **gözlendiğine** dair betimleyici
   açıklama — **önceden tanımlı sayısal eşik OLMADAN**. Eşik sonuçtan sonra
   seçilseydi, gözlemi doğrulayan eşik seçilmiş olurdu. Açıklama niteldir ve
   hangi dizilerde görüldüğünü ada göre sayar.

Bu dört madde **kapalı bir listedir**. Sonuç görüldükten sonra **beşincisi
eklenmez**.

---

## 5. Yardımcı gözlem — `0032`

`2011_09_26_drive_0032_sync`, §2'deki **aynı temel alanlarla** ve **aynı
protokolle**, **ayrı bir "yardımcı gözlem" tablosunda** verilebilir.

Buna karşılık `0032`:

* ön kayıtlı istatistiklere **katılmaz**,
* §4'ün izin verdiği aralıklara **girmez**,
* dizi seçimi için **kanıt olarak kullanılmaz**,
* **6/6 tamamlanma sayımına girmez** (6/6 yalnızca §1'in altısıdır).

Yardımcı tablo, başlığında bu dört kısıtı **yazılı olarak** taşır.

---

## 6. Bilimsel sınırlar — final raporda zorunlu

Final rapor aşağıdaki beş sınırı **açıkça** taşır. Kısaltılamaz, dipnota
indirilemez.

1. **`same OXTS family != independent ground truth`.** Referans ile filtreye
   verilen GNSS aynı OXTS/INS çözüm ailesinden gelir.
2. **Withholding, doğrudan ölçüm-noktasında-değerlendirme örtüşmesini azaltır,
   bağımsız gerçek (independent truth) oluşturmaz.** Withheld konumlar da aynı
   aileden gelir.
3. **Mevcut dondurulmuş RL seyrek taban çizgisi, uzun ve dinamik dizilerde adil
   bir "hangi kestirimci daha doğru" kıyası değildir.** Bu bir kod hatası değil,
   deney kurulumunun sonucudur.
4. **Yönelim sürüklenmesi → artık yer çekimi sızıntısı yalnızca bir mekanizma
   adayıdır; nedenselliği izole edilmemiştir.** Süre ile dönme dinamiği eldeki
   dizilerle ayrıştırılamıyor.
5. **`evo` mevcut dışa aktarımla ölçüt-aritmetiği çapraz kontrolüdür; bağımsız
   zaman-eşleştirme doğrulaması değildir.** Dışa aktarım kestirimleri ortak
   destekteki referans damgalarıyla yeniden damgalar, dolayısıyla eşleştirme
   zaten sabittir. Eşleştirme sözleşmesinin güvencesi birim testleridir.

---

## 7. Tekrarlanabilirlik durumu — her artefakt sınıflandırılır

Final raporun sunduğu **her** artefakt şu üç sınıftan **birini** taşır:

| sınıf | anlamı |
|---|---|
| **A — yalnız commit'lenmiş koddan üretilebilir** | harici veri gerekmez; depodan yeniden üretilir |
| **B — harici KITTI verisi gerektirir** | kod commit'li, veri değil; veri edinilirse yeniden üretilir |
| **C — teşhis kaydı, `make results` tarafından üretilmiyor** | tek seferlik kayıt; üreticisi depoda yok |

Bu sözleşmenin yazıldığı andaki sınıflandırma:

| artefakt | sınıf |
|---|---|
| `gnss_sampling` seçim politikası ve testleri | **A** |
| withheld CLI sözleşmesi ve testleri | **A** |
| `trajectory_eval` ölçüt ve eşleştirme testleri | **A** |
| dizi envanteri (alanlar 3–9) | **B** |
| withheld RMSE / maks, RL tekrarları, digest'ler | **B** |
| `evo` çapraz kontrolü | **B** |
| [`baseline_audit.md`](baseline_audit.md) | **C** |

`baseline_audit.md` **C sınıfında kalır**. Bu içeriğin Faz 2 tekrarlanabilirlik
kapsamına alınıp alınmayacağı **F2.5 kapsamı belirlenirken** karara bağlanacaktır;
**bu sözleşme o kararı vermez.**

---

## 8. Tamamlanma tanımı

**F2.4-D yalnızca şu yedisi birlikte sağlandığında tamamlanmıştır:**

- [ ] §1'deki **6/6** ön kayıtlı dizi mevcut ve koşturulmuş
- [ ] her dizi için **tüm kabul kapıları raporlanmış** (PASS veya FAIL + gerekçe)
- [ ] `0032` **ayrı** yardımcı tabloda, ön kayıtlı istatistiklerin dışında
- [ ] sonuç sözleşmesine sonuçtan sonra **hiçbir ölçüt eklenmemiş** (§4)
- [ ] final dokümanlar gerçek sonuçlarla **doldurulmuş** (bu sözleşme boş kalır)
- [ ] **tam yerel doğrulama** geçmiş (CLAUDE.md §6'nın dört komutu)
- [ ] **PR CI yeşil**

Bir madde eksikse F2.4-D **devam ediyor** demektir; "büyük ölçüde tamam" diye
raporlanmaz.

---

`same OXTS family != independent ground truth`
