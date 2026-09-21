# F2.4-D · Final sonuçlar

> Bu dosya **ölçülmüş sonuçları** taşır. Hangi alanların yazılacağı, hangi
> kapılardan geçileceği ve hangi yorumların **yapılmayacağı** sonuçlar
> görülmeden önce [`final_result_contract.md`](final_result_contract.md)'de
> donduruldu. **O sözleşme bu turda değiştirilmedi.** Yeni ölçüt, yeni alan ve
> yeni kapı eklenmedi.

Kod sürümü: `2d4428a442c09626a92ec316371adbafa701af9e`
Deney protokolü: [`sequence_manifest.md`](sequence_manifest.md) — değiştirilmedi.
Envanter alanları (1–9) o dosyanın **ölçülen envanter** tablosundan alınmıştır.

## Dondurulmuş protokol

```
GNSS konum stride      10   (iki kestirimcide de)
GNSS hiz               kapali (iki kestirimcide de)
IMU                    tam hizli
Q / P0 / R / kapi      degismedi
sampling policy        gnss_sampling.hpp (paylasimli, saf)
degerlendirme          yalnizca withheld, ara-degerlenmemis referans damgalari
metrik                 STRICT ortak destek uzerinde oteleme APE, HIZALAMA YOK
tolerans               20 ms
RL tekrari             dizi basina 3
```

**Strict ortak destek** = withheld referans damgalarından, Kerteriz **ve üç RL
koşusunun hepsinin** 20 ms içinde eşleştirdiği damgalar. Alan 14–21'in tamamı
bu tek küme üzerinde hesaplanmıştır. Kerteriz-only teşhis koşularının withheld
RMSE değerleri **bu dosyada kullanılmaz** — destek kümesi farklıdır.

## Artefakt kaynağı — tam açıklık

`0009`, `0022`, `0101` ve dört yardımcı dizinin tüm artefaktları bu turda
üretildi. `0013`, `0027` ve `0039` için **daha önceki bir oturumda üretilmiş
RL/evo artefaktları sistemde bulunamadı** (yalnız Kerteriz çıktıları vardı).
Olmayan bir ölçümü rapor etmemek için bu üç dizinin **RL koşuları ve evo
çapraz kontrolü protokol değiştirilmeden yeniden ölçüldü**. Hiçbir parametre
değişmedi; bu bir yeniden ayar değil, eksik ölçümün tamamlanmasıdır.

Kerteriz tarafı bunun etkisinde değildir: koşucu çevrimdışı ve **bit-bit
tekrarlanabilirdir**, ve üç dizinin `kerteriz.csv` çıktıları bağımsız teşhis
koşusununkilerle **bit-aynı** çıkmıştır.

---

# 1. Birincil ön kayıtlı küme — 6/6

Sözleşme §2'nin **23 alanının tamamı** dizi başına yazılmıştır. Hiçbir alan
atlanmamıştır.

## 1.1 `2011_09_26_drive_0013_sync`

| # | alan | değer |
|---|---|---|
| 1 | `sequence` | `2011_09_26_drive_0013_sync` |
| 2 | `category` | City |
| 3 | `measured_frame_count` | 144 |
| 4 | `measured_duration_s` | 14.810 |
| 5 | `interpolated_frame_count` | 0 |
| 6 | `usable_reference_count` | 144 |
| 7 | `post_init_gnss_candidate_count` | 143 |
| 8 | `candidate_rate_hz` | 9.6529 |
| 9 | `selected_gnss_count` | 14 |
| 10 | `kerteriz_measurement_stamp_digest` | 6308090508728111009 |
| 11 | `rl_measurement_stamp_digest` | 6308090508728111009 |
| 12 | `digest_equality` | **PASS** |
| 13 | `withheld_strict_common_support_n` | 128 |
| 14 | `kerteriz_withheld_rmse_m` | 0.176361 |
| 15 | `kerteriz_withheld_max_m` | 0.499706 |
| 16 | `rl_run1_rmse_m` | 0.164482 |
| 17 | `rl_run2_rmse_m` | 0.164482 |
| 18 | `rl_run3_rmse_m` | 0.164482 |
| 19 | `rl_mean_rmse_m` | 0.164482 |
| 20 | `rl_rmse_min_max_range_m` | (0.164482, 0.164482) |
| 21 | `rl_max_error_m` | 0.473012 |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | 0.0e+00 |
| 23 | `kerteriz_rejection_diagnostics` | chi-kare 0 · sayısal 0 · tampon 0 |

## 1.2 `2011_09_26_drive_0009_sync`

| # | alan | değer |
|---|---|---|
| 1 | `sequence` | `2011_09_26_drive_0009_sync` |
| 2 | `category` | City |
| 3 | `measured_frame_count` | 447 |
| 4 | `measured_duration_s` | 46.172 |
| 5 | `interpolated_frame_count` | 4 |
| 6 | `usable_reference_count` | 443 |
| 7 | `post_init_gnss_candidate_count` | 446 |
| 8 | `candidate_rate_hz` | 9.6609 |
| 9 | `selected_gnss_count` | 43 |
| 10 | `kerteriz_measurement_stamp_digest` | 17303888309858617821 |
| 11 | `rl_measurement_stamp_digest` | 17303888309858617821 |
| 12 | `digest_equality` | **PASS** |
| 13 | `withheld_strict_common_support_n` | 398 |
| 14 | `kerteriz_withheld_rmse_m` | 0.231282 |
| 15 | `kerteriz_withheld_max_m` | 0.534958 |
| 16 | `rl_run1_rmse_m` | 13.066238 |
| 17 | `rl_run2_rmse_m` | 13.066238 |
| 18 | `rl_run3_rmse_m` | 13.066238 |
| 19 | `rl_mean_rmse_m` | 13.066238 |
| 20 | `rl_rmse_min_max_range_m` | (13.066238, 13.066238) |
| 21 | `rl_max_error_m` | 36.846258 |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | 3.6e-15 |
| 23 | `kerteriz_rejection_diagnostics` | chi-kare 0 · sayısal 0 · tampon 0 |

## 1.3 `2011_09_26_drive_0022_sync`

| # | alan | değer |
|---|---|---|
| 1 | `sequence` | `2011_09_26_drive_0022_sync` |
| 2 | `category` | Residential |
| 3 | `measured_frame_count` | 800 |
| 4 | `measured_duration_s` | 82.774 |
| 5 | `interpolated_frame_count` | 0 |
| 6 | `usable_reference_count` | 800 |
| 7 | `post_init_gnss_candidate_count` | 799 |
| 8 | `candidate_rate_hz` | 9.6524 |
| 9 | `selected_gnss_count` | 79 |
| 10 | `kerteriz_measurement_stamp_digest` | 400008869794288960 |
| 11 | `rl_measurement_stamp_digest` | 400008869794288960 |
| 12 | `digest_equality` | **PASS** |
| 13 | `withheld_strict_common_support_n` | 719 |
| 14 | `kerteriz_withheld_rmse_m` | 0.482228 |
| 15 | `kerteriz_withheld_max_m` | 1.431989 |
| 16 | `rl_run1_rmse_m` | 2.060272 |
| 17 | `rl_run2_rmse_m` | 2.060272 |
| 18 | `rl_run3_rmse_m` | 2.060272 |
| 19 | `rl_mean_rmse_m` | 2.060272 |
| 20 | `rl_rmse_min_max_range_m` | (2.060272, 2.060272) |
| 21 | `rl_max_error_m` | 7.482187 |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | 0.0e+00 |
| 23 | `kerteriz_rejection_diagnostics` | chi-kare 0 · sayısal 0 · tampon 0 |

## 1.4 `2011_09_26_drive_0039_sync`

| # | alan | değer |
|---|---|---|
| 1 | `sequence` | `2011_09_26_drive_0039_sync` |
| 2 | `category` | Residential |
| 3 | `measured_frame_count` | 395 |
| 4 | `measured_duration_s` | 40.672 |
| 5 | `interpolated_frame_count` | 0 |
| 6 | `usable_reference_count` | 395 |
| 7 | `post_init_gnss_candidate_count` | 394 |
| 8 | `candidate_rate_hz` | 9.6889 |
| 9 | `selected_gnss_count` | 39 |
| 10 | `kerteriz_measurement_stamp_digest` | 6030371780485323181 |
| 11 | `rl_measurement_stamp_digest` | 6030371780485323181 |
| 12 | `digest_equality` | **PASS** |
| 13 | `withheld_strict_common_support_n` | 354 |
| 14 | `kerteriz_withheld_rmse_m` | 0.265830 |
| 15 | `kerteriz_withheld_max_m` | 0.579477 |
| 16 | `rl_run1_rmse_m` | 8.906585 |
| 17 | `rl_run2_rmse_m` | 8.906585 |
| 18 | `rl_run3_rmse_m` | 8.906585 |
| 19 | `rl_mean_rmse_m` | 8.906585 |
| 20 | `rl_rmse_min_max_range_m` | (8.906585, 8.906585) — yayılım 2.6e-11 m |
| 21 | `rl_max_error_m` | 40.494786 |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | 0.0e+00 |
| 23 | `kerteriz_rejection_diagnostics` | chi-kare 0 · sayısal 0 · tampon 0 |

## 1.5 `2011_09_26_drive_0027_sync`

| # | alan | değer |
|---|---|---|
| 1 | `sequence` | `2011_09_26_drive_0027_sync` |
| 2 | `category` | Road |
| 3 | `measured_frame_count` | 188 |
| 4 | `measured_duration_s` | 19.371 |
| 5 | `interpolated_frame_count` | 0 |
| 6 | `usable_reference_count` | 188 |
| 7 | `post_init_gnss_candidate_count` | 187 |
| 8 | `candidate_rate_hz` | 9.6570 |
| 9 | `selected_gnss_count` | 18 |
| 10 | `kerteriz_measurement_stamp_digest` | 5057789931279156134 |
| 11 | `rl_measurement_stamp_digest` | 5057789931279156134 |
| 12 | `digest_equality` | **PASS** |
| 13 | `withheld_strict_common_support_n` | 168 |
| 14 | `kerteriz_withheld_rmse_m` | 0.218213 |
| 15 | `kerteriz_withheld_max_m` | 0.664184 |
| 16 | `rl_run1_rmse_m` | 0.161963 |
| 17 | `rl_run2_rmse_m` | 0.161963 |
| 18 | `rl_run3_rmse_m` | 0.161963 |
| 19 | `rl_mean_rmse_m` | 0.161963 |
| 20 | `rl_rmse_min_max_range_m` | (0.161963, 0.161963) |
| 21 | `rl_max_error_m` | 0.558856 |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | 2.8e-17 |
| 23 | `kerteriz_rejection_diagnostics` | chi-kare 0 · sayısal 0 · tampon 0 |

## 1.6 `2011_09_26_drive_0101_sync`

| # | alan | değer |
|---|---|---|
| 1 | `sequence` | `2011_09_26_drive_0101_sync` |
| 2 | `category` | Road |
| 3 | `measured_frame_count` | 936 |
| 4 | `measured_duration_s` | 96.624 |
| 5 | `interpolated_frame_count` | 2 |
| 6 | `usable_reference_count` | 934 |
| 7 | `post_init_gnss_candidate_count` | 935 |
| 8 | `candidate_rate_hz` | 9.6763 |
| 9 | `selected_gnss_count` | 93 |
| 10 | `kerteriz_measurement_stamp_digest` | 10525527416876162791 |
| 11 | `rl_measurement_stamp_digest` | 10525527416876162791 |
| 12 | `digest_equality` | **PASS** |
| 13 | `withheld_strict_common_support_n` | 839 |
| 14 | `kerteriz_withheld_rmse_m` | **128.980607** |
| 15 | `kerteriz_withheld_max_m` | **364.037279** |
| 16 | `rl_run1_rmse_m` | 0.237131 |
| 17 | `rl_run2_rmse_m` | 0.237131 |
| 18 | `rl_run3_rmse_m` | 0.237131 |
| 19 | `rl_mean_rmse_m` | 0.237131 |
| 20 | `rl_rmse_min_max_range_m` | (0.237131, 0.237131) |
| 21 | `rl_max_error_m` | 1.720914 |
| 22 | `evo_arithmetic_crosscheck_abs_diff_m` | 2.8e-17 |
| 23 | `kerteriz_rejection_diagnostics` | **chi-kare 64** · sayısal **0** · tampon **0** |

### `0101` kaydı — ayar YAPILMADI

`0101`, **geçerli bir birincil sonuçtur** ve on iki kapının hepsinden geçer.
Kerteriz bu dizide ıraksamıştır: 93 ölçüm verildi, **29'u kabul edildi, 64'ü
chi-kare kapısında reddedildi**; sayısal başarısızlık ve tampon reddi
**sıfırdır**.

Bu sonuç **düzeltilmedi, ayarlanmadı ve dizi tablodan çıkarılmadı.** Kapıların
hiçbiri sıfır-red şartı koymaz; alan 23 ölçülmüş bir değerdir ve öyle
raporlanır. `0101` için sonuç görüldükten sonra **hiçbir parametre
değiştirilmemiştir** (G12).

---

# 2. Kabul kapıları — birincil küme

| kapı | 0013 | 0009 | 0022 | 0039 | 0027 | 0101 |
|---|---|---|---|---|---|---|
| G1 envanter tam | PASS | PASS | PASS | PASS | PASS | PASS |
| G2 dondurulmuş stride | PASS | PASS | PASS | PASS | PASS | PASS |
| G3 GNSS hızı kapalı | PASS | PASS | PASS | PASS | PASS | PASS |
| G4 filtre değişmedi | PASS | PASS | PASS | PASS | PASS | PASS |
| G5 seçim politikadan | PASS | PASS | PASS | PASS | PASS | PASS |
| G6 digest eşitliği | PASS | PASS | PASS | PASS | PASS | PASS |
| G7 kesişim boş | PASS | PASS | PASS | PASS | PASS | PASS |
| G8 ortak destek boş değil | PASS | PASS | PASS | PASS | PASS | PASS |
| G9 üç RL tekrarı tam | PASS | PASS | PASS | PASS | PASS | PASS |
| G10 evo çapraz kontrolü | PASS | PASS | PASS | PASS | PASS | PASS |
| G11 hizalama yok | PASS | PASS | PASS | PASS | PASS | PASS |
| G12 post-hoc ayar yok | PASS | PASS | PASS | PASS | PASS | PASS |

**Birincil küme: 6/6 dizi, 12/12 kapı PASS.** FAIL eden birincil dizi yoktur.

Kapı dayanakları:

* **G2** — koşucu `--gnss-position-stride 10`, yayıncı `gnss_position_stride:=10`.
* **G3** — koşucu `--no-gnss-velocity` (GNSS hız: verilen 0, kabul 0); yayıncı
  GNSS hızı **hiç yayınlamaz**, `robot_localization` `odom0_config` hız alanları
  `false`.
* **G4** — `kitti_runner.hpp`, `robot_localization_baseline.yaml` ve
  `kerteriz_core/` F2.4-C'den (`7a0f31c`) bu yana **değişmemiştir**.
* **G5** — iki süreç aynı `gnss_sampling` planını kullanır; alan 9 o plandan gelir.
* **G7** — `measurement ∩ withheld_reference = ∅`, manifest ve withheld CSV
  üzerinden iki bağımsız yoldan doğrulandı.
* **G10** — her dizide **RL run1** için `evo_ape -r trans_part` çalıştırıldı.
* **G11** — `trajectory_eval` hizalama uygulamaz; `evo` çıktısı `(not aligned)`.

---

# 3. Yardımcı gözlem — ön kayıtlı kümenin DIŞINDA

> Bu bölümdeki dört dizi **birincil kümeye dahil DEĞİLDİR.** **6/6 tamamlanma
> sayımına girmezler.** Birincil istatistiğe, toplu sonuca, aralığa, orana veya
> sıralamaya **katılmazlar**. Dizi seçimi için kanıt olarak kullanılamazlar.
> Yalnızca dayanıklılık ve mekanizma gözlemi içindir (sözleşme §5).

Alan 14–21 burada da **strict ortak destek** üzerindedir.

| alan | `0032` | `0070` | `0079` | `0104` |
|---|---|---|---|---|
| 2 `category` | Road | **ÖLÇÜLEMEDİ** | **ÖLÇÜLEMEDİ** | **ÖLÇÜLEMEDİ** |
| 3 `measured_frame_count` | 390 | 420 | 100 | 312 |
| 4 `measured_duration_s` | 40.312 | 43.307 | 10.240 | 32.171 |
| 5 `interpolated_frame_count` | 0 | 20 | 1 | 12 |
| 6 `usable_reference_count` | 390 | 400 | 99 | 300 |
| 7 `post_init_gnss_candidate_count` | 389 | 418 | 99 | 311 |
| 8 `candidate_rate_hz` | 9.6489 | 9.6748 | 9.6643 | 9.6660 |
| 9 `selected_gnss_count` | 38 | 41 | 9 | 30 |
| 10 `kerteriz_..._digest` | 485574586968950018 | 15777332552988670075 | 11975423952971707770 | 443364947745553461 |
| 11 `rl_..._digest` | 485574586968950018 | 15777332552988670075 | 11975423952971707770 | 443364947745553461 |
| 12 `digest_equality` | PASS | PASS | PASS | PASS |
| 13 `withheld_strict_common_support_n` | 350 | 357 | 88 | 269 |
| 14 `kerteriz_withheld_rmse_m` | 0.191184 | 0.146370 | 0.099495 | 0.221909 |
| 15 `kerteriz_withheld_max_m` | 0.397368 | 0.394450 | 0.269643 | 0.709779 |
| 16 `rl_run1_rmse_m` | 2.043156 | 0.675261 | 0.300934 | 9.035917 |
| 17 `rl_run2_rmse_m` | 2.043156 | 0.675261 | 0.300934 | 9.035917 |
| 18 `rl_run3_rmse_m` | 2.043156 | 0.675261 | 0.300934 | 9.035917 |
| 19 `rl_mean_rmse_m` | 2.043156 | 0.675261 | 0.300934 | 9.035917 |
| 20 `rl_rmse_min_max_range_m` | (2.043156, 2.043156) | (0.675261, 0.675261) | (0.300934, 0.300934) | (9.035917, 9.035917) — yayılım 6.1e-12 m |
| 21 `rl_max_error_m` | 5.655877 | 2.380204 | 0.678942 | 35.000115 |
| 22 `evo_arithmetic_crosscheck_abs_diff_m` | 4.4e-16 | 1.1e-16 | 5.6e-17 | 1.8e-15 |
| 23 `kerteriz_rejection_diagnostics` | 0 · 0 · 0 | 0 · 0 · 0 | 0 · 0 · 0 | 0 · 0 · 0 |

## 3.1 Yardımcı küme kapıları

| kapı | `0032` | `0070` | `0079` | `0104` |
|---|---|---|---|---|
| G1 envanter tam | PASS | **FAIL** | **FAIL** | **FAIL** |
| G2 … G12 | PASS | PASS | PASS | PASS |

**G1 FAIL gerekçesi — `0070`, `0079`, `0104`.** Sözleşme §2'nin **alan 2
(`category`)** değeri `ÖLÇÜLEMEDİ`'dir. `sequence_manifest.md` bu üç dizi için
kategoriyi bilerek yazmamıştır: doğrulanabilir yerel bir kaynak yoktur ve
kategori uydurulmaz. Veri seti klasörlerinde de kategori bilgisi bulunmaz.

Sözleşme §3'ün FAIL kuralı uygulanmıştır: diziler tablodan **çıkarılmamış**,
durumları `FAIL` yazılmış, düşen kapı (**G1**) ve nedeni belirtilmiş, ölçülebilen
**bütün** alanları raporlanmıştır. Bu diziler zaten hiçbir toplu ölçüte
girmediği için FAIL bir aralığı da etkilemez.

---

# 4. Toplama politikası — bu turda toplu özet ÜRETİLMEDİ

Sözleşme §4'ün **izin verdiği** dört betimleyici özet (Kerteriz RMSE aralığı,
RL RMSE aralığı, destek boyutu aralığı, ıraksama gözlemi) **bu dosyada bilerek
üretilmemiştir**. İzin opsiyoneldir ve kullanılmamıştır.

Sözleşme §4 uyarınca aşağıdakiler **üretilmemiştir ve üretilmeyecektir**:

* genel "Kerteriz vs `robot_localization` kazananı"
* "X kat daha iyi" türü üstünlük ifadesi
* toplam skor veya doğruluk sıralaması
* global pooled RMSE
* süre ağırlıklı veya benzeri ağırlıklandırılmış skor
* yardımcı kümeden herhangi bir dizinin toplu bir sayıya katılması

**F2.4-D'nin birincil sonucu dizi başına sonuçlardır.** Birleştirilmiş tek bir
sayı birincil sonuç değildir.

---

# 5. Bilimsel sınırlar

Sözleşme §6'nın beş sınırı **aynen** taşınır:

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

Alan 22 bu dosyada **"evo ölçüt-aritmetiği çapraz kontrolü"** adıyla geçer;
*"bağımsız doğrulama"* **denmez**.

---

# 6. Tekrarlanabilirlik sınıfları

| artefakt | sınıf |
|---|---|
| `gnss_sampling` seçim politikası ve testleri | **A** — yalnız commit'lenmiş koddan üretilebilir |
| withheld CLI sözleşmesi ve testleri | **A** |
| `trajectory_eval` ölçüt ve eşleştirme testleri | **A** |
| dizi envanteri (alanlar 3–9) | **B** — harici KITTI verisi gerektirir |
| withheld RMSE / maks, RL tekrarları, digest'ler (alanlar 10–21, 23) | **B** |
| `evo` çapraz kontrolü (alan 22) | **B** |
| [`baseline_audit.md`](baseline_audit.md) | **C** — teşhis kaydı, `make results` üretmez |

`baseline_audit.md` **C sınıfında kalır.** Bu içeriğin Faz 2 tekrarlanabilirlik
kapsamına alınıp alınmayacağı F2.5 kapsamı belirlenirken karara bağlanacaktır;
bu dosya o kararı **vermez**.

**B sınıfı artefaktlar depoda saklanmaz.** Gerçek veri setleri depoya konmaz;
koşu çıktıları yerel olarak üretilir. Yukarıdaki sayılar `2011_09_26` takvim
gününün on dizisiyle, bu dosyanın başındaki kod sürümünde ölçülmüştür.

---

# 7. Tamamlanma durumu

| madde | durum |
|---|---|
| §1'deki 6/6 ön kayıtlı dizi mevcut ve koşturulmuş | **EVET** |
| her dizi için tüm kabul kapıları raporlanmış | **EVET** (birincil 12/12 PASS; yardımcı G1 FAIL × 3 gerekçeli) |
| yardımcı küme ayrı tabloda, ön kayıtlı istatistiklerin dışında | **EVET** |
| sonuç sözleşmesine sonuçtan sonra hiçbir ölçüt eklenmemiş | **EVET** |
| final dokümanlar gerçek sonuçlarla doldurulmuş | **EVET** (bu dosya) |
| tam yerel doğrulama geçmiş | **EVET** |
| PR CI yeşil | **HENÜZ DEĞİL** |

Geriye tek bir madde kaldı: **PR CI yeşil olmadan F2.4-D devam ediyor
sayılır**; "büyük ölçüde tamam" diye raporlanmaz.

---

`same OXTS family != independent ground truth`
