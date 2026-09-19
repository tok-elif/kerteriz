# Kerteriz

[![CI](https://github.com/tok-elif/kerteriz/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/tok-elif/kerteriz/actions/workflows/ci.yml)

**Çoğu kestirimci nerede olduğunu söyler. Kerteriz'in hedefi *ne kadar
güvenebileceğini* de söylemek.**

ROS 2 için çok-sensörlü durum kestirim çerçevesi. **Nihai hedefi**, kestirim hatası için
çalışma anında sayısal bir üst sınır (*protection level*) üretmek, tutarlılığı Monte Carlo
NEES/NIS ile ölçmek ve SE₂(3) tabanlı değişmez (invariant) kestirim mimarisini
desteklemektir.

Bugün bunların hangisinin **uygulanmış** olduğu aşağıda [Bugün ne var](#bugün-ne-var)
bölümünde tek tek yazılıdır: Faz 1 sonunda çalışan füzyon **ESKF**'tir; InEKF, Monte Carlo
NEES ve protection level henüz **yoktur**.

> **Durum: Faz 0 ve Faz 1 kapandı — Faz 2 (kanıt) aktif.** Mimari dondurulmuştur (ADR-1…24).
> ESKF, Faz 1 ölçüm modelleri, ölçüm tamponu ve gerçek KITTI yolu **çalışır durumdadır**;
> Monte Carlo/NEES, `evo` ve protection level henüz **yoktur** (Faz 2 ve Faz 4).
> Faz 1 kapanış kararı ve ölçülen ATE farkı: [`docs/SPEC.md`](docs/SPEC.md) §9.1.

---

## Neden

Açık kaynakta IMU + GNSS füzyonu yapan yüzlerce repo var ve neredeyse hepsi aynı şeyi
yapıyor: bir yörünge çiziyor, grafik koyuyor, "çalışıyor" diyor.

Hiçbiri şunu göstermiyor: **filtrenin ürettiği kovaryans doğru mu?**

Bu önemsiz bir ayrıntı değil. Kestirimciyi tüketen her şey — planlama, kontrol, emniyet
mantığı — kovaryansa güvenerek karar verir. Aşırı-iyimser bir kovaryans, sapmış bir
konumdan daha tehlikelidir: sistem yanlış olduğunu bilmez.

## Üç katman — hedeflenen tasarım

Aşağıdaki üç katman projenin **hedefidir**, bugünkü durumu değil. Her satırın ne kadarının
uygulandığı "Durum" sütununda; ayrıntı için [Bugün ne var](#bugün-ne-var).

| Katman | Ne | Durum |
|---|---|---|
| **Yapısal olarak daha iyi koşullu** | Navigasyon çekirdeği SE₂(3) üzerinde; değişmez hata yapısı linearizasyon kaynaklı tutarsızlığı azaltmayı hedefler | Durum SE₂(3) üzerinde **var**; değişmez (InEKF) backend **Faz 3** |
| **Tutarlılığı ölçülmüş** | Monte Carlo NEES ve çevrimiçi NIS, χ² güven bantlarıyla — avantaj varsayılmaz, ölçülür | Çevrimiçi NIS ve χ² kapılama **var**; Monte Carlo NEES **Faz 2** |
| **Çalışma anında farkında** | NIS tabanlı χ² kapılama, arıza tespiti ve izolasyonu, protection level, kademeli bozulma | χ² kapılama **var**; FDI ve protection level **Faz 4** |

## Bugün ne var

Faz 1 sonunda **uygulanmış ve test edilmiş** olanlar:

| Katman | İçerik |
|---|---|
| Çekirdek | ROS'suz C++17 kestirim çekirdeği, SE₂(3) navigasyon durumu (15 DoF + sınırlı augmentation) |
| Yayılım | `ImuPropagator` — birinci mertebe strapdown, tam rotasyon, ayrık F/Q |
| Backend | `EskfBackend` — Joseph formu, simetrizasyon, LDLT, χ² kapılama |
| Ölçümler | `GnssPosition`, `GnssVelocity`, `WheelVelocity`, `NonHolonomic`, `ZeroVelocity` |
| Ardışık düzen | `MeasurementBuffer` (sırasız ölçüm, geri sarma, yeniden yayılım) + `Estimator` |
| Bütünlük | NIS/kapılama altyapısı ve `NisMonitor`/`FaultDetector` snapshot iskeleti (**pasif**) |
| Doğrulama | Sentetik ESKF yakınsama smoke testi |
| Veri seti | KITTI raw OXTS adaptörü, NCLT adaptörü (IMU + GNSS) |
| Gerçek veri | KITTI çevrimdışı koşucusu — üretim zincirinden geçer |
| Kıyas | **Harici** `robot_localization` karşılaştırma yolu |
| Ölçüt | Öteleme ATE aracı (hizalamasız) |

**Henüz yok:** Monte Carlo/NEES, `evo`, otomatik rapor (Faz 2); InEKF, göreli poz,
çevrimiçi teker kalibrasyonu (Faz 3); FDI ve protection level'ın çalışma anındaki
kullanımı (Faz 4).

## Mimari

Tek belirleyici kural: **kestirim çekirdeği ROS bilmez.**

```
kerteriz_core/     saf C++17 — Eigen + manif. ROS kurulu olmayan makinede derlenir.
kerteriz_ros/      ince ROS 2 adaptörü: lifecycle node, component, TF2, teşhis
kerteriz_msgs/     teşhis ve bütünlük mesajları
kerteriz_sim/      deterministik yörünge ve sensör üreteci
kerteriz_eval/     Python: evo, NEES/NIS, Stanford diyagramı, rapor
kerteriz_bringup/  launch dosyaları, YAML, veri seti adaptörleri
```

## Derleme

**Hedef ortam:** Ubuntu 22.04 + ROS 2 Humble (CI ayrıca Jazzy'de koşar).

### Çekirdek — ROS olmadan

Bu yol, ROS bağımsızlığının yapısal kanıtıdır:

```bash
cmake -S kerteriz_core -B build/core -DCMAKE_BUILD_TYPE=Release
cmake --build build/core -j
ctest --test-dir build/core --output-on-failure
```

### Tüm çalışma alanı — colcon

```bash
colcon build --symlink-install
colcon test
colcon test-result --all --verbose
```

Bağımlılıklar: Eigen 3.4 (sistem; yoksa indirilir), [manif](https://github.com/artivis/manif)
0.0.5 ve GoogleTest 1.14.0 (FetchContent, sürümler pinli).

---

## Sentetik doğrulama — önce burası

R6: her filtre gerçek veriden **önce** sentetik veride doğrulanır. Sentetik ESKF
yakınsama testi `kerteriz_sim` içindedir:

```bash
cmake -S kerteriz_sim -B build/sim
cmake --build build/sim -j
ctest --test-dir build/sim --output-on-failure -R test_eskf_smoke
```

Test yakınsamayı, NIS özetini, determinizmi (aynı tohum → bit-bit aynı sonuç) ve
kovaryans sağlığını doğrular.

---

## KITTI Raw ile gerçek veri

### Beklenen veri seti biçimi

```
<drive>_sync/
└── oxts/
    ├── timestamps.txt
    └── data/
        ├── 0000000000.txt
        ├── 0000000001.txt
        └── ...
```

- **Görüntü klasörleri gerekmez** (`image_00` … `image_03` okunmaz).
- **Velodyne gerekmez** (`velodyne_points` okunmaz).
- **Gerçek veri setleri depoya konmaz.** Depodaki tek veri
  `kerteriz_bringup/test/fixtures/` altındaki *sentetik biçim fixture'larıdır*; bunlar
  bir kıyaslama sonucu değildir.

Veri seti yolu ve çıktı yolu **dışarıdan** verilir; koda gömülü yol yoktur.

### Kerteriz koşucusu

```bash
ros2 run kerteriz_bringup kerteriz_kitti_runner --dataset <drive>_sync --output /tmp/kerteriz_traj.csv --reference /tmp/reference.csv
```

Çıktı sütunları: `timestamp_ns, px, py, pz, vx, vy, vz`.

Zincir **üretim kodudur**; alternatif bir filtre yoktur:

```
KITTI adaptoru -> DatasetEvent -> MeasurementBuffer -> Estimator -> EskfBackend
```

Yalnızca-konum kipi (harici taban çizgisiyle eşdeğer bilgi kümesi):

```bash
ros2 run kerteriz_bringup kerteriz_kitti_runner --dataset <drive>_sync --output /tmp/kerteriz_posonly.csv --no-gnss-velocity
```

Harici taban çizgisi için başlangıç durumu parametrelerini üretmek:

```bash
ros2 run kerteriz_bringup kerteriz_kitti_runner --dataset <drive>_sync --output /tmp/kerteriz_posonly.csv --no-gnss-velocity --emit-baseline-params /tmp/baseline_init.yaml
```

Bu dosya, Kerteriz'in kullandığı **aynı** başlatma kaydından üretilir; iki tarafın
başlangıç bilgisi elle senkronize edilen iki dosyaya değil tek kod yoluna dayanır.

**Ara-değerleme politikası.** KITTI kısa OXTS kesintilerinde tüm değerleri doğrusal
ara-değerler ve son üç kipi `-1` yapar. Koşucu bunu açık bir politikayla ele alır:
ara-değerlenmiş GNSS konum/hız **ölçüm olarak kullanılmaz**, ara-değerlenmiş referans
**ATE'ye girmez**, IMU ise **düşürülmez** (yayılım zaman sürekliliği gerektirir) ama ayrı
sayılır ve gerçek gözlem gibi raporlanmaz.

---

## Harici `robot_localization` taban çizgisi

`robot_localization` **Kerteriz'in kestirimcisinin parçası değildir.** Kerteriz onu
çağırmaz, sonucunu okumaz ve ona bağımlı değildir; `kerteriz_core` bu paketi tanımaz.
İki süreç aynı veri setini bağımsız tüketir:

```
ayni KITTI veri seti
  |
  +-> Kerteriz ESKF          -> yorunge_K
  |
  +-> robot_localization EKF -> yorunge_RL
```

```bash
ros2 launch kerteriz_bringup robot_localization_baseline.launch.py \
  dataset_dir:=<drive>_sync \
  output_csv:=/tmp/baseline_traj.csv \
  initial_state_params:=/tmp/baseline_init.yaml \
  gnss_position_stride:=0
```

`gnss_position_stride` **zorunludur ve varsayılanı yoktur**: `0` tam hızlı (legacy)
koşudur, pozitif bir değer F2.4-C seyreltmesidir ve Kerteriz koşucusuna verilen
`--gnss-position-stride` ile **aynı** olmalıdır. Sessiz bir varsayılan, iki sürecin
farklı GNSS setiyle koşup sonucun yine de makul görünmesi anlamına gelirdi.

Filtre ayarları [`kerteriz_bringup/config/robot_localization_baseline.yaml`](kerteriz_bringup/config/robot_localization_baseline.yaml)
içindedir. Bilgi kümesi farkları (GNSS hızının verilememesi, bias durumunun olmaması,
süreç gürültüsünün eşlenememesi) o dosyada **açıkça** yazılıdır; başlangıç kovaryansı
gizli pakete varsayılanına bırakılmaz.

---

## ATE

```bash
ros2 run kerteriz_bringup kerteriz_ate --reference /tmp/reference.csv --estimate /tmp/kerteriz_posonly.csv --label "Kerteriz"
ros2 run kerteriz_bringup kerteriz_ate --reference /tmp/reference.csv --estimate /tmp/baseline_traj.csv  --label "robot_localization"
```

Öteleme RMSE'si; **hiçbir hizalama uygulanmaz** (Sim(3) veya rijit fit yok). Zaman
eşleştirme deterministiktir: en yakın damga, tolerans dışı örnek dışlanır.

### Faz 1 kapanışında ölçülen ham sonuçlar

`2011_09_26_drive_0013_sync` · 144 kare · 14.81 s · 20 ms tolerans · hizalama yok:

| koşu | ATE RMSE | maks | eşleşen |
|---|---|---|---|
| Kerteriz — konum + hız | 0.329732 m | 0.490325 m | 143 |
| Kerteriz — yalnız konum | 0.120213 m | 0.257459 m | 143 |
| `robot_localization` (harici) | 0.029949 m | 0.084202 m | 143 |

**Tekrarlanabilirlik — iki taraf aynı değil.** Kerteriz koşucusu çevrimdışı ve
**bit-bit tekrarlanabilirdir**: aynı veri seti aynı sayıyı verir. Harici taban çizgisi ise
canlı bir ROS ardışık düzeninden geçer (`/clock`, zamanlayıcı, mesaj teslimi), dolayısıyla
**bit-bit tekrarlanabilir değildir**; aynı girdiyle koşudan koşuya küçük fark görülür —
ölçülen yayılım RMSE'de `0.029949 → 0.031462 m`, maksimumda `0.084202 → 0.113537 m`.
Yukarıdaki tablo Faz 1 kapanış anındaki tek bir koşudur; taban çizgisi satırı bu yayılımla
birlikte okunmalıdır.

**Bunlar bağımsız bilimsel doğrulama değildir.** KITTI'de referans poz ile filtreye
verilen GNSS aynı OXTS çözümünden gelir. Bu, Faz 1 entegrasyon/MVP doğrulamasıdır.
İki filtre de bu dizi için **ayarlanmamıştır**. Performans denkliği elde edilmemiştir;
ayar, birden çok dizi ve daha güçlü değerlendirme Faz 2'ye devredilmiştir
([`docs/SPEC.md`](docs/SPEC.md) §9.1).

## Dokümantasyon

| Dosya | İçerik |
|---|---|
| [`docs/SPEC.md`](docs/SPEC.md) | Kendi kendine yeten proje tanımı |
| [`docs/CONVENTIONS.md`](docs/CONVENTIONS.md) | Çerçeve, kuaterniyon, `J_res`, Joseph, zaman, birim |
| [`docs/INTERFACES.md`](docs/INTERFACES.md) | `kerteriz_core` arayüz sözleşmesi |
| [`docs/INTEGRATION.md`](docs/INTEGRATION.md) | Yeni bir platforma entegrasyon |
| [`docs/design/DECISIONS.md`](docs/design/DECISIONS.md) | ADR-1…24 — dizin ve henüz bölünmemiş kararlar |
| [`docs/design/ADR-0001-ros-free-core.md`](docs/design/ADR-0001-ros-free-core.md) | ADR-1 · ROS'suz kestirim çekirdeği |
| [`docs/design/ADR-0002-se23-state.md`](docs/design/ADR-0002-se23-state.md) | ADR-2 · Durum SE₂(3) × R⁶ üzerinde |
| [`docs/PHASE0.md`](docs/PHASE0.md) | Faz 0 planı — tamamlandı |
| [`docs/SPEC.md#91-faz-1-kapanış-kararı--ate-ölçütü`](docs/SPEC.md) | Faz 1 kapanış kararı ve ölçülen ATE farkı |
| [`CLAUDE.md`](CLAUDE.md) | Katkıda bulunan ajanlar için kurallar |

## Lisans

MIT — [`LICENSE`](LICENSE)
