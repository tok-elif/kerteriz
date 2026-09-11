# Kerteriz

**Çoğu kestirimci nerede olduğunu söyler. Kerteriz *ne kadar güvenebileceğini* de söyler.**

ROS 2 için çok-sensörlü durum kestirim çerçevesi: kestirim hatasına çalışma anında sayısal
üst sınır (*protection level*) üreten, tutarlılığı Monte Carlo NEES analiziyle doğrulanan,
SE₂(3) üzerinde değişmez (invariant) EKF tabanlı füzyon.

> **Durum: Faz 0 — iskelet.** Mimari dondurulmuştur (ADR-1…22). Filtre henüz yoktur;
> yol haritası için [`docs/PHASE0.md`](docs/PHASE0.md).

---

## Neden

Açık kaynakta IMU + GNSS füzyonu yapan yüzlerce repo var ve neredeyse hepsi aynı şeyi
yapıyor: bir yörünge çiziyor, grafik koyuyor, "çalışıyor" diyor.

Hiçbiri şunu göstermiyor: **filtrenin ürettiği kovaryans doğru mu?**

Bu önemsiz bir ayrıntı değil. Kestirimciyi tüketen her şey — planlama, kontrol, emniyet
mantığı — kovaryansa güvenerek karar verir. Aşırı-iyimser bir kovaryans, sapmış bir
konumdan daha tehlikelidir: sistem yanlış olduğunu bilmez.

## Üç katman

| Katman | Ne |
|---|---|
| **Yapısal olarak daha iyi koşullu** | Navigasyon çekirdeği SE₂(3) üzerinde; değişmez hata yapısı linearizasyon kaynaklı tutarsızlığı azaltmayı hedefler |
| **Tutarlılığı ölçülmüş** | Monte Carlo NEES ve çevrimiçi NIS, χ² güven bantlarıyla — avantaj varsayılmaz, ölçülür |
| **Çalışma anında farkında** | NIS tabanlı χ² kapılama, arıza tespiti ve izolasyonu, protection level, kademeli bozulma |

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
colcon test --packages-select kerteriz_core
colcon test-result --verbose
```

Bağımlılıklar: Eigen 3.4 (sistem; yoksa indirilir), [manif](https://github.com/artivis/manif)
0.0.5 ve GoogleTest 1.14.0 (FetchContent, sürümler pinli).

## Dokümantasyon

| Dosya | İçerik |
|---|---|
| [`docs/SPEC.md`](docs/SPEC.md) | Kendi kendine yeten proje tanımı |
| [`docs/CONVENTIONS.md`](docs/CONVENTIONS.md) | Çerçeve, kuaterniyon, `J_res`, Joseph, zaman, birim |
| [`docs/INTERFACES.md`](docs/INTERFACES.md) | `kerteriz_core` arayüz sözleşmesi |
| [`docs/INTEGRATION.md`](docs/INTEGRATION.md) | Yeni bir platforma entegrasyon |
| [`docs/design/DECISIONS.md`](docs/design/DECISIONS.md) | ADR-1…22 |
| [`docs/PHASE0.md`](docs/PHASE0.md) | Aktif faz planı |
| [`CLAUDE.md`](CLAUDE.md) | Katkıda bulunan ajanlar için kurallar |

## Lisans

MIT — [`LICENSE`](LICENSE)
