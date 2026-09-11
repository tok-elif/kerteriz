# Yeni bir platforma entegrasyon

> Kerteriz'in tasarım hedefi: **yeni bir robota geçmek kod yazmayı değil, YAML yazmayı gerektirir.**
> Bu dosya o sözün nasıl tutulduğunu ve yeni bir sisteme geçerken izlenecek sırayı anlatır.

---

## 0. Taşınabilirliğin dayandığı dört karar

| Karar | Sonucu |
|---|---|
| Çekirdek ROS'suz (R1, ADR-1) | Kestirimci ROS 2, ROS 1 köprüsü, çıplak Linux veya gömülü hedefte aynı koddur |
| Ölçüm eklenti arayüzü (R4, ADR-4) | Sensör kümesi platformdan platforma değişir, backend değişmez |
| Platform sabitleri YAML'da (R5) | Tekerlek yarıçapı, extrinsic, gürültü — hiçbiri derlenmiş kodda değil |
| **Sınırlı kapasiteli augmentation (ADR-14)** | Kalibrasyon durumu **çalışma anında** kaydedilir. `estimate_*_online: true` salt YAML değişikliğidir, yeniden derleme gerektirmez. |

> **"Aynı ikili dosya" sözünün sınırı:** augmentation deposu `kMaxAugmentDof` ile sınırlıdır
> (derleme zamanı sabiti). YAML'dan açılan kalibrasyon blokları bu kapasiteyi aşarsa
> yapılandırma **hata verir**, sessizce kırpılmaz. Varsayılan kapasite normal bir platform
> için fazlasıyla yeterlidir; aşırı durumda tek bir sabiti değiştirip yeniden derlersin.

**Sonuç:** aynı ikili dosya, farklı YAML ile bir Segway'de, bir arabada ve bir İHA'da çalışır.

---

## 1. Entegrasyon sırası

Bu sırayı atlama. Her adımın bir doğrulama çıktısı var; geçmeden sonrakine geçme.

### Adım 1 — Sensör envanteri

Neyin var, hangi hızda, hangi ROS mesaj tipinde?

| Sensör | Tipik hız | Kerteriz'deki karşılığı |
|---|---|---|
| IMU | 100–400 Hz | süreç modeli (zorunlu) |
| GNSS konum | 1–10 Hz | `gnss_position` |
| GNSS hız (Doppler) | 1–10 Hz | `gnss_velocity` |
| Teker enkoderi (ham) | 10–100 Hz | `wheel_velocity` |
| `/odom` (entegre) | 10–50 Hz | `relative_pose` |
| Manyetometre | 10–100 Hz | `magnetometer` |
| Barometre | 10–50 Hz | `barometer` |
| LiDAR/görsel odometri | 10–20 Hz | `relative_pose` |

**IMU zorunludur.** Kerteriz IMU-merkezli bir kestirimcidir: IMU yayılımı yapar,
diğer her şey güncellemedir. IMU yoksa bu çerçeve senin problemin için doğru araç değil.

### Adım 1.5 — Dünya çerçevesi orijini

`W`'nin orijini GNSS'e bağlı değildir; politika YAML'dan seçilir (`CONVENTIONS.md` §1.1).

```yaml
world:
  origin_policy: gnss_first_fix   # gnss_first_fix | manual_llh | first_pose | identity
```

| Durumun | Politika |
|---|---|
| Dış mekân, GNSS var | `gnss_first_fix` |
| Tekrarlanabilir deney, harita hizalama | `manual_llh` |
| **İç mekân, GNSS yok** | `first_pose` — ilk `relative_pose` ölçümünün başlangıcı orijin olur |
| Simülasyon, birim testi | `identity` |

`first_pose` ve `identity` modlarında `W` yalnızca **yerel** tutarlıdır; coğrafi bağ yoktur.
Bu modlarda GNSS ve manyetometre ölçümleri ya kapalıdır ya da orijin kurulana kadar reddedilir
(`RejectReason::kOriginNotSet`).

### Adım 2 — Çerçeveler ve extrinsic'ler

Her sensörün `base_link`'e göre konumu ve yönelimi gerekir.

```yaml
sensors:
  - name: gnss_main
    type: gnss_position
    extrinsic: {xyz: [0.10, 0.0, 0.85], rpy_deg: [0, 0, 0]}   # anten kolu
```

- CAD'den ölçebiliyorsan ölç. Birkaç cm hata GNSS için kabul edilebilir.
- Bilmiyorsan veya güvenmiyorsan: `estimate_extrinsic_online: true` ile duruma ekle.
  Gözlemlenebilirlik hareketten gelir — düz gitmek yetmez, dönüş gerekir.
- Kamera/LiDAR ↔ IMU için [Kalibr](https://github.com/ethz-asl/kalibr) kullan, tahmin etme.

**Doğrulama:** robotu elle döndür, `/tf` ağacında sensörler doğru yerde görünmeli.

### Adım 3 — IMU gürültü kalibrasyonu

Veri sayfasındaki değerleri **kullanma**; ölçülen değerler genelde 2–5 kat farklıdır.

```bash
# IMU'yu 2+ saat hareketsiz bırak, kaydet
ros2 bag record /imu/data -o imu_static

# Allan varyansı ile parametreleri çıkar
# (Kalibr'in imu_utils'i veya kerteriz_eval/allan_variance.py)
```

Çıkan dört sayıyı YAML'a yaz (`CONVENTIONS.md` §6).

**Doğrulama:** Allan sapması grafiğinde -1/2 ve +1/2 eğimli bölgeler görünmeli.

### Adım 4 — Hangi ölçümler açık

```yaml
sensors:
  - {name: gnss_main, type: gnss_position,  enabled: true}
  - {name: wheel,     type: wheel_velocity, enabled: true}
  - {name: nhc,       type: non_holonomic,  enabled: true}
  - {name: zupt,      type: zero_velocity,  enabled: true}
```

Platforma göre:

| Platform | Aç | Kapat |
|---|---|---|
| Tekerlekli yer robotu | `wheel_velocity`, `non_holonomic`, `zero_velocity` | — |
| Araba | `wheel_velocity`, `non_holonomic` | `zero_velocity` (durma nadir) |
| Paletli / kayarak dönen | `wheel_velocity` (yüksek gürültüyle) | **`non_holonomic`** — palet yanal kayar |
| İHA / drone | — | `wheel_velocity`, `non_holonomic`, `zero_velocity` |
| İç mekân, GNSS yok | `relative_pose` (LiDAR/görsel odometri) | `gnss_*` |

> **Uyarı:** `non_holonomic`, aracın yana kaymadığı varsayımıdır. Paletli araçta, buzda,
> veya agresif viraj alan bir yarış aracında bu varsayım bozulur ve filtre sapar.
> Emin değilsen kapat; kapalıyken sapma artar ama sonuç güvenilir kalır.

### Adım 5 — Başlangıç ve yakınsama

```yaml
initialization:
  method: gnss_static        # gnss_static | gnss_motion | manual
  static_duration_s: 3.0     # yerçekiminden roll/pitch, jiroskop biasi
  yaw_source: gnss_course    # gnss_course | magnetometer | manual
```

Yaw en zor olanıdır: yerçekimi roll ve pitch verir, yaw vermez.
Kaynaklar: GNSS rota açısı (hareket gerekir), manyetometre (bozulmaya açık),
çift anten GNSS (en iyi), ya da başlangıçta büyük belirsizlikle bırakıp yakınsamasını beklemek.

**InEKF'in burada somut üstünlüğü var:** büyük yaw belirsizliğinde ESKF tutarlılığını kaybeder,
InEKF kaybetmez. Kalkışta yaw'ı bilmiyorsan InEKF kullan.

### Adım 6 — Doğrulama: filtre gerçekten çalışıyor mu

**"Grafik güzel görünüyor" kabul kriteri değildir.** Sırayla:

1. **NIS kontrolü** — her sensör için `/kerteriz/diagnostics`'teki NIS ortalaması
   serbestlik derecesine yakın olmalı. 3 boyutlu GNSS için ≈ 3.
   - NIS çok yüksek → gürültü modeli fazla iyimser, ya da extrinsic yanlış
   - NIS çok düşük → gürültü modeli fazla kötümser, ölçümden faydalanmıyorsun
2. **Kabul oranı** — sabit bir band yoktur; kriter **yapılandırdığın güvene** bağlıdır
   (`CONVENTIONS.md` §10). `gate.chi2_confidence = c` ise beklenen oran `c`'dir ve sonlu
   örneklemde bandı `c ± 3√(c(1−c)/N)`'dir.
   Örnek: `c = 0.997`, `N = 1000` → beklenen `[99.2%, 100%]`.
   *Sabit "%95–99" kriteri kullanma — `c = 0.997` için doğru çalışan filtreyi hatalı işaretler.*
   Oran bandın altındaysa bir şey yanlış; kapıyı genişletmek çözüm değil, semptomu gizlemek.
3. **Bias yakınsaması** — jiroskop ve ivmeölçer biasları sabit bir değere oturmalı.
   Sürüklenmeye devam ediyorsa gözlemlenebilirlik yetersiz veya model yanlış.
4. **Ground truth varsa** — `kerteriz_eval` ile ATE/RPE ve NEES.

---

## 2. Sıfırdan yeni bir sensör tipi eklemek

Bu, yeni platform desteklemenin tek "kod yazılan" hâlidir ve tek dosyadır.

```cpp
// kerteriz_core/include/kerteriz/measurements/my_sensor.hpp
class MySensor final : public Measurement {
 public:
  TimeNs           stamp_ns()     const override { return stamp_; }
  int              residual_dim() const override { return 3; }
  std::string_view name()         const override { return name_; }

  // Mutlak ölçüm: klon gerekmiyor. Göreli ölçümse required_clones() doldurulur.

  void evaluate(const StateBundle& states, MeasurementWorkspace& w) const override {
    const NavState& x = states.current();
    w.dim = residual_dim();

    // w.r     = z ⊖ h(x)
    // w.J_res = ∂r(x ⊞ δ)/∂δ      ← residual Jacobian, SAĞ pertürbasyon
    //           (CONVENTIONS.md §3.2 — "H" DEĞİL; işareti ters)
    // w.R     = ölçüm gürültüsü

    w.r.head(w.dim)                      = /* ... */;
    w.J_res.topLeftCorner(w.dim, x.active_dof()).setZero();
    w.J_res.block(0, /*ofset*/ 0, w.dim, 3) = /* ... */;
    w.R.topLeftCorner(w.dim, w.dim)      = R_;
    // Tahsis yok: yalnızca w'ye yazılır (ADR-16).
  }

 private:
  Eigen::Matrix<Scalar, 3, 3> R_;   // sabit boyutlu — değerle MatX döndürme
};
```

**Jacobian sütun ofsetleri.** `w.J_res` durumun tam aktif düzenine göredir. Çekirdek
bloklar sabit ofsettedir; kalibrasyon ve klon blokları için ofseti runtime'da al:

```cpp
const int off = x.calibration_offset("wheel_scale");   // veya x.clone_offset(id)
w.J_res.block(0, off, w.dim, 2) = /* ... */;
```

Sonra üç adım:

1. Fabrikaya kaydet: `registry.add("my_sensor", &MySensor::from_yaml);`
   Kayıt aynı zamanda Jacobian testini de kaydeder — testsiz ölçüm test paketini düşürür
   (`CLAUDE.md` §6.1, denetim 4).
2. Sayısal residual Jacobian testini yaz (R2)
3. YAML'a ekle. Kalibrasyon durumu gerekiyorsa `register_calibration()` ile kaydettir.

Backend, düğüm, tampon — hiçbiri değişmez. Değişiyorsa arayüz yanlış kullanılmıştır.

---

## 3. ROS 2 dışında kullanım

`kerteriz_core` bağımsız bir CMake hedefidir:

```cmake
find_package(kerteriz_core REQUIRED)
target_link_libraries(my_app PRIVATE kerteriz::core)
```

Kendi döngünü kurarsın:

```cpp
auto backend = std::make_unique<kerteriz::EskfBackend>(config);
kerteriz::Estimator estimator(std::move(backend), kerteriz::NisMonitor{},
                              kerteriz::FaultDetector{});
kerteriz::MeasurementBuffer buffer(/*window_ns=*/200'000'000);

while (running) {
  buffer.add_imu(read_imu());
  buffer.add_measurement(std::make_unique<GnssPosition>(read_gnss()));
  const auto results = buffer.process(estimator);
  (void)results;  // istersen teşhis/log için tüket
  publish(estimator.backend().state());
}
```

Not: örnekteki `std::make_unique` ölçüm kuyruğunun **sahiplik** katmanındadır; ADR-22'nin
allocation-free sayısal hot path'i `predict/update/evaluate` içidir. Hard-real-time hedefte
ölçüm sahipliği ayrıca pool/variant ile tahsissiz yapılabilir.

Bu yolla kullanılabileceği yerler: ROS 1 düğümü, micro-ROS, çıplak Linux servisi,
kayıt sonrası toplu işleme, birim testleri, simülasyon.

---

## 4. Kendi robotuna geçiş kontrol listesi

Donanım geldiğinde sırayla:

- [ ] IMU 2 saat statik kayıt → Allan varyansı → YAML
- [ ] Sensör extrinsic'leri ölçüldü veya Kalibr ile kalibre edildi
- [ ] Tekerlek yarıçapı ve iz genişliği ölçüldü (`estimate_scale_online: true` ile doğrula)
- [ ] Zaman senkronizasyonu doğrulandı — sensörler aynı saati mi kullanıyor?
      (Farklı saatler kestirimi sessizce bozar; PTP/NTP ya da donanım tetikleme)
- [ ] Açık alanda düz bir tur: NIS ve kabul oranı kontrol edildi
- [ ] Sekiz çizerek tur: bias ve extrinsic yakınsaması gözlendi
- [ ] GNSS'i elle kapatma denemesi: sapma hızı ve protection level büyümesi makul mü
- [ ] Statik test: ZUPT devrede mi, konum kayıyor mu

**Zaman senkronizasyonu en çok göz ardı edilen ve en çok zarar veren maddedir.**
10 ms'lik sabit bir IMU–GNSS gecikmesi, 10 m/s hızda 10 cm'lik sistematik hata demektir.
Şüpheleniyorsan gecikmeyi duruma ekleyip çevrimiçi kestir.
